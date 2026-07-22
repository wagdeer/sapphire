#include <sapphire/backend/pose_graph.hpp>
#include <sapphire/mapping/occupancy_grid.hpp>
#include <sapphire/odometry/voxel_filter.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace sapphire {
namespace {

double rotationAngle(const Isometry3d& transform) {
    Eigen::Quaterniond rotation(transform.rotation());
    rotation.normalize();
    return 2.0 * std::atan2(
        rotation.vec().norm(), std::abs(rotation.w()));
}

gtsam::Pose3 toGtsam(const Isometry3d& pose) {
    return gtsam::Pose3(pose.matrix());
}

Isometry3d fromGtsam(const gtsam::Pose3& pose) {
    return Isometry3d(pose.matrix());
}

PointCloudPtr transformedCloud(
    const PointCloudConstPtr& cloud,
    const Isometry3d& transform)
{
    auto output = std::make_shared<PointCloud>();
    pcl::transformPointCloud(
        *cloud, *output, transform.matrix());
    for (Point& point : output->points) {
        point.data[3] = 1.0f;
    }
    return output;
}

PointCloudPtr voxelized(
    const PointCloudConstPtr& cloud,
    double leaf_size)
{
    if (leaf_size <= 0.0) {
        return std::make_shared<PointCloud>(*cloud);
    }
    return deterministicVoxelDownsample(*cloud, leaf_size);
}

}  // namespace

class PoseGraphBackend::Impl {
public:
    explicit Impl(const Config::Pgo& config)
        : config_(config)
    {
        if (!config_.enabled) {
            return;
        }

        gtsam::ISAM2Params params;
        params.relinearizeThreshold = 0.01;
        params.relinearizeSkip = 1;
        isam2_ = std::make_unique<gtsam::ISAM2>(params);

        gtsam::Vector6 prior_variances;
        prior_variances << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
        prior_noise_ =
            gtsam::noiseModel::Diagonal::Variances(prior_variances);

        gtsam::Vector6 odom_variances;
        odom_variances << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
        odom_noise_ =
            gtsam::noiseModel::Diagonal::Variances(odom_variances);

        if (config_.occupancy.enabled) {
            occupancy_ = std::make_unique<OccupancyGrid>(config_.occupancy);
            spdlog::info(
                "[pgo] occupancy grid enabled (reso={:.2f}m, d_max={:.2f}m)",
                config_.occupancy.resolution,
                config_.occupancy.d_max);
        }

        worker_ = std::thread([this]() { workerLoop(); });
        spdlog::info("[pgo] asynchronous pose-graph backend enabled");
    }

    ~Impl() {
        stop_.store(true, std::memory_order_release);
        wake_cv_.notify_all();
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    bool enabled() const {
        return config_.enabled;
    }

    void addFrame(
        const PointCloudConstPtr& cloud_odom,
        const Isometry3d& T_odom_lidar,
        double stamp)
    {
        if (!config_.enabled || !cloud_odom || cloud_odom->empty()
            || !T_odom_lidar.matrix().allFinite()
            || !std::isfinite(stamp)) {
            return;
        }

        std::lock_guard<std::mutex> lock(input_mutex_);
        if (!input_frames_.empty()) {
            const Isometry3d delta =
                input_frames_.back().T_odom_lidar.inverse() * T_odom_lidar;
            if (delta.translation().norm() < config_.keyframe_distance
                && rotationAngle(delta) < config_.keyframe_rotation) {
                return;
            }
        }

        const double travel_distance = input_frames_.empty()
            ? 0.0
            : input_frames_.back().travel_distance
                + (T_odom_lidar.translation()
                   - input_frames_.back().T_odom_lidar.translation()).norm();
        input_frames_.push_back({
            cloud_odom,
            T_odom_lidar,
            stamp,
            travel_distance,
        });
    }

    Isometry3d correction() const {
        std::lock_guard<std::mutex> lock(output_mutex_);
        return T_map_odom_;
    }

    PoseGraphStats stats() const {
        std::scoped_lock lock(input_mutex_, output_mutex_);
        PoseGraphStats result = stats_;
        result.keyframes = input_frames_.size();
        return result;
    }

    PoseGraphSnapshot snapshot() const {
        PoseGraphSnapshot result;
        {
            std::lock_guard<std::mutex> lock(output_mutex_);
            result.stats = stats_;
            result.optimized_poses = optimized_poses_snapshot_;
            result.loop_edges = loop_edges_snapshot_;
        }
        {
            std::lock_guard<std::mutex> lock(input_mutex_);
            result.stats.keyframes = input_frames_.size();
        }
        return result;
    }

    void requestSnapshot() {
        if (config_.enabled) {
            snapshot_requested_.store(true, std::memory_order_release);
            wake_cv_.notify_one();
        }
    }

    void requestGlobalMap() {
        if (config_.enabled) {
            map_requested_.store(true, std::memory_order_release);
            wake_cv_.notify_one();
        }
    }

    PointCloudConstPtr latestGlobalMap() const {
        std::lock_guard<std::mutex> lock(output_mutex_);
        return global_map_;
    }

    void requestOccupancyGrid() {
        if (config_.enabled && occupancy_) {
            occupancy_requested_.store(true, std::memory_order_release);
            wake_cv_.notify_one();
        }
    }

    std::shared_ptr<const OccupancyGridMsg> latestOccupancyGrid() const {
        std::lock_guard<std::mutex> lock(occupancy_mutex_);
        return occupancy_msg_;
    }

private:
    struct InputFrame {
        PointCloudConstPtr cloud_odom;
        Isometry3d T_odom_lidar = Isometry3d::Identity();
        double stamp = 0.0;
        double travel_distance = 0.0;
    };

    struct GraphFrame {
        PointCloudConstPtr cloud_lidar;
        Isometry3d T_odom_lidar = Isometry3d::Identity();
        double stamp = 0.0;
        double travel_distance = 0.0;
    };

    void workerLoop() {
        const auto period =
            std::chrono::duration<double>(config_.update_period_sec);
        while (!stop_.load(std::memory_order_acquire)) {
            std::unique_lock<std::mutex> lock(wake_mutex_);
            wake_cv_.wait_for(lock, period, [this]() {
                return stop_.load(std::memory_order_acquire)
                    || snapshot_requested_.load(std::memory_order_acquire)
                    || map_requested_.load(std::memory_order_acquire)
                    || occupancy_requested_.load(std::memory_order_acquire);
            });
            lock.unlock();
            if (stop_.load(std::memory_order_acquire)) {
                break;
            }

            try {
                processPending();
            } catch (const std::exception& error) {
                spdlog::error("[pgo] backend update failed: {}", error.what());
            }
        }
    }

    void copyPendingFrames() {
        std::vector<InputFrame> pending;
        {
            std::lock_guard<std::mutex> lock(input_mutex_);
            pending.assign(
                input_frames_.begin()
                    + static_cast<std::ptrdiff_t>(frames_.size()),
                input_frames_.end());
        }
        for (const InputFrame& input : pending) {
            frames_.push_back({
                transformedCloud(
                    input.cloud_odom, input.T_odom_lidar.inverse()),
                input.T_odom_lidar,
                input.stamp,
                input.travel_distance,
            });
        }
    }

    void processPending() {
        copyPendingFrames();
        if (frames_.empty()) {
            return;
        }

        gtsam::NonlinearFactorGraph odom_graph;
        gtsam::Values initial;
        bool odom_updated = false;
        if (buildOdometryGraph(odom_graph, initial)) {
            updateIsam(odom_graph, initial);
            updateCorrection();
            odom_updated = true;
        }

        const size_t loops_before = loop_edge_count_;
        gtsam::NonlinearFactorGraph loop_graph;
        if (buildLoopEdges(loop_graph)) {
            updateIsam(loop_graph, {});
            updateCorrection();
        }
        const bool loop_updated = loop_edge_count_ > loops_before;

        // Occupancy must never block correction/loop work above. Policy:
        //   odom  → cheap incremental append of new keyframes only
        //   loop  → full rebuild (historical poses moved)
        // Export (below) only serializes; it must not trigger a full rebuild
        // just because ISAM nudged poses or RViz polls every second.
        if (occupancy_) {
            if (loop_updated) {
                rebuildOccupancyMap();
            } else if (odom_updated) {
                appendOccupancyFrames();
            }
        }

        if (snapshot_requested_.exchange(false, std::memory_order_acq_rel)
            && !optimized_.empty()) {
            rebuildSnapshot();
        }
        if (map_requested_.exchange(false, std::memory_order_acq_rel)
            && !optimized_.empty()) {
            rebuildGlobalMap();
        }
        if (occupancy_requested_.exchange(false, std::memory_order_acq_rel)
            && occupancy_) {
            publishOccupancySnapshot();
        }
    }

    bool buildOdometryGraph(
        gtsam::NonlinearFactorGraph& graph,
        gtsam::Values& initial)
    {
        if (added_odom_id_ >= frames_.size()) {
            return false;
        }

        if (added_odom_id_ == 0) {
            const gtsam::Pose3 first = toGtsam(frames_.front().T_odom_lidar);
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(
                0, first, prior_noise_));
            initial.insert(0, first);
            added_odom_id_ = 1;
        }

        const size_t last_optimized_id =
            optimized_.empty() ? 0 : optimized_.size() - 1;
        const Isometry3d last_optimized = optimized_.empty()
            ? frames_.front().T_odom_lidar
            : fromGtsam(
                optimized_.at<gtsam::Pose3>(last_optimized_id));

        for (size_t i = added_odom_id_; i < frames_.size(); ++i) {
            const Isometry3d relative =
                frames_[i - 1].T_odom_lidar.inverse()
                * frames_[i].T_odom_lidar;
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                i - 1, i, toGtsam(relative), odom_noise_));

            const Isometry3d estimate =
                last_optimized
                * frames_[last_optimized_id].T_odom_lidar.inverse()
                * frames_[i].T_odom_lidar;
            initial.insert(i, toGtsam(estimate));
            ++added_odom_id_;
        }
        return !graph.empty();
    }

    int searchLoopTarget(size_t query_id, const Isometry3d& query_pose) const {
        int best_id = -1;
        double best_squared_distance =
            config_.loop_search_radius * config_.loop_search_radius;
        const Eigen::Quaterniond query_rotation(query_pose.rotation());

        // A loop target must be historical. When several frames are copied in
        // one worker cycle, optimized_ also contains nodes newer than an early
        // query; admitting those creates duplicate reciprocal loop edges.
        const size_t candidate_count =
            std::min(query_id, optimized_.size());
        for (size_t candidate = 0; candidate < candidate_count; ++candidate) {
            const Isometry3d target =
                fromGtsam(optimized_.at<gtsam::Pose3>(candidate));
            const double squared_distance =
                (query_pose.translation() - target.translation()).squaredNorm();
            if (squared_distance > best_squared_distance) {
                continue;
            }
            const double time_difference =
                std::abs(frames_[query_id].stamp - frames_[candidate].stamp);
            const double travel_difference = std::abs(
                frames_[query_id].travel_distance
                - frames_[candidate].travel_distance);
            const Eigen::Quaterniond target_rotation(target.rotation());
            const double angle_difference =
                query_rotation.angularDistance(target_rotation);
            if (time_difference > config_.loop_min_time_separation
                && travel_difference > config_.loop_min_travel_distance
                && angle_difference < config_.loop_max_rotation) {
                best_squared_distance = squared_distance;
                best_id = static_cast<int>(candidate);
            }
        }
        return best_id;
    }

    PointCloudPtr buildTargetCloud(int target_id) const {
        auto merged = std::make_shared<PointCloud>();
        const int begin =
            std::max(0, target_id - config_.target_frame_count);
        const int end = std::min(
            static_cast<int>(optimized_.size()) - 1,
            target_id + config_.target_frame_count);
        for (int i = begin; i <= end; ++i) {
            const Isometry3d pose =
                fromGtsam(optimized_.at<gtsam::Pose3>(i));
            *merged += *transformedCloud(frames_[i].cloud_lidar, pose);
        }
        return voxelized(merged, config_.target_voxel_size);
    }

    bool registerLoop(
        const Isometry3d& initial_pose,
        const PointCloudConstPtr& source,
        const PointCloudConstPtr& target,
        Isometry3d& result,
        double& fitness) const
    {
        pcl::IterativeClosestPoint<Point, Point> icp;
        icp.setMaximumIterations(50);
        // Match the DLIO/SimpleLoopClosure setup. The default PCL
        // correspondence distance is far too small for accumulated odometry
        // drift at the end of a large loop.
        icp.setMaxCorrespondenceDistance(
            config_.loop_search_radius * 2.0);
        icp.setTransformationEpsilon(1e-4);
        icp.setEuclideanFitnessEpsilon(1e-4);
        icp.setRANSACIterations(0);
        icp.setInputSource(source);
        icp.setInputTarget(target);
        PointCloud aligned;
        icp.align(aligned, initial_pose.matrix().cast<float>());
        result = Isometry3d(icp.getFinalTransformation().cast<double>());
        fitness = icp.getFitnessScore();
        return icp.hasConverged()
            && result.matrix().allFinite()
            && std::isfinite(fitness)
            && fitness < config_.fitness_threshold;
    }

    bool buildLoopEdges(gtsam::NonlinearFactorGraph& graph) {
        if (searched_loop_id_ >= frames_.size() || optimized_.empty()) {
            return false;
        }

        const size_t last_optimized_id = optimized_.size() - 1;
        const Isometry3d last_optimized =
            fromGtsam(optimized_.at<gtsam::Pose3>(last_optimized_id));
        const size_t stride =
            static_cast<size_t>(std::max(1, config_.loop_search_stride));

        for (size_t query_id = searched_loop_id_;
             query_id < frames_.size();
             query_id += stride) {
            const Isometry3d query_pose =
                last_optimized
                * frames_[last_optimized_id].T_odom_lidar.inverse()
                * frames_[query_id].T_odom_lidar;
            const int target_id = searchLoopTarget(query_id, query_pose);
            if (target_id < 0) {
                continue;
            }
            ++loop_candidate_count_;

            const PointCloudPtr source =
                voxelized(
                    frames_[query_id].cloud_lidar,
                    config_.source_voxel_size);
            const PointCloudPtr target = buildTargetCloud(target_id);
            Isometry3d registered_pose;
            double fitness = std::numeric_limits<double>::infinity();
            if (!registerLoop(
                    query_pose,
                    source,
                    target,
                    registered_pose,
                    fitness)) {
                ++loop_registration_failure_count_;
                last_loop_fitness_ =
                    std::isfinite(fitness) ? fitness : -1.0;
                spdlog::debug(
                    "[pgo] loop ICP rejected: {} -> {}, fitness={:.4f}",
                    query_id,
                    target_id,
                    fitness);
                continue;
            }
            last_loop_fitness_ = fitness;

            const Isometry3d target_pose =
                fromGtsam(optimized_.at<gtsam::Pose3>(target_id));
            const Isometry3d relative =
                registered_pose.inverse() * target_pose;
            gtsam::Vector6 variances;
            variances.setConstant(std::max(fitness, 1e-9));
            const auto loop_noise = gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::Cauchy::Create(1.0),
                gtsam::noiseModel::Diagonal::Variances(variances));
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                query_id,
                static_cast<size_t>(target_id),
                toGtsam(relative),
                loop_noise));
            loop_edges_.push_back({
                query_id,
                static_cast<size_t>(target_id),
            });
            ++loop_edge_count_;
            spdlog::info(
                "[pgo] loop detected: {} -> {}, fitness={:.4f}",
                query_id,
                target_id,
                fitness);
        }
        searched_loop_id_ = frames_.size();
        syncLoopStats();
        return !graph.empty();
    }

    void syncLoopStats() {
        std::lock_guard<std::mutex> lock(output_mutex_);
        stats_.loop_candidates = loop_candidate_count_;
        stats_.loop_registration_failures =
            loop_registration_failure_count_;
        stats_.last_loop_fitness = last_loop_fitness_;
    }

    void updateIsam(
        const gtsam::NonlinearFactorGraph& graph,
        const gtsam::Values& initial)
    {
        if (initial.empty()) {
            isam2_->update(graph);
        } else {
            isam2_->update(graph, initial);
        }
        isam2_->update();
        optimized_ = isam2_->calculateEstimate();
    }

    void rebuildSnapshot() {
        std::vector<Isometry3d> poses;
        poses.reserve(optimized_.size());
        for (size_t i = 0; i < optimized_.size(); ++i) {
            poses.push_back(fromGtsam(optimized_.at<gtsam::Pose3>(i)));
        }

        std::lock_guard<std::mutex> lock(output_mutex_);
        optimized_poses_snapshot_ = std::move(poses);
        loop_edges_snapshot_ = loop_edges_;
    }

    void rebuildGlobalMap() {
        auto merged = std::make_shared<PointCloud>();
        const size_t stride =
            static_cast<size_t>(config_.map_frame_stride);
        for (size_t i = 0; i < optimized_.size(); i += stride) {
            const Isometry3d pose =
                fromGtsam(optimized_.at<gtsam::Pose3>(i));
            *merged += *transformedCloud(frames_[i].cloud_lidar, pose);
        }
        const PointCloudPtr sparse =
            voxelized(merged, config_.map_voxel_size);
        {
            std::lock_guard<std::mutex> lock(output_mutex_);
            global_map_ = sparse;
        }
        spdlog::debug(
            "[pgo] sparse map rebuilt: poses={}, points={}",
            optimized_.size(),
            sparse->size());
    }

    void updateCorrection() {
        if (optimized_.empty()) {
            return;
        }
        const size_t last_id = optimized_.size() - 1;
        const Isometry3d optimized_pose =
            fromGtsam(optimized_.at<gtsam::Pose3>(last_id));
        const Isometry3d correction =
            optimized_pose * frames_[last_id].T_odom_lidar.inverse();

        std::lock_guard<std::mutex> lock(output_mutex_);
        T_map_odom_ = correction;
        stats_.optimized_poses = optimized_.size();
        stats_.loop_edges = loop_edge_count_;
        stats_.correction_translation = correction.translation().norm();
        stats_.correction_rotation = rotationAngle(correction);
        ++stats_.revision;
    }

    void appendOccupancyFrames() {
        if (!occupancy_ || optimized_.empty()) {
            return;
        }
        while (occupancy_inserted_id_ < optimized_.size()) {
            const Isometry3d pose = fromGtsam(
                optimized_.at<gtsam::Pose3>(occupancy_inserted_id_));
            occupancy_->insertScan(
                frames_[occupancy_inserted_id_].cloud_lidar, pose);
            ++occupancy_inserted_id_;
        }
    }

    void rebuildOccupancyMap() {
        if (!occupancy_) {
            return;
        }
        occupancy_->clear();
        occupancy_inserted_id_ = 0;
        appendOccupancyFrames();
        spdlog::info(
            "[pgo] occupancy grid rebuilt from {} keyframes (rev={})",
            occupancy_inserted_id_,
            occupancy_->revision());
    }

    void publishOccupancySnapshot() {
        if (!occupancy_) {
            return;
        }
        // Catch up new keyframes only. Full rebuild is reserved for loop
        // closure in processPending — never re-raycast the whole trajectory
        // on a visualization poll.
        if (occupancy_inserted_id_ < optimized_.size()) {
            appendOccupancyFrames();
        }
        auto msg = std::make_shared<OccupancyGridMsg>(occupancy_->toMsg());
        std::lock_guard<std::mutex> lock(occupancy_mutex_);
        occupancy_msg_ = std::move(msg);
    }

    Config::Pgo config_;

    mutable std::mutex input_mutex_;
    std::vector<InputFrame> input_frames_;
    std::vector<GraphFrame> frames_;

    std::unique_ptr<gtsam::ISAM2> isam2_;
    gtsam::SharedNoiseModel prior_noise_;
    gtsam::SharedNoiseModel odom_noise_;
    gtsam::Values optimized_;
    size_t added_odom_id_ = 0;
    size_t searched_loop_id_ = 0;
    size_t loop_candidate_count_ = 0;
    size_t loop_registration_failure_count_ = 0;
    size_t loop_edge_count_ = 0;
    double last_loop_fitness_ = -1.0;
    std::vector<PoseGraphEdge> loop_edges_;

    mutable std::mutex output_mutex_;
    Isometry3d T_map_odom_ = Isometry3d::Identity();
    PoseGraphStats stats_;
    std::vector<Isometry3d> optimized_poses_snapshot_;
    std::vector<PoseGraphEdge> loop_edges_snapshot_;
    PointCloudConstPtr global_map_;
    std::unique_ptr<OccupancyGrid> occupancy_;
    size_t occupancy_inserted_id_ = 0;
    /// Occupancy export is isolated from output_mutex_ so dense grid copies
    /// cannot stall frontend T_map_odom() / IMU propagation publishers.
    mutable std::mutex occupancy_mutex_;
    std::shared_ptr<const OccupancyGridMsg> occupancy_msg_;
    std::atomic<bool> snapshot_requested_{false};
    std::atomic<bool> map_requested_{false};
    std::atomic<bool> occupancy_requested_{false};

    std::atomic<bool> stop_{false};
    std::mutex wake_mutex_;
    std::condition_variable wake_cv_;
    std::thread worker_;
};

PoseGraphBackend::PoseGraphBackend(const Config::Pgo& config)
    : impl_(std::make_unique<Impl>(config))
{
}

PoseGraphBackend::~PoseGraphBackend() = default;

bool PoseGraphBackend::enabled() const {
    return impl_->enabled();
}

void PoseGraphBackend::addFrame(
    const PointCloudConstPtr& cloud_odom,
    const Isometry3d& T_odom_lidar,
    double stamp)
{
    impl_->addFrame(cloud_odom, T_odom_lidar, stamp);
}

Isometry3d PoseGraphBackend::T_map_odom() const {
    return impl_->correction();
}

PoseGraphStats PoseGraphBackend::stats() const {
    return impl_->stats();
}

PoseGraphSnapshot PoseGraphBackend::snapshot() const {
    return impl_->snapshot();
}

void PoseGraphBackend::requestSnapshot() {
    impl_->requestSnapshot();
}

void PoseGraphBackend::requestGlobalMap() {
    impl_->requestGlobalMap();
}

PointCloudConstPtr PoseGraphBackend::latestGlobalMap() const {
    return impl_->latestGlobalMap();
}

void PoseGraphBackend::requestOccupancyGrid() {
    impl_->requestOccupancyGrid();
}

std::shared_ptr<const OccupancyGridMsg>
PoseGraphBackend::latestOccupancyGrid() const {
    return impl_->latestOccupancyGrid();
}

}  // namespace sapphire
