#include <sapphire/backend/pose_graph.hpp>
#include <sapphire/mapping/occupancy_grid.hpp>
#include <sapphire/odometry/voxel_filter.hpp>

#include <SO3.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/ISAM2UpdateParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <pcl/common/transforms.h>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/normal_estimation.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
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
    return lie::SO3d::log(lie::SO3d(transform.rotation())).norm();
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

/// Neighbor count for loop-cloud covariances. Not a toml parameter.
constexpr int kLoopCovarianceNeighbors = 20;
/// Translation (m) and rotation (rad) convergence, fixed to match the
/// intern GICP registrar. small_gicp checks rotation against rotation_eps
/// and translation against translation_eps.
constexpr double kLoopConvergenceEpsilon = 0.01;

/// ISAM2 looks up a per-character relinearization threshold. Integer keys
/// never match 'x', so every pose variable uses Symbol('x', id).
constexpr unsigned char kPoseSymbol = 'x';
constexpr double kRelinearizeRotationRad = 0.01;
constexpr double kRelinearizeTranslationM = 0.05;
/// Gauss-Newton wildfire. This is also the GTSAM default. Dogleg is not used.
constexpr double kGaussNewtonWildfire = 0.001;
/// Empty updates after a new factor, until no variable still exceeds the
/// relinearization threshold. The intern default is 20 and toml does not
/// override it. The installed GTSAM has no predictUpdateInfo; the stop
/// check below is the full-graph threshold test that call uses.
constexpr int kIsamRelinearizeIterations = 20;
/// Nonlinear error above this is divergence. The intern batch optimizer
/// aborts a trial at the same 1e12 cutoff.
constexpr double kMaxGraphError = 1.0e12;
/// Nearest historical frames tried per query. Intern spatial_max_candidates.
/// One query still contributes at most one loop factor.
constexpr int kLoopMaxCandidates = 5;

struct ResidualSummary {
    bool finite = false;
    const char* reason = "not evaluated";
    size_t constraints = 0;
    double linear = 0.0;
    double linear_ratio = 0.0;
    double linear_sigma = 0.0;
    std::uint64_t linear_from = 0;
    std::uint64_t linear_to = 0;
    double angular = 0.0;
    double angular_ratio = 0.0;
    double angular_sigma = 0.0;
    std::uint64_t angular_from = 0;
    std::uint64_t angular_to = 0;
};

bool estimateFinite(const gtsam::Values& values) {
    for (const auto& key_value : values) {
        const gtsam::Pose3 pose = key_value.value.cast<gtsam::Pose3>();
        if (!pose.matrix().allFinite()) {
            return false;
        }
    }
    return true;
}

/// Largest per-edge residual / sigma on the optimized graph. Translation is
/// the max absolute component of (measurement - relative pose), and rotation
/// is the geodesic angle. Standard deviations are the square roots of the
/// largest covariance diagonal in each block. GTSAM order is rotation then
/// translation; the intern offset of 0 is translation-first and does not apply.
ResidualSummary summarizeConstraintResiduals(
    const gtsam::NonlinearFactorGraph& factors,
    const gtsam::Values& estimate)
{
    ResidualSummary summary;
    summary.finite = true;
    summary.reason = "ok";
    for (const auto& factor : factors) {
        if (!factor) {
            continue;
        }
        const auto* between =
            dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3>*>(
                factor.get());
        if (between == nullptr) {
            continue;
        }
        ++summary.constraints;

        const gtsam::Key from_key = between->key1();
        const gtsam::Key to_key = between->key2();
        if (!estimate.exists(from_key) || !estimate.exists(to_key)) {
            summary.finite = false;
            summary.reason = "missing pose";
            return summary;
        }
        const gtsam::Pose3 from = estimate.at<gtsam::Pose3>(from_key);
        const gtsam::Pose3 to = estimate.at<gtsam::Pose3>(to_key);
        const gtsam::Pose3 relative = from.inverse() * to;
        const gtsam::Pose3& measured = between->measured();
        if (!relative.matrix().allFinite() || !measured.matrix().allFinite()) {
            summary.finite = false;
            summary.reason = "non-finite residual";
            return summary;
        }

        const Eigen::Vector3d translation_error =
            measured.translation() - relative.translation();
        const double linear = translation_error.cwiseAbs().maxCoeff();
        const gtsam::Rot3 rotation_error =
            relative.rotation().inverse() * measured.rotation();
        const double angular = gtsam::Rot3::Logmap(rotation_error).norm();
        if (!std::isfinite(linear) || !std::isfinite(angular)) {
            summary.finite = false;
            summary.reason = "non-finite residual";
            return summary;
        }

        const auto gaussian =
            std::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(
                between->noiseModel());
        if (!gaussian) {
            summary.finite = false;
            summary.reason = "non-gaussian constraint";
            return summary;
        }
        const gtsam::Matrix covariance = gaussian->covariance();
        if (covariance.rows() != 6 || covariance.cols() != 6
            || !covariance.allFinite()) {
            summary.finite = false;
            summary.reason = "non-finite covariance";
            return summary;
        }
        const double rotation_variance =
            covariance.diagonal().head<3>().maxCoeff();
        const double translation_variance =
            covariance.diagonal().tail<3>().maxCoeff();
        if (!(rotation_variance > 0.0) || !(translation_variance > 0.0)) {
            summary.finite = false;
            summary.reason = "non-positive variance";
            return summary;
        }
        const double linear_sigma = std::sqrt(translation_variance);
        const double angular_sigma = std::sqrt(rotation_variance);
        const double linear_ratio = linear / linear_sigma;
        const double angular_ratio = angular / angular_sigma;
        const std::uint64_t from_id = gtsam::Symbol(from_key).index();
        const std::uint64_t to_id = gtsam::Symbol(to_key).index();
        if (linear_ratio > summary.linear_ratio) {
            summary.linear = linear;
            summary.linear_ratio = linear_ratio;
            summary.linear_sigma = linear_sigma;
            summary.linear_from = from_id;
            summary.linear_to = to_id;
        }
        if (angular_ratio > summary.angular_ratio) {
            summary.angular = angular;
            summary.angular_ratio = angular_ratio;
            summary.angular_sigma = angular_sigma;
            summary.angular_from = from_id;
            summary.angular_to = to_id;
        }
    }
    return summary;
}

gtsam::Key poseKey(std::uint64_t id) {
    return gtsam::Symbol(kPoseSymbol, id);
}

struct LoopAlignment {
    bool accepted = false;
    Isometry3d T_target_source = Isometry3d::Identity();
    /// Covariance of `T_target_source` in its right tangent, GTSAM order
    /// (rotation, then translation), after the diagonal floors.
    Eigen::Matrix<double, 6, 6> covariance =
        Eigen::Matrix<double, 6, 6>::Zero();
    double fitness = std::numeric_limits<double>::infinity();
    size_t source_points = 0;
    size_t target_points = 0;
    size_t inliers = 0;
    double overlap = 0.0;
    double min_eigenvalue = std::numeric_limits<double>::quiet_NaN();
    const char* reason = "not attempted";
};

/// Invert a floored registration covariance into a Gaussian information
/// model. Returns false when the covariance is not symmetric positive
/// definite. `information_out` is the matrix passed to the noise model.
bool loopInformationNoise(
    const Eigen::Matrix<double, 6, 6>& covariance,
    gtsam::SharedNoiseModel& noise,
    Eigen::Matrix<double, 6, 6>& information_out)
{
    if (!covariance.allFinite()) {
        return false;
    }
    const Eigen::Matrix<double, 6, 6> symmetric =
        0.5 * (covariance + covariance.transpose());
    const Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(symmetric);
    if (llt.info() != Eigen::Success) {
        return false;
    }
    Eigen::Matrix<double, 6, 6> information =
        llt.solve(Eigen::Matrix<double, 6, 6>::Identity());
    information = 0.5 * (information + information.transpose());
    if (!information.allFinite()) {
        return false;
    }
    information_out = information;
    noise = gtsam::noiseModel::Gaussian::Information(information);
    return static_cast<bool>(noise);
}

std::shared_ptr<small_gicp::PointCloud> toGicpCloud(const PointCloud& cloud) {
    auto output = std::make_shared<small_gicp::PointCloud>();
    output->resize(cloud.size());
    size_t count = 0;
    for (const Point& point : cloud.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)
            || !std::isfinite(point.z)) {
            continue;
        }
        output->point(count) << point.x, point.y, point.z, 1.0;
        ++count;
    }
    output->resize(count);
    return output;
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

        gtsam::ISAM2GaussNewtonParams gauss_newton(kGaussNewtonWildfire);
        gtsam::ISAM2Params params(gauss_newton);
        gtsam::ISAM2ThresholdMap thresholds;
        thresholds[kPoseSymbol] =
            (gtsam::Vector(6) << kRelinearizeRotationRad,
             kRelinearizeRotationRad,
             kRelinearizeRotationRad,
             kRelinearizeTranslationM,
             kRelinearizeTranslationM,
             kRelinearizeTranslationM)
                .finished();
        params.relinearizeThreshold = thresholds;
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

    void recordOdometrySample(const OdometryResult& sample) {
        if (!config_.enabled || !sample.converged
            || !std::isfinite(sample.stamp)
            || !sample.T_world_lidar.matrix().allFinite()
            || !sample.v_world.allFinite()) {
            return;
        }

        size_t keyframe = 0;
        {
            std::lock_guard<std::mutex> lock(input_mutex_);
            if (input_frames_.empty()) {
                return;
            }
            keyframe = input_frames_.size() - 1;
        }

        std::lock_guard<std::mutex> lock(samples_mutex_);
        samples_.push_back({
            sample.stamp,
            keyframe,
            sample.T_world_lidar,
            sample.v_world,
        });
    }

    MapTrajectory mapTrajectory() const {
        std::vector<OdometrySample> samples;
        {
            std::lock_guard<std::mutex> lock(samples_mutex_);
            samples = samples_;
        }

        std::vector<Isometry3d> map_poses;
        std::vector<Isometry3d> odom_poses;
        Isometry3d latest_correction = Isometry3d::Identity();
        size_t revision = 0;
        {
            std::lock_guard<std::mutex> lock(output_mutex_);
            map_poses = keyframe_map_poses_;
            odom_poses = keyframe_odom_poses_;
            latest_correction = T_map_odom_;
            revision = stats_.revision;
        }

        MapTrajectory trajectory;
        trajectory.revision = revision;
        trajectory.samples.reserve(samples.size());
        for (const OdometrySample& sample : samples) {
            const Isometry3d correction = sampleCorrection(
                sample.keyframe,
                map_poses,
                odom_poses,
                latest_correction);
            MapTrajectorySample mapped;
            mapped.stamp = sample.stamp;
            mapped.keyframe = sample.keyframe;
            mapped.T_map_lidar = correction * sample.T_odom_lidar;
            mapped.v_map = correction.rotation() * sample.v_odom;
            if (!mapped.T_map_lidar.matrix().allFinite()
                || !mapped.v_map.allFinite()) {
                continue;
            }
            trajectory.samples.push_back(std::move(mapped));
        }
        return trajectory;
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

    /// Odom-frame lidar pose. `keyframe` indexes input_frames_ / frames_.
    struct OdometrySample {
        double stamp = 0.0;
        size_t keyframe = 0;
        Isometry3d T_odom_lidar = Isometry3d::Identity();
        Eigen::Vector3d v_odom = Eigen::Vector3d::Zero();
    };

    /// A GICP-accepted loop that is not yet in the committed ISAM2.
    /// `edge.source` is the query and `edge.target` is the historical frame.
    struct PendingLoop {
        PoseGraphEdge edge;
        size_t source_points = 0;
        size_t target_points = 0;
        size_t source_frames = 0;
        size_t inliers = 0;
        double overlap = 0.0;
        double fitness = 0.0;
        double min_eigenvalue = 0.0;
        Eigen::Matrix<double, 6, 1> info_diag =
            Eigen::Matrix<double, 6, 1>::Zero();
    };

    struct LoopLinearization {
        int passes = 0;
        size_t still_moving = 0;
        size_t moved_over_1cm = 0;
        double max_shift = 0.0;
        size_t max_id = 0;
        double mid_shift = 0.0;
        size_t mid_id = 0;
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
            optimized_ = updateIsam(
                *isam2_, odom_graph, initial, false, nullptr);
            updateCorrection();
            odom_updated = true;
        }

        gtsam::NonlinearFactorGraph loop_graph;
        std::vector<PendingLoop> pending_loops;
        bool loop_updated = false;
        if (buildLoopEdges(loop_graph, pending_loops)) {
            // Odometry is already in the committed ISAM2. The copy sees that
            // update, then the loop factors only. A rejection deletes the copy.
            if (tryCommitLoops(loop_graph, pending_loops)) {
                updateCorrection();
                loop_updated = true;
            }
            syncLoopStats();
        }

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
                poseKey(0), first, prior_noise_));
            initial.insert(poseKey(0), first);
            added_odom_id_ = 1;
        }

        const size_t last_optimized_id =
            optimized_.empty() ? 0 : optimized_.size() - 1;
        const Isometry3d last_optimized = optimized_.empty()
            ? frames_.front().T_odom_lidar
            : fromGtsam(optimizedPose(last_optimized_id));

        for (size_t i = added_odom_id_; i < frames_.size(); ++i) {
            const Isometry3d relative =
                frames_[i - 1].T_odom_lidar.inverse()
                * frames_[i].T_odom_lidar;
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                poseKey(i - 1),
                poseKey(i),
                toGtsam(relative),
                odom_noise_));

            const Isometry3d estimate =
                last_optimized
                * frames_[last_optimized_id].T_odom_lidar.inverse()
                * frames_[i].T_odom_lidar;
            initial.insert(poseKey(i), toGtsam(estimate));
            ++added_odom_id_;
        }
        return !graph.empty();
    }

    /// Odometry links consecutive keyframes. Committed loops are stored as
    /// query → historical frame. Either one disqualifies another loop.
    bool alreadyLinked(size_t query_id, size_t candidate_id) const {
        if (candidate_id + 1 == query_id) {
            return true;
        }
        for (const PoseGraphEdge& edge : loop_edges_) {
            const bool same_pair =
                (edge.source == query_id && edge.target == candidate_id)
                || (edge.source == candidate_id && edge.target == query_id);
            if (same_pair) {
                return true;
            }
        }
        return false;
    }

    /// Historical frames inside the search radius, nearest first, at most
    /// five. Time, travel, and heading gates stay. A frame that already has
    /// an edge with the query is left out before the cap, so it does not
    /// consume one of the five attempts.
    std::vector<size_t> searchLoopCandidates(
        size_t query_id, const Isometry3d& query_pose) const
    {
        const double radius_squared =
            config_.loop_search_radius * config_.loop_search_radius;
        const Eigen::Quaterniond query_rotation(query_pose.rotation());
        struct Ranked {
            size_t id = 0;
            double squared_distance = 0.0;
        };
        std::vector<Ranked> ranked;

        // A loop target must be historical. When several frames are copied in
        // one worker cycle, optimized_ also contains nodes newer than an early
        // query; admitting those creates duplicate reciprocal loop edges.
        const size_t candidate_count =
            std::min(query_id, optimized_.size());
        ranked.reserve(candidate_count);
        for (size_t candidate = 0; candidate < candidate_count; ++candidate) {
            if (alreadyLinked(query_id, candidate)) {
                continue;
            }
            const Isometry3d target = fromGtsam(optimizedPose(candidate));
            const double squared_distance =
                (query_pose.translation() - target.translation()).squaredNorm();
            if (squared_distance > radius_squared) {
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
                ranked.push_back({candidate, squared_distance});
            }
        }

        const size_t count = std::min(
            ranked.size(), static_cast<size_t>(kLoopMaxCandidates));
        std::partial_sort(
            ranked.begin(),
            ranked.begin() + static_cast<std::ptrdiff_t>(count),
            ranked.end(),
            [](const Ranked& a, const Ranked& b) {
                if (a.squared_distance != b.squared_distance) {
                    return a.squared_distance < b.squared_distance;
                }
                // Same distance: prefer the later historical frame, matching
                // the intern spatial candidate tie break.
                return a.id > b.id;
            });
        std::vector<size_t> ids;
        ids.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            ids.push_back(ranked[i].id);
        }
        return ids;
    }

    PointCloudPtr buildTargetCloud(int target_id) const {
        auto merged = std::make_shared<PointCloud>();
        const int begin =
            std::max(0, target_id - config_.target_frame_count);
        const int end = std::min(
            static_cast<int>(optimized_.size()) - 1,
            target_id + config_.target_frame_count);
        for (int i = begin; i <= end; ++i) {
            const Isometry3d pose = fromGtsam(
                optimizedPose(static_cast<size_t>(i)));
            *merged += *transformedCloud(frames_[i].cloud_lidar, pose);
        }
        return voxelized(merged, config_.target_voxel_size);
    }

    /// Inclusive index window of keyframes merged into the query cloud.
    /// Travel is monotonic, so the frames inside the radius are contiguous.
    /// Expanding one step backward then one forward keeps a stationary run
    /// centered on the query once the frame cap binds.
    void queryCloudWindow(
        size_t query_id, size_t& begin, size_t& end) const
    {
        const double travel = frames_[query_id].travel_distance;
        const double radius = std::max(0.0, config_.source_travel_radius_m);
        const size_t cap = static_cast<size_t>(
            std::max(1, config_.source_max_frames));
        begin = query_id;
        end = query_id;
        const auto inside = [&](size_t id) {
            return std::abs(frames_[id].travel_distance - travel) <= radius;
        };
        while (end - begin + 1 < cap) {
            const bool backward = begin > 0 && inside(begin - 1);
            const bool forward =
                end + 1 < frames_.size() && inside(end + 1);
            if (!backward && !forward) {
                break;
            }
            if (backward) {
                --begin;
            }
            if (end - begin + 1 >= cap) {
                break;
            }
            if (forward) {
                ++end;
            }
        }
    }

    /// Query scan plus keyframes within the travel window, expressed in the
    /// query body by odometry. Points farther than source_max_range_m in the
    /// scan that observed them are dropped first. The voxel is unchanged.
    PointCloudPtr buildQueryCloud(
        size_t query_id, size_t& frame_count) const
    {
        size_t begin = query_id;
        size_t end = query_id;
        queryCloudWindow(query_id, begin, end);
        frame_count = end - begin + 1;

        const Isometry3d& T_query = frames_[query_id].T_odom_lidar;
        const double max_range_sq = config_.source_max_range_m
            * config_.source_max_range_m;
        auto merged = std::make_shared<PointCloud>();
        for (size_t id = begin; id <= end; ++id) {
            const PointCloud& scan = *frames_[id].cloud_lidar;
            auto kept = std::make_shared<PointCloud>();
            kept->reserve(scan.size());
            for (const Point& point : scan.points) {
                if (!std::isfinite(point.x) || !std::isfinite(point.y)
                    || !std::isfinite(point.z)) {
                    continue;
                }
                const double range_sq =
                    static_cast<double>(point.x) * point.x
                    + static_cast<double>(point.y) * point.y
                    + static_cast<double>(point.z) * point.z;
                if (range_sq > max_range_sq) {
                    continue;
                }
                Point copy = point;
                copy.data[3] = 1.0f;
                kept->push_back(copy);
            }
            if (kept->empty()) {
                continue;
            }
            const Isometry3d T_query_scan =
                T_query.inverse() * frames_[id].T_odom_lidar;
            *merged += *transformedCloud(kept, T_query_scan);
        }
        return voxelized(merged, config_.source_voxel_size);
    }

    LoopAlignment registerLoop(
        const Isometry3d& initial_pose,
        const PointCloudConstPtr& source,
        const PointCloudConstPtr& target) const
    {
        LoopAlignment alignment;
        const auto source_gicp = toGicpCloud(
            source ? *source : PointCloud{});
        const auto target_gicp = toGicpCloud(
            target ? *target : PointCloud{});
        alignment.source_points = source_gicp->size();
        alignment.target_points = target_gicp->size();
        const size_t min_points = static_cast<size_t>(
            std::max(1, config_.lidar_min_inliers));
        if (alignment.source_points < min_points
            || alignment.target_points < min_points
            || !initial_pose.matrix().allFinite()) {
            alignment.reason = "too few points or non-finite initial pose";
            return alignment;
        }

        small_gicp::estimate_covariances(
            *source_gicp, kLoopCovarianceNeighbors);
        small_gicp::estimate_covariances(
            *target_gicp, kLoopCovarianceNeighbors);
        const small_gicp::KdTree<small_gicp::PointCloud> tree(target_gicp);

        small_gicp::Registration<
            small_gicp::GICPFactor,
            small_gicp::SerialReduction> solver;
        solver.optimizer.max_iterations = config_.lidar_max_iterations;
        const double max_correspondence = config_.lidar_max_correspondence_m;
        solver.rejector.max_dist_sq =
            max_correspondence * max_correspondence;
        solver.criteria.translation_eps = kLoopConvergenceEpsilon;
        solver.criteria.rotation_eps = kLoopConvergenceEpsilon;

        const small_gicp::RegistrationResult result = solver.align(
            *target_gicp, *source_gicp, tree, initial_pose);
        alignment.inliers = result.num_inliers;
        alignment.overlap = static_cast<double>(alignment.inliers)
            / static_cast<double>(alignment.source_points);
        alignment.T_target_source = result.T_target_source;

        // Fitness is a second pass over inlier nearest-neighbor distances.
        // It is not result.num_inliers and not PCL getFitnessScore().
        double error = 0.0;
        size_t fitness_count = 0;
        for (size_t i = 0; i < source_gicp->size(); ++i) {
            size_t neighbor = 0;
            double distance = 0.0;
            const Eigen::Vector4d point =
                result.T_target_source * source_gicp->point(i);
            if (small_gicp::traits::nearest_neighbor_search(
                    tree, point, &neighbor, &distance)
                && distance <= solver.rejector.max_dist_sq) {
                error += distance;
                ++fitness_count;
            }
        }
        alignment.fitness = fitness_count == 0
            ? std::numeric_limits<double>::infinity()
            : error / static_cast<double>(fitness_count);

        const Eigen::Matrix<double, 6, 6> information =
            (result.H + result.H.transpose())
            / (2.0 * static_cast<double>(
                std::max<size_t>(1, result.num_inliers)));
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>>
            eigenvalues(information);
        const bool curvature_ok = eigenvalues.info() == Eigen::Success
            && eigenvalues.eigenvalues().allFinite();
        if (curvature_ok) {
            alignment.min_eigenvalue = eigenvalues.eigenvalues().minCoeff();
        }

        if (!result.converged) {
            alignment.reason = "not converged";
            return alignment;
        }
        if (!alignment.T_target_source.matrix().allFinite()) {
            alignment.reason = "non-finite result";
            return alignment;
        }
        if (alignment.inliers < min_points) {
            alignment.reason = "too few inliers";
            return alignment;
        }
        if (alignment.overlap < config_.lidar_min_overlap) {
            alignment.reason = "overlap below minimum";
            return alignment;
        }
        if (!std::isfinite(alignment.fitness)
            || alignment.fitness > config_.lidar_max_fitness_m2) {
            alignment.reason = "fitness above maximum";
            return alignment;
        }
        if (!curvature_ok || alignment.min_eigenvalue <= 0.0) {
            alignment.reason = "singular Hessian";
            return alignment;
        }

        // H is already divided by the inlier count. Invert in GTSAM order
        // and add the floors there: rotation on the first three diagonal
        // entries, translation on the last three. Do not permute to
        // translation-first; the factor below consumes this order directly.
        Eigen::Matrix<double, 6, 6> covariance =
            eigenvalues.eigenvectors()
            * eigenvalues.eigenvalues().cwiseInverse().asDiagonal()
            * eigenvalues.eigenvectors().transpose();
        covariance = 0.5 * (covariance + covariance.transpose());
        const double rotation_floor = config_.lidar_rotation_sigma_rad
            * config_.lidar_rotation_sigma_rad;
        const double translation_floor = config_.lidar_translation_sigma_m
            * config_.lidar_translation_sigma_m;
        covariance.diagonal().head<3>().array() += rotation_floor;
        covariance.diagonal().tail<3>().array() += translation_floor;
        if (!covariance.allFinite()) {
            alignment.reason = "singular Hessian";
            return alignment;
        }
        alignment.covariance = covariance;
        alignment.accepted = true;
        alignment.reason = "accepted";
        return alignment;
    }

    bool buildLoopEdges(
        gtsam::NonlinearFactorGraph& graph,
        std::vector<PendingLoop>& pending) {
        if (searched_loop_id_ >= frames_.size() || optimized_.empty()) {
            return false;
        }

        const size_t last_optimized_id = optimized_.size() - 1;
        const Isometry3d last_optimized =
            fromGtsam(optimizedPose(last_optimized_id));
        const size_t stride =
            static_cast<size_t>(std::max(1, config_.loop_search_stride));

        for (size_t query_id = searched_loop_id_;
             query_id < frames_.size();
             query_id += stride) {
            const Isometry3d query_pose =
                last_optimized
                * frames_[last_optimized_id].T_odom_lidar.inverse()
                * frames_[query_id].T_odom_lidar;
            const std::vector<size_t> target_ids =
                searchLoopCandidates(query_id, query_pose);
            if (target_ids.empty()) {
                continue;
            }

            // One thickened query cloud for every candidate of this query.
            size_t source_frames = 0;
            const PointCloudPtr source =
                buildQueryCloud(query_id, source_frames);
            for (const size_t target_index : target_ids) {
                const int target_id = static_cast<int>(target_index);
                ++loop_candidate_count_;
                const PointCloudPtr target = buildTargetCloud(target_id);
                const LoopAlignment alignment =
                    registerLoop(query_pose, source, target);
                last_loop_source_points_ = alignment.source_points;
                if (!alignment.accepted) {
                    ++loop_registration_failure_count_;
                    last_loop_fitness_ = std::isfinite(alignment.fitness)
                        ? alignment.fitness
                        : -1.0;
                    spdlog::warn(
                        "[pgo] loop GICP rejected: {} -> {}, points={}/{}, "
                        "source_frames={}, inliers={}, overlap={:.3f}, "
                        "fitness={:.4f}, min_eig={:.3e} ({})",
                        query_id,
                        target_id,
                        alignment.source_points,
                        alignment.target_points,
                        source_frames,
                        alignment.inliers,
                        alignment.overlap,
                        alignment.fitness,
                        alignment.min_eigenvalue,
                        alignment.reason);
                    continue;
                }
                last_loop_fitness_ = alignment.fitness;

                const Isometry3d target_pose = fromGtsam(
                    optimizedPose(static_cast<size_t>(target_id)));
                // The target cloud is already in the world frame, so
                // T_target_source is T_world_query, not the relative pose.
                // BetweenFactor(target, query) wants
                // T_target_query = T_world_target^{-1} * T_world_query.
                // A right perturbation of T_world_query is the same tangent
                // vector on that relative pose, so the registration covariance
                // attaches directly. The inverse measurement would need Ad(T^{-1}).
                const Isometry3d measurement =
                    target_pose.inverse() * alignment.T_target_source;
                // prepare_loops drops a match whose relative translation is
                // longer than the search radius. max_loop_distance_m is left
                // at 0, so there is no second, tighter limit. The intern
                // checks translationNorm of that relative pose; here the same
                // length is ||T_world_target^{-1} * T_target_source||, not
                // the world-frame position of the query.
                const double translation = measurement.translation().norm();
                if (!measurement.matrix().allFinite()
                    || !std::isfinite(translation)) {
                    ++loop_registration_failure_count_;
                    spdlog::warn(
                        "[pgo] loop covariance rejected: {} -> {} "
                        "(non-finite measurement)",
                        query_id,
                        target_id);
                    continue;
                }
                if (translation > config_.loop_search_radius) {
                    ++loop_registration_failure_count_;
                    spdlog::warn(
                        "[pgo] loop translation rejected: {} -> {}, "
                        "translation={:.3f}m, limit={:.3f}m",
                        query_id,
                        target_id,
                        translation,
                        config_.loop_search_radius);
                    continue;
                }
                gtsam::SharedNoiseModel loop_noise;
                Eigen::Matrix<double, 6, 6> information;
                if (!loopInformationNoise(
                        alignment.covariance, loop_noise, information)) {
                    ++loop_registration_failure_count_;
                    spdlog::warn(
                        "[pgo] loop covariance rejected: {} -> {} "
                        "(indefinite covariance)",
                        query_id,
                        target_id);
                    continue;
                }
                graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                    poseKey(static_cast<std::uint64_t>(target_id)),
                    poseKey(query_id),
                    toGtsam(measurement),
                    loop_noise));
                // Count and publish only after the residual trial accepts.
                // Visualization stores query → historical frame. The factor
                // above is historical frame → query. The first accepted
                // candidate ends this query; later candidates are not added.
                PendingLoop loop;
                loop.edge = {query_id, static_cast<size_t>(target_id)};
                loop.source_frames = source_frames;
                loop.source_points = alignment.source_points;
                loop.target_points = alignment.target_points;
                loop.inliers = alignment.inliers;
                loop.overlap = alignment.overlap;
                loop.fitness = alignment.fitness;
                loop.min_eigenvalue = alignment.min_eigenvalue;
                loop.info_diag = information.diagonal();
                pending.push_back(loop);
                break;
            }
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
        stats_.loop_residual_rejections = loop_residual_rejection_count_;
        stats_.last_loop_fitness = last_loop_fitness_;
        stats_.last_loop_source_points = last_loop_source_points_;
    }

    void logCommittedLoop(const PendingLoop& loop) const {
        spdlog::info(
            "[pgo] loop detected: factor {} -> {} gaussian information, "
            "points={}/{}, source_frames={}, inliers={}, overlap={:.3f}, "
            "fitness={:.4f}, min_eig={:.3e}, info_diag=[{:.4e}, {:.4e}, "
            "{:.4e}, {:.4e}, {:.4e}, {:.4e}]",
            loop.edge.target,
            loop.edge.source,
            loop.source_points,
            loop.target_points,
            loop.source_frames,
            loop.inliers,
            loop.overlap,
            loop.fitness,
            loop.min_eigenvalue,
            loop.info_diag(0),
            loop.info_diag(1),
            loop.info_diag(2),
            loop.info_diag(3),
            loop.info_diag(4),
            loop.info_diag(5));
    }

    void logLinearization(
        const LoopLinearization& linearization,
        const ResidualSummary& residual) const
    {
        if (linearization.still_moving > 0) {
            spdlog::warn(
                "[pgo] loop relinearization stopped at {} passes "
                "with {} variables still above threshold, "
                "keyframes_moved>1cm={}, max_shift={:.3f}m at {}, "
                "mid_shift={:.3f}m at {}, "
                "max_linear_ratio={:.3f} ({}->{}), "
                "max_angular_ratio={:.3f} ({}->{})",
                linearization.passes,
                linearization.still_moving,
                linearization.moved_over_1cm,
                linearization.max_shift,
                linearization.max_id,
                linearization.mid_shift,
                linearization.mid_id,
                residual.linear_ratio,
                residual.linear_from,
                residual.linear_to,
                residual.angular_ratio,
                residual.angular_from,
                residual.angular_to);
            return;
        }
        spdlog::info(
            "[pgo] loop relinearization passes={}, "
            "keyframes_moved>1cm={}, max_shift={:.3f}m at {}, "
            "mid_shift={:.3f}m at {}, "
            "max_linear_ratio={:.3f} ({}->{}), "
            "max_angular_ratio={:.3f} ({}->{})",
            linearization.passes,
            linearization.moved_over_1cm,
            linearization.max_shift,
            linearization.max_id,
            linearization.mid_shift,
            linearization.mid_id,
            residual.linear_ratio,
            residual.linear_from,
            residual.linear_to,
            residual.angular_ratio,
            residual.angular_from,
            residual.angular_to);
    }

    /// Returns true only when the trial replaces the committed ISAM2.
    /// The official graph, T_map_odom, and occupancy stay put on failure.
    bool tryCommitLoops(
        const gtsam::NonlinearFactorGraph& loop_graph,
        const std::vector<PendingLoop>& pending)
    {
        std::unique_ptr<gtsam::ISAM2> trial;
        gtsam::Values estimate;
        LoopLinearization linearization;
        ResidualSummary residual;
        const char* reject_reason = nullptr;
        try {
            trial = std::make_unique<gtsam::ISAM2>(*isam2_);
            estimate = updateIsam(
                *trial, loop_graph, {}, true, &linearization);
            if (!estimateFinite(estimate)) {
                reject_reason = "non-finite pose";
            } else {
                const double error =
                    trial->getFactorsUnsafe().error(estimate);
                if (!std::isfinite(error) || std::abs(error) > kMaxGraphError) {
                    reject_reason = "diverged graph error";
                } else {
                    residual = summarizeConstraintResiduals(
                        trial->getFactorsUnsafe(), estimate);
                    // Chain edges, loops already committed, and this trial.
                    const size_t expected = estimate.empty()
                        ? 0
                        : (estimate.size() - 1)
                            + loop_edge_count_
                            + pending.size();
                    if (residual.constraints != expected) {
                        residual.finite = false;
                        residual.reason = "incomplete constraint set";
                    }
                    if (!residual.finite) {
                        reject_reason = residual.reason;
                    } else if (
                        residual.linear_ratio
                            > config_.max_constraint_error_sigma
                        || residual.angular_ratio
                            > config_.max_constraint_error_sigma) {
                        reject_reason =
                            "constraint error above sigma limit";
                    }
                }
            }
        } catch (const std::exception& error) {
            spdlog::warn(
                "[pgo] loop trial failed: {} (discarded)",
                error.what());
            loop_residual_rejection_count_ += pending.size();
            return false;
        }

        if (reject_reason != nullptr) {
            spdlog::warn(
                "[pgo] loop residual rejected: {} factor(s), reason={}, "
                "linear {}->{} error={:.4f}m sigma={:.4f} ratio={:.3f}, "
                "angular {}->{} error={:.4f}rad sigma={:.4f} ratio={:.3f}, "
                "threshold={:.3f}, relinearization passes={}, "
                "still_moving={}, keyframes_moved>1cm={}, "
                "max_shift={:.3f}m at {}, mid_shift={:.3f}m at {}",
                pending.size(),
                reject_reason,
                residual.linear_from,
                residual.linear_to,
                residual.linear,
                residual.linear_sigma,
                residual.linear_ratio,
                residual.angular_from,
                residual.angular_to,
                residual.angular,
                residual.angular_sigma,
                residual.angular_ratio,
                config_.max_constraint_error_sigma,
                linearization.passes,
                linearization.still_moving,
                linearization.moved_over_1cm,
                linearization.max_shift,
                linearization.max_id,
                linearization.mid_shift,
                linearization.mid_id);
            loop_residual_rejection_count_ += pending.size();
            return false;
        }

        isam2_.swap(trial);
        optimized_ = std::move(estimate);
        for (const PendingLoop& loop : pending) {
            loop_edges_.push_back(loop.edge);
            ++loop_edge_count_;
            logCommittedLoop(loop);
        }
        logLinearization(linearization, residual);
        return true;
    }

    /// Keys whose linear delta still exceeds the per-axis threshold.
    /// Matches ISAM2's full relinearization check for a Symbol map:
    /// a component must be strictly larger than its threshold.
    size_t keysNeedingRelinearization(const gtsam::ISAM2& isam) const {
        const gtsam::VectorValues& delta = isam.getDelta();
        size_t count = 0;
        for (const auto& entry : delta) {
            const gtsam::Vector& value = entry.second;
            if (value.size() != 6) {
                continue;
            }
            const bool rotation =
                value.head<3>().cwiseAbs().maxCoeff()
                > kRelinearizeRotationRad;
            const bool translation =
                value.tail<3>().cwiseAbs().maxCoeff()
                > kRelinearizeTranslationM;
            if (rotation || translation) {
                ++count;
            }
        }
        return count;
    }

    gtsam::Values updateIsam(
        gtsam::ISAM2& isam,
        const gtsam::NonlinearFactorGraph& graph,
        const gtsam::Values& initial,
        bool force_relinearize,
        LoopLinearization* linearization)
    {
        std::vector<Eigen::Vector3d> translation_before;
        if (linearization != nullptr) {
            translation_before.reserve(optimized_.size());
            for (size_t i = 0; i < optimized_.size(); ++i) {
                translation_before.push_back(
                    fromGtsam(optimizedPose(i)).translation());
            }
        }

        gtsam::ISAM2UpdateParams update_params;
        update_params.force_relinearize = force_relinearize;
        isam.update(graph, initial, update_params);

        int passes = 1;
        while (passes < kIsamRelinearizeIterations
               && keysNeedingRelinearization(isam) > 0) {
            isam.update(
                gtsam::NonlinearFactorGraph(),
                gtsam::Values(),
                gtsam::ISAM2UpdateParams{});
            ++passes;
        }
        gtsam::Values estimate = isam.calculateEstimate();
        if (linearization == nullptr) {
            return estimate;
        }

        linearization->passes = passes;
        linearization->still_moving = keysNeedingRelinearization(isam);
        linearization->moved_over_1cm = 0;
        linearization->max_shift = 0.0;
        linearization->max_id = 0;
        const size_t comparable = std::min(
            translation_before.size(), estimate.size());
        linearization->mid_id = comparable / 2;
        linearization->mid_shift = 0.0;
        for (size_t i = 0; i < comparable; ++i) {
            const double shift =
                (fromGtsam(estimate.at<gtsam::Pose3>(poseKey(i))).translation()
                 - translation_before[i])
                    .norm();
            if (shift > 0.01) {
                ++linearization->moved_over_1cm;
            }
            if (shift > linearization->max_shift) {
                linearization->max_shift = shift;
                linearization->max_id = i;
            }
            if (i == linearization->mid_id) {
                linearization->mid_shift = shift;
            }
        }
        return estimate;
    }

    gtsam::Pose3 optimizedPose(size_t id) const {
        return optimized_.at<gtsam::Pose3>(poseKey(id));
    }

    void rebuildSnapshot() {
        std::vector<Isometry3d> poses;
        poses.reserve(optimized_.size());
        for (size_t i = 0; i < optimized_.size(); ++i) {
            poses.push_back(fromGtsam(optimizedPose(i)));
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
            const Isometry3d pose = fromGtsam(optimizedPose(i));
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

    static Isometry3d sampleCorrection(
        size_t keyframe,
        const std::vector<Isometry3d>& map_poses,
        const std::vector<Isometry3d>& odom_poses,
        const Isometry3d& latest_correction)
    {
        if (keyframe >= map_poses.size() || keyframe >= odom_poses.size()) {
            return latest_correction;
        }
        return map_poses[keyframe] * odom_poses[keyframe].inverse();
    }

    void updateCorrection() {
        if (optimized_.empty()) {
            return;
        }
        const size_t last_id = optimized_.size() - 1;
        const Isometry3d optimized_pose =
            fromGtsam(optimizedPose(last_id));
        const Isometry3d correction =
            optimized_pose * frames_[last_id].T_odom_lidar.inverse();

        // Same poses the trajectory replay reads. A sample owned by keyframe
        // i uses T_map_i * T_odom_i^{-1}; i == last_id is T_map_odom_.
        std::vector<Isometry3d> map_poses;
        std::vector<Isometry3d> odom_poses;
        map_poses.reserve(optimized_.size());
        odom_poses.reserve(optimized_.size());
        for (size_t i = 0; i < optimized_.size(); ++i) {
            map_poses.push_back(fromGtsam(optimizedPose(i)));
            odom_poses.push_back(frames_[i].T_odom_lidar);
        }

        std::lock_guard<std::mutex> lock(output_mutex_);
        T_map_odom_ = correction;
        keyframe_map_poses_ = std::move(map_poses);
        keyframe_odom_poses_ = std::move(odom_poses);
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
            const Isometry3d pose =
                fromGtsam(optimizedPose(occupancy_inserted_id_));
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

    mutable std::mutex samples_mutex_;
    std::vector<OdometrySample> samples_;

    std::unique_ptr<gtsam::ISAM2> isam2_;
    gtsam::SharedNoiseModel prior_noise_;
    gtsam::SharedNoiseModel odom_noise_;
    gtsam::Values optimized_;
    size_t added_odom_id_ = 0;
    size_t searched_loop_id_ = 0;
    size_t loop_candidate_count_ = 0;
    size_t loop_registration_failure_count_ = 0;
    size_t loop_residual_rejection_count_ = 0;
    size_t loop_edge_count_ = 0;
    double last_loop_fitness_ = -1.0;
    size_t last_loop_source_points_ = 0;
    std::vector<PoseGraphEdge> loop_edges_;

    mutable std::mutex output_mutex_;
    Isometry3d T_map_odom_ = Isometry3d::Identity();
    /// Optimized map pose and the odom pose that produced it, one entry per
    /// optimized keyframe. Updated together with T_map_odom_.
    std::vector<Isometry3d> keyframe_map_poses_;
    std::vector<Isometry3d> keyframe_odom_poses_;
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

void PoseGraphBackend::recordOdometrySample(
    const OdometryResult& odom_frame_sample)
{
    impl_->recordOdometrySample(odom_frame_sample);
}

MapTrajectory PoseGraphBackend::mapTrajectory() const {
    return impl_->mapTrajectory();
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
