#include "pose_graph.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "local_gridmap.hpp"
#include "memory.hpp"
#include "occ_layer.hpp"

namespace {

gtsam::Pose3 toGtsam(const Eigen::Isometry3d &pose) { return gtsam::Pose3(pose.matrix()); }

Eigen::Isometry3d fromGtsam(const gtsam::Pose3 &pose) { return Eigen::Isometry3d(pose.matrix()); }

gtsam::KeySet changedFinalPoses(const gtsam::Values &oldEstimate, const gtsam::Values &newEstimate, const gtsam::KeySet &affectedKeys) {
  gtsam::KeySet changed;
  for (const gtsam::Key key : affectedKeys) {
    if (!oldEstimate.exists(key) || !newEstimate.exists(key)) {
      continue;
    }
    const gtsam::Vector6 difference = oldEstimate.at<gtsam::Pose3>(key).localCoordinates(newEstimate.at<gtsam::Pose3>(key));
    if (difference.lpNorm<Eigen::Infinity>() > 0.0) {
      changed.insert(key);
    }
  }
  return changed;
}

Eigen::Isometry3f toIsometry3f(const Eigen::Isometry3d &pose) {
  Eigen::Isometry3f result = Eigen::Isometry3f::Identity();
  result.matrix() = pose.matrix().cast<float>();
  return result;
}

std::shared_ptr<vvec<float, 3>> transformedCloud(const std::shared_ptr<const vvec<float, 3>> &cloud, const Eigen::Isometry3d &transform) {
  auto output = std::make_shared<vvec<float, 3>>();
  output->reserve(cloud->size());
  for (const Eigen::Vector3f &point : *cloud) {
    const Eigen::Vector3d transformed = transform * point.cast<double>();
    output->emplace_back(static_cast<float>(transformed.x()), static_cast<float>(transformed.y()), static_cast<float>(transformed.z()));
  }
  return output;
}

std::shared_ptr<vvec<float, 3>> voxelized(std::shared_ptr<vvec<float, 3>> cloud, double leaf_size) {
  if (leaf_size > 0.0) {
    down_sampling_voxel(*cloud, 1.0 / leaf_size);
  }
  return cloud;
}

}  // namespace

class PoseGraphBackend::Impl {
 public:
  Impl(const PoseGraphParameters &config, const NaviMapParameters &navi_map, const std::string &database_path,
       NavigationGridCallback navigation_grid_callback)
      : config_(config), navigation_grid_callback_(std::move(navigation_grid_callback)) {
    if (!config_.enabled) {
      return;
    }

    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    isam2_ = std::make_unique<gtsam::ISAM2>(params);

    gtsam::Vector6 prior_variances;
    prior_variances << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    prior_noise_ = gtsam::noiseModel::Diagonal::Variances(prior_variances);

    gtsam::Vector6 odom_variances;
    odom_variances << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    odom_noise_ = gtsam::noiseModel::Diagonal::Variances(odom_variances);
    memory_ = std::make_unique<Memory>(database_path, static_cast<size_t>(std::max(1, config_.memory_stm_size)),
                                       static_cast<size_t>(std::max(0, config_.memory_wm_size)));

    if (navi_map.enabled) {
      local_grid_maker_ = std::make_unique<LocalGridMaker>(navi_map);
      occupancy_ = std::make_unique<OccupancyGrid>(navi_map);
      occupancy_resolution_ = navi_map.resolution;
      spdlog::info("[pgo] occupancy grid enabled (reso={:.2f}m, d_max={:.2f}m)", navi_map.resolution, navi_map.d_max);
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

  bool enabled() const { return config_.enabled; }

  void addFrame(const std::shared_ptr<const vvec<float, 3>> &cloud_lidar, const Eigen::Isometry3d &T_odom_lidar, double stamp) {
    if (!config_.enabled || !cloud_lidar || cloud_lidar->empty() || !T_odom_lidar.matrix().allFinite() || !std::isfinite(stamp)) {
      return;
    }

    std::lock_guard<std::mutex> lock(input_mutex_);
    if (keyframe_count_ > 0) {
      input_travel_distance_ += (T_odom_lidar.translation() - last_input_pose_.translation()).norm();
    }
    last_input_pose_ = T_odom_lidar;
    input_frames_.push_back({cloud_lidar, T_odom_lidar, stamp, input_travel_distance_});
    ++keyframe_count_;
    input_requested_.store(true, std::memory_order_release);
    wake_cv_.notify_one();
  }

  Eigen::Isometry3d correction() const {
    std::lock_guard<std::mutex> lock(output_mutex_);
    return T_map_odom_;
  }

  void requestGlobalMap() {
    if (config_.enabled) {
      map_requested_.store(true, std::memory_order_release);
      wake_cv_.notify_one();
    }
  }

  std::shared_ptr<const vvec<float, 3>> latestGlobalMap() const {
    std::lock_guard<std::mutex> lock(output_mutex_);
    return global_map_;
  }

  std::shared_ptr<const NavigationGrid> latestOccupancyGrid() const {
    std::lock_guard<std::mutex> lock(occupancy_mutex_);
    return occupancy_msg_;
  }

 private:
  struct InputFrame {
    std::shared_ptr<const vvec<float, 3>> cloud_lidar;
    Eigen::Isometry3d T_odom_lidar = Eigen::Isometry3d::Identity();
    double stamp = 0.0;
    double travel_distance = 0.0;
  };

  struct GraphFrame {
    std::shared_ptr<const vvec<float, 3>> cloud_lidar;
    Eigen::Isometry3d T_odom_lidar = Eigen::Isometry3d::Identity();
    double stamp = 0.0;
    double travel_distance = 0.0;
  };

  struct IsamOutput {
    gtsam::Values estimate;
    gtsam::KeySet affected_keys;
  };

  void workerLoop() {
    const auto period = std::chrono::duration<double>(config_.update_period_sec);
    while (true) {
      std::unique_lock<std::mutex> lock(wake_mutex_);
      wake_cv_.wait_for(lock, period, [this]() {
        return stop_.load(std::memory_order_acquire) || input_requested_.load(std::memory_order_acquire) ||
               map_requested_.load(std::memory_order_acquire);
      });
      const bool stopping = stop_.load(std::memory_order_acquire);
      lock.unlock();
      try {
        processPending();
      } catch (const std::exception &error) {
        spdlog::error("[pgo] backend update failed: {}", error.what());
      }
      if (stopping) {
        break;
      }
    }
  }

  void copyPendingFrames() {
    std::deque<InputFrame> pending;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      pending.swap(input_frames_);
    }
    size_t index = 0;
    try {
      for (; index < pending.size(); ++index) {
        InputFrame &input = pending[index];
        const size_t frame_id = frames_.size();
        GraphFrame frame{input.cloud_lidar, input.T_odom_lidar, input.stamp, input.travel_distance};
        LocalGrid grid(static_cast<float>(occupancy_resolution_));
        if (local_grid_maker_) {
          local_grid_maker_->createLocalMap(*frame.cloud_lidar, toIsometry3f(input.T_odom_lidar), grid);
        }
        memory_->saveKeyframe(frame_id + 1, input.stamp, toIsometry3f(input.T_odom_lidar), frame.cloud_lidar, grid);
        frames_.push_back(std::move(frame));
      }
    } catch (...) {
      std::lock_guard<std::mutex> lock(input_mutex_);
      input_frames_.insert(input_frames_.begin(), std::make_move_iterator(pending.begin() + index), std::make_move_iterator(pending.end()));
      input_requested_.store(true, std::memory_order_release);
      throw;
    }
  }

  void processPending() {
    input_requested_.store(false, std::memory_order_release);
    copyPendingFrames();
    if (frames_.empty()) {
      return;
    }

    gtsam::NonlinearFactorGraph odom_graph;
    gtsam::Values initial;
    const size_t previous_estimate_size = optimized_.size();
    if (buildOdometryGraph(odom_graph, initial)) {
      IsamOutput output = updateIsam(odom_graph, initial);
      const gtsam::KeySet changed = changedFinalPoses(optimized_, output.estimate, output.affected_keys);
      pending_changed_keys_.insert(changed.begin(), changed.end());
      optimized_ = std::move(output.estimate);
      for (size_t key = previous_estimate_size; key < optimized_.size(); ++key) {
        pending_changed_keys_.insert(static_cast<gtsam::Key>(key));
      }
      added_odom_id_ = frames_.size();
      updateCorrection();
    }

    gtsam::NonlinearFactorGraph loop_graph;
    const bool searchedNewFrames = searched_loop_id_ < frames_.size() && !optimized_.empty();
    if (buildLoopEdges(loop_graph)) {
      IsamOutput output = updateIsam(loop_graph, {});
      const gtsam::KeySet changed = changedFinalPoses(optimized_, output.estimate, output.affected_keys);
      pending_changed_keys_.insert(changed.begin(), changed.end());
      optimized_ = std::move(output.estimate);
      updateCorrection();
    }
    if (searchedNewFrames) {
      finalizeLoopSearch();
    }

    bool occupancy_updated = false;
    if (!pending_changed_keys_.empty()) {
      saveOptimizedPoses(pending_changed_keys_);
      if (occupancy_) {
        occupancy_updated = updateOccupancyFrames(pending_changed_keys_);
      }
      pending_changed_keys_.clear();
    }
    if (occupancy_updated) {
      publishOccupancySnapshot();
    }

    if (map_requested_.exchange(false, std::memory_order_acq_rel) && !optimized_.empty()) {
      rebuildGlobalMap();
    }
  }

  bool buildOdometryGraph(gtsam::NonlinearFactorGraph &graph, gtsam::Values &initial) {
    if (added_odom_id_ >= frames_.size()) {
      return false;
    }

    size_t first_new_id = added_odom_id_;
    if (added_odom_id_ == 0) {
      const gtsam::Pose3 first = toGtsam(frames_.front().T_odom_lidar);
      graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, first, prior_noise_));
      initial.insert(0, first);
      first_new_id = 1;
    }

    const size_t last_optimized_id = optimized_.empty() ? 0 : optimized_.size() - 1;
    const Eigen::Isometry3d last_optimized =
        optimized_.empty() ? frames_.front().T_odom_lidar : fromGtsam(optimized_.at<gtsam::Pose3>(last_optimized_id));

    for (size_t i = first_new_id; i < frames_.size(); ++i) {
      const Eigen::Isometry3d relative = frames_[i - 1].T_odom_lidar.inverse() * frames_[i].T_odom_lidar;
      graph.add(gtsam::BetweenFactor<gtsam::Pose3>(i - 1, i, toGtsam(relative), odom_noise_));
      memory_->saveLink(static_cast<int>(i), static_cast<int>(i + 1), 0, toIsometry3f(relative));
      const Eigen::Isometry3d estimate = last_optimized * frames_[last_optimized_id].T_odom_lidar.inverse() * frames_[i].T_odom_lidar;
      initial.insert(i, toGtsam(estimate));
    }
    return !graph.empty();
  }

  int searchLoopTarget(size_t query_id, const Eigen::Isometry3d &query_pose) const {
    int best_id = -1;
    double best_squared_distance = config_.loop_search_radius * config_.loop_search_radius;
    const Eigen::Quaterniond query_rotation(query_pose.rotation());
    const size_t candidate_count = std::min(query_id, optimized_.size());

    for (size_t candidate = 0; candidate < candidate_count; ++candidate) {
      const Eigen::Isometry3d target = fromGtsam(optimized_.at<gtsam::Pose3>(candidate));
      const double squared_distance = (query_pose.translation() - target.translation()).squaredNorm();
      if (squared_distance > best_squared_distance) {
        continue;
      }
      const double time_difference = std::abs(frames_[query_id].stamp - frames_[candidate].stamp);
      const double travel_difference = std::abs(frames_[query_id].travel_distance - frames_[candidate].travel_distance);
      const Eigen::Quaterniond target_rotation(target.rotation());
      const double angle_difference = query_rotation.angularDistance(target_rotation);
      if (time_difference > config_.loop_min_time_separation && travel_difference > config_.loop_min_travel_distance &&
          angle_difference < config_.loop_max_rotation) {
        best_squared_distance = squared_distance;
        best_id = static_cast<int>(candidate);
      }
    }
    return best_id;
  }

  std::shared_ptr<const vvec<float, 3>> cloudForFrame(size_t frame_id) const {
    if (frames_[frame_id].cloud_lidar) {
      return frames_[frame_id].cloud_lidar;
    }
    return memory_->loadCloud(static_cast<int>(frame_id + 1));
  }

  std::shared_ptr<vvec<float, 3>> buildTargetCloud(int target_id) const {
    auto merged = std::make_shared<vvec<float, 3>>();
    const int begin = std::max(0, target_id - config_.target_frame_count);
    const int end = std::min(static_cast<int>(optimized_.size()) - 1, target_id + config_.target_frame_count);
    for (int i = begin; i <= end; ++i) {
      const Eigen::Isometry3d pose = fromGtsam(optimized_.at<gtsam::Pose3>(i));
      const std::shared_ptr<vvec<float, 3>> transformed = transformedCloud(cloudForFrame(i), pose);
      merged->insert(merged->end(), transformed->begin(), transformed->end());
    }
    return voxelized(merged, config_.keyframe_voxel_size);
  }

  bool registerLoop(const Eigen::Isometry3d &initial_pose, const std::shared_ptr<const vvec<float, 3>> &source,
                    const std::shared_ptr<const vvec<float, 3>> &target, Eigen::Isometry3d &result, double &fitness) const {
    const size_t min_inliers = static_cast<size_t>(std::max(1, config_.gicp_min_inliers));
    if (source->size() < min_inliers || target->size() < min_inliers) {
      return false;
    }

    const int num_threads = std::max(1, config_.gicp_num_threads);
    const int num_neighbors = std::max(5, config_.gicp_k_correspondences);
    auto target_points = std::make_shared<small_gicp::PointCloud>(*target);
    auto source_points = std::make_shared<small_gicp::PointCloud>(*source);
    small_gicp::KdTree<small_gicp::PointCloud> target_tree(target_points, small_gicp::KdTreeBuilderOMP(num_threads));
    small_gicp::KdTree<small_gicp::PointCloud> source_tree(source_points, small_gicp::KdTreeBuilderOMP(num_threads));
    small_gicp::estimate_covariances_omp(*target_points, target_tree, num_neighbors, num_threads);
    small_gicp::estimate_covariances_omp(*source_points, source_tree, num_neighbors, num_threads);

    small_gicp::RegistrationSetting setting;
    setting.type = small_gicp::RegistrationSetting::GICP;
    setting.num_threads = num_threads;
    setting.max_iterations = std::max(1, config_.gicp_max_iterations);
    setting.max_correspondence_distance = config_.gicp_max_correspondence_distance;
    setting.translation_eps = config_.gicp_transformation_epsilon;
    setting.rotation_eps = config_.gicp_rotation_epsilon;
    const small_gicp::RegistrationResult registration = small_gicp::align(*target_points, *source_points, target_tree, initial_pose, setting);
    result = registration.T_target_source;

    const double max_fitness_squared_distance = config_.gicp_max_correspondence_distance;
    double squared_error = 0.0;
    size_t fitness_inliers = 0;
    for (const Eigen::Vector4d &point : source_points->points) {
      size_t index;
      double squared_distance;
      const Eigen::Vector4d transformed = result.matrix() * point;
      if (small_gicp::traits::nearest_neighbor_search(target_tree, transformed, &index, &squared_distance) &&
          squared_distance <= max_fitness_squared_distance) {
        squared_error += squared_distance;
        ++fitness_inliers;
      }
    }
    fitness = fitness_inliers > 0 ? squared_error / static_cast<double>(fitness_inliers) : std::numeric_limits<double>::infinity();
    return registration.converged && registration.num_inliers >= min_inliers && result.matrix().allFinite() && std::isfinite(fitness) &&
           fitness < config_.fitness_threshold;
  }

  bool buildLoopEdges(gtsam::NonlinearFactorGraph &graph) {
    if (searched_loop_id_ >= frames_.size() || optimized_.empty()) {
      return false;
    }

    const size_t last_optimized_id = optimized_.size() - 1;
    const Eigen::Isometry3d last_optimized = fromGtsam(optimized_.at<gtsam::Pose3>(last_optimized_id));
    const size_t stride = static_cast<size_t>(std::max(1, config_.loop_search_stride));

    for (size_t query_id = searched_loop_id_; query_id < frames_.size(); query_id += stride) {
      const Eigen::Isometry3d query_pose = last_optimized * frames_[last_optimized_id].T_odom_lidar.inverse() * frames_[query_id].T_odom_lidar;
      const int target_id = searchLoopTarget(query_id, query_pose);
      if (target_id < 0) {
        continue;
      }

      const std::shared_ptr<const vvec<float, 3>> source = cloudForFrame(query_id);
      const std::shared_ptr<vvec<float, 3>> target = buildTargetCloud(target_id);
      Eigen::Isometry3d registered_pose;
      double fitness = std::numeric_limits<double>::infinity();
      if (!registerLoop(query_pose, source, target, registered_pose, fitness)) {
        spdlog::debug("[pgo] loop GICP rejected: {} -> {}, fitness={:.4f}", query_id, target_id, fitness);
        continue;
      }

      const Eigen::Isometry3d target_pose = fromGtsam(optimized_.at<gtsam::Pose3>(target_id));
      const Eigen::Isometry3d relative = registered_pose.inverse() * target_pose;
      gtsam::Vector6 variances;
      variances.setConstant(std::max(fitness, 1e-9));
      const auto loop_noise =
          gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(1.0), gtsam::noiseModel::Diagonal::Variances(variances));
      graph.add(gtsam::BetweenFactor<gtsam::Pose3>(query_id, static_cast<size_t>(target_id), toGtsam(relative), loop_noise));
      memory_->saveLink(static_cast<int>(query_id + 1), target_id + 1, 1, toIsometry3f(relative));
      spdlog::info("[pgo] GICP loop detected: {} -> {}, fitness={:.4f}", query_id, target_id, fitness);
    }
    return !graph.empty();
  }

  void finalizeLoopSearch() {
    searched_loop_id_ = frames_.size();
    for (GraphFrame &frame : frames_) {
      frame.cloud_lidar.reset();
    }
  }

  IsamOutput updateIsam(const gtsam::NonlinearFactorGraph &graph, const gtsam::Values &initial) {
    if (initial.empty()) {
      isam2_->update(graph);
    } else {
      isam2_->update(graph, initial);
    }
    isam2_->calculateEstimate();
    isam2_->update();
    auto output = isam2_->calculateEstimateWithAffectedKeys();
    return {std::move(output.first), std::move(output.second)};
  }

  void rebuildGlobalMap() {
    auto merged = std::make_shared<vvec<float, 3>>();
    const size_t stride = static_cast<size_t>(config_.map_frame_stride);
    for (size_t i = 0; i < optimized_.size(); i += stride) {
      const Eigen::Isometry3d pose = fromGtsam(optimized_.at<gtsam::Pose3>(i));
      const std::shared_ptr<vvec<float, 3>> transformed = transformedCloud(cloudForFrame(i), pose);
      merged->insert(merged->end(), transformed->begin(), transformed->end());
    }
    const std::shared_ptr<vvec<float, 3>> sparse = voxelized(merged, config_.map_voxel_size);
    {
      std::lock_guard<std::mutex> lock(output_mutex_);
      global_map_ = sparse;
    }
    spdlog::debug("[pgo] sparse map rebuilt: poses={}, points={}", optimized_.size(), sparse->size());
  }

  void updateCorrection() {
    if (optimized_.empty()) {
      return;
    }
    const size_t last_id = optimized_.size() - 1;
    const Eigen::Isometry3d optimized_pose = fromGtsam(optimized_.at<gtsam::Pose3>(last_id));
    const Eigen::Isometry3d correction = optimized_pose * frames_[last_id].T_odom_lidar.inverse();

    std::lock_guard<std::mutex> lock(output_mutex_);
    T_map_odom_ = correction;
  }

  void saveOptimizedPoses(const gtsam::KeySet &changedKeys) {
    std::vector<std::pair<int, Eigen::Isometry3f>> poses;
    poses.reserve(changedKeys.size());
    for (const gtsam::Key key : changedKeys) {
      if (!optimized_.exists(key)) {
        continue;
      }
      poses.emplace_back(static_cast<int>(key + 1), toIsometry3f(fromGtsam(optimized_.at<gtsam::Pose3>(key))));
    }
    memory_->saveOptimizedPoses(poses);
  }

  bool updateOccupancyFrames(const gtsam::KeySet &changedKeys) {
    if (!occupancy_ || !local_grid_maker_ || optimized_.empty()) {
      return false;
    }

    bool updated = false;
    GridMapUpdate update = occupancy_->update([this](int nodeId, LocalGrid &grid) { return memory_->loadLocalGrid(nodeId, grid); });
    bool hasHistoricalChanges = false;
    for (const gtsam::Key key : changedKeys) {
      const size_t frameId = static_cast<size_t>(key);
      if (frameId >= occupancy_inserted_id_ || !optimized_.exists(key)) {
        continue;
      }

      const Eigen::Isometry3f pose = toIsometry3f(fromGtsam(optimized_.at<gtsam::Pose3>(key)));
      const int nodeId = static_cast<int>(frameId + 1);
      LocalGrid grid(static_cast<float>(occupancy_resolution_));
      if (!memory_->loadLocalGrid(nodeId, grid)) {
        throw std::runtime_error("missing LocalGrid for changed occupancy frame");
      }
      update -= nodeId;
      update += GridFrame(nodeId, pose, grid);
      hasHistoricalChanges = true;
    }
    if (hasHistoricalChanges) {
      if (!update.commit()) {
        throw std::runtime_error("failed to commit occupancy frame updates");
      }
      updated = true;
    }

    while (occupancy_inserted_id_ < optimized_.size()) {
      const size_t frameId = occupancy_inserted_id_;
      const Eigen::Isometry3f pose = toIsometry3f(fromGtsam(optimized_.at<gtsam::Pose3>(frameId)));
      const int nodeId = static_cast<int>(frameId + 1);
      LocalGrid grid(static_cast<float>(occupancy_resolution_));
      if (!memory_->loadLocalGrid(nodeId, grid)) {
        throw std::runtime_error("missing LocalGrid for new occupancy frame");
      }
      if (!occupancy_->append(GridFrame(nodeId, pose, grid))) {
        throw std::runtime_error("failed to append occupancy frame");
      }
      ++occupancy_inserted_id_;
      updated = true;
    }
    if (updated) {
      ++occupancy_revision_;
    }
    return updated;
  }

  void publishOccupancySnapshot() {
    if (!occupancy_) {
      return;
    }
    const OccupancyGridData map = occupancy_->getMap();
    auto msg = std::make_shared<NavigationGrid>();
    msg->resolution = map.resolution;
    msg->origin_x = map.originX;
    msg->origin_y = map.originY;
    msg->width = map.width;
    msg->height = map.height;
    msg->revision = occupancy_revision_;
    msg->data = map.cells;
    {
      std::lock_guard<std::mutex> lock(occupancy_mutex_);
      occupancy_msg_ = msg;
    }
    if (navigation_grid_callback_) {
      navigation_grid_callback_(std::move(msg));
    }
  }

  PoseGraphParameters config_;

  mutable std::mutex input_mutex_;
  std::deque<InputFrame> input_frames_;
  size_t keyframe_count_ = 0;
  Eigen::Isometry3d last_input_pose_ = Eigen::Isometry3d::Identity();
  double input_travel_distance_ = 0.0;
  std::vector<GraphFrame> frames_;
  std::unique_ptr<Memory> memory_;

  std::unique_ptr<gtsam::ISAM2> isam2_;
  gtsam::SharedNoiseModel prior_noise_;
  gtsam::SharedNoiseModel odom_noise_;
  gtsam::Values optimized_;
  gtsam::KeySet pending_changed_keys_;
  size_t added_odom_id_ = 0;
  size_t searched_loop_id_ = 0;

  mutable std::mutex output_mutex_;
  Eigen::Isometry3d T_map_odom_ = Eigen::Isometry3d::Identity();
  std::shared_ptr<const vvec<float, 3>> global_map_;
  std::unique_ptr<LocalGridMaker> local_grid_maker_;
  std::unique_ptr<OccupancyGrid> occupancy_;
  size_t occupancy_inserted_id_ = 0;
  double occupancy_resolution_ = 0.1;
  size_t occupancy_revision_ = 0;
  mutable std::mutex occupancy_mutex_;
  std::shared_ptr<const NavigationGrid> occupancy_msg_;
  NavigationGridCallback navigation_grid_callback_;
  std::atomic<bool> map_requested_{false};
  std::atomic<bool> input_requested_{false};

  std::atomic<bool> stop_{false};
  std::mutex wake_mutex_;
  std::condition_variable wake_cv_;
  std::thread worker_;
};

PoseGraphBackend::PoseGraphBackend(const PoseGraphParameters &config, const NaviMapParameters &navi_map, const std::string &database_path,
                                   NavigationGridCallback navigation_grid_callback)
    : impl_(std::make_unique<Impl>(config, navi_map, database_path, std::move(navigation_grid_callback))) {}

PoseGraphBackend::~PoseGraphBackend() = default;

bool PoseGraphBackend::enabled() const { return impl_->enabled(); }

void PoseGraphBackend::addFrame(const std::shared_ptr<const vvec<float, 3>> &cloud_lidar, const Eigen::Isometry3d &T_odom_lidar, double stamp) {
  impl_->addFrame(cloud_lidar, T_odom_lidar, stamp);
}

Eigen::Isometry3d PoseGraphBackend::T_map_odom() const { return impl_->correction(); }

void PoseGraphBackend::requestGlobalMap() { impl_->requestGlobalMap(); }

std::shared_ptr<const vvec<float, 3>> PoseGraphBackend::latestGlobalMap() const { return impl_->latestGlobalMap(); }

std::shared_ptr<const NavigationGrid> PoseGraphBackend::latestOccupancyGrid() const { return impl_->latestOccupancyGrid(); }
