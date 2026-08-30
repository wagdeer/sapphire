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
#include <cstdint>
#include <deque>
#include <iterator>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "local_gridmap.hpp"
#include "loop_closure.hpp"
#include "memory.hpp"
#include "occ_layer.hpp"

namespace sapphire {

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

double elapsedMilliseconds(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

enum class LoopAttemptOutcome : std::uint8_t {
  kNone = 0,
  kMissingCloud,
  kBbsRejected,
  kGicpRejected,
  kAccepted,
};

const char *loopAttemptOutcomeName(LoopAttemptOutcome outcome) noexcept {
  switch (outcome) {
    case LoopAttemptOutcome::kNone:
      return "no_candidate";
    case LoopAttemptOutcome::kMissingCloud:
      return "missing_cloud";
    case LoopAttemptOutcome::kBbsRejected:
      return "bbs_rejected";
    case LoopAttemptOutcome::kGicpRejected:
      return "gicp_rejected";
    case LoopAttemptOutcome::kAccepted:
      return "accepted";
  }
  return "unknown";
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

    gpu::initialize_device();

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
    loop_noise_ = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(10.0),
        gtsam::noiseModel::Diagonal::Variances(odom_variances));
    memory_ = std::make_unique<Memory>(database_path);

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

  void addFrame(SubmapFrame submap) {
    const LioFrame &lio = submap.lio();
    if (!config_.enabled || !lio.pcd || lio.pcd->empty() || !lio.T_odom_base.matrix().allFinite() || !std::isfinite(lio.timestamp)) {
      return;
    }

    std::lock_guard<std::mutex> lock(input_mutex_);
    if (submap.id() != submap_count_) {
      spdlog::error("[pgo] rejected out-of-order submap: expected={}, received={}", submap_count_, submap.id());
      return;
    }
    input_frames_.push_back({std::move(submap)});
    ++submap_count_;
    input_requested_.store(true, std::memory_order_release);
    wake_cv_.notify_one();
  }

  Eigen::Isometry3d correction() const {
    std::lock_guard<std::mutex> lock(output_mutex_);
    return T_map_odom_;
  }

  std::shared_ptr<const NavigationGrid> latestOccupancyGrid() const {
    std::lock_guard<std::mutex> lock(occupancy_mutex_);
    return occupancy_msg_;
  }

 private:
  struct InputFrame {
    SubmapFrame submap;
  };

  struct GraphFrame {
    Eigen::Isometry3d T_odom_lidar = Eigen::Isometry3d::Identity();
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
        return stop_.load(std::memory_order_acquire) || input_requested_.load(std::memory_order_acquire);
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
    frames_.reserve(frames_.size() + pending.size());
    size_t index = 0;
    try {
      for (; index < pending.size(); ++index) {
        InputFrame &input = pending[index];
        const SubmapFrame &submap = input.submap;
        const LioFrame &lio = submap.lio();
        const size_t frame_id = static_cast<size_t>(submap.id());
        if (frame_id != frames_.size()) {
          throw std::logic_error("non-contiguous submap sequence");
        }
        GraphFrame frame{lio.T_odom_base};
        LocalGrid grid(static_cast<float>(occupancy_resolution_));
        if (local_grid_maker_) {
          local_grid_maker_->createLocalMap(*lio.pcd, toIsometry3f(lio.T_odom_base), grid);
        }
        memory_->saveSubmap(submap, grid);
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
    if (!pending_changed_keys_.empty()) {
      saveOptimizedPoses(pending_changed_keys_);
    }

    gtsam::NonlinearFactorGraph loop_graph;
    const bool searchedNewFrames = searched_loop_id_ < frames_.size() && !optimized_.empty();
    if (buildLoopEdges(loop_graph)) {
      IsamOutput output = updateIsam(loop_graph, {});
      const gtsam::KeySet changed = changedFinalPoses(optimized_, output.estimate, output.affected_keys);
      pending_changed_keys_.insert(changed.begin(), changed.end());
      optimized_ = std::move(output.estimate);
      saveOptimizedPoses(changed);
      updateCorrection();
    }
    if (searchedNewFrames) {
      finalizeLoopSearch();
    }

    bool occupancy_updated = false;
    if (!pending_changed_keys_.empty()) {
      if (occupancy_) {
        occupancy_updated = updateOccupancyFrames(pending_changed_keys_);
      }
      pending_changed_keys_.clear();
    }
    if (occupancy_updated) {
      publishOccupancySnapshot();
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

  bool buildLoopEdges(gtsam::NonlinearFactorGraph &graph) {
    if (searched_loop_id_ >= frames_.size() || optimized_.empty()) {
      return false;
    }

    const size_t last_optimized_id = optimized_.size() - 1;
    const Eigen::Isometry3d last_optimized = fromGtsam(optimized_.at<gtsam::Pose3>(last_optimized_id));

    for (size_t query_id = searched_loop_id_; query_id < frames_.size(); ++query_id) {
      const auto query_start = std::chrono::steady_clock::now();
      const Eigen::Isometry3d query_pose = last_optimized * frames_[last_optimized_id].T_odom_lidar.inverse() * frames_[query_id].T_odom_lidar;
      LoopCandidateBatch batch = prepareLoopCandidates(*memory_, query_id, query_pose);
      std::size_t cold_load_count = batch.stats.cold_loads;
      std::size_t gicp_count = 0;
      LoopAttemptOutcome outcome = LoopAttemptOutcome::kNone;
      std::int64_t selected_target = -1;
      LoopCandidate *top_candidate = nullptr;
      BbsResult bbs_result;
      GicpResult gicp_result;

      std::vector<gpu::LocalSearchTarget> bbs_targets;
      bbs_targets.reserve(batch.candidates.size());
      for (LoopCandidate &candidate : batch.candidates) {
        bbs_targets.push_back({candidate.target_id, &candidate.target_voxelmaps, candidate.target_T_query_initial});
      }
      if (batch.query_cloud && !bbs_targets.empty()) {
        bbs_result = alignLoopBbs(*batch.query_cloud, bbs_targets);
      }
      if (bbs_result.accepted) {
        const auto found = std::find_if(batch.candidates.begin(), batch.candidates.end(), [&](const LoopCandidate &candidate) {
          return candidate.target_id == bbs_result.target_id;
        });
        if (found != batch.candidates.end()) {
          top_candidate = &*found;
          selected_target = static_cast<std::int64_t>(bbs_result.target_id);
        }
      } else if (!batch.candidates.empty()) {
        outcome = LoopAttemptOutcome::kBbsRejected;
        spdlog::debug("[pgo] GPU BBS rejected query {}: candidates={}, best_overlap={:.3f}, elapsed_ms={:.3f}", query_id,
                      batch.candidates.size(), bbs_result.overlap, bbs_result.elapsed_ms);
      }

      if (top_candidate) {
        const size_t target_id = static_cast<size_t>(top_candidate->target_id);
        selected_target = static_cast<std::int64_t>(target_id);
        const std::shared_ptr<const vvec<float, 3>> target_cloud = memory_->loadCloud(target_id);
        ++cold_load_count;
        if (!target_cloud || target_cloud->empty()) {
          outcome = LoopAttemptOutcome::kMissingCloud;
          spdlog::debug("[pgo] loop candidate rejected: {} -> {}, reason={}", query_id, target_id, loopAttemptOutcomeName(outcome));
        } else {
          ++gicp_count;
          gicp_result = refineLoopGicp(*batch.query_cloud, *target_cloud, bbs_result.T_target_query);
          if (!gicp_result.accepted) {
            outcome = LoopAttemptOutcome::kGicpRejected;
            spdlog::debug("[pgo] loop candidate rejected: {} -> {}, reason={}, fitness={:.4f}", query_id, target_id,
                          loopAttemptOutcomeName(outcome), gicp_result.fitness);
          } else {
            outcome = LoopAttemptOutcome::kAccepted;
            const Eigen::Isometry3d relative = gicp_result.T_target_query.inverse();
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(query_id, target_id, toGtsam(relative), loop_noise_));
            memory_->saveLink(static_cast<int>(query_id + 1), static_cast<int>(target_id + 1), 1, toIsometry3f(relative));
            spdlog::info("[pgo] BBS/GICP loop detected: {} -> {}, overlap={:.3f}, fitness={:.4f}", query_id, target_id, bbs_result.overlap,
                         gicp_result.fitness);
          }
        }
      }

      spdlog::info(
          "[pgo] loop recall query={} target={} spatial_examined={} hard_filtered={} history_filtered={} adjacent_filtered={} "
          "candidates={} roots={} tolerant_overlap={:.3f} cold_loads={} bbs_count={} gicp_count={} "
          "initial_overlap={:.3f} bbs_elapsed_ms={:.3f} "
          "gicp_points={}/{} gicp_ms={:.3f}({:.3f}+{:.3f}) "
          "expanded=[{},{},{},{}] pruned=[{},{},{},{}] "
          "bbs_termination={} total_elapsed_ms={:.3f} result={}",
          query_id, selected_target, batch.stats.spatial_matches, batch.stats.rejected(), batch.stats.rejected_non_historical,
          batch.stats.rejected_adjacent, batch.candidates.size(), bbs_result.root_nodes, top_candidate ? bbs_result.overlap : 0.0,
          cold_load_count, bbs_targets.size(), gicp_count, bbs_result.initial_overlap, bbs_result.elapsed_ms,
          gicp_result.query_points, gicp_result.target_points, gicp_result.total_elapsed_ms,
          gicp_result.preparation_elapsed_ms, gicp_result.alignment_elapsed_ms,
          bbs_result.expanded_nodes[0], bbs_result.expanded_nodes[1], bbs_result.expanded_nodes[2], bbs_result.expanded_nodes[3],
          bbs_result.pruned_nodes[0], bbs_result.pruned_nodes[1], bbs_result.pruned_nodes[2], bbs_result.pruned_nodes[3],
          gpu::search_termination_name(bbs_result.termination), elapsedMilliseconds(query_start),
          loopAttemptOutcomeName(outcome));
    }
    return !graph.empty();
  }

  void finalizeLoopSearch() { searched_loop_id_ = frames_.size(); }

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
    GridMapUpdate update = occupancy_->update(
        [this](int nodeId, LocalGrid &grid) { return nodeId > 0 && memory_->loadLocalGrid(static_cast<std::uint64_t>(nodeId - 1), grid); });
    bool hasHistoricalChanges = false;
    for (const gtsam::Key key : changedKeys) {
      const size_t frameId = static_cast<size_t>(key);
      if (frameId >= occupancy_inserted_id_ || !optimized_.exists(key)) {
        continue;
      }

      const Eigen::Isometry3f pose = toIsometry3f(fromGtsam(optimized_.at<gtsam::Pose3>(key)));
      const int nodeId = static_cast<int>(frameId + 1);
      LocalGrid grid(static_cast<float>(occupancy_resolution_));
      if (!memory_->loadLocalGrid(frameId, grid)) {
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
      if (!memory_->loadLocalGrid(frameId, grid)) {
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
  size_t submap_count_ = 0;
  std::vector<GraphFrame> frames_;
  std::unique_ptr<Memory> memory_;

  std::unique_ptr<gtsam::ISAM2> isam2_;
  gtsam::SharedNoiseModel prior_noise_;
  gtsam::SharedNoiseModel odom_noise_;
  gtsam::SharedNoiseModel loop_noise_;
  gtsam::Values optimized_;
  gtsam::KeySet pending_changed_keys_;
  size_t added_odom_id_ = 0;
  size_t searched_loop_id_ = 0;

  mutable std::mutex output_mutex_;
  Eigen::Isometry3d T_map_odom_ = Eigen::Isometry3d::Identity();
  std::unique_ptr<LocalGridMaker> local_grid_maker_;
  std::unique_ptr<OccupancyGrid> occupancy_;
  size_t occupancy_inserted_id_ = 0;
  double occupancy_resolution_ = 0.1;
  size_t occupancy_revision_ = 0;
  mutable std::mutex occupancy_mutex_;
  std::shared_ptr<const NavigationGrid> occupancy_msg_;
  NavigationGridCallback navigation_grid_callback_;
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

void PoseGraphBackend::addFrame(SubmapFrame frame) { impl_->addFrame(std::move(frame)); }

Eigen::Isometry3d PoseGraphBackend::T_map_odom() const { return impl_->correction(); }

std::shared_ptr<const NavigationGrid> PoseGraphBackend::latestOccupancyGrid() const { return impl_->latestOccupancyGrid(); }

}  // namespace sapphire
