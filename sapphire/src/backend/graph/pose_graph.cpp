#include "backend/graph/pose_graph.hpp"

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
#include <cstring>
#include <deque>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "backend/grid/ground_estimator.hpp"
#include "backend/grid/local_gridmap.hpp"
#include "backend/registration/loop_closure.hpp"
#include "backend/storage/memory.hpp"
#include "backend/grid/occ_layer.hpp"
#include "backend/visual/feature/scene_features.hpp"
#include "tools/timer.hpp"

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
       NavigationGridCallback navigation_grid_callback, std::function<void()> failure_notification, std::function<void()> ready_notification)
      : config_(config), navi_map_(navi_map), navigation_grid_callback_(std::move(navigation_grid_callback)), failure_notification_(std::move(failure_notification)), ready_notification_(std::move(ready_notification)) {
    const std::string config_identity = map_config_identity(config, navi_map);
    historical_ = config.map_mode == "resume";
    if (!historical_ && config.map_mode != "new") throw MapError(MapErrorCode::UnsupportedMode, "Unsupported map mode: " + config.map_mode);
    if (!config_.enabled && !historical_) {
      return;
    }

    // Exclusive map creation must reject an existing destination before GPU/runtime initialization.
    memory_ = std::make_unique<Memory>(database_path, RecallIndexAdapters{}, config_.storage, config_identity, config.map_mode,
                                       static_cast<float>(navi_map.resolution));
    progress_.committed_revision = memory_->committedRevision();
    progress_.map_uuid = memory_->mapUuid();
    if (!historical_) gpu::initialize_device();

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
    loop_noise_ = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(10.0, gtsam::noiseModel::mEstimator::Base::Block),
                                                    gtsam::noiseModel::Diagonal::Variances(odom_variances));
    if (historical_) {
      reconstructHistory(navi_map);
      return;
    }
    if (config_.visual.loopEnabled()) visual_index_ = std::make_unique<VisualSubmapIndex>(config_.visual, *memory_);

    if (navi_map.enabled) {
      ground_estimator_ = std::make_unique<GroundEstimator>(navi_map);
      local_grid_maker_ = std::make_unique<LocalGridMaker>(navi_map);
      occupancy_ = std::make_unique<OccupancyGrid>(navi_map);
      occupancy_resolution_ = navi_map.resolution;
      spdlog::info("[pgo] occupancy grid enabled (reso={:.2f}m, d_max={:.2f}m)", navi_map.resolution, navi_map.d_max);
    }

    worker_ = std::thread([this]() { workerLoop(); });
    spdlog::info("[pgo] asynchronous pose-graph backend enabled");
  }

  ~Impl() {
    try {
      finish();
    } catch (const std::exception &error) {
      spdlog::error("[pgo] storage finish failed: {}", error.what());
    } catch (...) {
      // finish() preserves the recorded failure and remains checked for callers.
      // Final destruction must not rethrow it; member cleanup still obeys W's
      // complete-close/admission rules, including deliberate invariant fail-stop.
      spdlog::error("[pgo] storage finish failed: non-standard exception");
    }
  }

  bool failed() const { return failed_.load(std::memory_order_acquire); }
  void requireHealthy() const {
    if (failed() || fenced_.load(std::memory_order_acquire))
      throw MapError(MapErrorCode::Lifecycle, "Backend is failed/stopped; DB reconstruction required");
  }
  void recordFailure(std::exception_ptr error) {
    {
      std::lock_guard<std::mutex> lock(failure_mutex_);
      if (!failure_) failure_ = error;
      error = failure_;
    }
    {
      std::lock_guard<std::mutex> input(input_mutex_);
      if (!progress_.first_failure) progress_.first_failure = error;
      failed_.store(true, std::memory_order_release);
      stop_.store(true, std::memory_order_release);
    }
    wake_cv_.notify_all();
    capacity_cv_.notify_all();
  }
  void drain() {
    std::lock_guard<std::mutex> finishing(finish_mutex_);
    stopAdmission(false);
    { std::lock_guard<std::mutex> input(input_mutex_); stop_.store(true, std::memory_order_release); }
    capacity_cv_.notify_all(); wake_cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    drained_ = true;
    std::lock_guard<std::mutex> lock(failure_mutex_);
    if (failure_) std::rethrow_exception(failure_);
  }
  void close() {
    std::lock_guard<std::mutex> finishing(finish_mutex_);
    if (!drained_) throw MapError(MapErrorCode::Lifecycle, "Close requires checked accepted-work drain first");
    fenced_.store(true, std::memory_order_release);
    std::lock_guard<std::recursive_mutex> lock(lifecycle_mutex_);
    std::exception_ptr visual_error;
    try { if (visual_index_ && !failed()) visual_index_->settle(); } catch (...) { visual_error = std::current_exception(); }
    visual_index_.reset();
    try { if (memory_) memory_->finish(); }
    catch (...) { recordFailure(std::current_exception()); throw; }
    if (visual_error) recordFailure(visual_error);
    if (visual_error) std::rethrow_exception(visual_error);
  }
  void finish() {
    std::exception_ptr first;
    try { drain(); } catch (...) { first = std::current_exception(); }
    try { close(); } catch (...) { if (!first) first = std::current_exception(); }
    if (first) std::rethrow_exception(first);
  }
  void notifyReady() noexcept {
    try { if (ready_notification_) ready_notification_(); } catch (...) {}
  }
  void observeCommit(std::uint64_t revision) {
    std::lock_guard<std::mutex> input(input_mutex_);
    progress_.committed_revision = revision; progress_.committed_outcome_known = true;
  }
  bool captureReady(bool navigation, bool try_only, std::string &uuid, std::uint64_t &revision,
                    std::uint64_t &generation, std::uint64_t &source, double &timestamp,
                    Eigen::Isometry3d &correction, Eigen::Isometry3d &pose,
                    std::shared_ptr<const NavigationGrid> &grid) const {
    std::unique_lock<std::recursive_mutex> lifecycle(lifecycle_mutex_, std::defer_lock);
    if (try_only) { if (!lifecycle.try_lock()) return false; } else lifecycle.lock();
    requireHealthy();
    if (!hasActiveCorrection() || frames_.empty()) return false;
    { std::lock_guard<std::mutex> input(input_mutex_);
      if (!progress_.ready_available || !progress_.completed_revision || !progress_.last_completed) return false;
      revision = progress_.ready_revision; source = *progress_.last_completed;
    }
    uuid = memory_->mapUuid(); generation = producer_generation_; timestamp = ready_timestamp_;
    const auto committed_pose = memory_->submapPose(frames_.size() - 1);
    if (!committed_pose || memory_->committedRevision() != revision) {
      auto error = std::make_exception_ptr(MapError(MapErrorCode::Lifecycle, "Invalid coherent ready owner"));
      const_cast<Impl *>(this)->recordFailure(error); std::rethrow_exception(error);
    }
    pose = committed_pose->cast<double>();
    correction = T_map_odom_;
    grid.reset();
    if (navigation) {
      grid = latestOccupancyGrid();
      if (!grid || grid->map_uuid != uuid || grid->source_graph_revision != revision) {
        auto error = std::make_exception_ptr(MapError(MapErrorCode::Lifecycle, "B3 grid source disagrees with coherent ready capture"));
        const_cast<Impl *>(this)->recordFailure(error); std::rethrow_exception(error);
      }
    }
    return true;
  }
  void promoteStorageToWritable() {
    std::lock_guard<std::recursive_mutex> lock(lifecycle_mutex_);
    requireHistorical();
    // Historical queries complete their training before returning. This mutex
    // fences new queries and joins all active query work before the handoff.
    try {
      memory_->promoteToWritable(static_cast<float>(navi_map_.resolution));
    } catch (...) {
      if (memory_->storageState() != MapDatabase::State::HistoricalReadOnly) recordFailure(std::current_exception());
      throw;
    }
  }
  std::uint64_t commitFinalizedSubmap(const SubmapFrame &submap, const LocalGrid &grid, const std::vector<std::uint8_t> &scene,
                                      const std::vector<std::pair<int, Eigen::Isometry3f>> &poses, const GraphLink &base,
                                      const std::optional<GraphLink> &loop, const std::string &uuid, std::uint64_t revision, int next_id,
                                      int chain_root) {
    std::lock_guard<std::recursive_mutex> lock(lifecycle_mutex_);
    requireHealthy();
    if (attached_) throw MapError(MapErrorCode::Lifecycle, "B1 root is attached; use the bounded continuation admission API");
    if (!historical_ || memory_->storageState() != MapDatabase::State::Writable)
      throw MapError(MapErrorCode::ReadOnly, "W commit requires promoted historical storage");
    requireHistorical();  // W must not reuse a runtime left behind a prior successful commit.
    bool committed = false;
    try {
      database_detail::writableStorageTestPoint("tentative-backend-solver", isam2_.get());
      const auto result = memory_->commitFinalizedSubmap(submap, grid, scene, poses, base, loop, uuid, revision, next_id,
                                                         map_config_identity(config_, navi_map_), chain_root);
      committed = true;
      observeCommit(result);
      { std::lock_guard<std::mutex> input(input_mutex_); progress_.last_commit_outcome = CommitOutcome::Committed; }
      runtime_current_ = false;  // W has no B2 materialization/continuation. Reconstruct before serving this new history.
      database_detail::writableStorageTestPoint("backend-postcommit-materialization", isam2_.get());
      return result;
    } catch (...) {
      const auto cause = std::current_exception();
      auto outcome = committed ? CommitOutcome::Committed : CommitOutcome::NotCommitted;
      try { std::rethrow_exception(cause); }
      catch (const MapError &error) { if (!committed) outcome = error.outcome(); }
      catch (...) {}
      // Memory retains authoritative C after Committed; Unknown exposes only
      // its last-known C. Never infer a resolution by querying under input_mutex_.
      if (outcome == CommitOutcome::Committed) observeCommit(memory_->committedRevision());
      { std::lock_guard<std::mutex> input(input_mutex_);
        progress_.last_commit_outcome = outcome;
        if (outcome == CommitOutcome::Unknown) progress_.committed_outcome_known = false;
      }
      recordFailure(cause);
      if (committed) throw MapError(MapErrorCode::CommittedFailure, "DB revision committed; backend runtime failed", CommitOutcome::Committed);
      throw;
    }
  }

  AttachmentResult attachFreshSessionAutomatically(SubmapFrame &query, const LocalGrid &grid, std::uint64_t generation, const SubmapFrame *observation) {
    std::lock_guard<std::recursive_mutex> lifecycle(lifecycle_mutex_);
    requireHistorical();
    if (attached_ || memory_->storageState() != MapDatabase::State::HistoricalReadOnly || cancel_requested_.load())
      throw MapError(MapErrorCode::Lifecycle, "Automatic association requires an unattached live historical backend");
    const auto reject = [](AttachmentStatus status, const char *message) {
      AttachmentResult result; result.status = status; result.message = message; return result;
    };
    if (frames_.empty()) return reject(AttachmentStatus::EmptyMap, "No historical association target");
    const auto rigid = [](const Eigen::Isometry3d &pose) {
      return pose.matrix().allFinite() && pose.matrix().cast<float>().allFinite() &&
          (pose.matrix().row(3)-Eigen::RowVector4d(0,0,0,1)).norm() < 1e-9 &&
          (pose.linear().transpose()*pose.linear()-Eigen::Matrix3d::Identity()).norm() < 1e-6 &&
          std::abs(pose.linear().determinant()-1) < 1e-6;
    };
    const auto &anchor = query.lio().T_odom_base;
    if (!boundedInput(query, grid) || !rigid(anchor) || query.id() == UINT64_MAX || query.odom_poses().empty() ||
        !query.lio().pcd || query.lio().pcd->empty() || !query.bounds().valid() ||
        !std::isfinite(query.lio().timestamp) || !std::isfinite(query.odom_poses().front().timestamp) ||
        !query.odom_poses().front().T_odom_base.matrix().allFinite() ||
        std::abs(query.lio().timestamp-query.odom_poses().front().timestamp)>1e-9 ||
        (anchor.matrix()-query.odom_poses().front().T_odom_base.matrix()).norm()>1e-9)
      return reject(AttachmentStatus::InvalidQuery, "Invalid or oversized fresh association evidence");
    if (observation && (observation->id() < query.id() || observation->lio().timestamp < query.lio().timestamp ||
        !std::isfinite(observation->lio().timestamp) || !rigid(observation->lio().T_odom_base) ||
        !boundedInput(*observation, LocalGrid{})))
      return reject(AttachmentStatus::InvalidQuery, "Invalid later same-domain visual observation");
    const auto candidates = visual_index_->queryFreshSession(query, observation ? &observation->visual_frames() : nullptr);
    std::vector<Eigen::Isometry3d> placements;
    int selected_target = 0;
    Eigen::Isometry3d selected = Eigen::Isometry3d::Identity();
    for (const auto &candidate : candidates) {
      if (!candidate.T_target_query) continue;
      const auto &seed = *candidate.T_target_query;
      if (!rigid(seed)) throw MapError(MapErrorCode::HistoricalData, "Invalid metric association seed");
      const auto pose = memory_->submapPose(candidate.target_id);
      const auto cloud = memory_->loadCloud(candidate.target_id);
      const auto pyramid = memory_->loadPyramidVoxel(candidate.target_id);
      if (!pose || !cloud || cloud->empty() || !pyramid.valid())
        throw MapError(MapErrorCode::HistoricalData, "Historical association geometry is missing");
      const double radius = query.bounds().minimum.cwiseAbs().cwiseMax(query.bounds().maximum.cwiseAbs()).cast<double>().norm();
      const double bound = radius + seed.translation().cwiseAbs().maxCoeff() + 6.;
      if (!std::isfinite(bound) || bound >= double(std::numeric_limits<int>::max()/2)*pyramid.min_level_resolution)
        throw MapError(MapErrorCode::HistoricalData, "Association seed exceeds integer voxel representation");
      gpu::initialize_device();
      auto score = scoreLoopSeed(*query.lio().pcd, pyramid, seed.cast<float>());
      database_detail::writableStorageTestPoint("association-after-score", &score);
      if (!acceptsLoopSeedScore(score)) continue;
      auto refined = refineLoopGicp(query.lio().pcd, cloud, seed);
      database_detail::writableStorageTestPoint("association-after-gicp", &refined);
      if (!refined.accepted) continue;
      if (!rigid(refined.T_target_query)) throw MapError(MapErrorCode::HistoricalData, "Nonrigid accepted association measurement");
      const Eigen::Isometry3d placement = pose->cast<double>() * refined.T_target_query;
      const Eigen::Isometry3d correction = placement * anchor.inverse();
      if (!rigid(correction)) throw MapError(MapErrorCode::HistoricalData, "Unrepresentable association correction");
      for (const auto &other : placements) {
        const double translation = (other.translation()-placement.translation()).norm();
        const double rotation = Eigen::AngleAxisd(other.linear().transpose()*placement.linear()).angle();
        if (translation > .5 || rotation > 5. * 3.141592653589793 / 180.)
          return reject(AttachmentStatus::AmbiguousAssociation, "Accepted historical candidates disagree on the new-session location");
      }
      if (!selected_target) { selected_target = int(candidate.target_id+1); selected = refined.T_target_query; }
      placements.push_back(placement);
    }
    if (!selected_target) return reject(AttachmentStatus::NoAssociation, "No metric visual candidate passed independent geometry");
    if (cancel_requested_.load()) throw MapError(MapErrorCode::Lifecycle, "Automatic association canceled before attachment");
    spdlog::info("[association] source={} candidates={} agreeing={} selected_sql_target={}",
                 query.id(), candidates.size(), placements.size(), selected_target);
    return attachFreshSession(query, grid, selected_target, selected, generation, true);
  }

  AttachmentResult attachFreshSession(SubmapFrame &query, const LocalGrid &grid, int target_node_id,
                                      const Eigen::Isometry3d &seed, std::uint64_t generation, bool registration_verified = false) {
    std::lock_guard<std::recursive_mutex> lifecycle(lifecycle_mutex_);
    requireHistorical();
    if (attached_ || memory_->storageState() != MapDatabase::State::HistoricalReadOnly)
      throw MapError(MapErrorCode::Lifecycle, "Attachment requires an unattached historical backend");
    const auto reject = [](AttachmentStatus status, const std::string &message) {
      AttachmentResult result;
      result.status = status;
      result.message = message;
      return result;
    };
    const auto rigid = [](const Eigen::Isometry3d &pose) {
      return pose.matrix().allFinite() && pose.matrix().row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-9) &&
             (pose.linear().transpose() * pose.linear()).isApprox(Eigen::Matrix3d::Identity(), 1e-6) &&
             std::abs(pose.linear().determinant() - 1.0) < 1e-6;
    };
    if (frames_.empty()) return reject(AttachmentStatus::EmptyMap, "An empty historical map has no attachment target");
    if (target_node_id <= 0 || std::size_t(target_node_id) > frames_.size() || !memory_->sceneActive(target_node_id-1))
      return reject(AttachmentStatus::InvalidTarget, "Target must be a committed historical SQL Node ID");
    if (!rigid(seed) || !seed.matrix().cast<float>().allFinite())
      return reject(AttachmentStatus::InvalidSeed, "T_H_Q seed must be finite, rigid and representable for metric registration");
    const Eigen::Isometry3d anchor = query.lio().T_odom_base;
    if (!rigid(anchor) || query.odom_poses().empty() ||
        (anchor.matrix() - query.odom_poses().front().T_odom_base.matrix()).cwiseAbs().maxCoeff() > 1e-9 ||
        !std::isfinite(query.lio().timestamp) || std::abs(query.lio().timestamp - query.odom_poses().front().timestamp) > 1e-9 ||
        !query.lio().pcd || query.lio().pcd->empty() || !query.bounds().valid() ||
        frames_.size() >= std::size_t(std::numeric_limits<int>::max()))
      return reject(AttachmentStatus::InvalidQuery, "Invalid fresh query anchor/cloud or exhausted Node ID domain");
    const auto source_sequence = query.id();
    const auto key = frames_.size();
    const int root_id = static_cast<int>(key + 1);
    const auto target_key = static_cast<std::uint64_t>(target_node_id - 1);
    const std::string uuid = memory_->mapUuid();
    const auto revision = memory_->committedRevision();
    Eigen::Isometry3d verified;
    std::vector<std::uint8_t> scene;
    try {
      const auto target_pose = memory_->submapPose(target_key);
      auto target_cloud = memory_->loadCloud(target_key);
      database_detail::writableStorageTestPoint("b1-target-cloud", &target_cloud);
      const auto pyramid = memory_->loadPyramidVoxel(target_key);
      if (!target_pose || !target_cloud || target_cloud->empty() || !pyramid.valid())
        return reject(AttachmentStatus::MissingGeometry, "Target has no usable committed registration geometry");
      // BBS discretizes transformed points into signed-int voxel coordinates.
      // Keep the seed, Q bounds and unchanged six-metre search window inside
      // that representation, with headroom for float rounding/coarse levels.
      const double query_radius = query.bounds().minimum.cwiseAbs().cwiseMax(query.bounds().maximum.cwiseAbs()).cast<double>().norm();
      const double coordinate_bound = query_radius + seed.translation().cwiseAbs().maxCoeff() + 6.0;
      const double voxel_limit = double(std::numeric_limits<int>::max() / 2) * pyramid.min_level_resolution;
      if (!std::isfinite(coordinate_bound) || coordinate_bound >= voxel_limit)
        return reject(AttachmentStatus::InvalidSeed, "Seed/query bounds exceed the BBS integer voxel domain");
      if (registration_verified) {
        // Only the internal automatic decision can enter here, under this same
        // lifecycle lock, with its already accepted exact GICP measurement.
        verified = seed;
      } else {
        database_detail::writableStorageTestPoint("b1-before-registration");
        gpu::initialize_device();
        auto coarse = alignLoopBbs(*query.lio().pcd, {{target_key, &pyramid, seed.cast<float>()}});
        database_detail::writableStorageTestPoint("b1-after-bbs", &coarse);
        if (!coarse.accepted) return reject(AttachmentStatus::BbsRejected, "BBS rejected the supplied attachment");
        auto refined = refineLoopGicp(query.lio().pcd, target_cloud, coarse.T_target_query);
        database_detail::writableStorageTestPoint("b1-after-gicp", &refined);
        if (!refined.accepted || !rigid(refined.T_target_query))
          return reject(AttachmentStatus::GicpRejected, "GICP rejected the supplied attachment");
        verified = refined.T_target_query;
      }
      // Prepare appearance evidence without inserting query target membership.
      scene = buildVisualScene(query)->encode();
      database_detail::writableStorageTestPoint("b1-before-solver", isam2_.get());
    } catch (const std::exception &error) {
      return reject(AttachmentStatus::RegistrationFailure, error.what());
    }

    bool solver_started = false;
    bool committed = false;
    const char *stage = "tentative optimization";
    try {
      // Q->H predicts inverse(X_Q)*X_H. Serialize the same measurement used by
      // reconstruction, and never create a prior or a cross-session odom edge.
      const GraphLink attachment{root_id, target_node_id, 1, verified.inverse().cast<float>()};
      gtsam::NonlinearFactorGraph graph;
      graph.add(gtsam::BetweenFactor<gtsam::Pose3>(key, target_key, toGtsam(attachment.transform.cast<double>()), loop_noise_));
      database_detail::writableStorageTestPoint("b1-before-values");
      gtsam::Values initial;
      initial.insert(key, toGtsam(memory_->submapPose(target_key)->cast<double>() * verified));
      // Local factor/graph/Values construction above is retryable. Mark the
      // boundary BEFORE entering the first potentially mutating operation: even
      // an exception within its first ISAM update makes solver state untrusted.
      solver_started = true;
      auto proposed = updateIsam(graph, initial).estimate;
      database_detail::writableStorageTestPoint("b1-after-solver", isam2_.get());
      if (proposed.size() != key + 1 || !std::isfinite(isam2_->getFactorsUnsafe().error(proposed)))
        throw MapError(MapErrorCode::ReconstructionDiscrepancy, "Invalid tentative attachment estimate");
      std::vector<std::pair<int, Eigen::Isometry3f>> poses;
      for (std::size_t id = 0; id <= key; ++id) {
        const auto pose = fromGtsam(proposed.at<gtsam::Pose3>(id));
        if (!rigid(pose) || !pose.matrix().cast<float>().allFinite())
          throw MapError(MapErrorCode::ReconstructionDiscrepancy, "Nonfinite/nonrigid tentative committed pose");
        const auto serialized = toIsometry3f(pose);
        if (id == key && !(serialized.cast<double>() * anchor.inverse()).matrix().allFinite())
          throw MapError(MapErrorCode::ReconstructionDiscrepancy, "Tentative root cannot yield a finite active correction");
        const auto old = memory_->submapPose(id);
        // Inspect ALL keys, including A2 solver differences outside affected_keys.
        if (!old || !(old->matrix().array() == serialized.matrix().array()).all())
          poses.emplace_back(static_cast<int>(id + 1), serialized);
      }
      query = std::move(query).withPersistentId(key);
      stage = "W promotion";
      database_detail::writableStorageTestPoint("b1-before-promotion");
      memory_->promoteToWritable(static_cast<float>(navi_map_.resolution));
      stage = "W finalized commit";
      auto committed_revision = memory_->commitFinalizedSubmap(query, grid, scene, poses, attachment, std::nullopt,
                                                                     uuid, revision, root_id,
                                                                     map_config_identity(config_, navi_map_), root_id);
      committed = true;
      observeCommit(committed_revision);
      runtime_current_ = false;
      stage = "committed runtime materialization";
      refreshScene(attachment);
      committed_revision=memory_->committedRevision();
      database_detail::writableStorageTestPoint("b1-postcommit-materialization");
      const auto start = std::chrono::steady_clock::now();
      frames_.clear();
      optimized_.clear();
      memory_->visitHistoricalNodes([this](int id, const auto &original, const auto &pose, const auto &) {
        frames_.push_back({original});
        optimized_.insert(id - 1, toGtsam(pose.template cast<double>()));
      });
      materializeCommittedHistory(true);
      spdlog::info("[pgo] B1 committed runtime rebuild nodes={} elapsed_ms={}", frames_.size(), elapsedMilliseconds(start));
      database_detail::writableStorageTestPoint("b1-runtime-ready");
      const Eigen::Isometry3d correction = memory_->submapPose(key)->cast<double>() * anchor.inverse();
      if (!correction.matrix().allFinite())
        throw MapError(MapErrorCode::CommittedFailure, "Committed root cannot yield a finite active correction", CommitOutcome::Committed);
      {
        std::lock_guard<std::mutex> lock(output_mutex_);
        T_map_odom_ = correction;
      }
      runtime_current_ = true;
      attached_ = true;
      active_root_ = key;
      producer_generation_ = generation;
      root_source_sequence_ = source_sequence;
      expected_source_ = source_sequence + 1;
      { std::lock_guard<std::mutex> input(input_mutex_);
        progress_.ready_revision = committed_revision; progress_.ready_available = true;
        progress_.completed_revision = committed_revision; progress_.last_completed = source_sequence;
        progress_.root_revision = committed_revision; progress_.root_source = source_sequence; progress_.generation = generation;
      }
      ready_timestamp_ = query.lio().timestamp;
      return {AttachmentStatus::Attached, {}, root_id, committed_revision, correction};
    } catch (const MapError &error) {
      if (!solver_started) return reject(AttachmentStatus::RegistrationFailure, error.what());
      if (error.outcome() == CommitOutcome::Committed) observeCommit(memory_->committedRevision());
      { std::lock_guard<std::mutex> input(input_mutex_);
        progress_.last_commit_outcome = committed && error.outcome()!=CommitOutcome::Unknown ? CommitOutcome::Committed : error.outcome();
        if (error.outcome() == CommitOutcome::Unknown) progress_.committed_outcome_known = false;
      }
      recordFailure(std::current_exception());
      if (committed && error.outcome()!=CommitOutcome::Unknown) throw MapError(MapErrorCode::CommittedFailure, "B1 root " + std::to_string(root_id) +
                                      " committed; runtime materialization failed", CommitOutcome::Committed);
      throw;
    } catch (const std::exception &error) {
      if (!solver_started) return reject(AttachmentStatus::RegistrationFailure, error.what());
      recordFailure(std::current_exception());
      throw MapError(committed ? MapErrorCode::CommittedFailure : MapErrorCode::Storage,
                     std::string("B1 ") + stage + " failed: " + error.what(),
                     committed ? CommitOutcome::Committed : CommitOutcome::NotCommitted);
    } catch (...) {
      if (!solver_started) throw;
      recordFailure(std::current_exception());
      throw MapError(committed ? MapErrorCode::CommittedFailure : MapErrorCode::Storage, "B1 failed with an unknown exception",
                     committed ? CommitOutcome::Committed : CommitOutcome::NotCommitted);
    }
  }

  bool hasActiveCorrection() const {
    std::lock_guard<std::recursive_mutex> lifecycle(lifecycle_mutex_);
    return !cancel_requested_.load() && !failed() && !fenced_.load(std::memory_order_acquire) && (historical_ ? attached_ && runtime_current_ : enabled() && !frames_.empty());
  }
  void resetFreshSession() {
    stopAdmission(true);
    {
      std::lock_guard<std::recursive_mutex> lifecycle(lifecycle_mutex_);
      if (!historical_) throw MapError(MapErrorCode::UnsupportedMode, "B1 reset requires a historical attachment backend");
      attached_ = false;
      fenced_.store(true, std::memory_order_release);
    }
    finish();
  }

  bool enabled() const { return config_.enabled && !historical_ && !failed() && !fenced_.load(std::memory_order_acquire); }
  bool historical() const { return historical_; }

  void addFrame(SubmapFrame submap) {
    const auto admission_start = std::chrono::steady_clock::now();
    requireHealthy();
    if (historical_) throw MapError(MapErrorCode::ReadOnly, "Historical backend rejects submap input; continuation is unavailable");
    const LioFrame &lio = submap.lio();
    if (!config_.enabled || !lio.pcd || lio.pcd->empty() || !lio.T_odom_base.matrix().allFinite() || !std::isfinite(lio.timestamp)) {
      return;
    }

    std::size_t bytes = 0;
    if (!boundedInput(submap, LocalGrid{}, &bytes)) throw std::length_error("Submap exceeds bounded backend admission envelope");
    std::unique_lock<std::mutex> lock(input_mutex_);
    database_detail::writableStorageTestPoint("legacy-admission-wait");
    capacity_cv_.wait(lock, [this] {
      return input_frames_.empty() || admission_stopped_ || failed() || fenced_.load(std::memory_order_acquire);
    });
    requireHealthy();
    if (admission_stopped_) throw MapError(MapErrorCode::Lifecycle, "Backend admission stopped");
    if (submap.id() != submap_count_) {
      spdlog::error("[pgo] rejected out-of-order submap: expected={}, received={}", submap_count_, submap.id());
      return;
    }
    input_frames_.emplace_back(std::move(submap), LocalGrid{}, bytes);
    progress_.last_accepted = submap_count_; ++progress_.accepted;
    progress_.high_water = std::max(progress_.high_water, input_frames_.size());
    ++submap_count_;
    input_requested_.store(true, std::memory_order_release);
    if (timer::enabled()) spdlog::info("[backend-profile] admission node={} wait_ms={:.3f}", progress_.last_accepted.value(), elapsedMilliseconds(admission_start));
    lock.unlock();
    wake_cv_.notify_one();
  }

  Eigen::Isometry3d correction() const {
    std::lock_guard<std::recursive_mutex> lifecycle(lifecycle_mutex_);
    requireHealthy();
    if (historical_ && (cancel_requested_.load() || !attached_ || !runtime_current_)) throw MapError(MapErrorCode::CorrectionUnavailable, "Historical map has no active odometry-session correction");
    std::lock_guard<std::mutex> lock(output_mutex_);
    return T_map_odom_;
  }

  std::shared_ptr<const NavigationGrid> latestOccupancyGrid() const {
    std::lock_guard<std::recursive_mutex> lifecycle(lifecycle_mutex_);
    requireHealthy();
    if (historical_) requireHistorical();
    std::lock_guard<std::mutex> lock(occupancy_mutex_);
    if (!occupancy_msg_ && occupancy_) {
      auto map = occupancy_->getMap();
      auto message = std::make_shared<NavigationGrid>();
      message->resolution = map.resolution; message->origin_x = map.originX; message->origin_y = map.originY;
      message->width = map.width; message->height = map.height; message->data = std::move(map.cells);
      message->revision = occupancy_->revision(); message->map_uuid = memory_->mapUuid();
      message->source_graph_revision = memory_->committedRevision();
      occupancy_msg_ = std::move(message);
    }
    return occupancy_msg_;
  }

  std::string mapUuid() const { std::lock_guard<std::recursive_mutex> lock(lifecycle_mutex_); return memory_ ? memory_->mapUuid() : std::string{}; }
  std::uint64_t graphRevision() const { std::lock_guard<std::recursive_mutex> lock(lifecycle_mutex_); return memory_ ? memory_->committedRevision() : 0; }
  ReconstructionDiagnostics diagnostics() const {
    std::lock_guard<std::recursive_mutex> lifecycle(lifecycle_mutex_);
    requireHealthy();
    requireHistorical();
    return diagnostics_;
  }
  std::optional<Eigen::Isometry3f> committedPose(std::uint64_t id) const {
    std::lock_guard<std::recursive_mutex> lifecycle(lifecycle_mutex_);
    requireHealthy();
    requireHistorical();
    return memory_->submapPose(id);
  }
  std::vector<SpatialMatch> spatialCandidates(std::uint64_t id) const {
    std::lock_guard<std::recursive_mutex> lifecycle(lifecycle_mutex_);
    requireHealthy();
    requireHistorical();
    const auto pose = memory_->submapPose(id);
    if (!pose) throw std::out_of_range("Unknown historical submap");
    auto matches = memory_->recallSpatial(id, *pose);
    matches.erase(std::remove_if(matches.begin(), matches.end(), [&](const auto &match) { return !memory_->eligibleLoop(id, match.submap_id, memory_->chainRoot(id)); }),
                  matches.end());
    return matches;
  }
  std::vector<VisualSubmapMatch> transientVisualCandidates(const SubmapFrame &q) {
    std::lock_guard<std::recursive_mutex> lifecycle(lifecycle_mutex_);
    requireHistorical(); return visual_index_->queryTransient(q);
  }
  std::vector<VisualSubmapMatch> visualCandidates(std::uint64_t id) {
    std::lock_guard<std::recursive_mutex> lifecycle(lifecycle_mutex_);
    requireHealthy();
    requireHistorical();
    if (id >= frames_.size()) throw std::out_of_range("Unknown historical submap");
    return visual_index_->query(id);
  }
  mapping::DescriptorArchive::IndexNodeMetadata visualMetadata() const {
    std::lock_guard<std::recursive_mutex> lifecycle(lifecycle_mutex_);
    requireHealthy();
    requireHistorical();
    return visual_index_->historyMetadata();
  }
  OccupancyGrid historicalOccupancy() const {
    std::lock_guard<std::recursive_mutex> lifecycle(lifecycle_mutex_);
    requireHealthy();
    requireHistorical();
    if (!occupancy_) throw MapError(MapErrorCode::UnsupportedMode, "Geometry-only map has no archived navigation grid");
    return *occupancy_;
  }

  void beginContinuation(std::uint64_t generation) {
    std::lock_guard<std::recursive_mutex> lifecycle(lifecycle_mutex_);
    requireHistorical();
    if (!attached_ || continuation_ || generation != producer_generation_ || root_source_sequence_ == UINT64_MAX)
      throw MapError(MapErrorCode::Lifecycle, "Continuation requires the live B1 producer");
    try {
      visual_index_->beginContinuation(active_root_, frames_.size());
      std::lock_guard<std::mutex> lock(input_mutex_);
      if (admission_stopped_) return; // Successful B1 remains ready; stop must not invent a failed handoff.
      continuation_ = true;
      worker_ = std::thread([this] { workerLoop(); });
    } catch (...) { recordFailure(std::current_exception()); throw; }
  }
  static bool boundedInput(const SubmapFrame &q, const LocalGrid &grid, std::size_t *measured = nullptr) {
    // Conservative live payload accounting, including pyramid bucket allocation.
    std::size_t bytes = sizeof(q) + sizeof(grid);
    const auto add = [&](std::size_t n, std::size_t stride) {
      constexpr std::size_t limit = 64 * 1024 * 1024;
      if (!n) return true;
      if (!stride) return false;
      if (n > (limit - std::min(bytes, limit)) / stride) return false;
      bytes += n * stride; return true;
    };
    if (!q.lio().pcd || !add(q.lio().pcd->capacity(), sizeof(GaussianPoint)) ||
        !add(q.odom_poses().capacity(), sizeof(OdomPoses::value_type)) ||
        !add(q.navigation().samples.capacity(), sizeof(TrajectoryPoint)) ||
        !add(q.navigation().spline.control_points.capacity(), sizeof(Eigen::Vector3f)) ||
        !add(q.navigation().spline.knots.capacity(), sizeof(float)) ||
        !add(q.pyramid_voxels().level_buckets.capacity(), sizeof(cpu::VoxelBuckets)) ||
        !add(q.visual_frames().capacity(), sizeof(VisualFrame))) return false;
    for (const auto &b : q.pyramid_voxels().level_buckets) if (!add(b.capacity(), sizeof(b.front()))) return false;
    for (const auto *v : {&grid.groundCells, &grid.obstacleCells, &grid.emptyCells}) if (!add(v->capacity(), sizeof(GridPoint))) return false;
    const auto matrix = [&](const cv::Mat &m) {
      // An ROI retains its entire parent allocation, not just rows*cols bytes.
      return m.datastart && m.datalimit ? add(static_cast<std::size_t>(m.datalimit - m.datastart), 1) : m.empty();
    };
    for (const auto &v : q.visual_frames())
      if (!matrix(v.descriptors()) || !add(v.points().capacity(), sizeof(VisualPoint)) || !add(v.image_png().capacity(), 1)) return false;
    for (const auto &tag : q.tags()) if (tag && !matrix(tag->descriptor())) return false;
    if (measured) *measured = bytes;
    return true;
  }
  ContinuationAdmission submitContinuation(SubmapFrame &query, LocalGrid &grid, std::uint64_t generation, bool wait) {
    std::unique_lock<std::mutex> producer(submitter_mutex_, std::try_to_lock);
    if (!producer.owns_lock()) return ContinuationAdmission::Busy;
    std::size_t bytes = 0;
    if (!boundedInput(query, grid, &bytes)) return ContinuationAdmission::Oversize;
    if (!query.lio().pcd || query.lio().pcd->empty() || !query.bounds().valid() || query.odom_poses().empty() ||
        !query.lio().T_odom_base.matrix().allFinite()) return ContinuationAdmission::Invalid;
    if (!std::isfinite(grid.cellSize) || grid.cellSize <= 0 || !grid.viewPoint.allFinite() ||
        !std::isfinite(query.lio().timestamp) || std::abs(query.lio().timestamp - query.odom_poses().front().timestamp) > 1e-9 ||
        (query.lio().T_odom_base.matrix() - query.odom_poses().front().T_odom_base.matrix()).cwiseAbs().maxCoeff() > 1e-9)
      return ContinuationAdmission::Invalid;
    for (const auto *points : {&grid.groundCells, &grid.obstacleCells, &grid.emptyCells})
      for (const auto &point : *points) if (!point.allFinite()) return ContinuationAdmission::Invalid;
    std::unique_lock<std::mutex> lock(input_mutex_);
    const auto refusal = [&] {
      if (failed()) return ContinuationAdmission::Failed;
      if (admission_stopped_ || cancel_requested_.load()) return ContinuationAdmission::Stopped;
      if (!continuation_ || generation != producer_generation_) return ContinuationAdmission::WrongProducer;
      if (query.id() != expected_source_) return ContinuationAdmission::WrongSequence;
      return ContinuationAdmission::Accepted;
    };
    auto result = refusal();
    if (result != ContinuationAdmission::Accepted) return result;
    if (wait) {
      database_detail::writableStorageTestPoint("b2-capacity-wait");
      capacity_cv_.wait(lock, [&] { return input_frames_.size() < 1 || refusal() != ContinuationAdmission::Accepted; });
    }
    result = refusal();
    if (result != ContinuationAdmission::Accepted) return result;
    if (input_frames_.size() >= 1) return ContinuationAdmission::Full;
    if (expected_source_ == UINT64_MAX) return ContinuationAdmission::WrongSequence;
    // Allocate queue slot before moving caller-owned evidence.
    input_frames_.emplace_back(std::move(query), std::move(grid), bytes);
    progress_.last_accepted = expected_source_++;
    ++progress_.accepted;
    progress_.high_water = std::max(progress_.high_water, input_frames_.size());
    input_requested_.store(true);
    lock.unlock(); wake_cv_.notify_all(); capacity_cv_.notify_all();
    return ContinuationAdmission::Accepted;
  }
  void retryContinuation(std::uint64_t sequence, std::uint64_t generation) {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (failed() || admission_stopped_ || !head_ || progress_.head != ContinuationHead::Retryable ||
        generation != producer_generation_ || head_->source_sequence != sequence)
      throw MapError(MapErrorCode::Lifecycle, "No retryable continuation head for this producer/sequence");
    head_failure_ = nullptr; progress_.head = ContinuationHead::Idle;
    input_requested_.store(true); wake_cv_.notify_all(); capacity_cv_.notify_all();
  }
  ContinuationProgress continuationProgress() const {
    std::lock_guard<std::mutex> lock(input_mutex_);
    auto result = progress_; result.waiting = input_frames_.size();
    result.head_bytes = head_ ? head_->bytes : 0;
    result.waiting_bytes = 0;
    for (const auto &input : input_frames_) result.waiting_bytes += input.bytes;
    return result;
  }
  void stopAdmission(bool abort) {
    { std::lock_guard<std::mutex> lock(input_mutex_);
      admission_stopped_ = true;
      if (abort) cancel_requested_.store(true, std::memory_order_release);
    }
    capacity_cv_.notify_all(); wake_cv_.notify_all();
  }

 private:
  void requireHistorical() const {
    requireHealthy();
    if (!runtime_current_) throw MapError(MapErrorCode::Lifecycle, "Committed storage advanced; reconstruct historical runtime before querying");
    if (!historical_) throw MapError(MapErrorCode::UnsupportedMode, "Historical query requires a read-only historical backend");
  }

  void reconstructHistory(const NaviMapParameters &navi_map) {
    try {
      memory_->visitHistoricalNodes([this](int id, const auto &anchor, const auto &committed, const auto &) {
        frames_.push_back({anchor});
        optimized_.insert(id - 1, toGtsam(committed.template cast<double>()));
      });
      diagnostics_.nodes = frames_.size();
      gtsam::NonlinearFactorGraph graph;
      if (!frames_.empty()) {
        diagnostics_.original_prior = frames_[0].T_odom_base;
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, toGtsam(frames_[0].T_odom_base), prior_noise_));
      }
      for (const auto &link : memory_->historicalLinks()) {
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(link.from_id - 1, link.to_id - 1, toGtsam(link.transform.cast<double>()),
                                                     link.type == 0 ? odom_noise_ : loop_noise_));
        if (link.type == 0)
          ++diagnostics_.odometry_factors;
        else
          ++diagnostics_.loop_factors;
      }
      diagnostics_.factors = graph.size();
      // Local fault/observation seams reuse the existing weak no-op test channel.
      database_detail::writableStorageTestPoint("a2-canonical-graph", &graph);
      database_detail::writableStorageTestPoint("a2-committed-values", &optimized_);
      const auto finiteNorm = [](const Eigen::Vector3d &v) {
        if (!v.allFinite()) throw MapError(MapErrorCode::ReconstructionDiscrepancy, "Nonfinite reconstruction diagnostic vector");
        const double norm = v.norm();
        if (!std::isfinite(norm)) throw MapError(MapErrorCode::ReconstructionDiscrepancy, "Nonfinite reconstruction diagnostic norm");
        return norm;
      };
      const auto validResidual = [](const gtsam::Vector &v) {
        return v.size() == 6 && v.allFinite();
      };
      if (!graph.empty()) {
        auto rebuilt = updateIsam(graph, optimized_).estimate;
        database_detail::writableStorageTestPoint("a2-rebuilt-values", &rebuilt);
        database_detail::writableStorageTestPoint("a2-restored-graph", isam2_.get());
        if (rebuilt.size() != optimized_.size() || isam2_->getFactorsUnsafe().size() != graph.size())
          throw MapError(MapErrorCode::ReconstructionDiscrepancy, "Rebuilt graph identity/count mismatch");
        for (gtsam::Key key : optimized_.keys()) {
          const auto &stored = optimized_.at<gtsam::Pose3>(key), &derived = rebuilt.at<gtsam::Pose3>(key);
          if (!stored.matrix().allFinite() || !derived.matrix().allFinite())
            throw MapError(MapErrorCode::ReconstructionDiscrepancy, "Nonfinite reconstruction pose");
          Eigen::Quaterniond a(stored.rotation().matrix()), b(derived.rotation().matrix());
          const auto normalize = [](Eigen::Quaterniond &q) {
            database_detail::writableStorageTestPoint("a2-quaternion", &q);
            const double norm = q.norm();
            if (!q.coeffs().allFinite() || !std::isfinite(norm) || norm <= 0)
              throw MapError(MapErrorCode::ReconstructionDiscrepancy, "Invalid reconstruction quaternion");
            q.normalize();
            if (!q.coeffs().allFinite())
              throw MapError(MapErrorCode::ReconstructionDiscrepancy, "Nonfinite normalized reconstruction quaternion");
          };
          normalize(a);
          normalize(b);
          Eigen::Quaterniond rotation = a.conjugate() * b;
          database_detail::writableStorageTestPoint("a2-relative-quaternion", &rotation);
          if (!rotation.coeffs().allFinite())
            throw MapError(MapErrorCode::ReconstructionDiscrepancy, "Nonfinite relative reconstruction quaternion");
          const Eigen::Vector3d translation = stored.translation() - derived.translation();
          const double distance = finiteNorm(translation);
          const double angle = 2 * std::atan2(finiteNorm(rotation.vec()), std::abs(rotation.w()));
          if (!std::isfinite(angle)) throw MapError(MapErrorCode::ReconstructionDiscrepancy, "Nonfinite reconstruction angle");
          diagnostics_.max_translation = std::max(diagnostics_.max_translation, distance);
          diagnostics_.max_rotation = std::max(diagnostics_.max_rotation, angle);
        }
        for (std::size_t i = 0; i < graph.size(); ++i) {
          const auto factor = std::dynamic_pointer_cast<gtsam::NoiseModelFactor>(graph[i]);
          const auto restored = std::dynamic_pointer_cast<gtsam::NoiseModelFactor>(isam2_->getFactorsUnsafe()[i]);
          if (!factor || !restored || typeid(*restored) != typeid(*factor) || restored->keys() != factor->keys() ||
              !factor->noiseModel() || !restored->noiseModel() || !restored->noiseModel()->equals(*factor->noiseModel(), 1e-15))
            throw MapError(MapErrorCode::ReconstructionDiscrepancy, "Rebuilt factor/model mismatch");
          // GTSAM Huber::equals compares k but omits Scalar/Block reweighting.
          const auto canonical_robust = std::dynamic_pointer_cast<gtsam::noiseModel::Robust>(factor->noiseModel());
          const auto restored_robust = std::dynamic_pointer_cast<gtsam::noiseModel::Robust>(restored->noiseModel());
          if (bool(canonical_robust) != bool(restored_robust) ||
              (canonical_robust && canonical_robust->robust()->reweightScheme() != restored_robust->robust()->reweightScheme()))
            throw MapError(MapErrorCode::ReconstructionDiscrepancy, "Rebuilt robust reweighting mismatch");
          gtsam::Vector before = factor->unwhitenedError(optimized_), after = factor->unwhitenedError(rebuilt);
          gtsam::Vector same_values = restored->unwhitenedError(optimized_);
          database_detail::writableStorageTestPoint("a2-residual-before", &before);
          database_detail::writableStorageTestPoint("a2-residual-after", &after);
          database_detail::writableStorageTestPoint("a2-residual-restored", &same_values);
          if (!validResidual(before) || !validResidual(after) || !validResidual(same_values) ||
              !(same_values.array() == before.array()).all())
            throw MapError(MapErrorCode::ReconstructionDiscrepancy, "Rebuilt factor/residual mismatch");
          const gtsam::Vector difference = after - before;
          if (!validResidual(difference))
            throw MapError(MapErrorCode::ReconstructionDiscrepancy, "Nonfinite reconstruction residual difference");
          const Eigen::Vector3d rotation = difference.head<3>(), translation = difference.tail<3>();
          const double angle = finiteNorm(rotation), distance = finiteNorm(translation);
          diagnostics_.max_residual_rotation = std::max(diagnostics_.max_residual_rotation, angle);
          diagnostics_.max_residual_translation = std::max(diagnostics_.max_residual_translation, distance);
        }
        diagnostics_.objective_committed = graph.error(optimized_);
        diagnostics_.objective_rebuilt = graph.error(rebuilt);
        database_detail::writableStorageTestPoint("a2-objectives", &diagnostics_);
        diagnostics_.objective_delta = diagnostics_.objective_rebuilt - diagnostics_.objective_committed;
        // Objective change is diagnostic, not a monotonicity acceptance gate.
      }
      database_detail::writableStorageTestPoint("a2-diagnostics", &diagnostics_);
      // Finite arithmetic is a hard requirement, independent of reproduction similarity.
      if (!std::isfinite(diagnostics_.objective_committed) || !std::isfinite(diagnostics_.objective_rebuilt) ||
          !std::isfinite(diagnostics_.objective_delta) || !std::isfinite(diagnostics_.max_translation) ||
          !std::isfinite(diagnostics_.max_rotation) || !std::isfinite(diagnostics_.max_residual_translation) ||
          !std::isfinite(diagnostics_.max_residual_rotation))
        throw MapError(MapErrorCode::ReconstructionDiscrepancy, "Nonfinite historical reconstruction diagnostic/objective");
      spdlog::info(
          "[pgo] historical reconstruction nodes={} factors={} translation={} rotation={} residual_translation={} residual_rotation={} "
          "objective_delta={}",
          diagnostics_.nodes, diagnostics_.factors, diagnostics_.max_translation, diagnostics_.max_rotation, diagnostics_.max_residual_translation,
          diagnostics_.max_residual_rotation, diagnostics_.objective_delta);
      const bool translation_exceeded = diagnostics_.max_translation > .035, rotation_exceeded = diagnostics_.max_rotation > .003;
      const bool residual_translation_exceeded = diagnostics_.max_residual_translation > .026,
                 residual_rotation_exceeded = diagnostics_.max_residual_rotation > .003;
      if (translation_exceeded || rotation_exceeded || residual_translation_exceeded || residual_rotation_exceeded)
        spdlog::warn(
            "[pgo] reconstruction reproduction discrepancy: opening map UUID={} revision={} classification=exceeds_reference "
            "translation={} m reference=0.035 m exceeded={} rotation={} rad reference=0.003 rad exceeded={} "
            "residual_translation={} m reference=0.026 m exceeded={} residual_rotation={} rad reference=0.003 rad exceeded={} "
            "nodes={} factors={} objective_delta={}; similarity exceedance alone does not reject opening",
            memory_->mapUuid(), memory_->committedRevision(), diagnostics_.max_translation, translation_exceeded,
            diagnostics_.max_rotation, rotation_exceeded, diagnostics_.max_residual_translation, residual_translation_exceeded,
            diagnostics_.max_residual_rotation, residual_rotation_exceeded, diagnostics_.nodes, diagnostics_.factors, diagnostics_.objective_delta);
      materializeCommittedHistory(false);
      progress_.ready_revision = memory_->committedRevision();
      progress_.committed_revision = progress_.ready_revision; progress_.ready_available = true;
      // searched_loop_id_ stays inactive: the current format cannot establish completion.
      memory_->validateReadIdentity();
    } catch (const MapError &) {
      throw;
    } catch (const std::bad_alloc &) {
      throw MapError(MapErrorCode::Storage, "Insufficient memory for historical reconstruction");
    } catch (const std::exception &error) {
      throw MapError(MapErrorCode::ReconstructionDiscrepancy, error.what());
    }
  }
  // Reuse A2's authoritative-data materialization; no optimizer reconstruction,
  // ground estimation, raycasting, new transaction or publication callback.
  void materializeCommittedHistory(bool attachment) {
    database_detail::writableStorageTestPoint("runtime-full-rebuild");
    memory_->rebuildCommittedIndexes();
    visual_index_ = std::make_unique<VisualSubmapIndex>(config_.visual, *memory_);
    if (attachment) visual_index_->materializeCommittedRoot(frames_.size());
    else visual_index_->restoreHistory(frames_.size());
    if (navi_map_.enabled) {
      occupancy_ = std::make_unique<OccupancyGrid>(navi_map_);
      occupancy_resolution_ = navi_map_.resolution;
      for (std::size_t i = 0; i < frames_.size(); ++i) {
        LocalGrid grid;
        if (!memory_->loadLocalGrid(i, grid) || !occupancy_->append(GridFrame(int(i + 1), *memory_->submapPose(i), grid)))
          throw MapError(MapErrorCode::ReconstructionDiscrepancy, "Cannot append persisted historical occupancy evidence");
      }
    }
    occupancy_msg_.reset();  // Lazy output: no dense publication allocation in map materialization.
    submap_count_ = added_odom_id_ = occupancy_inserted_id_ = frames_.size();
    occupancy_revision_ = occupancy_ ? occupancy_->revision() : 0;
  }
  struct InputFrame {
    std::uint64_t source_sequence;
    std::size_t bytes = 0;
    SubmapFrame submap;
    std::optional<LocalGrid> grid;
    InputFrame(SubmapFrame &&q) noexcept : source_sequence(q.id()), submap(std::move(q)) {}
    InputFrame(SubmapFrame &&q, LocalGrid &&g, std::size_t size) noexcept
        : source_sequence(q.id()), bytes(size), submap(std::move(q)), grid(std::move(g)) {}
  };

  struct GraphFrame {
    Eigen::Isometry3d T_odom_base = Eigen::Isometry3d::Identity();
  };

  struct IsamOutput {
    gtsam::Values estimate;
    gtsam::KeySet affected_keys;
  };

  void cancelAccepted(const char *reason) {
    // input_mutex_ held; a contiguous bounded tail replaces per-node status storage.
    if (head_ || !input_frames_.empty()) {
      progress_.canceled_first = head_ ? head_->source_sequence : input_frames_.front().source_sequence;
      progress_.canceled_last = input_frames_.empty() ? head_->source_sequence : input_frames_.back().source_sequence;
      progress_.cancellation_reason = reason;
      head_.reset(); input_frames_.clear();
    }
  }
  void continuationLoop() {
    for (;;) {
      {
        std::unique_lock<std::mutex> lock(input_mutex_);
        capacity_cv_.wait(lock, [&] {
          return failed() || cancel_requested_.load() || stop_.load() ||
                 (progress_.head != ContinuationHead::Retryable && (head_ || !input_frames_.empty()));
        });
        if (failed() || cancel_requested_.load()) {
          cancelAccepted(failed() ? "backend failed" : "producer reset/abort");
          progress_.head = failed() ? ContinuationHead::Failed : ContinuationHead::Idle;
          break;
        }
        if (stop_.load() && progress_.head == ContinuationHead::Retryable) {
          const auto cause = head_failure_;
          cancelAccepted("finish with retryable head awaiting explicit action");
          progress_.head = ContinuationHead::Failed;
          lock.unlock();
          recordFailure(cause);
          break;
        }
        if (!head_ && input_frames_.empty()) { if (stop_.load()) break; else continue; }
        if (!head_) { head_.emplace(std::move(input_frames_.front())); input_frames_.pop_front(); }
        progress_.head = ContinuationHead::Processing; progress_.head_sequence = head_->source_sequence;
      }
      capacity_cv_.notify_all();
      processContinuationHead();
      bool completed = false;
      { std::lock_guard<std::mutex> input(input_mutex_); completed = !head_ && progress_.head == ContinuationHead::Idle; }
      if (completed && !failed()) notifyReady();
    }
    capacity_cv_.notify_all();
  }
  void processContinuationHead() {
    std::lock_guard<std::recursive_mutex> lifecycle_guard(lifecycle_mutex_);
    bool mutation = false, committed = false;
    const auto started = std::chrono::steady_clock::now();
    double candidate_ms = 0, bbs_ms = 0, gicp_ms = 0, preparation_ms = 0, solver_ms = 0, reconciliation_ms = 0, writer_ms = 0, poses_ms = 0, visual_ms = 0, occupancy_ms = 0;
    try {
      std::lock_guard<std::recursive_mutex> lifecycle(lifecycle_mutex_);
      requireHistorical();
      const auto sequence = head_->source_sequence;
      const auto q = frames_.size();
      if (!attached_ || !q || q >= std::size_t(std::numeric_limits<int>::max()) ||
          memory_->chainRoot(q - 1) != active_root_ ||
          sequence != root_source_sequence_ + progress_.completed + 1 || !head_->grid ||
          memory_->committedRevision() >= std::uint64_t(std::numeric_limits<std::int64_t>::max()))
        throw MapError(MapErrorCode::Lifecycle, "Invalid live continuation predecessor/sequence/revision");
      const auto uuid = memory_->mapUuid();
      const auto revision = memory_->committedRevision();
      auto &query = head_->submap;
      query = std::move(query).withPersistentId(q);
      const auto anchor = query.lio().T_odom_base;
      const auto rigid = [](const Eigen::Isometry3d &p) {
        return p.matrix().allFinite() && p.matrix().row(3) == Eigen::RowVector4d(0,0,0,1) &&
               (p.linear().transpose() * p.linear() - Eigen::Matrix3d::Identity()).norm() <= 1e-6 &&
               std::abs(p.linear().determinant() - 1) <= 1e-6;
      };
      if (!rigid(anchor) || (anchor.matrix() - query.odom_poses().front().T_odom_base.matrix()).cwiseAbs().maxCoeff() > 1e-9 ||
          !std::isfinite(query.lio().timestamp) || std::abs(query.lio().timestamp - query.odom_poses().front().timestamp) > 1e-9)
        throw MapError(MapErrorCode::HistoricalData, "Continuation anchor disagrees with original odometry");
      const Eigen::Isometry3d relative = frames_.back().T_odom_base.inverse() * anchor;
      const GraphLink base{static_cast<int>(q), static_cast<int>(q + 1), 0, toIsometry3f(relative)};
      const Eigen::Isometry3d seed = memory_->submapPose(q - 1)->cast<double>() * relative;
      if (!rigid(seed) || !base.transform.matrix().allFinite()) throw MapError(MapErrorCode::HistoricalData, "Invalid continuation seed/odometry");
      database_detail::writableStorageTestPoint("b2-before-preparation");
      const auto scene = buildVisualScene(query)->encode();
      const auto visual = config_.visual.loopEnabled() ? visual_index_->queryTransient(query) : std::vector<VisualSubmapMatch>{};
      const auto candidate_start = std::chrono::steady_clock::now();
      auto batch = prepareLoopCandidates(*memory_, query, seed, visual);
      candidate_ms = elapsedMilliseconds(candidate_start);
      const auto loop = verifyLoop(batch, q, &bbs_ms, &gicp_ms);
      gtsam::NonlinearFactorGraph graph;
      graph.add(gtsam::BetweenFactor<gtsam::Pose3>(q - 1, q, toGtsam(base.transform.cast<double>()), odom_noise_));
      if (loop) graph.add(gtsam::BetweenFactor<gtsam::Pose3>(q, loop->to_id - 1, toGtsam(loop->transform.cast<double>()), loop_noise_));
      gtsam::Values initial;
      initial.insert(q, toGtsam(seed));
      frames_.reserve(q + 1); // capacity only; committed membership/anchors stay unchanged
      database_detail::writableStorageTestPoint("b2-before-solver");
      preparation_ms = elapsedMilliseconds(started);
      auto stage = std::chrono::steady_clock::now();
      mutation = true; // BEFORE the very first potentially mutating update entry.
      auto proposed = updateIsam(graph, initial).estimate;
      solver_ms = elapsedMilliseconds(stage); stage = std::chrono::steady_clock::now();
      database_detail::writableStorageTestPoint("b2-after-solver", &proposed);
      if (proposed.size() != q + 1 || !std::isfinite(isam2_->getFactorsUnsafe().error(proposed)))
        throw MapError(MapErrorCode::ReconstructionDiscrepancy, "Invalid tentative continuation estimate");
      std::vector<std::pair<int, Eigen::Isometry3f>> delta;
      for (std::size_t key = 0; key <= q; ++key) {
        const auto estimate = fromGtsam(proposed.at<gtsam::Pose3>(key));
        if (!rigid(estimate)) throw MapError(MapErrorCode::ReconstructionDiscrepancy, "Invalid continuation estimate pose");
        const auto serialized = toIsometry3f(estimate);
        if (!rigid(serialized.cast<double>())) throw MapError(MapErrorCode::ReconstructionDiscrepancy, "Unrepresentable continuation pose");
        const auto old = memory_->submapPose(key);
        if (key != q && !old) throw MapError(MapErrorCode::Lifecycle, "Missing committed comparison pose");
        if (key == q || std::memcmp(old->matrix().data(), serialized.matrix().data(), 16 * sizeof(float)) != 0)
          delta.emplace_back(key + 1, serialized);
      }
      const Eigen::Isometry3d correction = delta.back().second.cast<double>() * anchor.inverse();
      if (!correction.matrix().allFinite()) throw MapError(MapErrorCode::HistoricalData, "Invalid continuation correction");
      reconciliation_ms = elapsedMilliseconds(stage); stage = std::chrono::steady_clock::now();
      database_detail::writableStorageTestPoint("b2-before-w");
      auto new_revision = memory_->commitFinalizedSubmap(query, *head_->grid, scene, delta, base, loop, uuid, revision,
                                                              q + 1, map_config_identity(config_, navi_map_), active_root_ + 1);
      committed = true; observeCommit(new_revision); runtime_current_ = false;
      if(loop) refreshScene(*loop);
      new_revision=memory_->committedRevision();
      writer_ms = elapsedMilliseconds(stage); stage = std::chrono::steady_clock::now();
      database_detail::writableStorageTestPoint("b2-postcommit-materialization");
      memory_->materializeCommittedDelta(query, delta);
      for (const auto &[sql, pose] : delta) {
        if (sql == int(q + 1)) optimized_.insert(q, toGtsam(pose.cast<double>()));
        else optimized_.update(sql - 1, toGtsam(pose.cast<double>()));
      }
      frames_.push_back({anchor});
      poses_ms = elapsedMilliseconds(stage); stage = std::chrono::steady_clock::now();
      database_detail::writableStorageTestPoint("b2-before-visual");
      visual_index_->advanceContinuation(q + 1);
      visual_ms = elapsedMilliseconds(stage); stage = std::chrono::steady_clock::now();
      database_detail::writableStorageTestPoint("b2-before-occupancy");
      if (occupancy_) {
        auto update = occupancy_->update([this](int sql, LocalGrid &grid) { return sql > 0 && memory_->loadLocalGrid(sql - 1, grid); });
        bool moved = false;
        for (const auto &[sql, pose] : delta) if (sql != int(q + 1)) {
          LocalGrid grid;
          if (!occupancy_->containsNode(sql)) throw MapError(MapErrorCode::Lifecycle, "Missing moved occupancy owner");
          if (!memory_->loadLocalGrid(sql - 1, grid)) throw MapError(MapErrorCode::HistoricalData, "Missing moved-owner LocalGrid");
          update -= sql; update += GridFrame(sql, pose, grid); moved = true;
        }
        if (moved && !update.commit()) throw MapError(MapErrorCode::Lifecycle, "Occupancy replacement failed");
        if (!occupancy_->append(GridFrame(q + 1, delta.back().second, *head_->grid)))
          throw MapError(MapErrorCode::Lifecycle, "Occupancy append failed");
      }
      occupancy_ms = elapsedMilliseconds(stage);
      occupancy_msg_.reset();
      database_detail::writableStorageTestPoint("b2-runtime-ready");
      {
        std::lock_guard<std::mutex> output(output_mutex_);
        std::lock_guard<std::mutex> input(input_mutex_);
        // Reset's atomic fence also gates every correction accessor.
        T_map_odom_ = correction;
        progress_.ready_revision = new_revision; progress_.ready_available = true;
        progress_.completed_revision = new_revision; ready_timestamp_ = query.lio().timestamp;
        progress_.last_commit_outcome = CommitOutcome::Committed;
        runtime_current_ = true;
        progress_.last_completed = sequence; ++progress_.completed;
        progress_.head = ContinuationHead::Idle; progress_.head_sequence.reset(); head_.reset(); head_failure_ = nullptr;
      }
      database_detail::writableStorageTestPoint("b2-completed");
      spdlog::info("[b2] nodes={} loop={} delta={} full_rebuilds=0 head_ms={} prepare_ms={} solver_ms={} reconcile_ms={} W_ms={} poses_ms={} visual_ms={} occupancy_ms={} candidates_ms={} BBS_ms={} GICP_ms={}",
                   q + 1, bool(loop), delta.size(), elapsedMilliseconds(started), preparation_ms, solver_ms, reconciliation_ms,
                   writer_ms, poses_ms, visual_ms, occupancy_ms, candidate_ms, bbs_ms, gicp_ms);
    } catch (...) {
      const auto cause = std::current_exception();
      if (mutation) {
        std::lock_guard<std::recursive_mutex> lifecycle(lifecycle_mutex_);
        CommitOutcome outcome = committed ? CommitOutcome::Committed : CommitOutcome::NotCommitted;
        try { std::rethrow_exception(cause); } catch (const MapError &e) { outcome = committed && e.outcome()!=CommitOutcome::Unknown ? CommitOutcome::Committed : e.outcome(); } catch (...) {}
        if (outcome == CommitOutcome::Committed) observeCommit(memory_->committedRevision());
        { std::lock_guard<std::mutex> input(input_mutex_); progress_.last_commit_outcome = outcome; if (outcome == CommitOutcome::Unknown) progress_.committed_outcome_known = false; }
        runtime_current_ = false;
        recordFailure(cause);
      } else {
        std::lock_guard<std::mutex> input(input_mutex_);
        head_failure_ = cause; progress_.head = ContinuationHead::Retryable;
        database_detail::writableStorageTestPoint("b2-retryable");
      }
    }
    capacity_cv_.notify_all();
  }

  void workerLoop() noexcept {
    try {
      if (continuation_) {
        database_detail::writableStorageTestPoint("b2-worker-entry");
        continuationLoop();
      } else legacyWorkerLoop();
    } catch (...) {
      recordFailure(std::current_exception());
      if (continuation_) {
        try {
          std::lock_guard<std::mutex> input(input_mutex_);
          progress_.head = ContinuationHead::Failed;
          cancelAccepted("conservative worker guard");
        } catch (...) { /* first cause and any already recorded canceled range remain retained */ }
      }
      capacity_cv_.notify_all();
    }
    // Notification runs after all backend locks are released, never joins/replaces an owner.
    if (failed() && failure_notification_) {
      try { failure_notification_(); } catch (...) { /* original cause remains retained */ }
    }
  }
  void legacyWorkerLoop() {
    const auto period = std::chrono::duration<double>(config_.update_period_sec);
    while (true) {
      std::unique_lock<std::mutex> lock(wake_mutex_);
      wake_cv_.wait_for(lock, period, [this]() { return stop_.load(std::memory_order_acquire) || input_requested_.load(std::memory_order_acquire); });
      const bool stopping = stop_.load(std::memory_order_acquire);
      lock.unlock();
      try {
        if (!failed() && processPending()) notifyReady();
      } catch (const std::exception &error) {
        recordFailure(std::current_exception());
        spdlog::error("[pgo] backend update failed: {}", error.what());
      }
      if (stopping || failed()) {
        break;
      }
    }
  }

  void copyPendingFrames() {
    std::deque<InputFrame> pending;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      if (!input_frames_.empty()) { progress_.head = ContinuationHead::Processing; progress_.head_sequence = input_frames_.front().source_sequence; }
      pending.swap(input_frames_);
    }
    capacity_cv_.notify_all();
    frames_.reserve(frames_.size() + pending.size());
    size_t index = 0;
    try {
      for (; index < pending.size(); ++index) {
        InputFrame &input = pending[index];
        SubmapFrame &submap = input.submap;
        const LioFrame &lio = submap.lio();
        const size_t frame_id = static_cast<size_t>(submap.id());
        if (frame_id != frames_.size()) {
          throw std::logic_error("non-contiguous submap sequence");
        }
        GraphFrame frame{lio.T_odom_base};
        LocalGrid grid(static_cast<float>(occupancy_resolution_));
        if (local_grid_maker_) {
          local_grid_maker_->createLocalMap(*lio.pcd, toIsometry3f(lio.T_odom_base), grid, ground_estimator_->update(submap));
        }
        const auto scene = visual_index_ ? buildVisualScene(submap) : nullptr;
        memory_->saveSubmap(submap, grid, scene);
        observeCommit(memory_->committedRevision());
        if (visual_index_) {
          database_detail::writableStorageTestPoint("b3-legacy-visual-insert");
          visual_index_->addSubmap(submap);
        }
        ready_timestamp_ = lio.timestamp;
        frames_.push_back(std::move(frame));
      }
    } catch (...) {
      std::lock_guard<std::mutex> lock(input_mutex_);
      input_frames_.insert(input_frames_.begin(), std::make_move_iterator(pending.begin() + index), std::make_move_iterator(pending.end()));
      input_requested_.store(true, std::memory_order_release);
      throw;
    }
  }

  bool processPending() {
    std::lock_guard<std::recursive_mutex> lifecycle(lifecycle_mutex_);
    try {
    const auto profile_start = std::chrono::steady_clock::now();
    const auto previous_size = frames_.size();
    input_requested_.store(false, std::memory_order_release);
    copyPendingFrames();
    if (frames_.empty() || frames_.size() == previous_size) {
      return false;
    }
    const double copy_ms = elapsedMilliseconds(profile_start);
    auto profile_stage = std::chrono::steady_clock::now();

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
    }
    if (!pending_changed_keys_.empty()) {
      saveSubmapPoses(pending_changed_keys_);
      updateCorrection();
    }

    const double odom_ms = elapsedMilliseconds(profile_stage);
    profile_stage = std::chrono::steady_clock::now();
    gtsam::NonlinearFactorGraph loop_graph;
    const bool searchedNewFrames = searched_loop_id_ < frames_.size() && !optimized_.empty();
    const bool has_loops = buildLoopEdges(loop_graph);
    const double search_ms = elapsedMilliseconds(profile_stage);
    profile_stage = std::chrono::steady_clock::now();
    if (has_loops) {
      IsamOutput output = updateIsam(loop_graph, {});
      const gtsam::KeySet changed = changedFinalPoses(optimized_, output.estimate, output.affected_keys);
      pending_changed_keys_.insert(changed.begin(), changed.end());
      optimized_ = std::move(output.estimate);
      saveSubmapPoses(changed);
      updateCorrection();
    }
    if (searchedNewFrames) {
      finalizeLoopSearch();
    }

    const double optimize_ms = elapsedMilliseconds(profile_stage);
    profile_stage = std::chrono::steady_clock::now();
    bool occupancy_updated = false;
    if (!pending_changed_keys_.empty()) {
      if (occupancy_) {
        occupancy_updated = updateOccupancyFrames(pending_changed_keys_);
      }
      pending_changed_keys_.clear();
    }
    (void)occupancy_updated;
    const double occupancy_ms = elapsedMilliseconds(profile_stage);
    profile_stage = std::chrono::steady_clock::now();
    if (visual_index_) {
      database_detail::writableStorageTestPoint("b3-legacy-visual-settle");
      visual_index_->settle();
    }
    occupancy_msg_.reset();
    updateCorrection();
    const auto revision = memory_->committedRevision();
    { std::lock_guard<std::mutex> input(input_mutex_);
      progress_.committed_revision = revision; progress_.ready_revision = revision;
      progress_.ready_available = true; progress_.completed_revision = revision;
      progress_.last_completed = frames_.size() - 1; progress_.completed = frames_.size();
      progress_.head = ContinuationHead::Idle; progress_.head_sequence.reset();
    }
    database_detail::writableStorageTestPoint("b3-legacy-ready");
    if (timer::enabled()) spdlog::info(
        "[backend-profile] batch first={} count={} history={} copy_ms={:.3f} odom_ms={:.3f} search_ms={:.3f} optimize_ms={:.3f} occupancy_ms={:.3f} settle_ready_ms={:.3f} total_ms={:.3f}",
        previous_size, frames_.size() - previous_size, frames_.size(), copy_ms, odom_ms, search_ms, optimize_ms, occupancy_ms,
        elapsedMilliseconds(profile_stage), elapsedMilliseconds(profile_start));
    return true;
    } catch (...) {
      { std::lock_guard<std::mutex> input(input_mutex_);
        progress_.head = ContinuationHead::Failed;
        if (progress_.accepted > progress_.completed) {
          progress_.canceled_first = progress_.completed; progress_.canceled_last = progress_.last_accepted;
          progress_.cancellation_reason = "legacy processing failed";
        }
      }
      // Failure must be fenced before releasing lifecycle, including visual uncertainty.
      observeCommit(memory_->committedRevision());
      // A scene-only COMMIT may be uncertain after an earlier graph commit was
      // known. Preserve the last known revision without claiming it is latest.
      try { throw; } catch(const MapError &error) {
        if(error.outcome()==CommitOutcome::Unknown) {
          std::lock_guard<std::mutex> input(input_mutex_);
          progress_.last_commit_outcome=CommitOutcome::Unknown;
          progress_.committed_outcome_known=false;
        }
      } catch(...) {}
      recordFailure(std::current_exception()); throw;
    }
  }

  bool buildOdometryGraph(gtsam::NonlinearFactorGraph &graph, gtsam::Values &initial) {
    if (added_odom_id_ >= frames_.size()) {
      return false;
    }

    size_t first_new_id = added_odom_id_;
    if (added_odom_id_ == 0) {
      const gtsam::Pose3 first = toGtsam(frames_.front().T_odom_base);
      graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, first, prior_noise_));
      initial.insert(0, first);
      first_new_id = 1;
    }

    const size_t last_optimized_id = optimized_.empty() ? 0 : optimized_.size() - 1;
    const Eigen::Isometry3d last_optimized =
        optimized_.empty() ? frames_.front().T_odom_base : fromGtsam(optimized_.at<gtsam::Pose3>(last_optimized_id));

    for (size_t i = first_new_id; i < frames_.size(); ++i) {
      const Eigen::Isometry3d relative = frames_[i - 1].T_odom_base.inverse() * frames_[i].T_odom_base;
      graph.add(gtsam::BetweenFactor<gtsam::Pose3>(i - 1, i, toGtsam(relative), odom_noise_));
      pending_links_.push_back({static_cast<int>(i), static_cast<int>(i + 1), 0, toIsometry3f(relative)});
      const Eigen::Isometry3d estimate = last_optimized * frames_[last_optimized_id].T_odom_base.inverse() * frames_[i].T_odom_base;
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
      const Eigen::Isometry3d query_pose = last_optimized * frames_[last_optimized_id].T_odom_base.inverse() * frames_[query_id].T_odom_base;
      std::vector<VisualSubmapMatch> visual;
      if (visual_index_) {
        database_detail::writableStorageTestPoint("b3-legacy-visual-query");
        visual = visual_index_->query(query_id);
      }
      const double visual_ms = elapsedMilliseconds(query_start);
      const auto prepare_start = std::chrono::steady_clock::now();
      LoopCandidateBatch batch = prepareLoopCandidates(*memory_, query_id, query_pose, visual);
      const double prepare_ms = elapsedMilliseconds(prepare_start);
      const auto verify_start = std::chrono::steady_clock::now();
      double bbs_ms = 0, gicp_ms = 0;
      const auto loop = verifyLoop(batch, query_id, &bbs_ms, &gicp_ms);
      if (timer::enabled()) spdlog::info(
          "[backend-profile] loop query={} candidates={} accepted={} visual_ms={:.3f} prepare_ms={:.3f} verify_ms={:.3f} bbs_ms={:.3f} gicp_ms={:.3f}",
          query_id, batch.candidates.size(), bool(loop), visual_ms, prepare_ms, elapsedMilliseconds(verify_start), bbs_ms, gicp_ms);
      if (loop) {
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(query_id, loop->to_id - 1, toGtsam(loop->transform.cast<double>()), loop_noise_));
        pending_links_.push_back(*loop);
      }
      if (visual_index_) visual_index_->finishQuery(query_id);

    }
    return !graph.empty();
  }

  std::optional<GraphLink> verifyLoop(LoopCandidateBatch &batch, std::size_t query_id, double *bbs_ms = nullptr, double *gicp_ms = nullptr) {
    for (auto &candidate : batch.candidates) {
      const auto target_cloud = memory_->loadCloud(candidate.target_id);
      if (!target_cloud || target_cloud->empty() || !batch.query_cloud) continue;
      Eigen::Isometry3d initial;
      if (candidate.metric_visual) {
        const auto scoring_start = std::chrono::steady_clock::now();
        auto score = scoreLoopSeed(*batch.query_cloud, candidate.target_voxelmaps, candidate.target_T_query_initial);
        database_detail::writableStorageTestPoint("visual-seed-geometry", &score);
        spdlog::info("[visual-seed-geometry] query={} target={} overlap={} available={} ms={}",
                    query_id, candidate.target_id, score.overlap, score.available, elapsedMilliseconds(scoring_start));
        if (!acceptsLoopSeedScore(score)) continue;
        initial = candidate.target_T_query_initial.cast<double>();
      } else {
        gpu::LocalSearchTarget target{candidate.target_id, &candidate.target_voxelmaps, candidate.target_T_query_initial};
        std::optional<gpu::LocalSearchOptions> options;
        if (candidate.visual) {
          options.emplace();
          options->translation_half_window = Eigen::Vector3f(config_.visual.global_xy_window, config_.visual.global_xy_window, config_.visual.global_z_window);
          options->minimum_overlap = 0.5;
        }
        const auto stage = std::chrono::steady_clock::now();
        auto coarse = alignLoopBbs(*batch.query_cloud, {target}, options);
        if (bbs_ms) *bbs_ms += elapsedMilliseconds(stage);
        database_detail::writableStorageTestPoint("b2-after-bbs", &coarse);
        if (!coarse.accepted) continue;
        initial = coarse.T_target_query;
      }
      const auto stage = std::chrono::steady_clock::now();
      auto refined = refineLoopGicp(batch.query_cloud, target_cloud, initial);
      if (gicp_ms) *gicp_ms += elapsedMilliseconds(stage);
      database_detail::writableStorageTestPoint("b2-after-gicp", &refined);
      if (!refined.accepted) continue;
      return GraphLink{static_cast<int>(query_id + 1), static_cast<int>(candidate.target_id + 1), 1,
                       toIsometry3f(refined.T_target_query.inverse())};
    }
    return std::nullopt;
  }

  void finalizeLoopSearch() { searched_loop_id_ = frames_.size(); }

  IsamOutput updateIsam(const gtsam::NonlinearFactorGraph &graph, const gtsam::Values &initial) {
    database_detail::writableStorageTestPoint("isam-update-entry", isam2_.get());
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
    const Eigen::Isometry3d optimized_pose = memory_->submapPose(last_id)->cast<double>();
    const Eigen::Isometry3d correction = optimized_pose * frames_[last_id].T_odom_base.inverse();

    std::lock_guard<std::mutex> lock(output_mutex_);
    T_map_odom_ = correction;
  }

  void refreshScene(const GraphLink &loop) {
    if(!config_.scene_refresh || loop.type!=1) return;
    if(const auto revision=memory_->refreshCoveredScene(loop.to_id,loop.from_id)) {
      observeCommit(*revision);
      if(visual_index_) visual_index_->retireScene(loop.to_id-1);
      spdlog::info("[scene-refresh] retired_sql={} replacement_sql={} revision={}",loop.to_id,loop.from_id,*revision);
    }
  }

  void saveSubmapPoses(const gtsam::KeySet &changedKeys) {
    std::vector<std::pair<int, Eigen::Isometry3f>> poses;
    poses.reserve(changedKeys.size());
    for (const gtsam::Key key : changedKeys) {
      if (!optimized_.exists(key)) {
        continue;
      }
      poses.emplace_back(static_cast<int>(key + 1), toIsometry3f(fromGtsam(optimized_.at<gtsam::Pose3>(key))));
    }
    memory_->saveSubmapPoses(poses, pending_links_);
    observeCommit(memory_->committedRevision());
    for(const auto &link:pending_links_) refreshScene(link);
    pending_links_.clear();
  }

  bool updateOccupancyFrames(const gtsam::KeySet &changedKeys) {
    if (!occupancy_ || optimized_.empty()) {
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

  PoseGraphParameters config_;
  bool historical_ = false;
  bool attached_ = false;  // lifecycle_mutex_; never persisted or restored.
  ReconstructionDiagnostics diagnostics_;
  mutable std::recursive_mutex lifecycle_mutex_;
  std::mutex finish_mutex_;
  bool drained_ = false;
  mutable std::mutex failure_mutex_;
  std::exception_ptr failure_;
  std::atomic_bool failed_{false}, fenced_{false};
  bool runtime_current_ = true;
  NaviMapParameters navi_map_;
  std::vector<GraphLink> pending_links_;

  mutable std::mutex input_mutex_;
  std::deque<InputFrame> input_frames_;
  std::optional<InputFrame> head_;
  std::exception_ptr head_failure_;
  std::mutex submitter_mutex_;
  std::condition_variable capacity_cv_;
  bool continuation_ = false, admission_stopped_ = false;
  std::atomic_bool cancel_requested_{false};
  std::uint64_t active_root_ = 0, producer_generation_ = 0, root_source_sequence_ = 0, expected_source_ = 0;
  ContinuationProgress progress_;
  size_t submap_count_ = 0;
  std::vector<GraphFrame> frames_;
  std::unique_ptr<Memory> memory_;
  std::unique_ptr<VisualSubmapIndex> visual_index_;

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
  std::unique_ptr<GroundEstimator> ground_estimator_;
  std::unique_ptr<OccupancyGrid> occupancy_;
  size_t occupancy_inserted_id_ = 0;
  double occupancy_resolution_ = 0.1;
  size_t occupancy_revision_ = 0;
  mutable std::mutex occupancy_mutex_;
  mutable std::shared_ptr<const NavigationGrid> occupancy_msg_;
  NavigationGridCallback navigation_grid_callback_;
  std::function<void()> failure_notification_, ready_notification_;
  double ready_timestamp_ = 0;
  std::atomic<bool> input_requested_{false};

  std::atomic<bool> stop_{false};
  std::mutex wake_mutex_;
  std::condition_variable wake_cv_;
  std::thread worker_;
};

PoseGraphBackend::PoseGraphBackend(const PoseGraphParameters &config, const NaviMapParameters &navi_map, const std::string &database_path,
                                   NavigationGridCallback navigation_grid_callback, std::function<void()> failure_notification, std::function<void()> ready_notification)
    : impl_(std::make_unique<Impl>(config, navi_map, database_path, std::move(navigation_grid_callback), std::move(failure_notification), std::move(ready_notification))) {}

PoseGraphBackend::~PoseGraphBackend() = default;

AttachmentResult PoseGraphBackend::attachFreshSession(SubmapFrame query, const LocalGrid &grid, int target_node_id,
                                                         const Eigen::Isometry3d &seed, std::uint64_t generation) {
  auto result = impl_->attachFreshSession(query, grid, target_node_id, seed, generation);
  if (result.status == AttachmentStatus::Attached) impl_->notifyReady();
  return result;
}
void PoseGraphBackend::resetFreshSession() { impl_->resetFreshSession(); }
void PoseGraphBackend::promoteStorageToWritable() { impl_->promoteStorageToWritable(); }
void PoseGraphBackend::finish() { impl_->finish(); }
void PoseGraphBackend::drain() { impl_->drain(); }
void PoseGraphBackend::close() { impl_->close(); }
bool PoseGraphBackend::captureReady(bool navigation, bool try_only, std::string &uuid, std::uint64_t &revision,
    std::uint64_t &generation, std::uint64_t &source, double &timestamp, Eigen::Isometry3d &correction,
    Eigen::Isometry3d &pose, std::shared_ptr<const NavigationGrid> &grid) const {
  return impl_->captureReady(navigation, try_only, uuid, revision, generation, source, timestamp, correction, pose, grid);
}
bool PoseGraphBackend::failed() const { return impl_->failed(); }
std::uint64_t PoseGraphBackend::commitFinalizedSubmap(const SubmapFrame &submap, const LocalGrid &grid, const std::vector<std::uint8_t> &scene,
                                                      const std::vector<std::pair<int, Eigen::Isometry3f>> &poses, const GraphLink &base,
                                                      const std::optional<GraphLink> &loop, const std::string &uuid, std::uint64_t revision,
                                                      int next_id, int chain_root) {
  return impl_->commitFinalizedSubmap(submap, grid, scene, poses, base, loop, uuid, revision, next_id, chain_root);
}

bool PoseGraphBackend::enabled() const { return impl_->enabled(); }

void PoseGraphBackend::addFrame(SubmapFrame frame) { impl_->addFrame(std::move(frame)); }

Eigen::Isometry3d PoseGraphBackend::T_map_odom() const { return impl_->correction(); }

std::shared_ptr<const NavigationGrid> PoseGraphBackend::latestOccupancyGrid() const { return impl_->latestOccupancyGrid(); }

bool PoseGraphBackend::historical() const { return impl_->historical(); }
bool PoseGraphBackend::hasActiveCorrection() const { return impl_->hasActiveCorrection(); }
LoopDecisionStatus PoseGraphBackend::loopDecisionStatus() const { return LoopDecisionStatus::Unavailable; }
std::string PoseGraphBackend::mapUuid() const { return impl_->mapUuid(); }
std::uint64_t PoseGraphBackend::graphRevision() const { return impl_->graphRevision(); }
ReconstructionDiagnostics PoseGraphBackend::reconstructionDiagnostics() const { return impl_->diagnostics(); }
std::optional<Eigen::Isometry3f> PoseGraphBackend::committedPose(std::uint64_t id) const { return impl_->committedPose(id); }
std::vector<SpatialMatch> PoseGraphBackend::historicalSpatialCandidates(std::uint64_t id) const { return impl_->spatialCandidates(id); }
std::vector<VisualSubmapMatch> PoseGraphBackend::historicalVisualCandidates(std::uint64_t id) { return impl_->visualCandidates(id); }
mapping::DescriptorArchive::IndexNodeMetadata PoseGraphBackend::historicalVisualMetadata() const { return impl_->visualMetadata(); }
OccupancyGrid PoseGraphBackend::historicalOccupancy() const { return impl_->historicalOccupancy(); }

}  // namespace sapphire

namespace sapphire {
void PoseGraphBackend::beginContinuation(std::uint64_t g) { impl_->beginContinuation(g); }
ContinuationAdmission PoseGraphBackend::submitContinuation(SubmapFrame &q, LocalGrid &grid, std::uint64_t g, bool wait) { return impl_->submitContinuation(q, grid, g, wait); }
void PoseGraphBackend::retryContinuation(std::uint64_t s, std::uint64_t g) { impl_->retryContinuation(s, g); }
ContinuationProgress PoseGraphBackend::continuationProgress() const { return impl_->continuationProgress(); }
void PoseGraphBackend::stopAdmission(bool abort) { impl_->stopAdmission(abort); }
}

namespace sapphire {
AttachmentResult PoseGraphBackend::attachFreshSessionRetained(SubmapFrame &q, const LocalGrid &grid, int target,
                                                              const Eigen::Isometry3d &seed, std::uint64_t generation) {
  auto result = impl_->attachFreshSession(q, grid, target, seed, generation);
  if (result.status == AttachmentStatus::Attached) impl_->notifyReady();
  return result;
}
}

namespace sapphire { std::vector<VisualSubmapMatch> PoseGraphBackend::transientVisualCandidates(const SubmapFrame &q) { return impl_->transientVisualCandidates(q); } }

namespace sapphire {
AttachmentResult PoseGraphBackend::attachFreshSessionAutomatically(SubmapFrame &q, const LocalGrid &grid, std::uint64_t generation, const SubmapFrame *observation) {
  auto result = impl_->attachFreshSessionAutomatically(q, grid, generation, observation);
  if (result.status == AttachmentStatus::Attached) impl_->notifyReady();
  return result;
}
}

namespace sapphire {
std::optional<std::size_t> PoseGraphBackend::boundedInputBytes(const SubmapFrame &q, const LocalGrid &grid) {
  std::size_t bytes=0;
  return Impl::boundedInput(q,grid,&bytes) ? std::optional<std::size_t>(bytes) : std::nullopt;
}
}
