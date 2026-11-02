#include <spdlog/spdlog.h>
#include "pipeline/frontend.hpp"

#include <malloc.h>
#include <sys/resource.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

#include "tools/timer.hpp"
namespace sapphire {
namespace {

bool finite_state(const StateGroup& state) {
  return state.R.allFinite() && state.p.allFinite() && state.v.allFinite() && state.bg.allFinite() &&
         state.ba.allFinite() && state.g.allFinite() && state.cov.allFinite();
}

bool profiling_enabled() {
  static const bool enabled = [] {
    const char *setting = std::getenv("SAPPHIRE_PROFILE");
    if (!setting) setting = std::getenv("SAPPHIRE_PROFILE_BACKEND");
    return setting && std::strcmp(setting, "1") == 0;
  }();
  return enabled;
}

}  // namespace

struct FrontendRuntime::ScanCompletion {
  FrontendRuntime& owner;
  const MeasGroup& measures;
  ScanDisposition disposition = ScanDisposition::Initializing;
  RecoveryReason reason = RecoveryReason::None;
  int exceptions = std::uncaught_exceptions();
  tools::Timer timer;
  ~ScanCompletion() {
    if (std::uncaught_exceptions() > exceptions) disposition = ScanDisposition::Failed;
    owner.acknowledge(measures, disposition, reason);
    SlamPipeline::invokeCallback(owner.output_.scan_processed, measures.lidar_end_time, timer.elapsed_milliseconds());
    std::lock_guard<std::mutex> lock(owner.input_mutex_);
    owner.last_scan_finished_ = Clock::now();
  }
};

FrontendRuntime::FrontendRuntime(const SapphireParameters &parameters, const SlamPipeline::Callbacks &output, Handoff handoff)
    : parameters_(parameters),
      output_(output),
      handoff_(std::move(handoff)),
      synchronizer_(parameters_.sensor.max_scan_duration, 4096, parameters_.recovery.max_imu_gap_sec, parameters_.recovery.max_lidar_gap_sec),
      parallel_executor_(parameters_.local_submap.thread_num),
      local_map_(parameters_, parallel_executor_) {
  if (!handoff_.wait_for_capacity || !handoff_.lidar || !handoff_.seal || !handoff_.reset)
    throw std::invalid_argument("Frontend requires an ownership handoff and lifecycle callbacks");
  if (parameters_.odometry.gicp_fallback.enabled) ricp_ = std::make_unique<Ricp>(parameters_.odometry.gicp_fallback);
  imu_estimator_.configure(parameters_.sensor, parameters_.initializer, parameters_.odometry);
  extrinsic_.R = imu_estimator_.Lid_rot_to_IMU;
  extrinsic_.p = imu_estimator_.Lid_offset_to_IMU;

}
void FrontendRuntime::stop() {
  accepting_.store(false, std::memory_order_release);
  synchronizer_.stop_accepting();
  stopping_.store(true, std::memory_order_release);
  { std::lock_guard<std::mutex> lock(input_mutex_); }
  input_cv_.notify_all();
}
RecoveryProgress FrontendRuntime::recoveryProgress() const {
  std::lock_guard<std::mutex> lock(input_mutex_);
  return recovery_;
}
void FrontendRuntime::setPhase(TrackingPhase phase) {
  RecoveryProgress status;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (recovery_.phase == TrackingPhase::Failed) return;
    recovery_.phase = phase;
    status = recovery_;
  }
  SlamPipeline::invokeCallback(output_.recovery_changed, std::cref(status));
}
void FrontendRuntime::requestRecovery(RecoveryReason reason) {
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (stopping_.load() || recovery_.phase == TrackingPhase::Failed) return;
    if (!recovery_.pending) pending_reason_ = reason;
    recovery_.pending = true;
    // Fence the backend before the worker can observe the recovery request.
    if (handoff_.suspend_output) handoff_.suspend_output();
    input_revision_.fetch_add(1, std::memory_order_release);
  }
  input_cv_.notify_one();
}
void FrontendRuntime::markFailed() {
  stop();
  cancelQueued(RecoveryReason::EstimatorFailure);
  setPhase(TrackingPhase::Failed);
}
void FrontendRuntime::acknowledge(const MeasGroup& scan, ScanDisposition disposition, RecoveryReason reason) {
  const ScanResult result{scan.sequence, scan.generation, scan.lidar_end_time, disposition, reason};
  SlamPipeline::invokeCallback(output_.scan_finished, std::cref(result));
}
void FrontendRuntime::cancelQueued(RecoveryReason reason) {
  for (const auto& scan : synchronizer_.reset()) acknowledge(scan, ScanDisposition::Cancelled, reason);
}
bool FrontendRuntime::push_imu(ImuMeas imu) {
  RecoveryReason reason = RecoveryReason::None;
  bool accepted;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (!accepting_.load() || recovery_.pending || recovery_.phase == TrackingPhase::Resetting) return false;
    if (!seen_imu_ && !seen_lidar_) stream_started_ = Clock::now();
    accepted = synchronizer_.push_imu(std::move(imu), &reason);
    if (accepted) { imu_arrival_ = Clock::now(); seen_imu_ = true; }
    input_revision_.fetch_add(1, std::memory_order_release);
  }
  if (!accepted) { ++imu_refusals_; requestRecovery(reason); }
  input_cv_.notify_one();
  return accepted;
}
ScanAdmission FrontendRuntime::rejectLidar(double stamp, RecoveryReason reason) {
  ScanAdmission admission;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    admission = {++next_scan_sequence_, recovery_.generation, false, reason};
  }
  requestRecovery(reason);
  MeasGroup scan;
  scan.sequence = admission.sequence; scan.generation = admission.generation; scan.lidar_end_time = stamp;
  acknowledge(scan, ScanDisposition::Rejected, reason);
  return admission;
}
ScanAdmission FrontendRuntime::push_lidar(double timestamp, std::vector<LidarPoint>&& cloud, std::optional<double> end) {
  ScanAdmission admission;
  bool request = false;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    admission.sequence = ++next_scan_sequence_;
    admission.generation = recovery_.generation;
    if (!accepting_.load() || recovery_.pending || recovery_.phase == TrackingPhase::Resetting) {
      admission.reason = RecoveryReason::InputUnavailable;
    } else {
      if (!seen_imu_ && !seen_lidar_) stream_started_ = Clock::now();
      admission.accepted = synchronizer_.push_lidar(timestamp, std::move(cloud), end, admission.sequence, admission.generation, &admission.reason);
      request = !admission.accepted;
      if (admission.accepted) { lidar_arrival_ = Clock::now(); seen_lidar_ = true; }
    }
    input_revision_.fetch_add(1, std::memory_order_release);
  }
  if (!admission.accepted) {
    ++lidar_refusals_;
    if (request) requestRecovery(admission.reason);
    MeasGroup scan;
    scan.sequence = admission.sequence; scan.generation = admission.generation; scan.lidar_end_time = end.value_or(timestamp);
    acknowledge(scan, ScanDisposition::Rejected, admission.reason);
  }
  input_cv_.notify_one();
  return admission;
}
RecoveryReason FrontendRuntime::stalledSensor() const {
  std::lock_guard<std::mutex> lock(input_mutex_);
  const double budget = parameters_.recovery.input_timeout_sec;
  if (budget <= 0 || recovery_.pending || (!seen_imu_ && !seen_lidar_)) return RecoveryReason::None;
  const auto now = Clock::now();
  const auto started = stream_started_;
  const auto imu_deadline = std::max(last_scan_finished_, seen_imu_ ? imu_arrival_ : started);
  const auto lidar_deadline = std::max(last_scan_finished_, seen_lidar_ ? lidar_arrival_ : started);
  if (std::chrono::duration<double>(now - imu_deadline).count() >= budget) return RecoveryReason::ImuSilence;
  if (std::chrono::duration<double>(now - lidar_deadline).count() >= budget) return RecoveryReason::LidarSilence;
  return RecoveryReason::None;
}
void FrontendRuntime::checkInputHealth() {
  const auto reason = stalledSensor();
  if (reason != RecoveryReason::None) requestRecovery(reason);
}
bool FrontendRuntime::recover() {
  RecoveryReason reason;
  bool reopen;
  std::uint64_t generation;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (!recovery_.pending) return true;
    reason = pending_reason_;
    reopen = !recovery_episode_;
    recovery_episode_ = true;
    recovery_.reason = reason;
    recovery_.phase = TrackingPhase::Resetting;
    ++recovery_.input_epoch;
    if (reopen) { ++recovery_.generation; ++recovery_.resets; }
    generation = recovery_.generation;
  }
  // Quiesce optional and required adapter output before exposing the new epoch.
  if (output_.owner_reset) output_.owner_reset(true);
  setPhase(TrackingPhase::Resetting);
  cancelQueued(reason);
  if (reason != RecoveryReason::EstimatorFailure) sealWindow();
  system_reset();
  if (!handoff_.reset(generation)) return false;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    seen_imu_ = seen_lidar_ = false;
    last_scan_finished_ = Clock::now();
    pending_reason_ = RecoveryReason::None;
    recovery_.pending = false;
  }
  spdlog::warn("[recovery] generation={} reopen={} reason={}", generation, reopen, recoveryReasonName(reason));
  // The ordered boundary is admitted. The new estimator starts immediately;
  // backend publication remains generation-fenced until its owner catches up.
  if (output_.owner_reset) output_.owner_reset(false);
  setPhase(TrackingPhase::WaitingForInput);
  return !stopping_.load();
}

void FrontendRuntime::emit_local_scan(const vvec<double, 3> &points) {
  const auto status = recoveryProgress();
  if (status.pending || status.phase == TrackingPhase::Failed) return;
  SlamPipeline::invokeCallback(output_.odom_state, std::cref(current_state_));
  if (output_.local_scan) {
    try {
      if (points.size() > (16 * 1024 * 1024 - 65536) / 64) throw std::length_error("optional scan cap");

      output_.local_scan(std::make_shared<vvec<double, 3>>(points));
    } catch (...) {
      SlamPipeline::invokeCallback(output_.optional_drop, false);
    }
  }
}

void FrontendRuntime::emit_local_map(int marginal_count) {
  const auto status = recoveryProgress();
  if (status.pending || status.phase == TrackingPhase::Failed) return;
  if (output_.local_map) {
    try {
      constexpr std::size_t limit = (16 * 1024 * 1024 - 65536) / 64;
      std::size_t count = 0;
      for (int i = 0; i < marginal_count; ++i) {
        const auto n = local_map_.points(i).size() / 3 + (local_map_.points(i).size() % 3 != 0);
        if (n > limit - count) throw std::length_error("optional local-map cap");
        count += n;
      }

      auto map = std::make_shared<vvec<double, 3>>();
      map->reserve(count);
      for (int i = 0; i < marginal_count; ++i)
        for (size_t j = 0; j < local_map_.points(i).size(); j += 3) {
          const pointVar &point = local_map_.points(i).at(j);
          map->push_back(local_map_.state(i).R * point.pnt + local_map_.state(i).p);
        }
      output_.local_map(std::move(map));
    } catch (...) {
      SlamPipeline::invokeCallback(output_.optional_drop, true);
    }
  }
}

int FrontendRuntime::initialization(MeasGroup &measures, vvec<double, 3> &world_points) {
  std::deque<ImuMeas> &imus = measures.imu_buf;
  std::shared_ptr<std::vector<LidarPoint>> &cloud = measures.lidar_cloud;
  auto original = std::make_shared<std::vector<LidarPoint>>(*cloud);
  const bool had_imu_initialization = imu_estimator_.init_flag;
  if (imu_estimator_.process(current_state_, *cloud, imus) == 0) {
    return had_imu_initialization ? -2 : 0;
  }

  PointCloudPtr points(new PointCloud);
  const double init_down_size_inv = parameters_.initializer.down_size_inv;
  down_sampling_voxel(*cloud, init_down_size_inv);
  var_init(extrinsic_, *cloud, points, parameters_.odometry.dept_err, parameters_.odometry.beam_err);
  eskf_.observe_kdtree(current_state_, *points, init_down_size_inv);

  world_points.clear();
  pvec_update(points, current_state_, world_points);
  // Tentative initialization states are not valid odometry outputs.

  std::vector<LidarPoint> fallback = *original;
  down_sampling_close(*original, parameters_.odometry.down_size_inv);
  if (original->size() < 1000) {
    *original = std::move(fallback);
    down_sampling_close(*original, parameters_.odometry.down_size_inv * 2.0);
  }
  std::stable_sort(original->begin(), original->end(),
                   [](const LidarPoint &left, const LidarPoint &right) { return left.time_offset < right.time_offset; });

  MeasGroup init_measures;
  init_measures.lidar_begin_time = measures.lidar_begin_time;
  init_measures.lidar_end_time = measures.lidar_end_time;
  init_measures.imu_buf = std::move(imus);
  init_measures.lidar_cloud = std::move(original);
  local_map_.insert_initial(current_state_, points, std::move(init_measures), imu_estimator_.scale_gravity);
  if (!local_map_.full()) return 0;
  return local_map_.initialize(current_state_, extrinsic_, imu_estimator_.scale_gravity) ? 1 : -1;
}

bool FrontendRuntime::update_lidar_window(const PointCloudPtr &points, const std::deque<ImuMeas> &imus, double journey) {
  local_map_.append_imu(imus, imu_estimator_.scale_gravity);
  const tools::Timer observation_begin;
  ESKF::Rejection reason;
  bool accepted = eskf_.observe_voxelmap(current_state_, *points, local_map_.geometry(), &reason);
  if (!accepted && ricp_ && (reason == ESKF::Rejection::normal_support || reason == ESKF::Rejection::no_matches)) {
    const tools::Timer fallback_begin;
    const auto result = ricp_->observe(current_state_, *points, local_map_.geometry());
    accepted = result.accepted;
    spdlog::info(
        "[gicp-fallback] t={:.9f} accepted={} reason={} samples={} matches={} targets={} iterations={} rms={:.5f} correction={:.5f} ms={:.3f}",
        current_state_.t, accepted, result.reason, result.samples, result.matches, result.targets, result.iterations, result.rms, result.correction,
        fallback_begin.elapsed_milliseconds());
  }
  SlamPipeline::invokeCallback(output_.lidar_update, std::cref(current_state_), accepted, observation_begin.elapsed_milliseconds());
  if (!accepted) return false;
  if (!finite_state(current_state_)) { requestRecovery(RecoveryReason::EstimatorFailure); return false; }

  vvec<double, 3> world_points;
  pvec_update(points, current_state_, world_points);
  local_map_.insert(current_state_, points, journey, world_points);
  return true;
}

void FrontendRuntime::sealWindow() {
  if (!parameters_.pose_graph.enabled) return;
  while (auto frame = local_map_.seal_oldest())
    if (!handoff_.seal(std::move(*frame))) break;
}

void FrontendRuntime::system_reset() {
  local_map_.reset();
  eskf_.pl_tree.clear();
  current_state_.setZero();
  imu_estimator_ = ImuEstimator{};
  imu_estimator_.configure(parameters_.sensor, parameters_.initializer, parameters_.odometry);
  if (ricp_) ricp_ = std::make_unique<Ricp>(parameters_.odometry.gicp_fallback);
}

void FrontendRuntime::run() {
  vvec<double, 3> world_points;
  Eigen::Vector3d last_position = Eigen::Vector3d::Zero();
  double journey = 0.0;
  int motion_init_flag = 1;
  eskf_.pl_tree.clear();
  int degrade_count = 0;
  constexpr int marginal_count = 1;
  uint64_t observed_revision = input_revision_.load(std::memory_order_acquire);

  while (true) {
    if (recoveryProgress().pending) {
      if (!recover()) break;
      motion_init_flag = 1; degrade_count = 0; journey = 0;
      last_position.setZero(); world_points.clear();
      local_map_.geometry().release_retired(std::numeric_limits<std::size_t>::max());
      local_map_.geometry().trim_window_pool(0, std::numeric_limits<std::size_t>::max());
      malloc_trim(0);
      continue;
    }
    MeasGroup measures;
    const auto sync = synchronizer_.sync_packages(measures);
    if (sync.kind == SyncOutcome::Kind::Waiting) {
      const auto stalled = stalledSensor();
      if (stalled != RecoveryReason::None) { requestRecovery(stalled); continue; }
      if (stopping_.load(std::memory_order_acquire)) {
        break;
      }
      if (local_map_.geometry().release_retired(1000) > 0) {
        malloc_trim(0);
      } else if (local_map_.geometry().trim_window_pool(10000, 500) > 0) {
        malloc_trim(0);
      }
      std::unique_lock<std::mutex> lock(input_mutex_);
      const auto ready = [this, &observed_revision] {
        return stopping_.load(std::memory_order_acquire) || recovery_.pending || input_revision_.load(std::memory_order_acquire) != observed_revision;
      };
      if (parameters_.recovery.input_timeout_sec > 0)
        input_cv_.wait_for(lock, std::chrono::duration<double>(std::min(.1, parameters_.recovery.input_timeout_sec)), ready);
      else input_cv_.wait(lock, ready);
      observed_revision = input_revision_.load(std::memory_order_acquire);
      continue;
    }

    ScanCompletion completion{*this, measures};
    if (sync.kind == SyncOutcome::Kind::Invalid) {
      completion.disposition = ScanDisposition::Rejected; completion.reason = sync.reason;
      requestRecovery(sync.reason);
      continue;
    }
    if (recoveryProgress().pending) {
      completion.disposition = ScanDisposition::Cancelled;
      completion.reason = RecoveryReason::InputUnavailable;
      continue;
    }
    imu_estimator_.pcl_beg_time = measures.lidar_begin_time;
    imu_estimator_.pcl_end_time = measures.lidar_end_time;
    std::shared_ptr<std::vector<LidarPoint>> &cloud = measures.lidar_cloud;
    std::deque<ImuMeas> &imus = measures.imu_buf;

    if (motion_init_flag) {
      if (recoveryProgress().phase != TrackingPhase::Initializing) setPhase(TrackingPhase::Initializing);
      const int init = initialization(measures, world_points);
      if (!finite_state(current_state_)) {
        completion.disposition = ScanDisposition::Rejected; completion.reason = RecoveryReason::EstimatorFailure;
        requestRecovery(completion.reason); continue;
      }
      if (recoveryProgress().pending) {
        completion.disposition = ScanDisposition::Cancelled;
        completion.reason = RecoveryReason::InputUnavailable;
        continue;
      }
      if (init == 1) {
        motion_init_flag = 0;
        {
          std::lock_guard<std::mutex> lock(input_mutex_);
          recovery_episode_ = false;
        }
        setPhase(TrackingPhase::Tracking);
      } else {
        if (init < 0) {
          { std::lock_guard<std::mutex> lock(input_mutex_); ++recovery_.initialization_attempts; }
          completion.reason = init == -2 ? RecoveryReason::ImuCoverage : RecoveryReason::InitializationRejected;
          completion.disposition = ScanDisposition::Rejected;
          requestRecovery(completion.reason);
        }
        continue;
      }
    } else {
      if (imu_estimator_.process(current_state_, *cloud, imus) == 0) {
        completion.disposition = ScanDisposition::Rejected; completion.reason = RecoveryReason::ImuCoverage;
        requestRecovery(completion.reason);
        continue;
      }

      std::vector<LidarPoint> downsampled = *cloud;
      down_sampling_voxel(downsampled, parameters_.odometry.down_size_inv);
      if (downsampled.size() < 500) {
        downsampled = *cloud;
        down_sampling_voxel(downsampled, parameters_.odometry.down_size_inv * 2.0);
      }

      PointCloudPtr points(new PointCloud);
      var_init(extrinsic_, downsampled, points, parameters_.odometry.dept_err, parameters_.odometry.beam_err);
      const bool accepted = update_lidar_window(points, imus, journey);
      degrade_count = accepted ? 0 : degrade_count + 1;
      if (!accepted) {
        completion.disposition = ScanDisposition::ObservationRejected;
        completion.reason = RecoveryReason::ObservationRejected;
      }
      if (degrade_count >= parameters_.odometry.degrade_bound) {
        requestRecovery(RecoveryReason::ObservationRejected);
        continue;
      }
      if (!accepted) {
        // An IMU prediction without usable LiDAR is not a valid tracking output.
        // Retain it only inside the estimator until recovery or a good update.
        continue;
      }
    }

    completion.disposition = ScanDisposition::Tracked;
    if (recoveryProgress().pending) { completion.disposition = ScanDisposition::Cancelled; continue; }
    if (local_map_.full()) {
      local_map_.optimize(current_state_);
      if (!finite_state(current_state_)) {
        completion.disposition = ScanDisposition::Rejected; completion.reason = RecoveryReason::EstimatorFailure;
        requestRecovery(completion.reason); continue;
      }
      world_points.clear();
      for (const auto& point : local_map_.points(local_map_.size() - 1))
        world_points.push_back(current_state_.R * point.pnt + current_state_.p);
      emit_local_scan(world_points);
      emit_local_map(marginal_count);
      const tools::Timer wait_start;
      if (!handoff_.wait_for_capacity()) {
        completion.disposition = ScanDisposition::Cancelled;
        if (recoveryProgress().pending) continue;
        break;
      }
      const double wait_ms = wait_start.elapsed_milliseconds();
      if (profiling_enabled() && wait_ms > 1.0) spdlog::info("[backend-profile] marginal_wait stamp={:.9f} ms={:.3f}", measures.lidar_end_time, wait_ms);
      // The existing new-map/backend-off consumer discards frozen evidence.
      // Keep marginalization and queue/drain sequencing, omit only its payload.
      const bool emit_evidence = parameters_.pose_graph.enabled || parameters_.pose_graph.map_mode != "new";
      handoff_.lidar(local_map_.retire_oldest(journey, emit_evidence));

      if ((local_map_.retired_frames() + local_map_.size()) % 10 == 0) {
        const double distance = (current_state_.p - last_position).norm();
        if (distance > 0.5) {
          journey += distance;
          last_position = current_state_.p;
        }
      }

      // Enforce the disposable reuse-cache bound even while input is continuously
      // available; correctness must not depend on the worker reaching an idle gap.
      local_map_.geometry().trimFixed();
      const auto windows_released = local_map_.geometry().trim_window_pool(10000, std::numeric_limits<std::size_t>::max());
      if (windows_released) malloc_trim(0);

      if (profiling_enabled() && local_map_.retired_frames() % 100 == 0) {
        const auto memory = local_map_.geometry().memoryUsage();
        const auto heap = mallinfo2();
        rusage usage{};
        getrusage(RUSAGE_SELF, &usage);
        spdlog::info(
            "[memory-profile] fixed_bytes={} fixed_roots={} evicted_roots={} stamp={} trees={} active_windows={} pooled_windows={} tree_bytes={} window_bytes={} pooled_window_bytes={} "
            "point_capacity_bytes={} empty_slot_bytes={} leaf_point_bytes={} heap_used={} heap_free={} mmap_bytes={} peak_rss_bytes={}",
            local_map_.geometry().fixedBytes(), local_map_.geometry().fixedSize(), local_map_.geometry().evictedRoots(),
            measures.lidar_end_time, memory.trees, memory.active_windows, memory.pooled_windows, memory.tree_bytes, memory.window_bytes,
            memory.pooled_window_bytes, memory.point_capacity_bytes, memory.empty_slot_bytes, memory.leaf_point_bytes, heap.uordblks, heap.fordblks,
            heap.hblkhd, std::size_t(usage.ru_maxrss) * 1024);
      }
    }
  }

  cancelQueued(RecoveryReason::InputUnavailable);
  if (recoveryProgress().phase != TrackingPhase::Failed) sealWindow();
  if (recoveryProgress().phase != TrackingPhase::Failed) setPhase(TrackingPhase::Stopped);
  local_map_.geometry().clear();
  while (local_map_.geometry().release_retired(1000) > 0) {
  }
  // The odometry owner has stopped; retaining an empty reuse cache through the
  // backend's final drain only overlaps two independent memory peaks.
  local_map_.geometry().trim_window_pool(0, std::numeric_limits<std::size_t>::max());
  malloc_trim(0);
  // Caller signals end-of-input on both normal and exceptional exits.
}

}  // namespace sapphire
