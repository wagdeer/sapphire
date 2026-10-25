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
  const std::function<void(double, double)> &callback;
  double stamp;
  tools::Timer timer;
  ~ScanCompletion() { SlamPipeline::invokeCallback(callback, stamp, timer.elapsed_milliseconds()); }
};

FrontendRuntime::FrontendRuntime(const SapphireParameters &parameters, const SlamPipeline::Callbacks &output, Handoff handoff)
    : parameters_(parameters),
      output_(output),
      handoff_(std::move(handoff)),
      synchronizer_(parameters_.sensor.max_scan_duration, 4096),
      parallel_executor_(parameters_.local_submap.thread_num),
      local_map_(parameters_, parallel_executor_) {
  if (!handoff_.wait_for_capacity || !handoff_.lidar || !handoff_.reset || !handoff_.invalidate_domain)
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
void FrontendRuntime::invalidateOdometryDomain() { handoff_.invalidate_domain(); }
void FrontendRuntime::notifyInputLoss() {
  if (parameters_.pose_graph.map_mode == "resume") invalidateOdometryDomain();
}
bool FrontendRuntime::push_imu(ImuMeas imu) {
  if (!accepting_.load(std::memory_order_acquire)) {
    return false;
  }
  if (!synchronizer_.push_imu(std::move(imu))) {
    const auto count = ++imu_refusals_;
    if (count == 1 || count % 100 == 0) spdlog::warn("IMU admission refused (invalid/order/capacity), total={}", count);
    notifyInputLoss();
    return false;
  }
  input_revision_.fetch_add(1, std::memory_order_release);
  input_cv_.notify_one();
  return true;
}

bool FrontendRuntime::push_lidar(double timestamp, std::vector<LidarPoint> &&cloud, std::optional<double> end_timestamp) {
  if (!accepting_.load(std::memory_order_acquire)) {
    return false;
  }
  if (!synchronizer_.push_lidar(timestamp, std::move(cloud), end_timestamp)) {
    const auto count = ++lidar_refusals_;
    if (count == 1 || count % 100 == 0) spdlog::warn("LiDAR admission refused (invalid/order/capacity), total={}", count);
    notifyInputLoss();
    return false;
  }
  input_revision_.fetch_add(1, std::memory_order_release);
  input_cv_.notify_one();
  return true;
}

void FrontendRuntime::emit_local_scan(const vvec<double, 3> &points) {
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
  if (imu_estimator_.process(current_state_, *cloud, imus) == 0) {
    return 0;
  }

  PointCloudPtr points(new PointCloud);
  const double init_down_size_inv = parameters_.initializer.down_size_inv;
  down_sampling_voxel(*cloud, init_down_size_inv);
  var_init(extrinsic_, *cloud, points, parameters_.odometry.dept_err, parameters_.odometry.beam_err);
  eskf_.observe_kdtree(current_state_, *points, init_down_size_inv);

  world_points.clear();
  pvec_update(points, current_state_, world_points);
  emit_local_scan(world_points);

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

  vvec<double, 3> world_points;
  pvec_update(points, current_state_, world_points);
  emit_local_scan(world_points);
  local_map_.insert(current_state_, points, journey, world_points);
  return true;
}

void FrontendRuntime::system_reset(const std::deque<ImuMeas> &imus) {
  if (handoff_.suspend_output) handoff_.suspend_output();
  current_state_.setZero();
  current_state_.p = Eigen::Vector3d(0, 0, 30);
  imu_estimator_.mean_acc.setZero();
  imu_estimator_.init_num = 0;
  imu_estimator_.init(imus);
  current_state_.g = -imu_estimator_.mean_acc * imu_estimator_.scale_gravity;

  // imus may borrow the failed initialization window; consume it before reset.
  local_map_.reset();
  eskf_.pl_tree.clear();
  spdlog::warn("Reset");
}

void FrontendRuntime::run() {
  vvec<double, 3> world_points;
  Eigen::Vector3d last_position = Eigen::Vector3d::Zero();
  double journey = 0.0;
  int motion_init_flag = 1;
  eskf_.pl_tree.clear();
  bool release_flag = false;
  int degrade_count = 0;
  constexpr int marginal_count = 1;
  uint64_t observed_revision = input_revision_.load(std::memory_order_acquire);

  while (true) {
    MeasGroup measures;
    if (!synchronizer_.sync_packages(measures)) {
      if (parameters_.pose_graph.map_mode == "resume" && synchronizer_.takeContinuityLoss()) {
        invalidateOdometryDomain();
        break;
      }
      if (stopping_.load(std::memory_order_acquire)) {
        break;
      }
      if (local_map_.geometry().release_retired(1000) > 0) {
        malloc_trim(0);
      } else if (release_flag) {
        release_flag = false;
        local_map_.geometry().prune(journey, parameters_.odometry.map_retention_distance);
      } else if (local_map_.geometry().trim_window_pool(10000, 500) > 0) {
        malloc_trim(0);
      }
      std::unique_lock<std::mutex> lock(input_mutex_);
      input_cv_.wait(lock, [this, &observed_revision] {
        return stopping_.load(std::memory_order_acquire) || input_revision_.load(std::memory_order_acquire) != observed_revision;
      });
      observed_revision = input_revision_.load(std::memory_order_acquire);
      continue;
    }

    const ScanCompletion completion{output_.scan_processed, measures.lidar_end_time};
    imu_estimator_.pcl_beg_time = measures.lidar_begin_time;
    imu_estimator_.pcl_end_time = measures.lidar_end_time;
    std::shared_ptr<std::vector<LidarPoint>> &cloud = measures.lidar_cloud;
    std::deque<ImuMeas> &imus = measures.imu_buf;

    if (motion_init_flag) {
      const int init = initialization(measures, world_points);
      if (init == 1) {
        motion_init_flag = 0;
      } else {
        if (init == -1) {
          system_reset(local_map_.last_initial_imus());
        }
        continue;
      }
    } else {
      if (imu_estimator_.process(current_state_, *cloud, imus) == 0) {
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
      if (accepted) {
        if (degrade_count > 0) {
          --degrade_count;
        }
      } else {
        ++degrade_count;
      }

      if (degrade_count > parameters_.odometry.degrade_bound) {
        if (parameters_.pose_graph.map_mode == "resume") {
          invalidateOdometryDomain();
          break;  // old live-domain type-0 must end before frontend reset
        }
        degrade_count = 0;
        system_reset(imus);
        last_position = current_state_.p;
        journey = 0.0;
        handoff_.reset();
        motion_init_flag = 1;
        continue;
      }
      if (!accepted) {
        // Publish prediction only; scans belong to accepted window frames
        // and must not contaminate downstream maps.
        SlamPipeline::invokeCallback(output_.odom_state, std::cref(current_state_));
        continue;
      }
    }

    if (local_map_.full()) {
      local_map_.optimize(current_state_);
      emit_local_map(marginal_count);
      const tools::Timer wait_start;
      if (!handoff_.wait_for_capacity()) break;
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
          release_flag = true;
        }
      }

      // Enforce the disposable reuse-cache bound even while input is continuously
      // available; correctness must not depend on the worker reaching an idle gap.
      if (local_map_.geometry().trim_window_pool(10000, std::numeric_limits<std::size_t>::max()) > 0) malloc_trim(0);

      if (profiling_enabled() && local_map_.retired_frames() % 100 == 0) {
        const auto memory = local_map_.geometry().memoryUsage();
        const auto heap = mallinfo2();
        rusage usage{};
        getrusage(RUSAGE_SELF, &usage);
        spdlog::info(
            "[memory-profile] stamp={} trees={} active_windows={} pooled_windows={} tree_bytes={} window_bytes={} pooled_window_bytes={} "
            "point_capacity_bytes={} empty_slot_bytes={} leaf_point_bytes={} heap_used={} heap_free={} mmap_bytes={} peak_rss_bytes={}",
            measures.lidar_end_time, memory.trees, memory.active_windows, memory.pooled_windows, memory.tree_bytes, memory.window_bytes,
            memory.pooled_window_bytes, memory.point_capacity_bytes, memory.empty_slot_bytes, memory.leaf_point_bytes, heap.uordblks, heap.fordblks,
            heap.hblkhd, std::size_t(usage.ru_maxrss) * 1024);
      }
    }
  }

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
