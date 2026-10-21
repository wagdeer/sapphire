#include "frontend/runtime.hpp"

#include <malloc.h>
#include <sys/resource.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "frontend/eskf/lm_optimizer.hpp"
#include "frontend/initialize/initialize.hpp"
#include "frontend/tools/timer.hpp"
namespace sapphire {
using detail::invoke_output;
namespace {
ImageSyncParameters imageSyncParameters(const VisualLoopParameters &visual) {
  ImageSyncParameters result;
  if (!visual.enabled) return result;
  for (std::size_t id = 0; id < visual.cameraCount(); ++id) result.cameras.push_back(visual.camera(id));
  result.stereo = visual.mode == "stereo";
  result.interval = visual.image_interval;
  result.capacity = 64;
  return result;
}

struct ScanCompletion {
  const std::function<void(double, double)> &callback;
  double stamp;
  timer::Clock::time_point start = timer::Clock::now();
  ~ScanCompletion() { invoke_output(callback, stamp, timer::ms(start)); }
};

}  // namespace

FrontendRuntime::FrontendRuntime(const SapphireParameters &parameters, OutputSink output, Handoff handoff)
    : parameters_(parameters),
      output_(std::move(output)),
      handoff_(std::move(handoff)),
      synchronizer_(parameters_.sensor.max_scan_duration, 4096, imageSyncParameters(parameters_.pose_graph.visual)),
      parallel_executor_(parameters_.local_submap.thread_num),
      voxel_map_(parameters_.odometry, parameters_.local_submap, parallel_executor_) {
  if (!handoff_.wait_for_capacity || !handoff_.lidar || !handoff_.reset || !handoff_.invalidate_domain)
    throw std::invalid_argument("Frontend requires an ownership handoff and lifecycle callbacks");
  if (parameters_.odometry.gicp_fallback.enabled) ricp_ = std::make_unique<Ricp>(parameters_.odometry.gicp_fallback);
  down_size_inv_ = parameters_.odometry.down_size_inv;
  degrade_bound_ = parameters_.odometry.degrade_bound;
  imu_estimator_.configure(parameters_.sensor, parameters_.initializer, parameters_.odometry);
  extrinsic_.R = imu_estimator_.Lid_rot_to_IMU;
  extrinsic_.p = imu_estimator_.Lid_offset_to_IMU;
  window_size_ = parameters_.local_submap.win_size;

  init_buffer_.reserve(window_size_);
}
FrontendRuntime::~FrontendRuntime() { clear_imu_factors(); }
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

bool FrontendRuntime::push_image(ImageMeas image) {
  if (!parameters_.pose_graph.visual.enabled || !accepting_.load(std::memory_order_acquire)) return false;
  return synchronizer_.push_image(std::move(image));
}

void FrontendRuntime::emit_local_scan(const vvec<double, 3> &points) {
  invoke_output(output_.odom_state, std::cref(current_state_));
  if (output_.local_scan) {
    try {
      if (points.size() > (16 * 1024 * 1024 - 65536) / 64) throw std::length_error("optional scan cap");

      output_.local_scan(std::make_shared<vvec<double, 3>>(points));
    } catch (...) {
      invoke_output(output_.optional_drop, false);
    }
  }
}

void FrontendRuntime::emit_local_map(int marginal_count) {
  if (output_.local_map) {
    try {
      constexpr std::size_t limit = (16 * 1024 * 1024 - 65536) / 64;
      std::size_t count = 0;
      for (int i = 0; i < marginal_count; ++i) {
        const auto n = point_buffer_[i]->size() / 3 + (point_buffer_[i]->size() % 3 != 0);
        if (n > limit - count) throw std::length_error("optional local-map cap");
        count += n;
      }

      auto map = std::make_shared<vvec<double, 3>>();
      map->reserve(count);
      for (int i = 0; i < marginal_count; ++i)
        for (size_t j = 0; j < point_buffer_[i]->size(); j += 3) {
          const pointVar &point = point_buffer_[i]->at(j);
          map->push_back(state_buffer_[i].R * point.pnt + state_buffer_[i].p);
        }
      output_.local_map(std::move(map));
    } catch (...) {
      invoke_output(output_.optional_drop, true);
    }
  }
}

void FrontendRuntime::clear_imu_factors() {
  pending_imu_factor_.reset();
  for (ImuFactor *factor : imu_factor_buffer_) {
    delete factor;
  }
  imu_factor_buffer_.clear();
}

int FrontendRuntime::initialization(MeasGroup &measures, Eigen::MatrixXd &hess, LidarFactor &voxel_hessian, vvec<double, 3> &world_points) {
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
  ++window_count_;
  state_buffer_.push_back(current_state_);
  point_buffer_.push_back(points);
  time_buffer_.push_back(measures.lidar_end_time);
  journey_buffer_.push_back(0.0);
  emit_local_scan(world_points);

  if (window_count_ > 1) {
    imu_factor_buffer_.push_back(new ImuFactor(state_buffer_[window_count_ - 2].bg, state_buffer_[window_count_ - 2].ba));
    imu_factor_buffer_[window_count_ - 2]->push_imu(imus, imu_estimator_.scale_gravity, parameters_.local_submap);
  }

  std::vector<LidarPoint> fallback = *original;
  down_sampling_close(*original, down_size_inv_);
  if (original->size() < 1000) {
    *original = std::move(fallback);
    down_sampling_close(*original, down_size_inv_ * 2.0);
  }
  std::stable_sort(original->begin(), original->end(),
                   [](const LidarPoint &left, const LidarPoint &right) { return left.time_offset < right.time_offset; });

  MeasGroup init_measures;
  init_measures.lidar_begin_time = measures.lidar_begin_time;
  init_measures.lidar_end_time = measures.lidar_end_time;
  init_measures.imu_buf = std::move(imus);
  init_measures.lidar_cloud = std::move(original);
  init_buffer_.push_back(std::move(init_measures));

  if (window_count_ < window_size_) {
    return 0;
  }
  const int success = Initialization::instance().motion_init(
      init_buffer_, &hess, voxel_hessian, state_buffer_, voxel_map_, point_buffer_, window_size_, current_state_, imu_factor_buffer_, extrinsic_,
      parameters_.initializer, parameters_.odometry, parameters_.local_submap, imu_estimator_.scale_gravity, parallel_executor_);
  if (success == 0) {
    return -1;
  }
  init_buffer_.clear();
  return 1;
}

bool FrontendRuntime::update_lidar_window(const PointCloudPtr &points, const std::deque<ImuMeas> &imus, double journey, LidarFactor &voxel_hessian) {
  // Each processed IMU interval is already clipped to consecutive scan ends.
  // Preserve its constraint even when the intervening LiDAR observation fails.
  if (window_count_ > 0) {
    if (!pending_imu_factor_) {
      const StateGroup &anchor = state_buffer_.back();
      pending_imu_factor_ = std::make_unique<ImuFactor>(anchor.bg, anchor.ba);
    }
    pending_imu_factor_->push_imu(imus, imu_estimator_.scale_gravity, parameters_.local_submap);
  }
  const auto observation_begin = timer::Clock::now();
  ESKF::Rejection reason;
  bool accepted = eskf_.observe_voxelmap(current_state_, *points, voxel_map_, &reason);
  if (!accepted && ricp_ && (reason == ESKF::Rejection::normal_support || reason == ESKF::Rejection::no_matches)) {
    const auto fallback_begin = timer::Clock::now();
    const auto result = ricp_->observe(current_state_, *points, voxel_map_);
    accepted = result.accepted;
    spdlog::info(
        "[gicp-fallback] t={:.9f} accepted={} reason={} samples={} matches={} targets={} iterations={} rms={:.5f} correction={:.5f} ms={:.3f}",
        current_state_.t, accepted, result.reason, result.samples, result.matches, result.targets, result.iterations, result.rms, result.correction,
        timer::ms(fallback_begin));
  }
  invoke_output(output_.lidar_update, std::cref(current_state_), accepted, timer::ms(observation_begin));
  if (!accepted) return false;

  vvec<double, 3> world_points;
  pvec_update(points, current_state_, world_points);
  emit_local_scan(world_points);
  ++window_count_;
  state_buffer_.push_back(current_state_);
  point_buffer_.push_back(points);
  time_buffer_.push_back(current_state_.t);
  journey_buffer_.push_back(journey);
  if (pending_imu_factor_) {
    imu_factor_buffer_.push_back(pending_imu_factor_.get());
    pending_imu_factor_.release();
  }

  voxel_hessian.clear();
  voxel_hessian.win_size = window_size_;
  voxel_map_.cut_voxel_multi(point_buffer_[window_count_ - 1], window_count_ - 1, window_size_, world_points);
  voxel_map_.recut_multi(window_count_, state_buffer_, voxel_hessian, parameters_.odometry.min_eigen_value,
                         parameters_.local_submap.plane_eigen_value_thre_inv);
  return true;
}

void FrontendRuntime::system_reset(const std::deque<ImuMeas> &imus) {
  if (handoff_.suspend_output) handoff_.suspend_output();
  voxel_map_.reset();
  current_state_.setZero();
  current_state_.p = Eigen::Vector3d(0, 0, 30);
  imu_estimator_.mean_acc.setZero();
  imu_estimator_.init_num = 0;
  imu_estimator_.init(imus);
  current_state_.g = -imu_estimator_.mean_acc * imu_estimator_.scale_gravity;

  clear_imu_factors();
  state_buffer_.clear();
  point_buffer_.clear();
  time_buffer_.clear();
  journey_buffer_.clear();
  init_buffer_.clear();
  eskf_.pl_tree.clear();
  window_base_ = 0;
  window_count_ = 0;
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
  LidarFactor voxel_hessian(window_size_);
  LI_BA_Optimizer optimizer(parameters_.local_submap, parallel_executor_);
  constexpr int marginal_count = 1;
  Eigen::MatrixXd hessian;
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
      if (voxel_map_.release_retired(1000) > 0) {
        malloc_trim(0);
      } else if (release_flag) {
        release_flag = false;
        voxel_map_.prune(journey, parameters_.odometry.map_retention_distance);
      } else if (voxel_map_.trim_window_pool(10000, 500) > 0) {
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
      const int init = initialization(measures, hessian, voxel_hessian, world_points);
      if (init == 1) {
        motion_init_flag = 0;
      } else {
        if (init == -1) {
          system_reset(init_buffer_.back().imu_buf);
        }
        continue;
      }
    } else {
      if (imu_estimator_.process(current_state_, *cloud, imus) == 0) {
        continue;
      }

      std::vector<LidarPoint> downsampled = *cloud;
      down_sampling_voxel(downsampled, down_size_inv_);
      if (downsampled.size() < 500) {
        downsampled = *cloud;
        down_sampling_voxel(downsampled, down_size_inv_ * 2.0);
      }

      PointCloudPtr points(new PointCloud);
      var_init(extrinsic_, downsampled, points, parameters_.odometry.dept_err, parameters_.odometry.beam_err);
      const bool accepted = update_lidar_window(points, imus, journey, voxel_hessian);
      if (accepted) {
        if (degrade_count > 0) {
          --degrade_count;
        }
      } else {
        ++degrade_count;
      }

      if (degrade_count > degrade_bound_) {
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
        invoke_output(output_.odom_state, std::cref(current_state_));
        continue;
      }
    }

    if (window_count_ >= window_size_) {
      optimizer.damping_iter(state_buffer_, voxel_hessian, imu_factor_buffer_, &hessian);

      Eigen::Matrix<double, 6, 1> marginal_variance = hessian.block<POSE_DOF, POSE_DOF>(0, STATE_DOF).diagonal();
      for (int i = 0; i < 6; ++i) {
        marginal_variance[i] = 1.0 / std::abs(marginal_variance[i]);
      }

      current_state_.R = state_buffer_[window_count_ - 1].R;
      current_state_.p = state_buffer_[window_count_ - 1].p;
      emit_local_map(marginal_count);
      const auto wait_start = timer::Clock::now();
      if (!handoff_.wait_for_capacity()) break;
      const double wait_ms = timer::ms(wait_start);
      if (timer::enabled() && wait_ms > 1.0) spdlog::info("[backend-profile] marginal_wait stamp={:.9f} ms={:.3f}", measures.lidar_end_time, wait_ms);
      // The existing new-map/backend-off consumer discards frozen evidence.
      // Keep marginalization and queue/drain sequencing, omit only its payload.
      const bool emit_evidence = parameters_.pose_graph.enabled || parameters_.pose_graph.map_mode != "new";
      GaussianCloud marginalized = voxel_map_.marginalize(journey, window_count_, marginal_count, state_buffer_, voxel_hessian, emit_evidence);

      if (marginalized.size() > 16 * 1024 * 1024 / sizeof(GaussianPoint)) throw std::length_error("Marginal evidence exceeds finite producer bound");
      handoff_.lidar(LidarFrame(state_buffer_[0], std::move(marginalized), time_buffer_[0], marginal_variance, journey_buffer_[0]));

      if ((window_base_ + window_count_) % 10 == 0) {
        const double distance = (current_state_.p - last_position).norm();
        if (distance > 0.5) {
          journey += distance;
          last_position = current_state_.p;
          release_flag = true;
        }
      }

      for (int i = marginal_count; i < window_count_; ++i) {
        state_buffer_[i - marginal_count] = state_buffer_[i];
        time_buffer_[i - marginal_count] = time_buffer_[i];
        journey_buffer_[i - marginal_count] = journey_buffer_[i];
        std::swap(point_buffer_[i - marginal_count], point_buffer_[i]);
      }
      for (int i = window_count_ - marginal_count; i < window_count_; ++i) {
        state_buffer_.pop_back();
        point_buffer_.pop_back();
        time_buffer_.pop_back();
        journey_buffer_.pop_back();
        delete imu_factor_buffer_.front();
        imu_factor_buffer_.pop_front();
      }
      window_base_ += marginal_count;
      window_count_ -= marginal_count;
      // Enforce the disposable reuse-cache bound even while input is continuously
      // available; correctness must not depend on the worker reaching an idle gap.
      if (voxel_map_.trim_window_pool(10000, std::numeric_limits<std::size_t>::max()) > 0) malloc_trim(0);

      if (timer::enabled() && window_base_ % 100 == 0) {
        const auto memory = voxel_map_.memoryUsage();
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

  voxel_map_.clear();
  while (voxel_map_.release_retired(1000) > 0) {
  }
  // The odometry owner has stopped; retaining an empty reuse cache through the
  // backend's final drain only overlaps two independent memory peaks.
  voxel_map_.trim_window_pool(0, std::numeric_limits<std::size_t>::max());
  malloc_trim(0);
  // Caller signals end-of-input on both normal and exceptional exits.
}

}  // namespace sapphire
