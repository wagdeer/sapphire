#include "backend/visual/feature/visual_features.hpp"
#include "backend/visual/utils/gray_image.hpp"
#include "backend/visual/solver/visual_stereo.hpp"
#include "pipeline.hpp"
#include "common/camera/camera.hpp"

#include <malloc.h>
#include <sys/resource.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

#include "frontend/initialize/initialize.hpp"
#include "backend/grid/ground_estimator.hpp"
#include "backend/grid/local_gridmap.hpp"
#include "frontend/eskf/lm_optimizer.hpp"
#include "backend/storage/map_database.hpp"
#include "backend/visual/visual_loop.hpp"
#include "backend/visual/feature/visual_tracker.hpp"
#include "tools/timer.hpp"

namespace sapphire {
namespace {

std::string current_time_filename() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time;
  localtime_r(&now_time, &local_time);
  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y-%m-%d_%H-%M-%S");
  return stream.str();
}


SapphireParameters validated(SapphireParameters parameters, bool attachment) {
  validate_parameters(parameters);
  if (attachment && parameters.pose_graph.automatic_attachment)
    throw std::invalid_argument("Choose explicit or automatic attachment, not both");
  if (parameters.pose_graph.map_mode != "new" && !(parameters.pose_graph.map_mode == "resume" &&
      (attachment || parameters.pose_graph.automatic_attachment))) {
    MapDatabase eligible(parameters.pose_graph.database_path, parameters.pose_graph.map_mode,
                         map_config_identity(parameters.pose_graph, parameters.navi_map));
    throw MapError(MapErrorCode::ResumeUnavailable,
                   "Historical frontend resume requires an explicit attachment or map.automatic_attachment=true");
  }
  return parameters;
}

template <typename Callback, typename... Args>
void invoke_output(const Callback &callback, Args &&...args) {
  if (!callback) {
    return;
  }
  try {
    callback(std::forward<Args>(args)...);
  } catch (const std::exception &error) {
    spdlog::error("Output callback failed: {}", error.what());
  } catch (...) {
    spdlog::error("Output callback failed with an unknown exception");
  }
}

struct ScanCompletion {
  const std::function<void(double, double)>& callback;
  double stamp;
  timer::Clock::time_point start = timer::Clock::now();
  ~ScanCompletion() { invoke_output(callback, stamp, timer::ms(start)); }
};

}  // namespace

SlamPipeline::SlamPipeline(SapphireParameters parameters, OutputSink output,
                           std::optional<std::pair<int, Eigen::Isometry3d>> attachment)
    : parameters_(validated(std::move(parameters), bool(attachment))),
      attachment_(std::move(attachment)),
      output_(std::move(output)),
      synchronizer_(parameters_.sensor.max_scan_duration),
      parallel_executor_(parameters_.local_submap.thread_num),
      voxel_map_(parameters_.odometry, parameters_.local_submap) {
  filename_ = current_time_filename();
  save_path_ = parameters_.general.save_path;
  save_map_ = parameters_.general.save_map;
  if (parameters_.odometry.gicp_fallback.enabled) ricp_ = std::make_unique<Ricp>(parameters_.odometry.gicp_fallback);
  down_size_inv_ = parameters_.odometry.down_size_inv;
  degrade_bound_ = parameters_.odometry.degrade_bound;
  imu_estimator_.configure(parameters_.sensor, parameters_.initializer, parameters_.odometry);
  extrinsic_.R = imu_estimator_.Lid_rot_to_IMU;
  extrinsic_.p = imu_estimator_.Lid_offset_to_IMU;
  window_size_ = parameters_.local_submap.win_size;

  if (!parameters_.pose_graph.database_path.empty()) create_pose_graph();
  if (save_map_) {
    std::filesystem::create_directories(std::filesystem::path(save_path_) / filename_);
  } else if (parameters_.pose_graph.enabled && parameters_.pose_graph.database_path.empty()) {
    std::filesystem::create_directories(std::filesystem::path(save_path_) / filename_);
  }
  if (parameters_.pose_graph.database_path.empty()) create_pose_graph();
  init_buffer_.reserve(window_size_);
  spdlog::info("filename: {}", filename_);

  try {
    if (parameters_.pose_graph.automatic_attachment) association_thread_ = std::thread([this] {
      try { thd_association(); }
      catch (...) {
        { std::lock_guard<std::mutex> lock(pose_graph_mutex_); if (!producer_failure_) producer_failure_ = std::current_exception(); }
        invalidateOdometryDomain();
      }
    });
    odometry_thread_ = std::thread([this] {
      try { thd_odometry(); } catch (...) { { std::lock_guard<std::mutex> lock(pose_graph_mutex_); if (!producer_failure_) producer_failure_ = std::current_exception(); } invalidateOdometryDomain(); }
      { std::lock_guard<std::mutex> lock(marginal_mutex_); odometry_done_.store(true); }
      marginal_cv_.notify_all();
    });
    mapping_thread_ = std::thread([this] {
      try { thd_mapping(); } catch (...) { { std::lock_guard<std::mutex> lock(pose_graph_mutex_); if (!producer_failure_) producer_failure_ = std::current_exception(); } invalidateOdometryDomain(); }
      { std::lock_guard<std::mutex> lock(association_mutex_); association_producer_done_ = true; }
      association_cv_.notify_all();
    });
  } catch (...) {
    { std::lock_guard<std::mutex> lock(association_mutex_); association_producer_done_ = true; }
    shutdown(); throw;
  }
}

SlamPipeline::~SlamPipeline() {
  shutdown();
  std::unique_ptr<PoseGraphBackend> owner;
  { std::lock_guard<std::mutex> lock(pose_graph_mutex_); owner = std::move(pose_graph_); }
  owner.reset(); // Never hold the owner mutex across backend join/close.
  clear_imu_factors();
}

bool SlamPipeline::push_imu(ImuMeas imu) {
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

bool SlamPipeline::push_lidar(double timestamp, std::vector<LidarPoint> &&cloud, std::optional<double> end_timestamp) {
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

bool SlamPipeline::push_image(ImageMeas image) {
  const auto &config = parameters_.pose_graph.visual;
  if (!config.enabled || !accepting_.load(std::memory_order_acquire)) return false;
  if (image.camera_id > (config.mode == "stereo" ? 1U : 0U)) return false;
  const auto &camera = config.camera(image.camera_id);
  image.timestamp += camera.time_offset;
  if (!std::isfinite(image.timestamp) || image.gray.type() != CV_8UC1 ||
      image.gray.cols != (camera.input_width ? camera.input_width : camera.width) ||
      image.gray.rows != (camera.input_height ? camera.input_height : camera.height))
    return false;
  std::lock_guard<std::mutex> lock(image_mutex_);
  if (image.timestamp <= last_image_time_[image.camera_id]) return false;
  if (image.timestamp - last_image_time_[image.camera_id] < ((config.tracking_enabled || config.keyframe_selection) ? config.tracking_interval : config.image_interval)) {
    ++image_samples_skipped_;
    if (image_samples_skipped_ == 1 || image_samples_skipped_ % 100 == 0)
      spdlog::info("[visual-loop] image interval sampling skipped total={}", image_samples_skipped_);
    return true;
  }
  last_image_time_[image.camera_id] = image.timestamp;
  // Own the bounded asynchronous packet even when a caller reuses its buffers.
  if (image.gray.cols == 2 * camera.width && image.gray.rows == 2 * camera.height)
    image.gray = halfGrayArea(image.gray);
  else if (image.gray.cols != camera.width || image.gray.rows != camera.height)
    cv::resize(image.gray, image.gray, cv::Size(camera.width, camera.height), 0, 0, cv::INTER_AREA);
  else
    image.gray = image.gray.clone();
  // Pixels now have their own allocation; do not retain an unaccounted raw ROS
  // message (or arbitrary caller owner) alongside the bounded processed image.
  image.owner.reset();
  constexpr std::size_t image_byte_budget = 32 * 1024 * 1024;
  const auto bytes = image.gray.total() * image.gray.elemSize();
  if (bytes > image_byte_budget) return false;
  while (image_buffer_.size() >= ((config.tracking_enabled || config.keyframe_selection) ? 64U : 16U) || image_buffer_bytes_ + bytes > image_byte_budget) {
    image_buffer_bytes_ -= image_buffer_.front().gray.total() * image_buffer_.front().gray.elemSize();
    image_buffer_.pop_front(); ++image_evictions_;
    if (image_evictions_ == 1 || image_evictions_ % 100 == 0)
      spdlog::warn("[visual-loop] image sampling queue eviction total={}", image_evictions_);
  }
  image_buffer_bytes_ += bytes;
  image_buffer_.emplace_back(std::move(image));
  return true;
}

void SlamPipeline::shutdown() {
  accepting_.store(false, std::memory_order_release);
  synchronizer_.stop_accepting();
  stopping_.store(true, std::memory_order_release);
  wakeStoppedProducers();
  // Resume capacity wait must be interrupted before joining its producer.
  // New-map's existing producer still flushes its final short builder on shutdown.
  { std::lock_guard<std::mutex> lock(pose_graph_mutex_);
    if (pose_graph_ && parameters_.pose_graph.map_mode == "resume" && !parameters_.pose_graph.automatic_attachment) pose_graph_->stopAdmission();
  }
  if (odometry_thread_.joinable()) {
    odometry_thread_.join();
  }
  if (mapping_thread_.joinable()) {
    mapping_thread_.join();
  }
  { std::lock_guard<std::mutex> lock(association_mutex_); association_producer_done_ = true; }
  association_cv_.notify_all();
  if (association_thread_.joinable()) association_thread_.join();
  if (parameters_.pose_graph.automatic_attachment) {
    std::lock_guard<std::mutex> lock(pose_graph_mutex_);
    if (pose_graph_) pose_graph_->stopAdmission();
  }
}

std::string SlamPipeline::backend_database_path() const {
  if (!parameters_.pose_graph.database_path.empty()) return parameters_.pose_graph.database_path;
  return (std::filesystem::path(save_path_) / filename_ / "map.db").string();
}

void SlamPipeline::create_pose_graph() {
  if (output_.owner_reset) output_.owner_reset(true);
  std::unique_ptr<PoseGraphBackend> old;
  { std::lock_guard<std::mutex> lock(pose_graph_mutex_); old = std::move(pose_graph_); }
  try {
    if (old) { old->finish(); old.reset(); }
    database_detail::writableStorageTestPoint("pipeline-old-backend-destroyed");
    auto replacement = std::make_unique<PoseGraphBackend>(
        parameters_.pose_graph, parameters_.navi_map, backend_database_path(),
        [this](std::shared_ptr<const NavigationGrid> map) { invoke_output(output_.navigation_grid, std::move(map)); },
        [this] {
          // Backend invokes this only outside its locks. Signal existing owners; never join/reenter backend here.
          failed_.store(true); accepting_.store(false); stopping_.store(true);
          synchronizer_.stop_accepting(); wakeStoppedProducers();
        }, output_.ready_changed);
    { std::lock_guard<std::mutex> lock(pose_graph_mutex_);
      pose_graph_ = std::move(replacement); owner_generation_ = session_id_.load();
    }
    output_domain_current_.store(true);
    if (output_.owner_reset) output_.owner_reset(false);
  } catch (...) {
    const auto cause = std::current_exception();
    { std::lock_guard<std::mutex> lock(pose_graph_mutex_);
      if (!producer_failure_) producer_failure_ = cause;
    }
    output_domain_current_.store(false);
    failed_.store(true, std::memory_order_release);
    accepting_.store(false, std::memory_order_release);
    synchronizer_.stop_accepting();
    stopping_.store(true, std::memory_order_release);
    wakeStoppedProducers();
    // End the pause on failure too. Failure/current-domain fences are already
    // visible; this releases final cleanup, not admission of a new output domain.
    invoke_output(output_.owner_reset, false);
    std::rethrow_exception(cause);
  }
}

void SlamPipeline::emit_local_trajectory(const vvec<double, 3> &points, double journey) {
  TrajectoryPoint point;
  point.x = static_cast<float>(current_state_.p.x());
  point.y = static_cast<float>(current_state_.p.y());
  point.z = static_cast<float>(current_state_.p.z());
  point.distance = static_cast<float>(journey);
  point.session = static_cast<float>(session_id_.load());
  trajectory_.push_back(point);

  invoke_output(output_.odom_state, std::cref(current_state_));
  if (output_.local_scan) {
    try {
      if (points.size() > (16 * 1024 * 1024 - 65536) / 64) throw std::length_error("optional scan cap");
      database_detail::writableStorageTestPoint("b3-scan-copy");
      const Eigen::Vector3d sensor_origin = current_state_.p + current_state_.R *
          Eigen::Map<const Eigen::Vector3d>(parameters_.sensor.extrinsic_tran.data());
      output_.local_scan(std::make_shared<vvec<double, 3>>(points), sensor_origin, current_state_.t, session_id_.load());
    } catch (...) { invoke_output(output_.optional_drop, false); }
  }
  if (output_.trajectory) {
    try {
      if (trajectory_.size() > (16 * 1024 * 1024 - 65536) / 64) throw std::length_error("optional trajectory cap");
      database_detail::writableStorageTestPoint("b3-trajectory-copy");
      output_.trajectory(std::make_shared<std::vector<TrajectoryPoint>>(trajectory_));
    } catch (...) { invoke_output(output_.optional_drop, false); }
  }
}

void SlamPipeline::emit_local_map(int marginal_count) {
  for (int i = 0; i < window_count_; ++i) {
    trajectory_[i + window_base_].x = static_cast<float>(state_buffer_[i].p.x());
    trajectory_[i + window_base_].y = static_cast<float>(state_buffer_[i].p.y());
    trajectory_[i + window_base_].z = static_cast<float>(state_buffer_[i].p.z());
  }

  if (output_.trajectory) {
    try {
      if (trajectory_.size() > (16 * 1024 * 1024 - 65536) / 64) throw std::length_error("optional trajectory cap");
      database_detail::writableStorageTestPoint("b3-trajectory-copy");
      output_.trajectory(std::make_shared<std::vector<TrajectoryPoint>>(trajectory_));
    } catch (...) { invoke_output(output_.optional_drop, false); }
  }
  if (output_.local_map) {
    try {
      constexpr std::size_t limit = (16 * 1024 * 1024 - 65536) / 64;
      std::size_t count = 0;
      for (int i = 0; i < marginal_count; ++i) {
        const auto n = point_buffer_[i]->size() / 3 + (point_buffer_[i]->size() % 3 != 0);
        if (n > limit - count) throw std::length_error("optional local-map cap");
        count += n;
      }
      database_detail::writableStorageTestPoint("b3-local-map-copy");
      auto map = std::make_shared<vvec<double, 3>>(); map->reserve(count);
      for (int i = 0; i < marginal_count; ++i)
        for (size_t j = 0; j < point_buffer_[i]->size(); j += 3) {
          const pointVar &point = point_buffer_[i]->at(j);
          map->push_back(state_buffer_[i].R * point.pnt + state_buffer_[i].p);
        }
      output_.local_map(std::move(map));
    } catch (...) { invoke_output(output_.optional_drop, true); }
  }
}

void SlamPipeline::clear_imu_factors() {
  pending_imu_factor_.reset();
  for (ImuFactor *factor : imu_factor_buffer_) {
    delete factor;
  }
  imu_factor_buffer_.clear();
}

int SlamPipeline::initialization(MeasGroup &measures, Eigen::MatrixXd &hess, LidarFactor &voxel_hessian, vvec<double, 3> &world_points) {
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
  emit_local_trajectory(world_points, 0.0);

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

bool SlamPipeline::update_lidar_window(const PointCloudPtr &points, const std::deque<ImuMeas> &imus,
                                       double journey, LidarFactor &voxel_hessian) {
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
  if (!accepted && ricp_ &&
      (reason == ESKF::Rejection::normal_support || reason == ESKF::Rejection::no_matches)) {
    const auto fallback_begin = timer::Clock::now();
    const auto result = ricp_->observe(current_state_, *points, voxel_map_);
    accepted = result.accepted;
    spdlog::info("[gicp-fallback] t={:.9f} accepted={} reason={} samples={} matches={} targets={} iterations={} rms={:.5f} correction={:.5f} ms={:.3f}",
      current_state_.t, accepted, result.reason, result.samples, result.matches, result.targets, result.iterations,
      result.rms, result.correction, timer::ms(fallback_begin));
  }
  invoke_output(output_.lidar_update, std::cref(current_state_), accepted, timer::ms(observation_begin));
  if (!accepted) return false;

  vvec<double, 3> world_points;
  pvec_update(points, current_state_, world_points);
  emit_local_trajectory(world_points, journey);
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

void SlamPipeline::system_reset(const std::deque<ImuMeas> &imus) {
  output_domain_current_.store(false);
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
  trajectory_.clear();
  spdlog::warn("Reset");
}

void SlamPipeline::thd_odometry() {
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
        invalidateOdometryDomain(); break;
      }
      if (stopping_.load(std::memory_order_acquire)) {
        break;
      }
      if (voxel_map_.release_retired(1000) > 0) {
        malloc_trim(0);
      } else if (release_flag) {
        release_flag = false;
        voxel_map_.prune(journey, 700.0);
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
          invalidateOdometryDomain(); break; // old live-domain type-0 must end before frontend reset
        }
        degrade_count = 0;
        system_reset(imus);
        last_position = current_state_.p;
        journey = 0.0;
        {
          std::lock_guard<std::mutex> lock(marginal_mutex_);
          reset_tail_.swap(marginal_frames_);
          reset_flag_ = 1;
        }
        marginal_cv_.notify_one();
        motion_init_flag = 1;
        continue;
      }
      if (!accepted) {
        // Publish prediction only; the indexed trajectory and scans belong to
        // accepted window frames and must not contaminate downstream maps.
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
      {
        const auto wait_start = timer::Clock::now();
        std::unique_lock<std::mutex> lock(marginal_mutex_);
        marginal_cv_.wait(lock, [this] { return marginal_frames_.size() < 2 || stopping_.load() || failed_.load(); });
        const double wait_ms = timer::ms(wait_start);
        if (timer::enabled() && wait_ms > 1.0)
          spdlog::info("[backend-profile] marginal_wait stamp={:.9f} ms={:.3f}", measures.lidar_end_time, wait_ms);
        if (stopping_.load() || failed_.load()) break;
      }
      GaussianCloud marginalized = voxel_map_.marginalize(journey, window_count_, marginal_count, state_buffer_, voxel_hessian);

      if (marginalized.size() > 16 * 1024 * 1024 / sizeof(GaussianPoint))
        throw std::length_error("Marginal evidence exceeds finite producer bound");
      {
        std::lock_guard<std::mutex> lock(marginal_mutex_);
        marginal_frames_.emplace_back(state_buffer_[0], std::move(marginalized), time_buffer_[0], marginal_variance, journey_buffer_[0]);
      }
      marginal_cv_.notify_one();

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
        spdlog::info("[memory-profile] stamp={} trees={} active_windows={} pooled_windows={} tree_bytes={} window_bytes={} pooled_window_bytes={} point_capacity_bytes={} empty_slot_bytes={} leaf_point_bytes={} heap_used={} heap_free={} mmap_bytes={} peak_rss_bytes={}",
            measures.lidar_end_time, memory.trees, memory.active_windows, memory.pooled_windows, memory.tree_bytes,
            memory.window_bytes, memory.pooled_window_bytes, memory.point_capacity_bytes, memory.empty_slot_bytes, memory.leaf_point_bytes,
            heap.uordblks, heap.fordblks, heap.hblkhd, std::size_t(usage.ru_maxrss) * 1024);
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
  // The thread entry records odometry_done under marginal_mutex_ on both exits.
}

void SlamPipeline::thd_mapping() {
  SubmapFrameBuffer submap_buffer(parameters_.pose_graph.submap_voxel_size_inv, parameters_.pose_graph.submap_travel_distance,
                                  parameters_.pose_graph.submap_max_point_range);
  const std::string initial_filename = filename_;
  const auto &visual = parameters_.pose_graph.visual;
  const bool attributes = visual.enabled && visual.attributes_only;
  std::array<std::unique_ptr<VisualKeyframeSelector>, 2> image_selectors;
  if (attributes) for (std::size_t id = 0; id < (visual.mode == "stereo" ? 2U : 1U); ++id)
    image_selectors[id] = std::make_unique<VisualKeyframeSelector>(visual, id);
  const bool tracking = visual.loopEnabled() && visual.tracking_enabled;
  const bool stereo_depth = visual.loopEnabled() && visual.stereo_depth;
  std::unique_ptr<StereoVisualProcessor> stereo;
  if (stereo_depth) stereo = std::make_unique<StereoVisualProcessor>(visual);
  std::array<std::unique_ptr<VisualTracker>, 2> trackers;
  std::array<Eigen::Isometry3d, 2> body_camera;
  if (tracking || stereo_depth || attributes) for (std::size_t id = 0; id < (visual.mode == "stereo" ? 2U : 1U); ++id) {
    VisualTrackingParameters options;
    options.max_features = std::min(600, visual.max_features);
    options.max_anchors = std::min(3, visual.max_frames);
    if (tracking) trackers[id] = std::make_unique<VisualTracker>(visual.camera(id), id, options);
    body_camera[id] = CameraModel(visual.camera(id)).cameraToImu().cast<double>();
  }
  std::optional<OdomPose> previous_visual_body;
  double last_usable_image = -std::numeric_limits<double>::infinity();
  std::uint64_t tracked_images = 0, described_images = 0, posed_images = 0, geometry_rows = 0, gap_resets = 0;
  double tracking_ms = 0, descriptor_ms = 0, affiliation_ms = 0;

  std::unique_ptr<GroundEstimator> ground;
  std::unique_ptr<LocalGridMaker> maker;
  if (parameters_.pose_graph.map_mode == "resume" && parameters_.navi_map.enabled) {
    ground = std::make_unique<GroundEstimator>(parameters_.navi_map);
    maker = std::make_unique<LocalGridMaker>(parameters_.navi_map);
  }
  bool attached = false;
  const auto submit_submap = [this, &ground, &maker, &attached](SubmapFrame submap) {
    const LioFrame lio = submap.lio();
    PoseGraphBackend *backend;
    { std::lock_guard<std::mutex> lock(pose_graph_mutex_); backend = pose_graph_.get(); }
    // Only this mapping thread replaces the owner. Shutdown requests stop before joining it.
    if (parameters_.pose_graph.map_mode == "resume") {
      if (stopping_.load() && !parameters_.pose_graph.automatic_attachment) { ++unaccepted_tail_drops_; return; }
      LocalGrid grid(static_cast<float>(parameters_.navi_map.resolution));
      if (maker) maker->createLocalMap(*lio.pcd, lio.T_odom_base.cast<float>(), grid, ground->update(submap));
      if (parameters_.pose_graph.automatic_attachment) {
        submitAutomaticSubmap(std::move(submap), std::move(grid)); return;
      }
      const auto generation = std::uint64_t(session_id_.load());
      if (!attached) {
        std::uint64_t tried = UINT64_MAX;
        for (;;) {
          std::pair<int, Eigen::Isometry3d> request;
          {
            std::unique_lock<std::mutex> lock(attachment_mutex_);
            attachment_cv_.wait(lock, [&] { return stopping_.load() || failed_.load() || (attachment_ && tried != attachment_attempt_); });
            if (stopping_.load() || failed_.load()) {
              ++unaccepted_tail_drops_;
              spdlog::warn("Unattached frozen sequence {} canceled before admission", submap.id()); return;
            }
            request = *attachment_; tried = attachment_attempt_;
          }
          const auto result = backend->attachFreshSessionRetained(submap, grid, request.first, request.second, generation);
          if (result.status == AttachmentStatus::Attached) {
            attached = true;
            if (!stopping_.load()) {
              try { backend->beginContinuation(generation); }
              catch (...) { if (!stopping_.load()) throw; }
            }
            invoke_output(output_.ready_changed); break;
          }
          spdlog::warn("B1 rejected; same frozen producer head retained for explicit target/seed retry: {}", result.message);
        }
      } else {
        const auto result = backend->submitContinuation(submap, grid, generation);
        if (result != ContinuationAdmission::Accepted) {
          if (result == ContinuationAdmission::Stopped && stopping_.load()) {
            ++unaccepted_tail_drops_;
            spdlog::warn("Frozen sequence {} refused before transfer: producer stopping", submap.id()); return;
          }
          throw MapError(MapErrorCode::Lifecycle, "Continuation admission refused: " + std::to_string(int(result)));
        }
      }
      if (!backend->hasActiveCorrection()) return;
    } else backend->addFrame(std::move(submap));

  };

  const auto flush_submap = [&](const char *reason) {
    if (submap_buffer.empty()) return;
    std::size_t representatives = 0, metric = 0;
    if (tracking) for (auto &tracker : trackers) if (tracker) {
      for (auto &frame : tracker->copySubmapEvidence()) {
        ++representatives;
        for (const auto &point : frame.points()) metric += point.geometry().has_value();
        submap_buffer.push_visual(std::move(frame), visual.max_frames);
      }
    }
    spdlog::info("[submap] reason={} frames={} points={} travel_m={} representatives={} runtime_metric_rows={}",
        reason, submap_buffer.buffered_frame_count(), submap_buffer.buffered_point_count(),
        submap_buffer.travel_distance(), representatives, metric);
    if (auto ready = submap_buffer.flush()) submit_submap(std::move(*ready));
    if (tracking) for (auto &tracker : trackers) if (tracker) tracker->beginSubmap();
    if (stereo) stereo->beginSubmap();
    for (auto &selector : image_selectors) if (selector) selector->beginSubmap();
  };

  while (true) {
    std::deque<MargiFrame, Eigen::aligned_allocator<MargiFrame>> reset_tail;
    std::optional<MargiFrame> marginal_frame;
    bool switch_session = false;
    {
      std::unique_lock<std::mutex> lock(marginal_mutex_);
      marginal_cv_.wait(lock, [this] {
        return reset_flag_ == 1 || !marginal_frames_.empty() ||
               (stopping_.load(std::memory_order_acquire) && odometry_done_.load(std::memory_order_acquire));
      });
      if (reset_flag_ == 1) {
        reset_flag_ = 0;
        reset_tail.swap(reset_tail_);
        switch_session = true;
      } else if (!marginal_frames_.empty()) {
        marginal_frame.emplace(std::move(marginal_frames_.front()));
        marginal_frames_.pop_front();
        marginal_cv_.notify_all();
      } else if (stopping_.load(std::memory_order_acquire) && odometry_done_.load(std::memory_order_acquire)) {
        break;
      }
    }

    if (failed_.load()) break;
    if (parameters_.pose_graph.map_mode == "resume" && !parameters_.pose_graph.automatic_attachment && stopping_.load()) {
      unaccepted_tail_drops_ += bool(marginal_frame); continue;
    }
    if (switch_session) {
      submap_buffer.clear();
      for (auto &tracker : trackers) if (tracker) tracker->reset();
      if (stereo) stereo->reset();
      for (auto &selector : image_selectors) if (selector) selector->reset();
      previous_visual_body.reset();
      last_usable_image = -std::numeric_limits<double>::infinity();
      {
        std::lock_guard<std::mutex> lock(image_mutex_);
        image_buffer_.clear();
        image_buffer_bytes_ = 0;
        last_image_time_.fill(-std::numeric_limits<double>::infinity());
      }
      const int session = session_id_.fetch_add(1) + 1;
      filename_ = initial_filename + std::to_string(session);
      if (save_map_) {
        std::filesystem::create_directories(std::filesystem::path(save_path_) / filename_);
      } else if (parameters_.pose_graph.enabled) {
        std::filesystem::create_directories(std::filesystem::path(save_path_) / filename_);
      }
      try {
        create_pose_graph();
      } catch (const std::exception &error) {
        spdlog::error("Backend replacement failed; pipeline stopped: {}", error.what());
        break;
      }
      continue;
    }

    if (!marginal_frame) {
      continue;
    }

    std::deque<ImageMeas> images;
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      for (auto it = image_buffer_.begin(); it != image_buffer_.end();) {
        if (it->timestamp <= marginal_frame->timestamp) {
          image_buffer_bytes_ -= it->gray.total() * it->gray.elemSize();
          images.emplace_back(std::move(*it));
          it = image_buffer_.erase(it);
        } else
          ++it;
      }
    }
    Eigen::Isometry3d current_body = Eigen::Isometry3d::Identity();
    current_body.linear() = marginal_frame->x.R;
    current_body.translation() = marginal_frame->x.p;
    for (ImageMeas &image : images) {
      if (attributes) {
        std::optional<Eigen::Isometry3d> camera_pose;
        if (previous_visual_body)
          camera_pose = interpolateVisualCameraPose(image.timestamp, previous_visual_body->timestamp,
              previous_visual_body->T_odom_base, marginal_frame->timestamp, current_body, body_camera[image.camera_id]);
        auto &selector = *image_selectors[image.camera_id];
        const auto decision = selector.evaluate(image, camera_pose);
        std::size_t bytes = 0;
        if (decision.selected) {
          auto frame = makeImageAttribute(image, visual.camera(image.camera_id), *camera_pose);
          bytes = frame.image_png().size();
          submap_buffer.push_visual(std::move(frame), visual.max_frames);
          selector.accept();
        }
        if (timer::enabled())
          spdlog::info("[visual-image] timestamp={} camera={} selected={} tracks={} coverage={} renewal={} translation_m={} rotation_deg={} flow_ms={} png_bytes={}",
              image.timestamp, image.camera_id, decision.selected, decision.tracks, decision.coverage, decision.renewal,
              decision.translation_m, decision.rotation_deg, decision.flow_ms, bytes);
      } else if (stereo) {
        std::optional<Eigen::Isometry3d> camera_pose;
        if (previous_visual_body)
          camera_pose = interpolateVisualCameraPose(image.timestamp, previous_visual_body->timestamp,
              previous_visual_body->T_odom_base, marginal_frame->timestamp, current_body, body_camera[image.camera_id]);
        auto result = stereo->push(std::move(image), camera_pose);
        if (!result) continue;
        const auto &r = *result;
        if (visual.keyframe_selection && timer::enabled())
          spdlog::info("[visual-keyframe] timestamp={} tracks={} shared={} coverage={} renewal={} translation_m={} rotation_deg={} selected={} usable={} seed={} gap={} flow_ms={} descriptor_ms={} stereo_ms={} total_ms={}",
              r.frames[0].timestamp(), r.selection.tracks, r.selection.shared, r.selection.coverage, r.selection.renewal,
              r.selection.translation_m, r.selection.rotation_deg, r.keyframe, r.selection.usable, r.selection.seed, r.selection.gap,
              r.selection.flow_ms, r.descriptor_ms, r.stereo_ms, r.total_ms);
        if (r.usable) last_usable_image = std::max(last_usable_image, r.frames[0].timestamp());
        if (!r.keyframe) continue;
        spdlog::info("[visual-stereo] timestamp={} left={} right={} matches={} metric={} posed={} usable={} candidate={} total_ms={} dropped={} pending={}",
            r.frames[0].timestamp(), r.frames[0].points().size(), r.frames[1].points().size(), r.matches, r.metric,
            bool(r.frames[0].T_odom_camera()) && bool(r.frames[1].T_odom_camera()), r.usable, r.submap_candidate,
            r.total_ms, stereo->dropped(), stereo->pending());
        if (r.usable) last_usable_image = std::max(last_usable_image, r.frames[0].timestamp());
        if (r.submap_candidate) flush_submap("stereo-visual");
        for (auto &frame : result->frames) if (frame.has_features()) submap_buffer.push_visual(std::move(frame), visual.max_frames);
      } else if (tracking) {
        std::optional<Eigen::Isometry3d> camera_pose;
        if (previous_visual_body)
          camera_pose = interpolateVisualCameraPose(image.timestamp, previous_visual_body->timestamp,
              previous_visual_body->T_odom_base, marginal_frame->timestamp, current_body, body_camera[image.camera_id]);
        auto result = trackers[image.camera_id]->process(image, camera_pose);
        ++tracked_images; posed_images += bool(camera_pose); described_images += bool(result.keyframe);
        geometry_rows += result.stats.triangulated; gap_resets += result.stats.reset_by_gap;
        tracking_ms += result.stats.tracking_ms; descriptor_ms += result.stats.descriptor_ms;
        affiliation_ms += result.stats.affiliation_ms;
        if (timer::enabled())
          spdlog::info("[visual-tracking-profile] camera={} timestamp={} posed={} tracks={} mature={} triangulated={} describe={} usable={} candidate={} tracking_ms={} descriptor_ms={} affiliation_ms={}",
              image.camera_id, image.timestamp, bool(camera_pose), result.stats.tracks, result.stats.mature_tracks,
              result.stats.triangulated, bool(result.keyframe), result.stats.usable, result.stats.submap_candidate,
              result.stats.tracking_ms, result.stats.descriptor_ms, result.stats.affiliation_ms);
        if (result.stats.usable) last_usable_image = std::max(last_usable_image, image.timestamp);
        if (result.stats.submap_candidate) flush_submap("visual");
      } else {
        try {
          auto frame = extractLoopFeatures(image, visual);
          if (frame.has_features()) submap_buffer.push_visual(std::move(frame), visual.max_frames);
        } catch (const std::exception &error) {
          spdlog::warn("[visual-loop] feature extraction rejected: {}", error.what());
        }
      }
    }
    previous_visual_body = OdomPose{marginal_frame->timestamp, current_body};

    // Bound stationary accumulation and conservative worst-case pyramid scratch.
    if (submap_buffer.buffered_frame_count() >= 256 ||
        submap_buffer.buffered_point_count() + marginal_frame->gaussians.size() > 24000) {
      flush_submap("resource");
    }
    if (marginal_frame->gaussians.size() > 24000) throw std::length_error("Marginal evidence exceeds builder point bound");
    const bool visual_unavailable = marginal_frame->timestamp - last_usable_image > 1.0;
    std::optional<SubmapFrame> submap = submap_buffer.push(std::move(*marginal_frame), !tracking && !stereo_depth);
    if (submap) {
      submit_submap(std::move(*submap));
      for (auto &selector : image_selectors) if (selector) selector->beginSubmap();
    }
    if ((tracking || stereo_depth) && visual_unavailable && submap_buffer.travel_distance() >= submap_buffer.target_distance())
      flush_submap("distance-fallback");
  }

  if (tracking) spdlog::info("[visual-tracking] images={} posed={} described={} triangulated_rows={} gap_resets={} tracking_ms={} descriptor_ms={} affiliation_ms={}",
      tracked_images, posed_images, described_images, geometry_rows, gap_resets, tracking_ms, descriptor_ms, affiliation_ms);
  if (tracking || stereo_depth || attributes) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    spdlog::info("[visual-tracking-queue] evicted={} sampled_out={} remaining={} remaining_bytes={}",
        image_evictions_, image_samples_skipped_, image_buffer_.size(), image_buffer_bytes_);
  }
  if (failed_.load(std::memory_order_acquire)) return;
  if (parameters_.pose_graph.map_mode == "resume" && !parameters_.pose_graph.automatic_attachment && stopping_.load()) {
    unaccepted_tail_drops_ += submap_buffer.buffered_frame_count();
    submap_buffer.clear(); return;
  }
  flush_submap("tail");
  malloc_trim(0);
}

}  // namespace sapphire

namespace sapphire {
void SlamPipeline::setAttachmentTarget(int target, const Eigen::Isometry3d &seed) {
  if (parameters_.pose_graph.automatic_attachment) throw std::logic_error("Manual target unavailable in automatic mode");
  if (target <= 0 || !seed.matrix().allFinite()) throw std::invalid_argument("Invalid explicit attachment target/seed");
  { std::lock_guard<std::mutex> lock(attachment_mutex_); attachment_ = std::make_pair(target, seed); ++attachment_attempt_; }
  attachment_cv_.notify_all();
}
void SlamPipeline::invalidateOdometryDomain() {
  output_domain_current_.store(false);
  failed_.store(true); accepting_.store(false); stopping_.store(true);
  { std::lock_guard<std::mutex> lock(pose_graph_mutex_); if (pose_graph_) pose_graph_->stopAdmission(true); }
  synchronizer_.stop_accepting();
  wakeStoppedProducers();
}
}

namespace sapphire {
void SlamPipeline::notifyInputLoss() {
  if (parameters_.pose_graph.map_mode == "resume") invalidateOdometryDomain();
}
std::exception_ptr SlamPipeline::producerFailure() const {
  std::lock_guard<std::mutex> lock(pose_graph_mutex_); return producer_failure_;
}
}

namespace sapphire {
ContinuationProgress SlamPipeline::continuationProgress() const {
  std::lock_guard<std::mutex> lock(pose_graph_mutex_);
  auto progress = pose_graph_ ? pose_graph_->continuationProgress() : ContinuationProgress{};
  if (parameters_.pose_graph.map_mode != "resume" && progress.completed_revision) progress.generation = owner_generation_;
  {
    std::lock_guard<std::mutex> queue(association_mutex_);
    progress.association_pending = association_queue_.size(); progress.association_bytes = association_bytes_;
    progress.association_high_water = association_high_water_; progress.association_attempts = association_attempts_;
    progress.automatically_attached = automatically_attached_;
  }
  return progress;
}
void SlamPipeline::retryContinuation(std::uint64_t sequence) {
  std::lock_guard<std::mutex> lock(pose_graph_mutex_); pose_graph_->retryContinuation(sequence, session_id_.load());
}
}

namespace sapphire {
void SlamPipeline::wakeStoppedProducers() {
  // Flag stores precede these lock handshakes. A waiter cannot miss the stop
  // between evaluating its predicate and atomically releasing its queue lock.
  { std::lock_guard<std::mutex> lock(input_mutex_); } input_cv_.notify_all();
  { std::lock_guard<std::mutex> lock(marginal_mutex_); } marginal_cv_.notify_all();
  { std::lock_guard<std::mutex> lock(attachment_mutex_); } attachment_cv_.notify_all();
  { std::lock_guard<std::mutex> lock(association_mutex_); } association_cv_.notify_all();
}
}

namespace sapphire {
ContinuationProgress SlamPipeline::drain() {
  shutdown();
  PoseGraphBackend *backend;
  { std::lock_guard<std::mutex> lock(pose_graph_mutex_); backend = pose_graph_.get(); }
  if (backend) backend->drain();
  if (auto error = producerFailure()) std::rethrow_exception(error);
  if (failed()) throw MapError(MapErrorCode::Lifecycle, "Pipeline failed before drain");
  auto progress = continuationProgress();
  if (progress.association_pending)
    throw MapError(MapErrorCode::ResumeUnavailable, "Automatic attachment unresolved: " +
        std::to_string(progress.association_pending) + " frozen observations remain uncommitted");
  if (progress.canceled_first || progress.waiting || progress.head_bytes ||
      progress.accepted != progress.completed || !progress.committed_outcome_known ||
      (progress.ready_available && progress.committed_revision != progress.ready_revision))
    throw MapError(MapErrorCode::Lifecycle, "Incomplete backend drain");
  return progress;
}
void SlamPipeline::close() {
  PoseGraphBackend *backend;
  { std::lock_guard<std::mutex> lock(pose_graph_mutex_); backend = pose_graph_.get(); }
  if (backend) backend->close();
}
bool SlamPipeline::captureReady(bool navigation, bool try_only, std::string &uuid, std::uint64_t &revision,
    std::uint64_t &generation, std::uint64_t &source, double &timestamp, Eigen::Isometry3d &correction,
    Eigen::Isometry3d &pose, std::shared_ptr<const NavigationGrid> &grid) const {
  std::unique_lock<std::mutex> lock(pose_graph_mutex_, std::defer_lock);
  if (try_only) { if (!lock.try_lock()) return false; } else lock.lock();
  if (!pose_graph_ || !output_domain_current_.load()) return false;
  auto result = pose_graph_->captureReady(navigation, try_only, uuid, revision, generation, source, timestamp, correction, pose, grid);
  if (parameters_.pose_graph.map_mode != "resume") generation = owner_generation_;
  return result;
}
}

namespace sapphire {
void SlamPipeline::submitAutomaticSubmap(SubmapFrame submap, LocalGrid grid) {
  const auto bytes = PoseGraphBackend::boundedInputBytes(submap,grid);
  if (!bytes) { ++unaccepted_tail_drops_; throw std::length_error("Automatic association item exceeds 64 MiB input bound"); }
  std::unique_lock<std::mutex> lock(association_mutex_);
  const auto fits = [&] { return association_queue_.size() < std::size_t(parameters_.pose_graph.association_pending_submaps) &&
      *bytes <= std::size_t(parameters_.pose_graph.association_pending_mb)*1024*1024 - association_bytes_; };
  if (!automatically_attached_ && !fits()) {
    ++unaccepted_tail_drops_; throw std::length_error("Automatic association pending budget exhausted before initial root recognition");
  }
  if (*bytes > std::size_t(parameters_.pose_graph.association_pending_mb)*1024*1024) {
    ++unaccepted_tail_drops_; throw std::length_error("Automatic association item exceeds configured pending byte bound");
  }
  association_cv_.wait(lock,[&] { return fits() || failed_.load() || association_producer_done_; });
  if (failed_.load() || association_producer_done_) throw MapError(MapErrorCode::Lifecycle,"Automatic producer has stopped");
  if (submap.id()!=association_received_) throw MapError(MapErrorCode::Lifecycle,"Noncontiguous automatic producer sequence");
  association_queue_.push_back({std::move(submap),std::move(grid),*bytes});
  ++association_received_; association_bytes_ += *bytes;
  association_high_water_ = std::max(association_high_water_,association_queue_.size());
  lock.unlock(); association_cv_.notify_all();
  invoke_output(output_.ready_changed);
}

void SlamPipeline::thd_association() {
  PoseGraphBackend *backend;
  { std::lock_guard<std::mutex> lock(pose_graph_mutex_); backend=pose_graph_.get(); }
  const auto generation=std::uint64_t(session_id_.load());
  std::uint64_t tried=0;
  const auto release_head = [this] {
    {
      std::lock_guard<std::mutex> lock(association_mutex_);
      association_bytes_-=association_queue_.front().bytes; association_queue_.pop_front();
    }
    association_cv_.notify_all();
    invoke_output(output_.ready_changed);
  };
  for (;;) {
    PendingAssociation *head, *view;
    bool attached;
    {
      std::unique_lock<std::mutex> lock(association_mutex_);
      association_cv_.wait(lock,[&] { return failed_.load() || association_producer_done_ ||
          (!association_queue_.empty() && (automatically_attached_ || association_received_>tried)); });
      if (failed_.load()) return;
      if (association_queue_.empty()) { if(association_producer_done_) return; else continue; }
      attached=automatically_attached_;
      if (!attached && tried==association_received_ && association_producer_done_) return;
      head=&association_queue_.front();
      view=attached ? head : &association_queue_.at(std::size_t(tried++));
    }
    // deque append preserves these element references; only this worker removes
    // or transfers elements. Producer reads counters, never in-flight payloads.
    if (!attached) {
      const auto result=backend->attachFreshSessionAutomatically(head->submap,head->grid,generation,&view->submap);
      { std::lock_guard<std::mutex> lock(association_mutex_); ++association_attempts_; }
      invoke_output(output_.ready_changed);
      if (result.status==AttachmentStatus::NoAssociation || result.status==AttachmentStatus::AmbiguousAssociation) {
        spdlog::info("[association] pending source={} status={} message={}",head->submap.id(),int(result.status),result.message);
        continue;
      }
      if (result.status!=AttachmentStatus::Attached)
        throw MapError(MapErrorCode::Lifecycle,"Automatic attachment failed: "+result.message);
      { std::lock_guard<std::mutex> lock(association_mutex_); automatically_attached_=true; }
      // B1 is already durable. Retire its pending accounting even if the
      // generation is canceled or the subsequent B2 handoff fails.
      release_head();
      if (failed_.load()) return;
      backend->beginContinuation(generation);
      continue;
    } else {
      for (;;) {
        if (failed_.load()) return;
        const auto admission=backend->submitContinuation(head->submap,head->grid,generation,false);
        if(admission==ContinuationAdmission::Accepted) break;
        if(admission!=ContinuationAdmission::Full)
          throw MapError(MapErrorCode::Lifecycle,"Automatic continuation refused: "+std::to_string(int(admission)));
        const auto progress=backend->continuationProgress();
        if(stopping_.load() && progress.head==ContinuationHead::Retryable)
          throw MapError(MapErrorCode::Lifecycle,"Automatic drain stopped at a retryable backend head");
        std::unique_lock<std::mutex> lock(association_mutex_);
        association_cv_.wait_for(lock,std::chrono::milliseconds(10));
      }
    }
    release_head();
  }
}
}
