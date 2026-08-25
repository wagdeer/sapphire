#include "pipeline.hpp"

#include <malloc.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>

#include "initialize.hpp"
#include "io_utils.hpp"
#include "lm_optimizer.hpp"

namespace sapphire {
namespace {

SapphireParameters validated(SapphireParameters parameters) {
  validate_parameters(parameters);
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

}  // namespace

bool Synchronizer::push_imu(ImuMeas imu) {
  if (!std::isfinite(imu.timestamp) || !imu.gyro.allFinite() || !imu.accel.allFinite()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!accepting_ || imu.timestamp <= last_imu_time_) {
    return false;
  }
  last_imu_time_ = imu.timestamp;
  imu_buf_.push_back(std::move(imu));
  return true;
}

bool Synchronizer::push_lidar(double timestamp, std::vector<LidarPoint> cloud) {
  if (!std::isfinite(timestamp) || cloud.empty()) {
    return false;
  }
  for (const LidarPoint &point : cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) || !std::isfinite(point.intensity) ||
        !std::isfinite(point.time_offset) || point.time_offset < 0.0F) {
      return false;
    }
  }
  std::stable_sort(cloud.begin(), cloud.end(), [](const LidarPoint &left, const LidarPoint &right) { return left.time_offset < right.time_offset; });
  const auto clipped =
      std::upper_bound(cloud.begin(), cloud.end(), 0.11F, [](float limit, const LidarPoint &point) { return limit < point.time_offset; });
  cloud.erase(clipped, cloud.end());
  if (cloud.empty()) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!accepting_ || timestamp <= last_lidar_time_) {
    return false;
  }
  last_lidar_time_ = timestamp;
  lidar_time_buf_.push_back(timestamp);
  lidar_buf_.push_back(std::make_shared<std::vector<LidarPoint>>(std::move(cloud)));
  return true;
}

bool Synchronizer::sync_packages(MeasGroup &measures) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!pending_.lidar_cloud) {
    if (lidar_buf_.empty()) {
      return false;
    }
    pending_.lidar_cloud = std::move(lidar_buf_.front());
    pending_.lidar_begin_time = lidar_time_buf_.front();
    lidar_buf_.pop_front();
    lidar_time_buf_.pop_front();
    pending_.lidar_end_time = pending_.lidar_begin_time + pending_.lidar_cloud->back().time_offset;
  }
  if (imu_buf_.empty() || imu_buf_.back().timestamp <= pending_.lidar_end_time) {
    return false;
  }

  while (!imu_buf_.empty()) {
    if (imu_buf_.front().timestamp > pending_.lidar_end_time) {
      break;
    }
    pending_.imu_buf.push_back(std::move(imu_buf_.front()));
    imu_buf_.pop_front();
  }
  measures = std::move(pending_);
  pending_.clear();
  return measures.imu_buf.size() > 4;
}

void Synchronizer::stop_accepting() {
  std::lock_guard<std::mutex> lock(mutex_);
  accepting_ = false;
}

SlamPipeline::SlamPipeline(SapphireParameters parameters, OutputSink output)
    : parameters_(validated(std::move(parameters))),
      output_(std::move(output)),
      parallel_executor_(parameters_.local_submap.thread_num),
      voxel_map_(parameters_.odometry, parameters_.local_submap) {
  filename_ = current_time_filename();
  save_path_ = parameters_.general.save_path;
  save_map_ = parameters_.general.save_map;
  down_size_inv_ = parameters_.odometry.down_size_inv;
  degrade_bound_ = parameters_.odometry.degrade_bound;
  imu_estimator_.configure(parameters_.sensor, parameters_.initializer, parameters_.odometry);
  extrinsic_.R = imu_estimator_.Lid_rot_to_IMU;
  extrinsic_.p = imu_estimator_.Lid_offset_to_IMU;
  window_size_ = parameters_.local_submap.win_size;

  if (save_map_) {
    prepare_output_directory(save_path_, filename_);
  } else if (parameters_.pose_graph.enabled) {
    std::filesystem::create_directories(std::filesystem::path(save_path_) / filename_);
  }
  create_pose_graph();
  init_buffer_.reserve(window_size_);
  spdlog::info("filename: {}", filename_);

  odometry_thread_ = std::thread(&SlamPipeline::thd_odometry, this);
  mapping_thread_ = std::thread(&SlamPipeline::thd_mapping, this);
}

SlamPipeline::~SlamPipeline() {
  shutdown();
  {
    std::lock_guard<std::mutex> lock(pose_graph_mutex_);
    pose_graph_.reset();
  }
  clear_imu_factors();
}

bool SlamPipeline::push_imu(ImuMeas imu) {
  if (!accepting_.load(std::memory_order_acquire)) {
    return false;
  }
  if (!synchronizer_.push_imu(std::move(imu))) {
    return false;
  }
  input_revision_.fetch_add(1, std::memory_order_release);
  input_cv_.notify_one();
  return true;
}

bool SlamPipeline::push_lidar(double timestamp, std::vector<LidarPoint> cloud) {
  if (!accepting_.load(std::memory_order_acquire)) {
    return false;
  }
  if (!synchronizer_.push_lidar(timestamp, std::move(cloud))) {
    return false;
  }
  input_revision_.fetch_add(1, std::memory_order_release);
  input_cv_.notify_one();
  return true;
}

void SlamPipeline::shutdown() {
  accepting_.store(false, std::memory_order_release);
  synchronizer_.stop_accepting();
  stopping_.store(true, std::memory_order_release);
  input_cv_.notify_all();
  keyframe_cv_.notify_all();
  if (odometry_thread_.joinable()) {
    odometry_thread_.join();
  }
  if (mapping_thread_.joinable()) {
    mapping_thread_.join();
  }
}

std::string SlamPipeline::backend_database_path() const { return (std::filesystem::path(save_path_) / filename_ / "map.db").string(); }

void SlamPipeline::create_pose_graph() {
  std::lock_guard<std::mutex> lock(pose_graph_mutex_);
  pose_graph_ = std::make_unique<PoseGraphBackend>(
      parameters_.pose_graph, parameters_.navi_map, backend_database_path(),
      [this](std::shared_ptr<const NavigationGrid> map) { invoke_output(output_.navigation_grid, std::move(map)); });
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
  invoke_output(output_.local_scan, std::make_shared<vvec<double, 3>>(points));
  invoke_output(output_.trajectory, std::make_shared<std::vector<TrajectoryPoint>>(trajectory_));
}

void SlamPipeline::emit_local_map(int marginal_count) {
  for (int i = 0; i < window_count_; ++i) {
    trajectory_[i + window_base_].x = static_cast<float>(state_buffer_[i].p.x());
    trajectory_[i + window_base_].y = static_cast<float>(state_buffer_[i].p.y());
    trajectory_[i + window_base_].z = static_cast<float>(state_buffer_[i].p.z());
  }

  auto map = std::make_shared<vvec<double, 3>>();
  for (int i = 0; i < marginal_count; ++i) {
    for (size_t j = 0; j < point_buffer_[i]->size(); j += 3) {
      const pointVar &point = point_buffer_[i]->at(j);
      map->push_back(state_buffer_[i].R * point.pnt + state_buffer_[i].p);
    }
  }

  invoke_output(output_.trajectory, std::make_shared<std::vector<TrajectoryPoint>>(trajectory_));
  invoke_output(output_.local_map, std::move(map));
}

void SlamPipeline::clear_imu_factors() {
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

void SlamPipeline::system_reset(const std::deque<ImuMeas> &imus) {
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
      if (eskf_.observe_voxelmap(current_state_, *points, voxel_map_)) {
        if (degrade_count > 0) {
          --degrade_count;
        }
      } else {
        ++degrade_count;
      }

      world_points.clear();
      pvec_update(points, current_state_, world_points);
      emit_local_trajectory(world_points, journey);
      ++window_count_;
      state_buffer_.push_back(current_state_);
      point_buffer_.push_back(points);
      time_buffer_.push_back(measures.lidar_end_time);
      if (window_count_ > 1) {
        imu_factor_buffer_.push_back(new ImuFactor(state_buffer_[window_count_ - 2].bg, state_buffer_[window_count_ - 2].ba));
        imu_factor_buffer_[window_count_ - 2]->push_imu(imus, imu_estimator_.scale_gravity, parameters_.local_submap);
      }

      voxel_hessian.clear();
      voxel_hessian.win_size = window_size_;
      voxel_map_.cut_voxel_multi(point_buffer_[window_count_ - 1], window_count_ - 1, window_size_, world_points);
      voxel_map_.recut_multi(window_count_, state_buffer_, voxel_hessian, parameters_.odometry.min_eigen_value,
                             parameters_.local_submap.plane_eigen_value_thre_inv);

      if (degrade_count > degrade_bound_) {
        degrade_count = 0;
        system_reset(imus);
        last_position = current_state_.p;
        journey = 0.0;
        {
          std::lock_guard<std::mutex> lock(keyframe_mutex_);
          reset_tail_.swap(keyframes_);
          reset_flag_ = 1;
        }
        keyframe_cv_.notify_one();
        motion_init_flag = 1;
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
      voxel_map_.marginalize(journey, window_count_, marginal_count, state_buffer_, voxel_hessian);

      {
        std::lock_guard<std::mutex> lock(keyframe_mutex_);
        keyframes_.emplace_back(state_buffer_[0], std::move(point_buffer_[0]), time_buffer_[0], marginal_variance);
      }
      keyframe_cv_.notify_one();

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
        std::swap(point_buffer_[i - marginal_count], point_buffer_[i]);
      }
      for (int i = window_count_ - marginal_count; i < window_count_; ++i) {
        state_buffer_.pop_back();
        point_buffer_.pop_back();
        time_buffer_.pop_back();
        delete imu_factor_buffer_.front();
        imu_factor_buffer_.pop_front();
      }
      window_base_ += marginal_count;
      window_count_ -= marginal_count;
    }
  }

  voxel_map_.clear();
  while (voxel_map_.release_retired(1000) > 0) {
  }
  malloc_trim(0);
  odometry_done_.store(true, std::memory_order_release);
  keyframe_cv_.notify_all();
}

void SlamPipeline::thd_mapping() {
  KeyframeBuffer keyframe_buffer(parameters_.pose_graph.keyframe_voxel_size_inv);
  int buffer_base = 0;
  const std::string initial_filename = filename_;
  if (save_map_) {
    FileReaderWriter::instance().open_session(save_path_, filename_);
  }

  while (true) {
    std::deque<MargiFrame, Eigen::aligned_allocator<MargiFrame>> reset_tail;
    std::optional<MargiFrame> marginal_frame;
    bool switch_session = false;
    {
      std::unique_lock<std::mutex> lock(keyframe_mutex_);
      keyframe_cv_.wait(lock, [this] {
        return reset_flag_ == 1 || !keyframes_.empty() ||
               (stopping_.load(std::memory_order_acquire) && odometry_done_.load(std::memory_order_acquire));
      });
      if (reset_flag_ == 1) {
        reset_flag_ = 0;
        reset_tail.swap(reset_tail_);
        switch_session = true;
      } else if (!keyframes_.empty()) {
        marginal_frame.emplace(std::move(keyframes_.front()));
        keyframes_.pop_front();
      } else if (stopping_.load(std::memory_order_acquire) && odometry_done_.load(std::memory_order_acquire)) {
        break;
      }
    }

    if (switch_session) {
      if (save_map_) {
        for (const MargiFrame &frame : reset_tail) {
          FileReaderWriter::instance().save_pose(frame);
        }
      }
      keyframe_buffer.clear();
      const int session = session_id_.fetch_add(1) + 1;
      filename_ = initial_filename + std::to_string(session);
      if (save_map_) {
        prepare_output_directory(save_path_, filename_);
        FileReaderWriter::instance().open_session(save_path_, filename_);
      } else if (parameters_.pose_graph.enabled) {
        std::filesystem::create_directories(std::filesystem::path(save_path_) / filename_);
      }
      buffer_base = 0;
      create_pose_graph();
      continue;
    }

    if (!marginal_frame) {
      continue;
    }
    if (save_map_) {
      FileReaderWriter::instance().save_pose(*marginal_frame);
    }
    ++buffer_base;

    LioFrame keyframe;
    if (!keyframe_buffer.push(std::move(*marginal_frame), keyframe)) {
      continue;
    }

    Eigen::Isometry3d map_odom = Eigen::Isometry3d::Identity();
    {
      std::lock_guard<std::mutex> lock(pose_graph_mutex_);
      pose_graph_->addFrame(keyframe.pcd, keyframe.T_odom_base, keyframe.timestamp);
      map_odom = pose_graph_->T_map_odom();
    }
    const Eigen::Isometry3d map_pose = map_odom * keyframe.T_odom_base;
    invoke_output(output_.map_odom, std::cref(map_odom));
    invoke_output(output_.map_pose, std::cref(map_pose), keyframe.timestamp);
    if (save_map_) {
      FileReaderWriter::instance().save_keyframe(keyframe, buffer_base - 1);
    }
  }

  if (save_map_) {
    FileReaderWriter::instance().close_session();
  }
  malloc_trim(0);
}

}  // namespace sapphire
