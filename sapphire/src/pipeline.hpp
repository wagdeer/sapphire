#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "common.hpp"
#include "eskf.hpp"
#include "imu_estimator.hpp"
#include "imu_factor.hpp"
#include "key_frame.hpp"
#include "parallel_executor.hpp"
#include "parameters.h"
#include "pose_graph.hpp"
#include "voxel_map.hpp"

namespace sapphire {

class Synchronizer {
 public:
  bool push_imu(ImuMeas imu);
  bool push_lidar(double timestamp, std::vector<LidarPoint> cloud);
  bool sync_packages(MeasGroup &measures);
  void stop_accepting();

 private:
  std::mutex mutex_;
  std::deque<ImuMeas> imu_buf_;
  std::deque<std::shared_ptr<std::vector<LidarPoint>>> lidar_buf_;
  std::deque<double> lidar_time_buf_;
  MeasGroup pending_;
  double last_imu_time_ = -std::numeric_limits<double>::infinity();
  double last_lidar_time_ = -std::numeric_limits<double>::infinity();
  bool accepting_ = true;
};

struct OutputSink {
  std::function<void(const StateGroup &)> odom_state;
  std::function<void(std::shared_ptr<const vvec<double, 3>>)> local_scan;
  std::function<void(std::shared_ptr<const std::vector<TrajectoryPoint>>)> trajectory;
  std::function<void(std::shared_ptr<const vvec<double, 3>>)> local_map;
  std::function<void(const Eigen::Isometry3d &)> map_odom;
  std::function<void(const Eigen::Isometry3d &, double)> map_pose;
  std::function<void(std::shared_ptr<const NavigationGrid>)> navigation_grid;
};

class SlamPipeline {
 public:
  explicit SlamPipeline(SapphireParameters parameters, OutputSink output = {});
  ~SlamPipeline();

  SlamPipeline(const SlamPipeline &) = delete;
  SlamPipeline &operator=(const SlamPipeline &) = delete;

  bool push_imu(ImuMeas imu);
  bool push_lidar(double timestamp, std::vector<LidarPoint> cloud);
  bool push_image(ImageMeas image);
  void shutdown();

 private:
  std::string backend_database_path() const;
  void create_pose_graph();
  void emit_local_trajectory(const vvec<double, 3> &points, double journey);
  void emit_local_map(int marginal_count);
  void clear_imu_factors();
  int initialization(MeasGroup &measures, Eigen::MatrixXd &hess, LidarFactor &voxel_hessian, vvec<double, 3> &points);
  void system_reset(const std::deque<ImuMeas> &imus);
  void thd_odometry();
  void thd_mapping();

  const SapphireParameters parameters_;
  OutputSink output_;
  Synchronizer synchronizer_;
  ParallelExecutor parallel_executor_;
  std::vector<TrajectoryPoint> trajectory_;
  StateGroup current_state_, extrinsic_;
  ImuEstimator imu_estimator_;
  ESKF eskf_;
  VoxelMap voxel_map_;
  double down_size_inv_ = 1.0;
  int window_size_ = 0;
  std::vector<StateGroup> state_buffer_;
  std::vector<PointCloudPtr> point_buffer_;
  std::vector<double> time_buffer_;
  std::vector<MeasGroup> init_buffer_;
  std::deque<ImuFactor *> imu_factor_buffer_;
  int window_count_ = 0;
  int window_base_ = 0;
  int degrade_bound_ = 10;

  std::mutex marginal_mutex_;
  std::condition_variable marginal_cv_;
  std::deque<MargiFrame, Eigen::aligned_allocator<MargiFrame>> marginal_frames_, reset_tail_;
  int reset_flag_ = 0;

  std::atomic_bool accepting_{true};
  std::atomic_bool stopping_{false};
  std::atomic_bool odometry_done_{false};

  std::atomic_int session_id_{0};
  std::atomic_uint64_t input_revision_{0};

  std::mutex input_mutex_;
  std::condition_variable input_cv_;

  std::mutex image_mutex_;
  std::deque<ImageMeas> image_buffer_;

  std::thread odometry_thread_;
  std::thread mapping_thread_;

  std::string filename_;
  std::string save_path_;
  int save_map_ = 0;
  mutable std::mutex pose_graph_mutex_;
  std::unique_ptr<PoseGraphBackend> pose_graph_;
};

using SapphirePipeline = SlamPipeline;

}  // namespace sapphire
