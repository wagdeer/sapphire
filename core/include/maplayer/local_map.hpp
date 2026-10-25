#pragma once

#include "common/lidar_frame.hpp"
#include "maplayer/local_map/voxel_map.hpp"
#include "module/synchronizer.hpp"
#include "solver/lm_optimizer.hpp"

namespace sapphire {

// Active odometry-frame geometry and its joint LiDAR/IMU optimization window.
// One odometry worker owns this object. Parameters and executor must outlive it.
// Retired observations leave once, by value; downstream Nodes never share this map.
class LocalMap final {
 public:
  LocalMap(const SapphireParameters &parameters, tools::ParallelExecutor<> &executor);
  LocalMap(const LocalMap &) = delete;
  LocalMap &operator=(const LocalMap &) = delete;

  int size() const { return static_cast<int>(states_.size()); }
  int capacity() const { return parameters_.local_submap.win_size; }
  bool full() const { return size() == capacity(); }
  std::size_t retired_frames() const { return retired_frames_; }
  const StateGroup &state(int index) const { return states_.at(index); }
  const PointCloud &points(int index) const { return *points_.at(index); }
  VoxelMap &geometry() { return geometry_; }

  // Accumulate every predicted interval, including intervals whose scan is rejected.
  void append_imu(const std::deque<ImuMeas> &imus, double gravity_scale);
  void insert(const StateGroup &state, const PointCloudPtr &points, double journey, const vvec<double, 3> &world_points);
  void insert_initial(const StateGroup &state, const PointCloudPtr &points, MeasGroup &&raw, double gravity_scale);
  bool initialize(StateGroup &current, StateGroup &extrinsic, double gravity_scale);
  const std::deque<ImuMeas> &last_initial_imus() const { return initial_measures_.back().imu_buf; }

  void optimize(StateGroup &current);
  // Requires a full, optimized window. Only the oldest frame's contributions
  // are frozen in odom coordinates; the remaining frames stay jointly optimizable.
  LidarFrame retire_oldest(double journey, bool emit_geometry);
  void reset();

 private:
  friend class Initialization;
  friend struct SlamPipelineTestAccess;
  void append_frame(const StateGroup &state, const PointCloudPtr &points, double stamp, double journey);

  const SapphireParameters &parameters_;
  tools::ParallelExecutor<> &executor_;
  VoxelMap geometry_;
  std::vector<StateGroup> states_;
  std::vector<PointCloudPtr> points_;
  std::vector<double> times_, journeys_;
  std::vector<MeasGroup> initial_measures_;
  std::deque<std::unique_ptr<ImuFactor>> imu_factors_;
  std::unique_ptr<ImuFactor> pending_imu_;
  LidarFactor lidar_factor_;
  LI_BA_Optimizer optimizer_;
  Eigen::MatrixXd hessian_;
  std::size_t retired_frames_ = 0;
  bool optimized_ = false;
};

}  // namespace sapphire
