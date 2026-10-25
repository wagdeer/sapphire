#include "maplayer/local_map.hpp"

#include <stdexcept>

#include "module/initialize.hpp"

namespace sapphire {

LocalMap::LocalMap(const SapphireParameters &parameters, tools::ParallelExecutor<> &executor)
    : parameters_(parameters), executor_(executor), geometry_(parameters.odometry, parameters.local_submap, executor),
      lidar_factor_(capacity()), optimizer_(parameters.local_submap, executor) {
  if (capacity() < 2) throw std::invalid_argument("LocalMap requires at least two frames for joint LiDAR/IMU optimization");
  states_.reserve(capacity());
  points_.reserve(capacity());
  times_.reserve(capacity());
  journeys_.reserve(capacity());
  initial_measures_.reserve(capacity());
}

void LocalMap::append_imu(const std::deque<ImuMeas> &imus, double gravity_scale) {
  if (states_.empty()) return;
  if (!pending_imu_) pending_imu_ = std::make_unique<ImuFactor>(states_.back().bg, states_.back().ba);
  pending_imu_->push_imu(imus, gravity_scale, parameters_.local_submap);
}

void LocalMap::append_frame(const StateGroup &state, const PointCloudPtr &points, double stamp, double journey) {
  if (full()) throw std::logic_error("LocalMap must retire its oldest frame before accepting another");
  if (!points) throw std::invalid_argument("LocalMap requires a point cloud");
  if (!states_.empty() && !pending_imu_) throw std::logic_error("LocalMap frame requires an IMU bridge from the preceding frame");
  states_.push_back(state);
  points_.push_back(points);
  times_.push_back(stamp);
  journeys_.push_back(journey);
  if (pending_imu_) imu_factors_.push_back(std::move(pending_imu_));
  optimized_ = false;
}

void LocalMap::insert(const StateGroup &state, const PointCloudPtr &points, double journey, const vvec<double, 3> &world_points) {
  append_frame(state, points, state.t, journey);
  lidar_factor_.clear();
  lidar_factor_.win_size = capacity();
  geometry_.cut_voxel_multi(points_.back(), size() - 1, capacity(), world_points);
  geometry_.recut_multi(size(), states_, lidar_factor_, parameters_.odometry.min_eigen_value,
                        parameters_.local_submap.plane_eigen_value_thre_inv);
}

void LocalMap::insert_initial(const StateGroup &state, const PointCloudPtr &points, MeasGroup &&raw, double gravity_scale) {
  append_imu(raw.imu_buf, gravity_scale);
  append_frame(state, points, raw.lidar_end_time, 0.0);
  initial_measures_.push_back(std::move(raw));
}

bool LocalMap::initialize(StateGroup &current, StateGroup &extrinsic, double gravity_scale) {
  if (!full() || initial_measures_.size() != states_.size()) throw std::logic_error("Initialization requires a complete raw window");
  const bool success = Initialization::instance().motion_init(*this, current, extrinsic, gravity_scale) != 0;
  if (success) initial_measures_.clear();
  return success;
}

void LocalMap::optimize(StateGroup &current) {
  if (!full() || imu_factors_.size() + 1 != states_.size()) throw std::logic_error("LocalMap optimization requires a full connected window");
  optimizer_.damping_iter(states_, lidar_factor_, imu_factors_, &hessian_);
  // Preserve the existing feedback contract: BA feeds pose into the live estimator.
  current.R = states_.back().R;
  current.p = states_.back().p;
  optimized_ = true;
}

LidarFrame LocalMap::retire_oldest(double journey, bool emit_geometry) {
  if (!full() || !optimized_) throw std::logic_error("Only a full optimized LocalMap can retire a frame");
  Eigen::Matrix<double, 6, 1> variance = hessian_.block<POSE_DOF, POSE_DOF>(0, STATE_DOF).diagonal();
  for (int i = 0; i < 6; ++i) variance[i] = 1.0 / std::abs(variance[i]);
  GaussianCloud frozen = geometry_.marginalize(journey, size(), 1, states_, lidar_factor_, emit_geometry);
  if (frozen.size() > 16 * 1024 * 1024 / sizeof(GaussianPoint)) throw std::length_error("Marginal evidence exceeds finite producer bound");
  LidarFrame frame(states_.front(), std::move(frozen), times_.front(), variance, journeys_.front());
  states_.erase(states_.begin());
  points_.erase(points_.begin());
  times_.erase(times_.begin());
  journeys_.erase(journeys_.begin());
  imu_factors_.pop_front();
  ++retired_frames_;
  optimized_ = false;
  return frame;
}

void LocalMap::reset() {
  geometry_.reset();
  pending_imu_.reset();
  imu_factors_.clear();
  lidar_factor_.clear();
  states_.clear();
  points_.clear();
  times_.clear();
  journeys_.clear();
  initial_measures_.clear();
  retired_frames_ = 0;
  optimized_ = false;
}

}  // namespace sapphire
