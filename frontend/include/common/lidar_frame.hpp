#pragma once

#include <utility>

#include "frontend/common/point.hpp"

namespace sapphire {

// Frozen handoff contract: x and Gaussian means/covariances are expressed in
// the original odometry domain (metres, seconds); N/radius/voxel attributes are
// retained. Move gaussians to transfer its allocation. No mutable VoxelMap or
// backend-corrected pose belongs here. v6 is legacy evidence, not a calibrated
// 6-DoF pose covariance.
struct LidarFrame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StateGroup x;
  GaussianCloud gaussians;
  Eigen::Matrix<double, 6, 1> v6;
  double timestamp = -1.0;
  double journey = 0.0;

  LidarFrame(const StateGroup &state, GaussianCloud &&cloud, double stamp, const Eigen::Matrix<double, 6, 1> &variance, double travel_distance)
      : x(state), gaussians(std::move(cloud)), v6(variance), timestamp(stamp), journey(travel_distance) {}
};

}  // namespace sapphire
