#pragma once

#include <utility>

#include "common/common_type.hpp"
#include "imu_state.hpp"

namespace sapphire {

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
