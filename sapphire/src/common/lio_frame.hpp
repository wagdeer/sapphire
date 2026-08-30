#pragma once

#include <cmath>
#include <cstddef>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common.hpp"

namespace sapphire {

struct MargiFrame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StateGroup x;
  PointCloudPtr pvec;
  Eigen::Matrix<double, 6, 1> v6;
  double timestamp = -1.0;
  double journey = 0.0;

  MargiFrame(const StateGroup &state, PointCloudPtr points, double stamp, const Eigen::Matrix<double, 6, 1> &variance, double travel_distance)
      : x(state), pvec(std::move(points)), v6(variance), timestamp(stamp), journey(travel_distance) {}
};

struct LioFrame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Isometry3d T_odom_base = Eigen::Isometry3d::Identity();
  std::shared_ptr<const vvec<float, 3>> pcd;
  double timestamp = -1.0;
};

}  // namespace sapphire
