#pragma once

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <cmath>
#include <cstddef>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/common.hpp"

namespace sapphire {

struct OdomPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp = -1.0;
  Eigen::Isometry3d T_odom_base = Eigen::Isometry3d::Identity();
};

using OdomPoses = std::vector<OdomPose, Eigen::aligned_allocator<OdomPose>>;

struct MargiFrame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StateGroup x;
  GaussianCloud gaussians;
  Eigen::Matrix<double, 6, 1> v6;
  double timestamp = -1.0;
  double journey = 0.0;

  MargiFrame(const StateGroup &state, GaussianCloud &&cloud, double stamp, const Eigen::Matrix<double, 6, 1> &variance, double travel_distance)
      : x(state), gaussians(std::move(cloud)), v6(variance), timestamp(stamp), journey(travel_distance) {}
};

struct LioFrame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Isometry3d T_odom_base = Eigen::Isometry3d::Identity();
  GaussianCloudPtr pcd;
  double timestamp = -1.0;
};

}  // namespace sapphire
