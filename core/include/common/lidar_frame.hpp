#pragma once

#include <utility>

#include "common/common_type.hpp"
#include "imu_state.hpp"

namespace sapphire {

struct LidarFrame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr std::size_t max_geometry_bytes = 16U * 1024 * 1024;

  std::uint64_t generation = 0; // Set by the FIFO handoff, not by the estimator.
  StateGroup x;
  GaussianCloud gaussians;
  double timestamp = -1.0;
  double journey = 0.0;

  LidarFrame(const StateGroup &state, GaussianCloud &&cloud, double stamp, double travel_distance)
      : x(state), gaussians(std::move(cloud)), timestamp(stamp), journey(travel_distance) {}
};

}  // namespace sapphire
