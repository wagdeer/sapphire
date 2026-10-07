#pragma once

#include "common/common.hpp"
#include "parameters.h"

namespace sapphire {
class VoxelMap;

// Optional GICP observation used after ESKF geometric rejection. Pipeline-owned,
// odometry-thread only; map is read-only and all geometry caches are call-local.
// Points: scan-end IMU coordinates. State: T_world_imu and full ESKF covariance.
class Ricp {
 public:
  struct Result {
    bool accepted = false;
    const char *reason = "invalid";
    int samples = 0, matches = 0, targets = 0, iterations = 0;
    double rms = 0, correction = 0;
  };

  explicit Ricp(const GicpFallbackParameters &parameters) : parameters_(parameters) {}
  // small_gicp owns GICP factors, LM iteration and convergence; this component
  // owns source/map adaptation, acceptance and one full-state fusion/commit.
  Result observe(StateGroup &state, const PointCloud &points, const VoxelMap &map) const;

 private:
  GicpFallbackParameters parameters_;
};
} // namespace sapphire
