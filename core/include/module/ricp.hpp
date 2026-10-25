#pragma once

#include "common/point.hpp"
#include "parameters.hpp"

namespace sapphire {
class VoxelMap;

class Ricp {
 public:
  struct Result {
    bool accepted = false;
    const char *reason = "invalid";
    int samples = 0, matches = 0, targets = 0, iterations = 0;
    double rms = 0, correction = 0;
  };

  explicit Ricp(const GicpFallbackParameters &parameters) : parameters_(parameters) {}

  Result observe(StateGroup &state, const PointCloud &points, const VoxelMap &map) const;

 private:
  GicpFallbackParameters parameters_;
};
} // namespace sapphire
