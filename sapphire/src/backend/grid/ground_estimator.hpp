#pragma once
#include <memory>
#include <optional>

#include "common/key_frame.hpp"
#include "parameters.h"

namespace sapphire {
// Returns the floor height in the submap's odometry frame. The tracker resets
// with each backend session and never writes a z correction into the SLAM graph.
class GroundEstimator {
 public:
  explicit GroundEstimator(const NaviMapParameters &grid);
  ~GroundEstimator();
  std::optional<float> update(const SubmapFrame &submap);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace sapphire
