#pragma once

#include "frontend/common/measurement.hpp"
#include <deque>
#include <memory>
#include <optional>

#include "backend/sensor/lidar/submap_frame.hpp"
#include "frontend/common/lidar_frame.hpp"
#include "app/parameters.h"

namespace sapphire {

// Synchronous algorithm owner on the pipeline's existing mapping worker.
// LiDAR controls submap boundaries. Images only enrich those same submaps.
// Queue admission, session replacement and graph commits belong to the caller.
class SubmapBuilder final {
 public:
  explicit SubmapBuilder(const PoseGraphParameters &parameters);
  ~SubmapBuilder();

  // Images are ingress-validated observations in their own camera clock.
  // A resource cut starts a fresh zero-travel submap; each push
  // therefore emits at most one completed submap.
  std::optional<SubmapFrame> push(LidarFrame frame, std::deque<ImageMeas> images = {});
  std::optional<SubmapFrame> flush();
  void reset();
  std::size_t pendingFrames() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace sapphire
