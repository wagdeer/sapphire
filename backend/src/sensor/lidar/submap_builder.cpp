#include "backend/sensor/lidar/submap_builder.hpp"

#include <algorithm>
#include <cmath>
#include <spdlog/spdlog.h>
#include <stdexcept>

#include "backend/sensor/lidar/submap_buffer.hpp"
#include "backend/sensor/camera/image_attribute.hpp"

namespace sapphire {

class SubmapBuilder::Impl {
 public:
  explicit Impl(const PoseGraphParameters &parameters)
      : visual(parameters.visual),
        submap_buffer(parameters.submap_voxel_size_inv, parameters.submap_travel_distance, parameters.submap_max_point_range) {}

  std::optional<SubmapFrame> flush(const char *reason) {
    if (submap_buffer.empty()) return {};
    spdlog::info("[submap] reason={} frames={} points={} travel_m={}", reason,
        submap_buffer.buffered_frame_count(), submap_buffer.buffered_point_count(), submap_buffer.travel_distance());
    return submap_buffer.flush();
  }

  std::optional<SubmapFrame> push(LidarFrame frame, std::deque<ImageMeas> images) {
    if (frame.gaussians.size() > 24000) throw std::length_error("LiDAR evidence exceeds builder point bound");
    if (!std::isfinite(frame.timestamp) || !std::isfinite(frame.journey) ||
        !frame.x.R.allFinite() || !frame.x.p.allFinite())
      throw std::invalid_argument("Invalid frozen LiDAR observation");
    std::optional<SubmapFrame> ready;
    if (submap_buffer.buffered_frame_count() >= 256 ||
        submap_buffer.buffered_point_count() + frame.gaussians.size() > 24000) {
      ready = flush("resource");
    }
    if (visual.enabled) processImages(images);
    // Delay closure until runtime visual evidence is attached, but only LiDAR
    // travel controls the decision. Visual novelty and image loss cannot cut it.
    (void)submap_buffer.push(std::move(frame), false);
    if (submap_buffer.travel_distance() >= submap_buffer.target_distance()) {
      return flush("distance");
    }
    return ready;
  }

  void processImages(const std::deque<ImageMeas> &images) {
    for (const auto &image : images)
      submap_buffer.push_visual(makeImageAttribute(image, visual.camera(image.camera_id)), visual.max_frames);
  }

  void reset() {
    submap_buffer.clear();
  }

  VisualLoopParameters visual;
  SubmapFrameBuffer submap_buffer;

};

SubmapBuilder::SubmapBuilder(const PoseGraphParameters &parameters) : impl_(std::make_unique<Impl>(parameters)) {}
SubmapBuilder::~SubmapBuilder() = default;
std::optional<SubmapFrame> SubmapBuilder::push(LidarFrame frame, std::deque<ImageMeas> images) {
  return impl_->push(std::move(frame), std::move(images));
}
std::optional<SubmapFrame> SubmapBuilder::flush() {
  return impl_->flush("tail");
}
void SubmapBuilder::reset() { impl_->reset(); }
std::size_t SubmapBuilder::pendingFrames() const { return impl_->submap_buffer.buffered_frame_count(); }

}  // namespace sapphire
