#pragma once

#include <array>
#include <memory>
#include <vector>
#include <opencv2/core/types.hpp>

#include <backend/common/state_group.hpp>

namespace sapphire::backend::camera {
class Perspective;

struct StereoPixel {
  cv::Point3f pixel{};  // u_left, u_right, v; negative u_right means monocular
  float sigma = 0;      // image-pixel standard deviation
  bool valid() const { return sigma > 0; }
};

// Immutable measurements for one capture/camera. Optimized frame poses live
// in the caller's pose table, never in this original measurement snapshot.
struct StereoFrame {
  // Original rectified geometry, shared by every pixel in this capture.
  // Left optical frame; common intrinsics and a horizontal baseline.
  const StateGroupf anchor_from_camera;
  const std::array<float, 5> calibration;  // fx, fy, cx, cy, fx*baseline
  const std::vector<StereoPixel> pixels;
  StereoFrame(StateGroupf anchor_from_camera, std::array<float, 5> calibration, std::vector<StereoPixel> pixels);
  size_t memory_size() const;
};

// Owning indexed view: copies retain their frame across map copies/merges.
// No raw pointer into a growing vector and no per-pixel pose/calibration copy.
struct StereoObservation {
  std::shared_ptr<const StereoFrame> frame;
  size_t row = 0;
  StereoObservation() = default;
  StereoObservation(std::shared_ptr<const StereoFrame> frame, size_t row);
  // Standalone measurement, when no frame batch is available.
  StereoObservation(StateGroupf anchor_from_camera, std::array<float, 5> calibration, cv::Point3f pixel, float sigma);
  const StereoPixel& measurement() const;
  const StateGroupf& anchor_from_camera() const;
  const std::array<float, 5>& calibration() const;
  const cv::Point3f& pixel() const { return measurement().pixel; }
  float sigma() const { return measurement().sigma; }
  bool valid() const { return frame && row < frame->pixels.size() && measurement().valid(); }
  // Explicit replacement leaves all views of the original frame unchanged.
  StereoObservation with_pixel(cv::Point3f pixel) const;
  StereoObservation with_sigma(float sigma) const;
  StereoObservation with_camera(StateGroupf anchor_from_camera, std::array<float, 5> calibration) const;
};

// Capture the existing rectified model in the measurement representation.
std::array<float, 5> stereo_calibration(const Perspective& camera);

// Recover disparity only from original sparse-stereo points, before optimization.
// Preserve feature row order, including invalid rows. A missing octave uses
// fallback_sigma (zero by default). Mono fallback is an explicit caller policy.
StereoPixel stereo_pixel(const Perspective& camera, const cv::KeyPoint& keypoint, const cv::Point3f& original_body_point, float sigma,
                         const StateGroupf& anchor_from_body = StateGroupf::identity());
std::shared_ptr<const StereoFrame> stereo_frame(const Perspective& camera, const std::vector<cv::KeyPoint>& keypoints,
                                               const std::vector<cv::Point3f>& original_body_points,
                                               const std::vector<double>& pixel_variances, bool allow_mono = false, float fallback_sigma = 0,
                                               const StateGroupf& anchor_from_body = StateGroupf::identity());

StereoObservation stereo_observation(const Perspective& camera, const cv::KeyPoint& pixel, const cv::Point3f& original_body_point, float sigma,
                                     const StateGroupf& anchor_from_body = StateGroupf::identity());

// Count each retained frame once within an owner's memory estimate.
size_t stereo_frame_memory(std::vector<const StereoFrame*> frames);

}  // namespace sapphire::backend::camera
