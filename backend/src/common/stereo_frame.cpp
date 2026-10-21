#include "backend/common/stereo_frame.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>
#include "backend/sensor/camera/perspective.hpp"

namespace sapphire::backend::camera {
StereoFrame::StereoFrame(StateGroupf pose, std::array<float, 5> calibration, std::vector<StereoPixel> pixels)
    : anchor_from_camera(std::move(pose)), calibration(calibration), pixels(std::move(pixels)) {}
size_t StereoFrame::memory_size() const { return sizeof(*this) + pixels.capacity() * sizeof(StereoPixel); }
StereoObservation::StereoObservation(std::shared_ptr<const StereoFrame> frame, size_t row) : frame(std::move(frame)), row(row) {}
StereoObservation::StereoObservation(StateGroupf pose, std::array<float, 5> calibration, cv::Point3f pixel, float sigma)
    : frame(std::make_shared<const StereoFrame>(pose, calibration, std::vector<StereoPixel>{{pixel, sigma}})) {}
const StateGroupf& StereoObservation::anchor_from_camera() const {
  static const StateGroupf empty;
  return frame ? frame->anchor_from_camera : empty;
}
const std::array<float, 5>& StereoObservation::calibration() const {
  static const std::array<float, 5> empty{};
  return frame ? frame->calibration : empty;
}
const StereoPixel& StereoObservation::measurement() const {
  static const StereoPixel empty;
  return frame && row < frame->pixels.size() ? frame->pixels[row] : empty;
}
StereoObservation StereoObservation::with_pixel(cv::Point3f value) const { return {anchor_from_camera(), calibration(), value, sigma()}; }
StereoObservation StereoObservation::with_sigma(float value) const { return {anchor_from_camera(), calibration(), pixel(), value}; }
StereoObservation StereoObservation::with_camera(StateGroupf pose, std::array<float, 5> values) const { return {pose, values, pixel(), sigma()}; }
std::array<float, 5> stereo_calibration(const Perspective& camera) {
  return {float(camera.fx()), float(camera.fy()), float(camera.cx()), float(camera.cy()), float(camera.focalXBaseline())};
}
StereoPixel stereo_pixel(const Perspective& camera, const cv::KeyPoint& keypoint, const cv::Point3f& body_point, float sigma,
                    const StateGroupf& anchor_from_body) {
  if (!(camera.focalXBaseline() > 0) || !camera.isValidForReprojection() || !camera.cameraToBase().isValid() || !anchor_from_body.isValid() ||
      !std::isfinite(sigma) || sigma <= 0 || !camera.isInImage(keypoint.pt.x, keypoint.pt.y)) return {};
  const auto optical = camera.cameraToBase().inverse().transformPoint({body_point.x, body_point.y, body_point.z});
  if (!optical.allFinite() || optical.z() <= 0) return {};
  const float right = keypoint.pt.x - camera.focalXBaseline() / optical.z();
  if (!std::isfinite(right) || right < 0 || right >= camera.imageSize().width) return {};
  return {{keypoint.pt.x, right, keypoint.pt.y}, sigma};
}
std::shared_ptr<const StereoFrame> stereo_frame(const Perspective& camera, const std::vector<cv::KeyPoint>& keys,
                                               const std::vector<cv::Point3f>& points, const std::vector<double>& variances,
                                               bool allow_mono, float fallback_sigma, const StateGroupf& anchor_from_body) {
  if (keys.size() != points.size()) throw std::invalid_argument("Stereo frame feature/point count mismatch");
  std::vector<StereoPixel> pixels;
  pixels.reserve(keys.size());
  for (size_t i = 0; i < keys.size(); ++i) {
    const auto& k = keys[i];
    const float sigma = k.octave >= 0 && size_t(k.octave) < variances.size() ? std::sqrt(variances[k.octave]) : fallback_sigma;
    auto pixel = stereo_pixel(camera, k, points[i], sigma, anchor_from_body);
    if (!pixel.valid() && allow_mono) pixel = {{k.pt.x, -1, k.pt.y}, sigma};
    pixels.push_back(pixel);
  }
  return std::make_shared<const StereoFrame>(anchor_from_body.compose(camera.cameraToBase()), stereo_calibration(camera), std::move(pixels));
}
StereoObservation stereo_observation(const Perspective& camera, const cv::KeyPoint& key, const cv::Point3f& point, float sigma,
                                     const StateGroupf& anchor_from_body) {
  const auto pixel = stereo_pixel(camera, key, point, sigma, anchor_from_body);
  if (!pixel.valid()) return {};
  return {std::make_shared<const StereoFrame>(anchor_from_body.compose(camera.cameraToBase()), stereo_calibration(camera), std::vector<StereoPixel>{pixel}), 0};
}
size_t stereo_frame_memory(std::vector<const StereoFrame*> frames) {
  std::sort(frames.begin(), frames.end(), std::less<const StereoFrame*>{});
  frames.erase(std::unique(frames.begin(), frames.end()), frames.end());
  size_t bytes = 0;
  for (const auto* frame : frames) if (frame) bytes += frame->memory_size();
  return bytes;
}
}  // namespace sapphire::backend::camera
