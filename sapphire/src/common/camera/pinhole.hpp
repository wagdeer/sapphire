#pragma once

#include "common/camera/calibration.hpp"

namespace sapphire {
// Pinhole projection with optional radial/tangential distortion.
class PinholeCameraModel final : public CameraCalibration {
 public:
  explicit PinholeCameraModel(const CameraParameters &calibration);
  cv::Point2f distortNormalized(cv::Point2f ray) const;
  // Raw distorted pixel <-> unit optical bearing. The current geometry path
  // uses the forward-facing hemisphere; image bounds are checked separately.
  std::optional<Eigen::Vector2f> project(const Eigen::Vector3f &point_camera) const;
  std::optional<Eigen::Vector3f> unproject(cv::Point2f raw) const;
  // false: normalized pinhole ray; true: rectified pinhole pixel under matrix().
  void undistortPixels(const std::vector<cv::Point2f> &raw,
                       std::vector<cv::Point2f> &out, bool rectified = false) const;
  void undistortImage(const cv::Mat &raw, cv::Mat &rectified) const;
};
}  // namespace sapphire
