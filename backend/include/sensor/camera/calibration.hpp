#pragma once

#include <array>
#include <cmath>
#include <optional>
#include <vector>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace sapphire {
struct CameraParameters;
// Float calibration for optical cameras (+x right, +y down, +z forward).
// Shared storage/geometry; concrete models own distortion and inverse projection.
class CameraCalibration {
 public:
  const cv::Mat &matrix() const noexcept { return matrix_; }
  const std::vector<float> &distortion() const noexcept { return distortion_; }
  int width() const noexcept { return width_; }
  int height() const noexcept { return height_; }
  float fx() const noexcept { return intrinsics_[0]; }
  float fy() const noexcept { return intrinsics_[1]; }
  float cx() const noexcept { return intrinsics_[2]; }
  float cy() const noexcept { return intrinsics_[3]; }
  bool contains(float x, float y) const noexcept { return x >= 0 && y >= 0 && x < width_ && y < height_; }
  std::optional<Eigen::Vector3f> rectifiedToBearing(cv::Point2f pixel) const;
  std::optional<Eigen::Vector2f> bearingToRectified(const Eigen::Vector3f &point_camera) const;
  // T_imu_camera maps optical-camera coordinates into the IMU frame.
  // Extrinsics are required only for this operation, not pixel conversion.
  Eigen::Isometry3f cameraToImu() const;
 protected:
  template<class Model> static std::optional<Eigen::Vector2f> projectRaw(const Model &model, const Eigen::Vector3f &point) {
    if (!point.allFinite() || point.z() <= 0) return {};
    const auto raw = model.distortNormalized({point.x()/point.z(), point.y()/point.z()});
    if (!std::isfinite(raw.x) || !std::isfinite(raw.y)) return {};
    return Eigen::Vector2f(raw.x, raw.y);
  }

  template<class Model> static std::optional<Eigen::Vector3f> unprojectRaw(const Model &model, cv::Point2f raw) {
    if (!std::isfinite(raw.x) || !std::isfinite(raw.y)) return {};
    std::vector<cv::Point2f> normalized;
    model.undistortPixels({raw}, normalized);
    Eigen::Vector3f ray(normalized[0].x, normalized[0].y, 1.0F);
    const float norm = ray.norm();
    if (!ray.allFinite() || !std::isfinite(norm)) return {};
    return ray / norm;
  }

  explicit CameraCalibration(const CameraParameters &calibration);
  void checkImageSize(const cv::Mat &raw) const;
  int width_, height_;
  std::array<float, 4> intrinsics_;
  std::array<float, 2> inverse_focal_;
  std::vector<float> distortion_;
  cv::Mat matrix_;
  std::optional<Eigen::Isometry3f> camera_to_imu_;
};
}  // namespace sapphire
