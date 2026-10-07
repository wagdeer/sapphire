#include "common/camera/calibration.hpp"
#include "parameters.h"

#include <cmath>
#include <stdexcept>

namespace sapphire {
CameraCalibration::CameraCalibration(const CameraParameters &c) : width_(c.width), height_(c.height) {
  if (width_ <= 0 || height_ <= 0 || c.intrinsics.size() != 4 || !valid_camera_distortion(c))
    throw std::invalid_argument("Invalid optical camera calibration");
  for (std::size_t i = 0; i < intrinsics_.size(); ++i) {
    intrinsics_[i] = static_cast<float>(c.intrinsics[i]);
    if (!std::isfinite(intrinsics_[i])) throw std::invalid_argument("Nonfinite camera intrinsics");
  }
  if (fx() <= 0 || fy() <= 0) throw std::invalid_argument("Camera focal length must be positive");
  inverse_focal_ = {1.0F / fx(), 1.0F / fy()};
  if (!std::isfinite(inverse_focal_[0]) || !std::isfinite(inverse_focal_[1]))
    throw std::invalid_argument("Camera inverse focal length must be finite");
  distortion_.assign(c.distortion.begin(), c.distortion.end());
  for (float v : distortion_) if (!std::isfinite(v)) throw std::invalid_argument("Nonfinite float distortion");
  matrix_ = (cv::Mat_<float>(3, 3) << fx(), 0, cx(), 0, fy(), cy(), 0, 0, 1);
  if (c.camera_to_imu_rotation.size() == 9 && c.camera_to_imu_translation.size() == 3) {
    Eigen::Isometry3f t = Eigen::Isometry3f::Identity();
    for (int i = 0; i < 3; ++i) {
      t.translation()[i] = static_cast<float>(c.camera_to_imu_translation[i]);
      for (int j = 0; j < 3; ++j) t.linear()(i,j) = static_cast<float>(c.camera_to_imu_rotation[i*3+j]);
    }
    if (t.matrix().allFinite() && (t.linear().transpose()*t.linear()-Eigen::Matrix3f::Identity()).norm() <= 1e-5F &&
        std::abs(t.linear().determinant()-1.0F) <= 1e-5F) camera_to_imu_ = t;
  }
}

void CameraCalibration::checkImageSize(const cv::Mat &raw) const {
  if (raw.cols != width_ || raw.rows != height_)
    throw std::invalid_argument("Image dimensions disagree with camera calibration");
}

Eigen::Isometry3f CameraCalibration::cameraToImu() const {
  if (!camera_to_imu_) throw std::invalid_argument("Missing or invalid camera-to-IMU extrinsic");
  return *camera_to_imu_;
}

std::optional<Eigen::Vector3f> CameraCalibration::rectifiedToBearing(cv::Point2f pixel) const {
  Eigen::Vector3f ray((pixel.x-cx())*inverse_focal_[0], (pixel.y-cy())*inverse_focal_[1], 1.0F);
  const float norm = ray.norm();
  if (!ray.allFinite() || !std::isfinite(norm)) return {};
  return ray / norm;
}

std::optional<Eigen::Vector2f> CameraCalibration::bearingToRectified(const Eigen::Vector3f &point) const {
  if (!point.allFinite() || point.z() <= 0) return {};
  const Eigen::Vector2f pixel(fx()*point.x()/point.z()+cx(), fy()*point.y()/point.z()+cy());
  if (!pixel.allFinite()) return {};
  return pixel;
}
}  // namespace sapphire
