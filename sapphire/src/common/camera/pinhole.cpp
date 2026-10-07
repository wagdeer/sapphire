#include "common/camera/pinhole.hpp"
#include "parameters.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <opencv2/calib3d.hpp>

namespace sapphire {
PinholeCameraModel::PinholeCameraModel(const CameraParameters &c) : CameraCalibration(c) {
  if (c.distortion_model != "radtan") throw std::invalid_argument("Pinhole requires radtan calibration");
}

cv::Point2f PinholeCameraModel::distortNormalized(cv::Point2f ray) const {
  float x = ray.x, y = ray.y;
  if (!distortion_.empty()) {
    const auto &d = distortion_;
    const float r2 = x*x + y*y, radial = 1 + d[0]*r2 + d[1]*r2*r2 + d[4]*r2*r2*r2;
    const float xd = x*radial + 2*d[2]*x*y + d[3]*(r2 + 2*x*x);
    y = y*radial + d[2]*(r2 + 2*y*y) + 2*d[3]*x*y;
    x = xd;
  }
  return {intrinsics_[0]*x + intrinsics_[2], intrinsics_[1]*y + intrinsics_[3]};
}

void PinholeCameraModel::undistortPixels(const std::vector<cv::Point2f> &raw,
    std::vector<cv::Point2f> &out, bool rectified) const {
  if (raw.empty()) { out.clear(); return; }
  cv::undistortPoints(raw, out, matrix_, distortion_, cv::noArray(), rectified ? matrix_ : cv::Mat());
}

void PinholeCameraModel::undistortImage(const cv::Mat &raw, cv::Mat &rectified) const {
  checkImageSize(raw);
  cv::undistort(raw, rectified, matrix_, distortion_, matrix_);
}

std::optional<Eigen::Vector2f> PinholeCameraModel::project(const Eigen::Vector3f &point) const { return projectRaw(*this, point); }

std::optional<Eigen::Vector3f> PinholeCameraModel::unproject(cv::Point2f raw) const { return unprojectRaw(*this, raw); }
}  // namespace sapphire
