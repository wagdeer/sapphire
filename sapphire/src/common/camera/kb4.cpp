#include "common/camera/kb4.hpp"
#include "parameters.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <opencv2/calib3d.hpp>

namespace sapphire {
Kb4CameraModel::Kb4CameraModel(const CameraParameters &c) : CameraCalibration(c) {
  if (c.distortion_model != "equidistant") throw std::invalid_argument("KB4 requires equidistant calibration");
}

cv::Point2f Kb4CameraModel::distortNormalized(cv::Point2f ray) const {
  float x = ray.x, y = ray.y;
  const float r = std::hypot(x, y), theta = std::atan(r), t2 = theta * theta;
  const auto &d = distortion_;
  const float scale = r > 1e-12F ? theta * (1 + t2 * (d[0] + t2 * (d[1] + t2 * (d[2] + t2 * d[3])))) / r : 1.0F;
  x *= scale; y *= scale;
  return {intrinsics_[0]*x + intrinsics_[2], intrinsics_[1]*y + intrinsics_[3]};
}

void Kb4CameraModel::undistortPixels(const std::vector<cv::Point2f> &raw,
    std::vector<cv::Point2f> &out, bool rectified) const {
  if (raw.empty()) { out.clear(); return; }
  cv::fisheye::undistortPoints(raw, out, matrix_, distortion_, cv::noArray(), rectified ? matrix_ : cv::Mat());
  // OpenCV's finite sentinel/clipped inverse cannot silently become a bearing.
  for (std::size_t i = 0; i < out.size(); ++i) {
    const cv::Point2f ray = rectified
        ? cv::Point2f((out[i].x-intrinsics_[2])/intrinsics_[0], (out[i].y-intrinsics_[3])/intrinsics_[1])
        : cv::Point2f(out[i]);
    const auto projected = distortNormalized(ray);
    if (!std::isfinite(ray.x) || !std::isfinite(ray.y) || !std::isfinite(projected.x) || !std::isfinite(projected.y) ||
        cv::norm(projected - cv::Point2f(raw[i])) > .01F)
      out[i] = cv::Point2f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
  }
}

void Kb4CameraModel::undistortImage(const cv::Mat &raw, cv::Mat &rectified) const {
  checkImageSize(raw);
  cv::fisheye::undistortImage(raw, rectified, matrix_, distortion_, matrix_, raw.size());
}

std::optional<Eigen::Vector2f> Kb4CameraModel::project(const Eigen::Vector3f &point) const { return projectRaw(*this, point); }

std::optional<Eigen::Vector3f> Kb4CameraModel::unproject(cv::Point2f raw) const { return unprojectRaw(*this, raw); }
}  // namespace sapphire
