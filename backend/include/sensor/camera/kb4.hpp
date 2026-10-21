#pragma once

#include <backend/common/common_type.hpp>
#include <Eigen/Core>
#include <array>
#include <opencv2/core/types.hpp>
#include <optional>

namespace sapphire::backend::camera {

// Kannala–Brandt: theta * (1 + k1 theta^2 + ... + k4 theta^8).
class KB4 final {
 public:
  KB4(double fx, double fy, double cx, double cy, const std::array<double, 4>& distortion = {});

  double fx() const noexcept { return k_[0]; }
  double fy() const noexcept { return k_[1]; }
  double cx() const noexcept { return k_[2]; }
  double cy() const noexcept { return k_[3]; }

  const std::array<double, 4>& distortion() const noexcept { return distortion_; }

  std::optional<cv::Point2d> project(const cv::Point3d& point, MatRC_t<2, 3, double>* jacobian = nullptr) const noexcept;

  std::optional<cv::Point3d> bearing(const cv::Point2d& pixel) const noexcept;

  // Optical Z depth, not distance along the ray; requires a forward ray.
  std::optional<cv::Point3d> backProject(const cv::Point2d& pixel, double zDepth) const noexcept;

 private:
  std::array<double, 4> k_;
  std::array<double, 4> distortion_;
};

}  // namespace sapphire::backend::camera
