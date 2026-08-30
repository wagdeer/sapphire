#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

namespace sapphire {

class CameraModel final {
 public:
  CameraModel(int width, int height, float fx, float fy, float cx, float cy, float alpha)
      : width_(width), height_(height), fx_(fx), fy_(fy), cx_(cx), cy_(cy), alpha_(alpha) {
    if (width_ <= 0 || height_ <= 0 || fx_ <= 0.0F || fy_ <= 0.0F || alpha_ < 0.0F || alpha_ > 1.0F) {
      throw std::invalid_argument("Invalid unified camera model parameters");
    }
  }

  std::optional<Eigen::Vector2f> project(const Eigen::Vector3f &point_camera) const noexcept {
    const float distance = point_camera.norm();
    if (distance <= std::numeric_limits<float>::epsilon()) {
      return std::nullopt;
    }
    const float denominator = alpha_ * distance + (1.0F - alpha_) * point_camera.z();
    if (denominator <= std::numeric_limits<float>::epsilon()) {
      return std::nullopt;
    }

    Eigen::Vector2f pixel(fx_ * point_camera.x() / denominator + cx_, fy_ * point_camera.y() / denominator + cy_);
    if (!pixel.allFinite()) {
      return std::nullopt;
    }
    return pixel;
  }

  std::optional<Eigen::Vector3f> unproject(const Eigen::Vector2f &pixel) const noexcept {
    const float x = (pixel.x() - cx_) / fx_;
    const float y = (pixel.y() - cy_) / fy_;
    const float radius_squared = x * x + y * y;
    const float radicand = 1.0F - (2.0F * alpha_ - 1.0F) * radius_squared;
    if (radicand < 0.0F) {
      return std::nullopt;
    }

    const float denominator = alpha_ * std::sqrt(radicand) + 1.0F - alpha_;
    if (std::abs(denominator) <= std::numeric_limits<float>::epsilon()) {
      return std::nullopt;
    }
    const float z = (1.0F - alpha_ * alpha_ * radius_squared) / denominator;
    Eigen::Vector3f bearing(x, y, z);
    const float norm = bearing.norm();
    if (norm <= std::numeric_limits<float>::epsilon() || !bearing.allFinite()) {
      return std::nullopt;
    }
    return bearing / norm;
  }

  void set_extrinsic(const Eigen::Matrix3f &rotation_base_camera, const Eigen::Vector3f &translation_base_camera) {
    rotation_base_camera_ = rotation_base_camera;
    translation_base_camera_ = translation_base_camera;
  }

  int width() const noexcept { return width_; }
  int height() const noexcept { return height_; }
  float alpha() const noexcept { return alpha_; }
  const Eigen::Matrix3f &rotation_base_camera() const noexcept { return rotation_base_camera_; }
  const Eigen::Vector3f &translation_base_camera() const noexcept { return translation_base_camera_; }

 private:
  int width_;
  int height_;
  float fx_;
  float fy_;
  float cx_;
  float cy_;
  float alpha_;
  Eigen::Matrix3f rotation_base_camera_ = Eigen::Matrix3f::Identity();
  Eigen::Vector3f translation_base_camera_ = Eigen::Vector3f::Zero();
};

}  // namespace sapphire