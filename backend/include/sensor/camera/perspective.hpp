#pragma once

#include <backend/common/common_type.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <optional>
#include <ostream>
#include <type_traits>
#include <vector>

#include "backend/common/state_group.hpp"
#include "backend/tools/simd.hpp"

namespace sapphire::backend::camera {

struct ImageBounds {
  double minX = 0.0;
  double maxX = 0.0;
  double minY = 0.0;
  double maxY = 0.0;
};

class Perspective {
 public:
  static StateGroupf opticalRotation() { return StateGroupf(StateGroupf::Coefficients{0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0}); }

  Perspective();
  Perspective(double fx, double fy, double cx, double cy, const StateGroupf& cameraToBase = opticalRotation(), const cv::Size& imageSize = {},
              double focalXBaseline = 0.0);

  bool isValidForProjection() const noexcept;
  bool isValidForReprojection() const noexcept;

  double fx() const noexcept { return focal_[0]; }
  double fy() const noexcept { return focal_[1]; }
  double cx() const noexcept { return principal_[0]; }
  double cy() const noexcept { return principal_[1]; }
  double fxInv() const noexcept { return inverseFocal_[0]; }
  double fyInv() const noexcept { return inverseFocal_[1]; }
  double focalXBaseline() const noexcept { return focalXBaseline_; }

  void setCameraToBase(const StateGroupf& transform);

  const StateGroupf& cameraToBase() const noexcept { return cameraToBase_; }

  void setImageSize(const cv::Size& size);
  const cv::Size& imageSize() const noexcept { return imageSize_; }
  const ImageBounds& imageBounds() const noexcept { return imageBounds_; }

  Mat33_t<double> cameraMatrix() const noexcept;

  std::optional<cv::Point2d> project(const cv::Point3d& point, MatRC_t<2, 3, double>* jacobian = nullptr) const noexcept;

  // Pixel Jacobian for T' = T Exp(delta), T = source_from_body,
  // delta = [translation, rotation]. Caller checks finite, positive optical depth.
  MatRC_t<2, 6, double> poseJacobian(const Vec3_t<double>& optical, const Vec3_t<double>& body,
                                         const Mat33_t<double>& cameraFromBodyRotation) const noexcept;

  static double squaredPixelError(const Mat33_t<double>& worldFromCameraRotation, const Vec3_t<double>& worldFromCameraTranslation,
                                  const Vec3_t<double>& point, const std::array<float, 5>& calibration,
                                  const Vec3_t<double>& measured) noexcept;

  std::optional<cv::Point3d> bearing(const cv::Point2d& pixel) const noexcept;

  std::optional<cv::Point3d> backProject(const cv::Point2d& pixel, double zDepth) const noexcept;

  bool isInImage(double imageX, double imageY) const noexcept;

  template <typename Packet, std::enable_if_t<std::is_same_v<Packet, simd::Float4> || std::is_same_v<Packet, simd::Double2>, int> = 0>
  auto isInImage(Packet imageX, Packet imageY) const noexcept {
    return (imageX > Packet(imageBounds_.minX)) & (imageX < Packet(imageBounds_.maxX)) &
           (imageY > Packet(imageBounds_.minY)) & (imageY < Packet(imageBounds_.maxY));
  }

  template <bool HasDisparityResidual, typename Packet>
  auto project(std::common_type_t<Packet> x, std::common_type_t<Packet> y, std::common_type_t<Packet> z,
               Packet& inverseZ, Packet& imageX, Packet& imageY, Packet& rightX) const noexcept {
    static_assert(std::is_same_v<Packet, simd::Float4> || std::is_same_v<Packet, simd::Double2>);
    inverseZ = Packet(1) / z;
    imageX = Packet(fx()) * x * inverseZ + Packet(cx());
    imageY = Packet(fy()) * y * inverseZ + Packet(cy());
    if constexpr (HasDisparityResidual) rightX = imageX - Packet(focalXBaseline_) * inverseZ;
    return z > Packet(0);
  }

  template <bool HasDisparityResidual, typename Scalar>
  bool projectScalar(Scalar x, Scalar y, Scalar z, Scalar& inverseZ, Scalar& imageX, Scalar& imageY, Scalar& rightX) const noexcept {
    static_assert(std::is_floating_point_v<Scalar>);
    if (!isValidForProjection() || !(z > Scalar(0)) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      inverseZ = imageX = imageY = rightX = Scalar(0);
      return false;
    }
    inverseZ = Scalar(1) / z;
    imageX = static_cast<Scalar>(fx()) * x * inverseZ + static_cast<Scalar>(cx());
    imageY = static_cast<Scalar>(fy()) * y * inverseZ + static_cast<Scalar>(cy());
    if constexpr (HasDisparityResidual) {
      rightX = imageX - static_cast<Scalar>(focalXBaseline_) * inverseZ;
      return std::isfinite(imageX) && std::isfinite(imageY) && std::isfinite(rightX);
    }
    return std::isfinite(imageX) && std::isfinite(imageY);
  }

  template <typename Scalar>
  bool projectVisibleScalar(std::common_type_t<Scalar> x, std::common_type_t<Scalar> y, std::common_type_t<Scalar> z,
                            Scalar& imageX, Scalar& imageY, Scalar& rightX) const noexcept {
    static_assert(std::is_same_v<Scalar, float> || std::is_same_v<Scalar, double>);
    Scalar inverseZ = 0;
    const bool visible = projectScalar<true>(x, y, z, inverseZ, imageX, imageY, rightX) && isInImage(imageX, imageY);
    if (!visible) imageX = imageY = rightX = 0;
    return visible;
  }

  simd::Mask32x4 projectVisible(simd::Float4 x, simd::Float4 y, simd::Float4 z, simd::Float4& imageX, simd::Float4& imageY,
                                simd::Float4& rightX) const noexcept;

  template <typename Packet>
  void backProject(std::common_type_t<Packet> imageX, std::common_type_t<Packet> imageY,
                   Packet& bearingX, Packet& bearingY, Packet& bearingZ) const noexcept {
    static_assert(std::is_same_v<Packet, simd::Float4> || std::is_same_v<Packet, simd::Double2>);
    const Packet normalizedX = (imageX - Packet(cx())) * Packet(fxInv());
    const Packet normalizedY = (imageY - Packet(cy())) * Packet(fyInv());
    simd::normalize(normalizedX, normalizedY, Packet(1), bearingX, bearingY, bearingZ);
  }

  template <typename Scalar>
  void convertPointsToBearings(const std::vector<cv::Point2f>& points, simd::SoA3<Scalar>& bearings) const;

  void convertKeypointsToBearings(const std::vector<cv::KeyPoint>& keypoints, simd::SoA3<float>& bearings) const;

  void convertBearingsToPoints(const simd::SoA3<float>& bearings, std::vector<cv::Point2f>& points) const;

  bool reprojectToBearing(const StateGroupd& cameraFromWorld, const Vec3_t<double>& worldPoint, Vec3_t<double>& outputBearing) const noexcept;

  void reprojectPointsToImage(const StateGroupf& cameraFromWorld, const simd::SoA3<float>& worldPoints, simd::SoA2<float>& imagePoints,
                              simd::aligned_vector<float>& rightX, std::vector<std::uint8_t>& visible) const;

  Perspective scaled(double scale) const;

  Perspective resized(const cv::Size& size) const;

  Perspective cropped(const cv::Rect& roi) const;

  // Field of view in radians.
  double fovX() const;
  double fovY() const;

 private:
  void refreshDerived() noexcept;

  cv::Size imageSize_;
  ImageBounds imageBounds_;
  alignas(simd::alignment) std::array<double, 2> focal_{0.0, 0.0};
  alignas(simd::alignment) std::array<double, 2> principal_{0.0, 0.0};
  alignas(simd::alignment) std::array<double, 2> inverseFocal_{0.0, 0.0};
  double focalXBaseline_ = 0.0;
  StateGroupf cameraToBase_;
};

std::ostream& operator<<(std::ostream& stream, const Perspective& model);

}  // namespace sapphire::backend::camera
