#include "backend/sensor/camera/perspective.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "backend/tools/logging.hpp"

namespace sapphire::backend::camera {
namespace {

double bearingAngle(const std::optional<cv::Point3d>& first, const std::optional<cv::Point3d>& second) {
  return first && second ? std::acos(std::clamp(first->dot(*second), -1.0, 1.0)) : 0.0;
}

}  // namespace

Perspective::Perspective() : cameraToBase_(opticalRotation()) {}

Perspective::Perspective(double fx, double fy, double cx, double cy, const StateGroupf& cameraToBase, const cv::Size& imageSize, double focalXBaseline)
    : imageSize_(imageSize), focal_{fx, fy}, principal_{cx, cy}, focalXBaseline_(focalXBaseline), cameraToBase_(cameraToBase) {
  EXT_ASSERT_MSG(fx > 0.0 && fy > 0.0, "fx={} fy={}", fx, fy);
  EXT_ASSERT_MSG(cx >= 0.0 && cy >= 0.0, "cx={} cy={}", cx, cy);
  EXT_ASSERT((imageSize.width == 0 && imageSize.height == 0) || (imageSize.width > 0 && imageSize.height > 0));
  EXT_ASSERT(!cameraToBase.isNull());
  refreshDerived();
}

bool Perspective::isValidForProjection() const noexcept {
  return std::isfinite(fx()) && std::isfinite(fy()) && std::isfinite(cx()) && std::isfinite(cy()) && fx() > 0.0 && fy() > 0.0 && cx() >= 0.0 && cy() >= 0.0;
}

bool Perspective::isValidForReprojection() const noexcept { return isValidForProjection() && imageSize_.width > 0 && imageSize_.height > 0; }

void Perspective::setCameraToBase(const StateGroupf& transform) {
  EXT_ASSERT(!transform.isNull());
  cameraToBase_ = transform;
}

void Perspective::setImageSize(const cv::Size& size) {
  EXT_ASSERT((size.width == 0 && size.height == 0) || (size.width > 0 && size.height > 0));
  imageSize_ = size;
  if (principal_[0] == 0.0 && size.width > 0) principal_[0] = static_cast<double>(size.width) * 0.5 - 0.5;
  if (principal_[1] == 0.0 && size.height > 0) principal_[1] = static_cast<double>(size.height) * 0.5 - 0.5;
  refreshDerived();
}

void Perspective::refreshDerived() noexcept {
  inverseFocal_[0] = focal_[0] != 0.0 ? 1.0 / focal_[0] : 0.0;
  inverseFocal_[1] = focal_[1] != 0.0 ? 1.0 / focal_[1] : 0.0;
  imageBounds_ = ImageBounds{0.0, static_cast<double>(imageSize_.width), 0.0, static_cast<double>(imageSize_.height)};
}

Eigen::Matrix3d Perspective::cameraMatrix() const noexcept {
  Eigen::Matrix3d matrix;
  matrix << fx(), 0.0, cx(), 0.0, fy(), cy(), 0.0, 0.0, 1.0;
  return matrix;
}

std::optional<cv::Point2d> Perspective::project(const cv::Point3d& point, Eigen::Matrix<double, 2, 3>* jacobian) const noexcept {
  if (jacobian) jacobian->setZero();
  double inverseZ = 0.0;
  double imageX = 0.0;
  double imageY = 0.0;
  double rightX = 0.0;
  if (!projectScalar<false>(point.x, point.y, point.z, inverseZ, imageX, imageY, rightX)) return std::nullopt;
  if (jacobian)
    *jacobian << fx() * inverseZ, 0, -fx() * point.x * inverseZ * inverseZ,
                 0, fy() * inverseZ, -fy() * point.y * inverseZ * inverseZ;
  return cv::Point2d(imageX, imageY);
}

Eigen::Matrix<double, 2, 6> Perspective::poseJacobian(const Eigen::Vector3d& optical, const Eigen::Vector3d& body,
                                                   const Eigen::Matrix3d& cameraFromBodyRotation) const noexcept {
  Eigen::Matrix<double, 2, 3> projection;
  project({optical.x(), optical.y(), optical.z()}, &projection);
  Eigen::Matrix3d cross;
  cross << 0, -body.z(), body.y(), body.z(), 0, -body.x(), -body.y(), body.x(), 0;
  Eigen::Matrix<double, 3, 6> motion;
  motion.leftCols<3>() = -Eigen::Matrix3d::Identity();
  motion.rightCols<3>() = cross;
  return projection * cameraFromBodyRotation * motion;
}

double Perspective::squaredPixelError(const Eigen::Matrix3d& world_from_camera_rotation,
                                 const Eigen::Vector3d& world_from_camera_translation,
                                 const Eigen::Vector3d& point, const std::array<float, 5>& k,
                                 const Eigen::Vector3d& measured) noexcept {
  if (!point.allFinite()) return std::numeric_limits<double>::infinity();
  const Eigen::Vector3d q = world_from_camera_rotation.transpose() * (point - world_from_camera_translation);
  if (q.z() <= 0) return std::numeric_limits<double>::infinity();
  const double d = 1.0 / q.z();
  if (measured.y() < 0) {
    const double u = q.x() * d, v = q.y() * d;
    return (Eigen::Vector2d(k[0] * u + 0.0 * v + k[2], k[1] * v + k[3]) - Eigen::Vector2d(measured.x(), measured.z())).squaredNorm();
  }
  // Keep the native stereo arithmetic order, including float baseline division.
  const double dfx = d * k[0], dfy = d * k[1], baseline = k[4] / k[0];
  return (Eigen::Vector3d(k[2] + dfx * q.x(), k[2] + dfx * (q.x() - baseline), k[3] + dfy * q.y()) - measured).squaredNorm();
}

std::optional<cv::Point3d> Perspective::bearing(const cv::Point2d& pixel) const noexcept {
  if (!isValidForProjection() || !std::isfinite(pixel.x) || !std::isfinite(pixel.y)) return std::nullopt;
  const double x = (pixel.x - cx()) * fxInv();
  const double y = (pixel.y - cy()) * fyInv();
  const double inverseNorm = 1.0 / std::sqrt(x * x + y * y + 1.0);
  return cv::Point3d(x * inverseNorm, y * inverseNorm, inverseNorm);
}

std::optional<cv::Point3d> Perspective::backProject(const cv::Point2d& pixel, double zDepth) const noexcept {
  const std::optional<cv::Point3d> ray = bearing(pixel);
  if (!ray || !std::isfinite(zDepth) || zDepth <= 0.0 || ray->z <= std::numeric_limits<double>::epsilon()) return std::nullopt;
  return *ray * (zDepth / ray->z);
}

bool Perspective::isInImage(double imageX, double imageY) const noexcept {
  return imageBounds_.minX < imageX && imageX < imageBounds_.maxX && imageBounds_.minY < imageY && imageY < imageBounds_.maxY;
}

simd::Mask32x4 Perspective::projectVisible(simd::Float4 x, simd::Float4 y, simd::Float4 z, simd::Float4& imageX, simd::Float4& imageY,
                                           simd::Float4& rightX) const noexcept {
  const simd::Float4 zero(0.0f);
  const simd::Mask32x4 validDepth = simd::is_finite(x) & simd::is_finite(y) & simd::is_finite(z) & (z > zero);
  const simd::Float4 safeZ = simd::select(validDepth, z, simd::Float4(1.0f));
  simd::Float4 inverseZ;
  project<true>(x, y, safeZ, inverseZ, imageX, imageY, rightX);
  const simd::Mask32x4 visible =
      validDepth & simd::is_finite(imageX) & simd::is_finite(imageY) & simd::is_finite(rightX) & isInImage(imageX, imageY);
  imageX = simd::select(visible, imageX, zero);
  imageY = simd::select(visible, imageY, zero);
  rightX = simd::select(visible, rightX, zero);
  return visible;
}

template <typename Scalar>
void Perspective::convertPointsToBearings(const std::vector<cv::Point2f>& points, simd::SoA3<Scalar>& bearings) const {
  static_assert(std::is_same_v<Scalar, float> || std::is_same_v<Scalar, double>);
  using Packet = std::conditional_t<std::is_same_v<Scalar, float>, simd::Float4, simd::Double2>;
  constexpr std::size_t width = simd::packet_width<Scalar>;
  bearings.resize(points.size());
  const std::size_t packedSize = simd::floor_packet<Scalar>(points.size());
  std::size_t index = 0U;
  for (; index < packedSize; index += width) {
    alignas(simd::alignment) Scalar imageX[width], imageY[width];
    for (std::size_t lane = 0; lane < width; ++lane) {
      imageX[lane] = points[index + lane].x;
      imageY[lane] = points[index + lane].y;
    }
    Packet x, y, z;
    backProject(Packet::load_aligned(imageX), Packet::load_aligned(imageY), x, y, z);
    x.store_aligned(bearings.x.data() + index);
    y.store_aligned(bearings.y.data() + index);
    z.store_aligned(bearings.z.data() + index);
  }
  for (; index < points.size(); ++index) {
    const auto ray = bearing(cv::Point2d(points[index]));
    if (ray) bearings.set(index, static_cast<Scalar>(ray->x), static_cast<Scalar>(ray->y), static_cast<Scalar>(ray->z));
  }
}

template void Perspective::convertPointsToBearings(const std::vector<cv::Point2f>&, simd::SoA3<float>&) const;
template void Perspective::convertPointsToBearings(const std::vector<cv::Point2f>&, simd::SoA3<double>&) const;

void Perspective::convertKeypointsToBearings(const std::vector<cv::KeyPoint>& keypoints, simd::SoA3<float>& bearings) const {
  std::vector<cv::Point2f> points;
  points.reserve(keypoints.size());
  for (const cv::KeyPoint& keypoint : keypoints) points.push_back(keypoint.pt);
  convertPointsToBearings(points, bearings);
}

void Perspective::convertBearingsToPoints(const simd::SoA3<float>& bearings, std::vector<cv::Point2f>& points) const {
  points.resize(bearings.size());
  const std::size_t packedSize = simd::floor_packet<float>(bearings.size());
  std::size_t index = 0U;
  for (; index < packedSize; index += simd::packet_width<float>) {
    simd::Float4 inverseZ;
    simd::Float4 imageX;
    simd::Float4 imageY;
    simd::Float4 rightX;
    project<false>(simd::Float4::load_aligned(bearings.x.data() + index), simd::Float4::load_aligned(bearings.y.data() + index),
                   simd::Float4::load_aligned(bearings.z.data() + index), inverseZ, imageX, imageY, rightX);
    for (std::size_t lane = 0; lane < simd::packet_width<float>; ++lane) points[index + lane] = cv::Point2f(imageX.lane(lane), imageY.lane(lane));
  }
  for (; index < bearings.size(); ++index) {
    const auto projected = project(cv::Point3d(bearings.x[index], bearings.y[index], bearings.z[index]));
    points[index] = projected ? cv::Point2f(*projected) : cv::Point2f();
  }
}

bool Perspective::reprojectToBearing(const StateGroupd& cameraFromWorld, const Eigen::Vector3d& worldPoint, Eigen::Vector3d& outputBearing) const noexcept {
  outputBearing = cameraFromWorld.transformPoint(worldPoint);
  if (!outputBearing.allFinite() || !(outputBearing.z() > 0.0)) return false;
  double inverseZ = 0.0;
  double imageX = 0.0;
  double imageY = 0.0;
  double rightX = 0.0;
  if (!projectScalar<false>(outputBearing.x(), outputBearing.y(), outputBearing.z(), inverseZ, imageX, imageY, rightX) || !isInImage(imageX, imageY)) {
    return false;
  }
  outputBearing.normalize();
  return true;
}

void Perspective::reprojectPointsToImage(const StateGroupf& cameraFromWorld, const simd::SoA3<float>& worldPoints, simd::SoA2<float>& imagePoints,
                                         simd::aligned_vector<float>& rightX, std::vector<std::uint8_t>& visible) const {
  imagePoints.resize(worldPoints.size());
  rightX.resize(worldPoints.size());
  visible.resize(worldPoints.size());
  const StateGroupf::Coefficients& transform = cameraFromWorld.coefficients();
  const std::size_t packedSize = simd::floor_packet<float>(worldPoints.size());
  std::size_t index = 0U;
  for (; index < packedSize; index += simd::packet_width<float>) {
    const simd::Float4 worldX = simd::Float4::load_aligned(worldPoints.x.data() + index);
    const simd::Float4 worldY = simd::Float4::load_aligned(worldPoints.y.data() + index);
    const simd::Float4 worldZ = simd::Float4::load_aligned(worldPoints.z.data() + index);
    const simd::Float4 cameraX =
        simd::Float4::multiply_add(simd::Float4(transform[0]), worldX,
                                   simd::Float4::multiply_add(simd::Float4(transform[1]), worldY,
                                                              simd::Float4::multiply_add(simd::Float4(transform[2]), worldZ, simd::Float4(transform[3]))));
    const simd::Float4 cameraY =
        simd::Float4::multiply_add(simd::Float4(transform[4]), worldX,
                                   simd::Float4::multiply_add(simd::Float4(transform[5]), worldY,
                                                              simd::Float4::multiply_add(simd::Float4(transform[6]), worldZ, simd::Float4(transform[7]))));
    const simd::Float4 cameraZ =
        simd::Float4::multiply_add(simd::Float4(transform[8]), worldX,
                                   simd::Float4::multiply_add(simd::Float4(transform[9]), worldY,
                                                              simd::Float4::multiply_add(simd::Float4(transform[10]), worldZ, simd::Float4(transform[11]))));
    simd::Float4 imageX;
    simd::Float4 imageY;
    simd::Float4 imageRightX;
    const simd::Mask32x4 visibility = projectVisible(cameraX, cameraY, cameraZ, imageX, imageY, imageRightX);
    imageX.store_aligned(imagePoints.x.data() + index);
    imageY.store_aligned(imagePoints.y.data() + index);
    imageRightX.store_aligned(rightX.data() + index);
    for (std::size_t lane = 0; lane < simd::packet_width<float>; ++lane) visible[index + lane] = visibility.lane(lane) ? 1U : 0U;
  }
  for (; index < worldPoints.size(); ++index) {
    const Eigen::Vector3f cameraPoint = cameraFromWorld.transformPoint(Eigen::Vector3f(worldPoints.x[index], worldPoints.y[index], worldPoints.z[index]));
    visible[index] =
        projectVisibleScalar(cameraPoint.x(), cameraPoint.y(), cameraPoint.z(), imagePoints.x[index], imagePoints.y[index], rightX[index]) ? 1U : 0U;
  }
}

Perspective Perspective::scaled(double scale) const {
  EXT_ASSERT(scale > 0.0);
  return resized(cv::Size(cvRound(imageSize_.width * scale), cvRound(imageSize_.height * scale)));
}

Perspective Perspective::resized(const cv::Size& size) const {
  EXT_ASSERT(imageSize_.width > 0 && imageSize_.height > 0 && size.width > 0 && size.height > 0);
  Perspective result = *this;
  const double scaleX = static_cast<double>(size.width) / imageSize_.width;
  const double scaleY = static_cast<double>(size.height) / imageSize_.height;
  result.focal_[0] *= scaleX;
  result.focal_[1] *= scaleY;
  result.principal_[0] *= scaleX;
  result.principal_[1] *= scaleY;
  result.focalXBaseline_ *= scaleX;
  result.imageSize_ = size;
  result.refreshDerived();
  return result;
}

Perspective Perspective::cropped(const cv::Rect& roi) const {
  EXT_ASSERT(roi.width > 0 && roi.height > 0);
  Perspective result = *this;
  result.principal_[0] -= roi.x;
  result.principal_[1] -= roi.y;
  result.imageSize_ = roi.size();
  result.refreshDerived();
  return result;
}

double Perspective::fovX() const {
  return isValidForReprojection() ? bearingAngle(bearing(cv::Point2d(0.0, cy())), bearing(cv::Point2d(imageSize_.width - 1.0, cy()))) : 0.0;
}

double Perspective::fovY() const {
  return isValidForReprojection() ? bearingAngle(bearing(cv::Point2d(cx(), 0.0)), bearing(cv::Point2d(cx(), imageSize_.height - 1.0))) : 0.0;
}

std::ostream& operator<<(std::ostream& stream, const Perspective& model) {
  return stream << "Size: " << model.imageSize().width << 'x' << model.imageSize().height << '\n'
                << "fx=" << model.fx() << " fy=" << model.fy() << " cx=" << model.cx() << " cy=" << model.cy() << '\n'
                << "CameraToBase= " << model.cameraToBase();
}

}  // namespace sapphire::backend::camera
