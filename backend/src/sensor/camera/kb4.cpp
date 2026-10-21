#include "backend/sensor/camera/kb4.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sapphire::backend::camera {
KB4::KB4(double fx, double fy, double cx, double cy, const std::array<double, 4>& distortion)
    : k_{fx, fy, cx, cy}, distortion_(distortion) {
  for (double v : k_) if (!std::isfinite(v)) throw std::invalid_argument("KB4: nonfinite intrinsics");
  if (fx <= 0 || fy <= 0) throw std::invalid_argument("KB4: focal lengths must be positive");
  for (double v : distortion_) if (!std::isfinite(v)) throw std::invalid_argument("KB4: nonfinite distortion");
}
namespace {
double radius(double t, const std::array<double, 4>& d) {
  const double t2 = t * t;
  return t * (1 + t2 * (d[0] + t2 * (d[1] + t2 * (d[2] + t2 * d[3]))));
}
double slope(double t, const std::array<double, 4>& d) {
  const double t2 = t * t;
  return 1 + t2 * (3*d[0] + t2 * (5*d[1] + t2 * (7*d[2] + t2 * 9*d[3])));
}
}  // namespace

std::optional<cv::Point2d> KB4::project(const cv::Point3d& p, Eigen::Matrix<double, 2, 3>* j) const noexcept {
  if (j) j->setZero();
  const double n = std::hypot(std::hypot(p.x, p.y), p.z);
  if (!std::isfinite(n) || n == 0) return std::nullopt;
  const double r = std::hypot(p.x, p.y);
  if (r < 1e-10 * n) {
    if (p.z <= 0) return std::nullopt;
    if (j) *j << fx()/p.z, 0, -fx()*p.x/(p.z*p.z), 0, fy()/p.z, -fy()*p.y/(p.z*p.z);
    return cv::Point2d(fx()*p.x/p.z + cx(), fy()*p.y/p.z + cy());
  }
  const double theta = std::atan2(r, p.z), rd = radius(theta, distortion_), derivative = slope(theta, distortion_);
  if (!std::isfinite(rd) || !std::isfinite(derivative) || rd < 0 || derivative <= 0) return std::nullopt;
  const double scale = rd / r;
  if (j) {
    const Eigen::RowVector3d dr(p.x/r, p.y/r, 0);
    const Eigen::RowVector3d dt(p.z/n*(p.x/r)/n, p.z/n*(p.y/r)/n, -r/n/n);
    const Eigen::RowVector3d ds = (derivative * dt - scale * dr) / r;
    j->row(0) = fx() * (p.x * ds + Eigen::RowVector3d(scale, 0, 0));
    j->row(1) = fy() * (p.y * ds + Eigen::RowVector3d(0, scale, 0));
  }
  const cv::Point2d pixel(fx()*scale*p.x + cx(), fy()*scale*p.y + cy());
  if (!std::isfinite(pixel.x) || !std::isfinite(pixel.y)) return std::nullopt;
  return pixel;
}

std::optional<cv::Point3d> KB4::bearing(const cv::Point2d& pixel) const noexcept {
  const double x = (pixel.x-cx())/fx(), y = (pixel.y-cy())/fy(), rd = std::hypot(x, y);
  if (!std::isfinite(rd)) return std::nullopt;
  if (rd < 1e-12) return cv::Point3d(x, y, 1);
  constexpr double pi = 3.14159265358979323846;
  double lower = 0, upper = pi;
  double theta = std::min(rd, pi/2);
  for (int i = 0; i < 50; ++i) {
    const double derivative = slope(theta, distortion_);
    const double error = radius(theta, distortion_) - rd;
    if (!std::isfinite(error) || !std::isfinite(derivative) || derivative <= 0) return std::nullopt;
    if (std::abs(error) <= 1e-12 * std::max(1.0, rd)) {
      const double scale = std::sin(theta)/rd;
      return cv::Point3d(scale*x, scale*y, std::cos(theta));
    }
    if (error > 0) upper = theta; else lower = theta;
    const double next = theta - error/derivative;
    // Safeguard Newton near the edge of wide fields of view. The calibrated
    // radial polynomial is assumed monotonic on the rays being inverted.
    theta = std::isfinite(next) && next > lower && next < upper ? next : 0.5*(lower+upper);
  }
  return std::nullopt;
}

std::optional<cv::Point3d> KB4::backProject(const cv::Point2d& pixel, double depth) const noexcept {
  if (!std::isfinite(depth) || depth <= 0) return std::nullopt;
  const auto ray = bearing(pixel);
  if (!ray || ray->z <= 1e-12) return std::nullopt;
  const cv::Point3d point = *ray * (depth/ray->z);
  if (!std::isfinite(point.x) || !std::isfinite(point.y)) return std::nullopt;
  return point;
}
}  // namespace sapphire::backend::camera
