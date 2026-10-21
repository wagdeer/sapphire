#include "backend/solver/triangulator.hpp"

#include <Eigen/LU>
#include <algorithm>
#include <cmath>

#include "backend/common/stereo_frame.hpp"

namespace sapphire::backend::solver {
bool Triangulator::triangulateRays(const Eigen::Vector3d& bearing_1, const Eigen::Vector3d& bearing_2, const Eigen::Matrix3d& rotation_21,
                                   const Eigen::Vector3d& translation_21, Eigen::Vector3d& point_1) {
  if (!bearing_1.allFinite() || !bearing_2.allFinite() || !rotation_21.allFinite() || !translation_21.allFinite() || translation_21.squaredNorm() == 0.0)
    return false;
  const Eigen::Vector3d translation_12 = -rotation_21.transpose() * translation_21;
  const Eigen::Vector3d bearing_2_in_1 = rotation_21.transpose() * bearing_2;

  Eigen::Matrix2d system;
  system(0, 0) = bearing_1.dot(bearing_1);
  system(1, 0) = bearing_1.dot(bearing_2_in_1);
  system(0, 1) = -system(1, 0);
  system(1, 1) = -bearing_2_in_1.dot(bearing_2_in_1);
  const Eigen::Vector2d rhs(bearing_1.dot(translation_12), bearing_2_in_1.dot(translation_12));

  const double scale = system.cwiseAbs().maxCoeff();
  if (scale <= 1e-12 || std::fabs(system.determinant()) <= 1e-12 * scale * scale) {
    return false;
  }

  const Eigen::Vector2d distance = system.inverse() * rhs;
  const Eigen::Vector3d point_on_first = distance.x() * bearing_1;
  const Eigen::Vector3d point_on_second = distance.y() * bearing_2_in_1 + translation_12;
  point_1 = 0.5 * (point_on_first + point_on_second);
  return point_1.allFinite();
}

bool Triangulator::stereo_seed_valid(const camera::StereoObservation& o) {
  return stereo_seed_valid(o.calibration(), o.measurement());
}
bool Triangulator::stereo_seed_valid(const std::array<float, 5>& calibration, const camera::StereoPixel& o) {
  const auto& k = calibration;
  const double disparity = double(o.pixel.x) - o.pixel.y;
  return o.valid() && std::isfinite(o.sigma) && k[0] > 0 && k[1] > 0 && k[4] > 0 && std::isfinite(k[0]) && std::isfinite(k[1]) && std::isfinite(k[4]) &&
         std::isfinite(o.pixel.x) && std::isfinite(o.pixel.y) && std::isfinite(o.pixel.z) && o.pixel.y >= 0 && disparity > 0 && disparity < k[0];
}

bool Triangulator::temporal_depth_support(const camera::StereoObservation& a, const StateGroupf& frame_from_a, const camera::StereoObservation& b,
                                          const StateGroupf& frame_from_b) {
  if (!a.valid() || !b.valid()) return false;
  const auto pa = frame_from_a.compose(a.anchor_from_camera());
  const auto pb = frame_from_b.compose(b.anchor_from_camera());
  if ((pa.translationVector() - pb.translationVector()).squaredNorm() < 1e-12) return false;
  const auto bearing = [](const camera::StereoObservation& o) {
    const auto& k = o.calibration();
    return Eigen::Vector3d((o.pixel().x - k[2]) / k[0], (o.pixel().z - k[3]) / k[1], 1).normalized().eval();
  };
  const Eigen::Vector3d ra = pa.rotationMatrix().cast<double>() * bearing(a);
  const Eigen::Vector3d rb = pb.rotationMatrix().cast<double>() * bearing(b);
  constexpr double degree = 0.017453292519943295;
  const double noise_angle = std::hypot(a.sigma() / std::min(a.calibration()[0], a.calibration()[1]), b.sigma() / std::min(b.calibration()[0], b.calibration()[1]));
  const double cosine = ra.dot(rb);
  return std::isfinite(cosine) && cosine > 0 && cosine < std::cos(std::max(degree, noise_angle));
}

bool Triangulator::stereo_depth_support(const camera::StereoObservation& o) {
  return stereo_depth_support(o.calibration(), o.measurement());
}
bool Triangulator::stereo_depth_support(const std::array<float, 5>& calibration, const camera::StereoPixel& o) {
  if (!o.valid() || !std::isfinite(o.sigma) || o.pixel.y < 0) return false;
  const auto& k = calibration;
  if (!(k[0] > 0 && k[1] > 0 && k[4] > 0)) return false;
  const double minimum = std::max(1.f, o.sigma);
  const double disparity = double(o.pixel.x) - o.pixel.y;
  if (!std::isfinite(disparity) || disparity < minimum || disparity >= k[0]) return false;
  const Eigen::Vector3d left((o.pixel.x - k[2]) / k[0], (o.pixel.z - k[3]) / k[1], 1);
  const Eigen::Vector3d right((o.pixel.y - k[2]) / k[0], (o.pixel.z - k[3]) / k[1], 1);
  const double angle = std::atan2(left.cross(right).norm(), left.dot(right));
  return std::isfinite(angle) && angle >= std::atan(minimum / std::max(k[0], k[1]));
}

}  // namespace sapphire::backend::solver
