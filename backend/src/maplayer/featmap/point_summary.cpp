#include "backend/maplayer/featmap/point_summary.hpp"

#include <Eigen/QR>
#include <stdexcept>

namespace sapphire::backend::featmap {

void PointSummary::validate() const {
  if (!reference.isValid() || !reference.matrix().allFinite() || !anchor.allFinite() || anchor.z() <= 0 ||
      !root.allFinite() || !residual.allFinite() ||
      !(reference.rotationMatrix().transpose() * reference.rotationMatrix()).isApprox(Mat33_t<Scalar>::Identity(), 1e-8) ||
      reference.rotationMatrix().determinant() <= 0)
    throw std::invalid_argument("Invalid point pixel summary");
}

Vec3_t<double> PointSummary::coordinates(const Vec3_t<Scalar>& point, Mat33_t<Scalar>* jacobian) const {
  const auto rotation = reference.rotationMatrix().transpose().eval();
  const Vec3_t<Scalar> p = rotation * (point - reference.translationVector());
  if (!p.allFinite() || p.z() <= 0) throw std::domain_error("Point outside summary reference half-space");
  const Scalar iz = 1 / p.z();
  if (jacobian) {
    Mat33_t<Scalar> d;
    d << iz, 0, -p.x() * iz * iz, 0, iz, -p.y() * iz * iz, 0, 0, -iz * iz;
    *jacobian = d * rotation;
  }
  return {p.x() * iz, p.y() * iz, iz};
}

Vec3_t<double> PointSummary::evaluate(const Vec3_t<Scalar>& point, Mat33_t<Scalar>* jacobian) const {
  Mat33_t<Scalar> d;
  const auto s = coordinates(point, jacobian ? &d : nullptr);
  if (jacobian) *jacobian = root * d;
  return root * (s - anchor) + residual;
}

PointSummary PointSummary::appended(const Vec3_t<Scalar>& point, const Vec3_t<Scalar>& error, const Mat33_t<Scalar>& jacobian) const {
  PointSummary next = *this;
  next.anchor = coordinates(point);
  // Analytic inverse of projective coordinates: p = (u,v,1)/rho.
  // No information-matrix inverse and no covariance propagation are involved.
  const Scalar z = 1 / next.anchor.z();
  Mat33_t<Scalar> point_from_coordinate;
  point_from_coordinate << z, 0, -next.anchor.x() * z * z,
                           0, z, -next.anchor.y() * z * z,
                           0, 0, -z * z;
  point_from_coordinate = (reference.rotationMatrix() * point_from_coordinate).eval();
  MatRC_t<6, 3, Scalar> rows;
  rows.topRows<3>() = root;
  rows.bottomRows<3>() = jacobian * point_from_coordinate;
  Vec6_t<Scalar> rhs;
  rhs.head<3>() = root * (next.anchor - anchor) + residual;
  rhs.tail<3>() = error;
  const Eigen::HouseholderQR<MatRC_t<6, 3, Scalar>> qr(rows);
  next.root = qr.matrixQR().topRows<3>().template triangularView<Eigen::Upper>();
  next.residual = (qr.householderQ().adjoint() * rhs).head<3>();
  next.validate();
  return next;
}

PointSummary PointSummary::from_observation(const camera::StereoObservation& observation, const StateGroupd& node_from_capture,
                                           const Vec3_t<Scalar>& point) {
  if (!observation.valid()) throw std::invalid_argument("Missing initial point pixel");
  PointSummary result;
  result.reference = node_from_capture.compose(StateGroupd(observation.anchor_from_camera()).normalized());
  result.anchor = result.coordinates(point);
  return result.with_observation(observation, node_from_capture, point);
}

PointSummary PointSummary::with_observation(const camera::StereoObservation& observation, const StateGroupd& node_from_capture,
                                            const Vec3_t<Scalar>& point) const {
  validate();
  if (!observation.valid() || !node_from_capture.isValid()) throw std::invalid_argument("Invalid summary observation");
  const auto camera = node_from_capture.compose(StateGroupd(observation.anchor_from_camera()).normalized());
  const Mat33_t<Scalar> rotation = camera.rotationMatrix().transpose();
  const Vec3_t<Scalar> p = rotation * (point - camera.translationVector());
  if (!p.allFinite() || p.z() <= 0) throw std::domain_error("Invalid summary observation depth");
  const auto& k = observation.calibration();
  const auto& pixel = observation.pixel();
  const Scalar sigma = observation.sigma(), iz = 1 / p.z();
  if (!std::isfinite(sigma) || sigma <= 0 || k[0] <= 0 || k[1] <= 0 || k[4] < 0)
    throw std::invalid_argument("Invalid summary pixel calibration or noise");
  const Scalar u = k[0] * p.x() * iz + k[2];
  Vec3_t<Scalar> error(u - pixel.x, pixel.y < 0 ? 0 : u - k[4] * iz - pixel.y, k[1] * p.y() * iz + k[3] - pixel.z);
  Mat33_t<Scalar> d = Mat33_t<Scalar>::Zero();
  d.row(0) << k[0] * iz, 0, -k[0] * p.x() * iz * iz;
  if (pixel.y >= 0) d.row(1) << k[0] * iz, 0, (k[4] - k[0] * p.x()) * iz * iz;
  d.row(2) << 0, k[1] * iz, -k[1] * p.y() * iz * iz;
  return appended(point, error / sigma, (d * rotation) / sigma);
}

PointSummary PointSummary::merged(const PointSummary& source, const StateGroupd& target_from_source, const Vec3_t<Scalar>& point) const {
  validate();
  source.validate();
  if (!target_from_source.isValid()) throw std::invalid_argument("Invalid summary alignment");
  Mat33_t<Scalar> d;
  const auto error = source.evaluate(target_from_source.inverse().transformPoint(point), &d);
  return appended(point, error, d * target_from_source.rotationMatrix().transpose());
}

PointSummary PointSummary::reanchored(const StateGroupd& new_from_old) const {
  PointSummary next = *this;
  next.reference = new_from_old.compose(reference);
  next.validate();
  return next;
}

}  // namespace sapphire::backend::featmap
