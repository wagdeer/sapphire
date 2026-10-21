#pragma once

#include "backend/common/stereo_frame.hpp"

namespace sapphire::backend::featmap {

// Fixed-size local approximation of accepted pixel residuals, conditional on
// committed capture poses. This is not a marginal covariance of the map.
// Coordinates are (x/z, y/z, 1/z) in reference; the ray remains optimizable.
struct PointSummary {
  using Scalar = double;
  StateGroupd reference;
  Vec3_t<Scalar> anchor = Vec3_t<Scalar>::Zero();
  Mat33_t<Scalar> root = Mat33_t<Scalar>::Zero();
  Vec3_t<Scalar> residual = Vec3_t<Scalar>::Zero();

  // Returns whitened historical pixel residuals; their point-independent
  // constant has been discarded. Rank-deficient (e.g. mono) roots are valid.
  Vec3_t<Scalar> evaluate(const Vec3_t<Scalar>& point, Mat33_t<Scalar>* jacobian = nullptr) const;
  void validate() const;
  static PointSummary from_observation(const camera::StereoObservation&, const StateGroupd& node_from_capture,
                                       const Vec3_t<Scalar>& point);
  // All updates return a new value. Call only on accepted, previously uncounted
  // evidence in the owning Node transaction; provenance belongs to that owner.
  PointSummary with_observation(const camera::StereoObservation&, const StateGroupd& node_from_capture,
                                const Vec3_t<Scalar>& point) const;
  PointSummary merged(const PointSummary& source, const StateGroupd& target_from_source, const Vec3_t<Scalar>& point) const;
  // A coordinate change transports only reference; it adds no information.
  PointSummary reanchored(const StateGroupd& new_from_old) const;

 private:
  Vec3_t<Scalar> coordinates(const Vec3_t<Scalar>& point, Mat33_t<Scalar>* jacobian = nullptr) const;
  PointSummary appended(const Vec3_t<Scalar>& point, const Vec3_t<Scalar>& error, const Mat33_t<Scalar>& jacobian) const;
};

}  // namespace sapphire::backend::featmap
