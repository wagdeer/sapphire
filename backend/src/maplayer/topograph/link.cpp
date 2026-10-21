#include "backend/maplayer/topograph/link.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <backend/common/common_type.hpp>
#include <backend/tools/logging.hpp>
#include <cmath>
#include <stdexcept>

namespace sapphire::backend {

const char* Link::typeName(Type type) noexcept {
  switch (type) {
    case kOdometry:
      return "Odometry";
    case kLoopClosure:
      return "LoopClosure";
    case kPosePrior:
      return "PosePrior";
    case kGravity:
      return "Gravity";
    default:
      return "Undefined";
  }
}

void Link::setType(Type type) {
  if (!isConstraintType(type)) throw std::invalid_argument("Unsupported Link type: " + std::to_string(int(type)));
  type_ = type;
}

Link::Link() : from_(0), to_(0), type_(kUndef), infMatrix_(Mat66_t<Scalar>::Identity()) {}
Link::Link(int from, int to, Type type, const StateGroupf& transform, const Mat66_t<Scalar>& infMatrix)
    : from_(from), to_(to), transform_(transform), type_(type) {
  setType(type);
  setInfMatrix(infMatrix);
}

double Link::marginalVariance(int offset, bool minimum) const {
  Vec3_t<Scalar> variances;
  // Reciprocal information diagonals are marginal variances only for diagonal noise.
  if (infMatrix_.isDiagonal(0.0))
    variances = infMatrix_.diagonal().segment<3>(offset).cwiseInverse();
  else
    variances = infMatrix_.llt().solve(Mat66_t<double>::Identity()).diagonal().segment<3>(offset);
  EXT_ASSERT(variances.allFinite() && (variances.array() > 0).all());
  if (minimum) return variances.minCoeff();
  // Preserve the ignored-axis convention for maximum variance queries.
  double maximum = 0;
  for (double variance : variances)
    if (variance < 10000.0) maximum = std::max(maximum, variance);
  return maximum > 0 ? maximum : 10000.0;
}

double Link::rotVariance(bool minimum) const { return marginalVariance(3, minimum); }
double Link::transVariance(bool minimum) const { return marginalVariance(0, minimum); }

void Link::setInfMatrix(const Mat66_t<Scalar>& infMatrix) {
  const Mat66_t<Scalar>& matrix = infMatrix;
  if (!matrix.allFinite() || !matrix.isApprox(matrix.transpose(), 1e-8)) throw std::invalid_argument("Link information must be finite and symmetric");
  const Mat66_t<Scalar> symmetric = (matrix + matrix.transpose()) * 0.5;
  if (Eigen::LLT<Mat66_t<Scalar>>(symmetric).info() != Eigen::Success) throw std::invalid_argument("Link information must be positive definite");
  // Own the buffer: later changes to a caller's covariance must not alter a factor.
  infMatrix_ = symmetric;
}

Link Link::merge(const Link& link) const {
  EXT_ASSERT_MSG(to_ == link.from(), "{}", fmt::format("merging this={}->{} to link={}->{}", from_, to_, link.from(), link.to()));
  if (from_ == to_ || link.from_ == link.to_ || !isOdometry() || !link.isOdometry())
    throw std::invalid_argument("Only odometry paths can be composed; node fusion requires reanchor");
  const Type outputType = kOdometry;
  EXT_ASSERT((link.transform().isNull() && transform_.isNull()) || (!link.transform().isNull() && !transform_.isNull()));
  if (transform_.isNull()) return Link(from_, link.to(), outputType, StateGroupf());
  // Independent measurements, first-order propagation in the target-local
  // right tangent. The second leg transports the first leg's uncertainty.
  const Mat66_t<Scalar> adjoint = StateGroupd(link.transform()).inverse().adjoint();
  const Mat66_t<Scalar> covariance =
      adjoint * infMatrix_.llt().solve(Mat66_t<double>::Identity()) * adjoint.transpose() + link.infMatrix().llt().solve(Mat66_t<double>::Identity());
  Link result(from_, link.to(), outputType, transform_.compose(link.transform()), covariance.llt().solve(Mat66_t<double>::Identity()));
  return result;
}

Link Link::inverse() const {
  Link result = *this;
  std::swap(result.from_, result.to_);
  result.transform_ = transform_.inverse();
  if (!transform_.isNull()) {
    const Mat66_t<Scalar> adjoint = StateGroupd(transform_).inverse().adjoint();
    result.setInfMatrix(adjoint.transpose() * infMatrix_ * adjoint);
  }
  return result;
}

Link Link::reanchor(int retained, int removed, const StateGroupf& retained_from_removed) const {
  if (from_ != removed && to_ != removed) return *this;
  if (retained == removed || retained_from_removed.isNull()) throw std::invalid_argument("Invalid link reanchor");
  Link result = *this;
  const auto inverse = retained_from_removed.inverse();
  if (from_ == removed && to_ == removed) {
    // Unary measurements are world_from_body, not between(body, body).
    result.from_ = result.to_ = retained;
    result.transform_ = transform_.compose(inverse);
    if (type_ == kGravity) result.transform_ = result.transform_.rotationPart();
  } else {
    if (from_ == removed) {
      result.from_ = retained;
      result.transform_ = retained_from_removed.compose(result.transform_);
    }
    if (to_ == removed) {
      result.to_ = retained;
      result.transform_ = result.transform_.compose(inverse);
    }
  }
  if (!transform_.isNull() && to_ == removed && type_ != kGravity) {
    // Right multiplication by T transports information with Ad(T).
    const Mat66_t<Scalar> adjoint = StateGroupd(retained_from_removed).inverse().adjoint();
    result.setInfMatrix(adjoint.transpose() * infMatrix_ * adjoint);
  }
  return result;
}

}  // namespace sapphire::backend
