#pragma once

#include <backend/common/state_group.hpp>
#include <optional>

namespace sapphire::backend {

class Link {
 public:
  using Scalar = double;

  enum Type : int {
    kOdometry = 0,     ///< Trajectory backbone, including pure-odometry composition.
    kLoopClosure = 1,  ///< Verified node-to-node measurement; search origin is logged by the producer.
    kPosePrior = 7,    ///< Absolute pose prior (from == to).
    kGravity = 9,      ///< Gravity direction (from == to).
    kUndef = 99        ///< Default invalid object, never persisted.
  };

  static constexpr bool isConstraintType(Type type) {
    switch (type) {
      case kOdometry:
      case kLoopClosure:
      case kPosePrior:
      case kGravity:
        return true;
      default:
        return false;
    }
  }
  bool isOdometry() const { return type_ == kOdometry; }
  bool isLoopClosure() const { return type_ == kLoopClosure; }

  static const char* typeName(Type type) noexcept;

  Link();

  Link(int from, int to, Type type, const StateGroupf& transform, const Mat66_t<Scalar>& infMatrix = Mat66_t<Scalar>::Identity());

  bool isValid() const { return from_ > 0 && to_ > 0 && !transform_.isNull() && isConstraintType(type_); }

  /** @return Source node id. */
  int from() const { return from_; }
  /** @return Target node id (positive capture ID). */
  int to() const { return to_; }
  /** @return Relative transform, from the @ref from() frame to the @ref to() frame. */
  const StateGroupf& transform() const { return transform_; }
  /** @return Link category (see @ref Type). */
  Type type() const { return type_; }
  const char* typeName() const noexcept { return typeName(type_); }
  /** Full SE(3) pose noise uses the target-local right tangent [translation, rotation], in metres/radians. */
  const Mat66_t<Scalar>& infMatrix() const { return infMatrix_; }

  /** @brief Minimum/maximum marginal rotation variance. Maximum ignores axes with variance >= 10000. */
  double rotVariance(bool minimum = true) const;
  /** @brief Minimum/maximum marginal translation variance. Maximum ignores axes with variance >= 10000. */
  double transVariance(bool minimum = true) const;

  void setFrom(int from) { from_ = from; }
  void setTo(int to) { to_ = to; }
  void setTransform(const StateGroupf& transform) { transform_ = transform; }
  void setType(Type type);
  /** @brief Sets the 6x6 double information matrix; must be finite, symmetric and positive definite. */
  void setInfMatrix(const Mat66_t<Scalar>& infMatrix);

  Link merge(const Link& link) const;
  /** @return Inverse pose constraint, with transported information. */
  Link inverse() const;

  /** Re-express an incident factor under X_removed = X_retained * retained_from_removed.
   *  The anchor change is deterministic; this is not probabilistic marginalization.
   */
  Link reanchor(int retained, int removed, const StateGroupf& retained_from_removed) const;

 private:
  double marginalVariance(int offset, bool minimum) const;

  int from_;
  int to_;
  StateGroupf transform_;
  Type type_;
  Mat66_t<Scalar> infMatrix_;  // Information matrix = covariance matrix ^ -1
};

}  // namespace sapphire::backend
