#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

namespace sapphire::solver {
// Position-only prior for a body pose.
class PositionFactor final : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  PositionFactor(gtsam::Key key, const gtsam::Point3& measured, const gtsam::SharedNoiseModel& noise)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(noise, key), measured_(measured) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& value, gtsam::OptionalMatrixType jacobian = {}) const override {
    return value.translation(jacobian) - measured_;
  }

 private:
  gtsam::Point3 measured_;
};
}  // namespace sapphire::solver
