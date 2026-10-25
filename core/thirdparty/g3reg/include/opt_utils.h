/**
 * Derived from G3Reg, Zhijian QIAO / HKUST Aerial Robotics.
 * Sapphire retains the serial FPFH/PAGOR/GNC route.
 */
#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
namespace gtsam {
class Point2PointFactor final : public NoiseModelFactorN<Pose3> {
 public:
  Point2PointFactor(Key key, Eigen::Vector3d source, Eigen::Vector3d target, const SharedNoiseModel& noise)
      : NoiseModelFactorN<Pose3>(noise, key), msrc_(source), mtgt_(target) {}
  NonlinearFactor::shared_ptr clone() const override { return std::make_shared<Point2PointFactor>(*this); }
  Vector evaluateError(const Pose3& X, OptionalMatrixType H = nullptr) const override;

 private:
  Eigen::Vector3d msrc_, mtgt_;
};
Eigen::Matrix4d gncSE3(const Eigen::Matrix3Xd& src, const Eigen::Matrix3Xd& tgt, Eigen::Matrix4d init);
Eigen::Matrix4d svdSE3(const Eigen::Matrix3Xd& src, const Eigen::Matrix3Xd& tgt);
}  // namespace gtsam
