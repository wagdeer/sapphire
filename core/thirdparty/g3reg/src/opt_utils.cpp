/**
 * Derived from G3Reg, Zhijian QIAO / HKUST Aerial Robotics.
 * Sapphire retains the serial FPFH/PAGOR/GNC route.
 */
#include "opt_utils.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
using gtsam::symbol_shorthand::X;
namespace gtsam {
Eigen::Matrix4d gncSE3(const Eigen::Matrix3Xd &src, const Eigen::Matrix3Xd &tgt, Eigen::Matrix4d init) {
  // Create a factor graph
  NonlinearFactorGraph graph;
  Values initial;
  initial.insert(X(0), Pose3(init));
  noiseModel::Diagonal::shared_ptr noise = noiseModel::Unit::Create(3);
  assert(src.cols() == tgt.cols());
  for (int i = 0; i < src.cols(); ++i) {
    graph.add(Point2PointFactor(X(0), src.col(i), tgt.col(i), noise));
  }
  GncParams<LevenbergMarquardtParams> gncParams;
  auto gnc = GncOptimizer<GncParams<LevenbergMarquardtParams>>(graph, initial, gncParams);
  Values estimate = gnc.optimize();
  Eigen::Matrix4d T = estimate.at<Pose3>(X(0)).matrix();
  return T;
}
Eigen::Matrix4d svdSE3(const Eigen::Matrix3Xd &src, const Eigen::Matrix3Xd &tgt) {
  Eigen::Vector3d src_mean = src.rowwise().mean();
  Eigen::Vector3d tgt_mean = tgt.rowwise().mean();
  const Eigen::Matrix3Xd &src_centered = src - src_mean.replicate(1, src.cols());
  const Eigen::Matrix3Xd &tgt_centered = tgt - tgt_mean.replicate(1, tgt.cols());
  Eigen::MatrixXd H = src_centered * tgt_centered.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd R_ = svd.matrixV() * svd.matrixU().transpose();
  if (R_.determinant() < 0) {
    Eigen::MatrixXd V = svd.matrixV();
    V.col(2) *= -1;
    R_ = V * svd.matrixU().transpose();
  }
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block(0, 0, 3, 3) = R_;
  T.block(0, 3, 3, 1) = tgt_mean - R_ * src_mean;
  return T;
}
Vector Point2PointFactor::evaluateError(const Pose3 &X, OptionalMatrixType H) const {
  const gtsam::Rot3 &R = X.rotation();
  gtsam::Vector3 mx = R * msrc_ + X.translation();
  gtsam::Vector3 error = mx - mtgt_;
  if (H) {
    *H = gtsam::Matrix(3, 6);
    (*H).block(0, 0, 3, 3) = -X.rotation().matrix() * skewSymmetric(msrc_);
    (*H).block(0, 3, 3, 3) = X.rotation().matrix();
  }
  return error;
}
}  // namespace gtsam
