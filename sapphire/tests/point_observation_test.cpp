#include "common/common.hpp"

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "frontend/eskf/voxel_map.hpp"

using namespace sapphire;
using M6 = Eigen::Matrix<double, 6, 6>;

// Adding computation to the observation must not add per-point runtime state.
struct alignas(16) PointStorage { Eigen::Vector3d pnt; Eigen::Matrix3d var; };
static_assert(sizeof(pointVar) == sizeof(PointStorage));

int main() {
  std::srand(19);
  double max_jacobian_error = 0.0;
  for (int sample = 0; sample < 100; ++sample) {
    const Eigen::Matrix3d R = lie::SO3d::exp(Eigen::Vector3d::Random()).R();
    const M6 A = M6::Random(), P = A * A.transpose();
    pointVar point;
    point.pnt = Eigen::Vector3d::Random() * 10.0;
    const Eigen::Matrix3d B = Eigen::Matrix3d::Random();
    point.var = B * B.transpose() * .02;
    const Eigen::Vector3d normal = Eigen::Vector3d::Random().normalized();
    Eigen::Matrix<double, POSE_DOF, 1> jacobian;
    const double variance = point.projectVariance(normal, R, P, jacobian);

    // Full world-covariance oracle includes nonzero pose cross covariance.
    Eigen::Matrix<double, 3, 6> J;
    J << -R * lie::SO3d::wedge(point.pnt), Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d C = R * point.var * R.transpose() + J * P * J.transpose();
    assert(std::abs(variance - normal.dot(C * normal)) < 1e-10);

    // Independently differentiate right attitude and world position errors.
    constexpr double eps = 1e-6;
    for (int axis = 0; axis < 6; ++axis) {
      Eigen::Vector3d plus, minus;
      const Eigen::Vector3d delta = Eigen::Vector3d::Unit(axis % 3) * eps;
      if (axis < 3) {
        plus = R * lie::SO3d::exp(delta).R() * point.pnt;
        minus = R * lie::SO3d::exp(-delta).R() * point.pnt;
      } else {
        plus = R * point.pnt + delta;
        minus = R * point.pnt - delta;
      }
      max_jacobian_error = std::max(max_jacobian_error, std::abs(normal.dot(plus - minus) / (2 * eps) - jacobian[axis]));
    }

    // A change of world coordinates must not change the residual variance.
    const Eigen::Matrix3d Q = lie::SO3d::exp(Eigen::Vector3d::Random()).R();
    M6 T = M6::Identity();
    T.bottomRightCorner<3, 3>() = Q;
    const M6 rotated_covariance = T * P * T.transpose();
    const Eigen::Matrix3d rotated_R = Q * R;
    assert(std::abs(point.projectVariance(Q * normal, rotated_R, rotated_covariance, jacobian) - variance) < 1e-10);
  }
  assert(max_jacobian_error < 1e-8);

  // Check the recursive matcher passes the uncertainty and winning Jacobian
  // through to its caller; a distant point must still fail the existing gate.
  OctoTree root(0, 5);
  root.octo_state = 1;
  for (double &value : root.voxel_center) value = 0;
  root.leaves[7] = std::make_unique<OctoTree>(1, 5);
  Plane &plane = root.leaves[7]->plane;
  plane.is_plane = true;
  plane.normal = Eigen::Vector3d::UnitZ();
  plane.center = Eigen::Vector3d(1, 2, 3);
  plane.radius = 100;
  plane.plane_var.setZero();
  plane.plane_var.bottomRightCorner<3, 3>().setIdentity();
  plane.plane_var *= .01;
  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  const M6 P = M6::Identity() * .001;
  pointVar point;
  point.pnt = plane.center;
  point.var = Eigen::Matrix3d::Identity() * .002;
  Eigen::Matrix<double, POSE_DOF, 1> jacobian;
  const double expected = .01 + point.projectVariance(plane.normal, R, P, jacobian);
  Plane *matched = nullptr;
  OctoTree *node = nullptr;
  double probability = 0, variance = 0;
  Eigen::Vector3d world = plane.center;
  assert(root.match(world, matched, probability, point, R, P, jacobian, variance, node) == 1);
  assert(matched == &plane && node == root.leaves[7].get());
  assert(std::abs(variance - expected) < 1e-12);
  world.z() += 4 * std::sqrt(expected);
  assert(root.match(world, matched, probability, point, R, P, jacobian, variance, node) == 0);
  std::cout << "PASS: covariance oracle, world-frame invariance, recursive matching; Jacobian error " << max_jacobian_error << '\n';
}
