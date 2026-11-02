#include "solver/lm_optimizer.hpp"

#include <cassert>
#include <iostream>
#include <stdexcept>

using namespace sapphire;

LidarFactor make_factors(int count) {
  LidarFactor factor(2);
  for (int voxel = 0; voxel < count; ++voxel) {
    std::vector<PointCluster> clusters(2);
    for (int frame = 0; frame < 2; ++frame) {
      for (int point = 0; point < 24; ++point) {
        const double x = .15 * (point % 6), y = .3 * (point / 6);
        clusters[frame].push(Eigen::Vector3d(x + .07 * voxel, y + .1 * frame, .04 * x + .02 * y + .001 * (point % 3)));
      }
    }
    PointCluster fixed, total;
    Eigen::Vector3d eigenvalues = Eigen::Vector3d::Zero();
    Eigen::Matrix3d eigenvectors = Eigen::Matrix3d::Identity();
    factor.push_voxel(clusters, fixed, 1 + .03 * voxel);
  }
  return factor;
}

int main() {
  std::vector<StateGroup> states(2);
  states[1].p = Eigen::Vector3d(.1, -.2, .03);
  states[1].R = lie::SO3d::exp(Eigen::Vector3d(.02, -.03, .01)).R();
  {
    auto factor = make_factors(4);
    Eigen::MatrixXd h0 = Eigen::MatrixXd::Zero(12, 12), h1 = h0;
    Eigen::VectorXd g0 = Eigen::VectorXd::Zero(12), g1 = g0;
    double r0, r1, trial_cost;
    factor.acc_evaluate2(states, 0, 4, h0, g0, r0);
    auto trial = states; trial[1].p += Eigen::Vector3d(.7, -.2, .9);
    factor.evaluate_only_residual(trial, 0, 4, trial_cost);
    factor.acc_evaluate2(states, 0, 4, h1, g1, r1);
    assert((h0-h1).norm() == 0 && (g0-g1).norm() == 0 && r0 == r1);
    for (int axis = 0; axis < 6; ++axis) {
      auto plus = states, minus = states;
      constexpr double eps = 1e-6;
      if (axis < 3) {
        Eigen::Vector3d delta = Eigen::Vector3d::Zero(); delta[axis] = eps;
        plus[1].R = states[1].R * lie::SO3d::exp(delta).R();
        minus[1].R = states[1].R * lie::SO3d::exp(-delta).R();
      } else { plus[1].p[axis-3] += eps; minus[1].p[axis-3] -= eps; }
      double rp, rm; factor.evaluate_only_residual(plus, 0, 4, rp); factor.evaluate_only_residual(minus, 0, 4, rm);
      assert(std::abs((rp-rm)/(2*eps) - g0[6+axis]) < 1e-7);
    }
  }
  for (size_t threads : {1, 2, 4}) {
    tools::ParallelExecutor<> executor(threads);
    LidarLmWorkspace workspace(executor);
    // Reuse workspace after a larger batch, including the no-LiDAR case.
    for (int count : {33, 17, 9, 8, 7, 1, 0}) {
      auto factor = make_factors(count), reference = factor;
      double expected_residual = 0;
      reference.evaluate_only_residual(states, 0, count, expected_residual);
      int imu_calls = 0;
      const double residual = workspace.evaluate_residual(states, factor, [&] { ++imu_calls; });
      assert(imu_calls == 1);
      assert(std::abs(residual - expected_residual) < 1e-11);
      Eigen::MatrixXd pose_hessian = Eigen::MatrixXd::Zero(12, 12);
      Eigen::VectorXd pose_jacobian = Eigen::VectorXd::Zero(12);
      reference.acc_evaluate2(states, 0, count, pose_hessian, pose_jacobian, expected_residual);
      Eigen::MatrixXd expected_hessian = Eigen::MatrixXd::Identity(30, 30);
      Eigen::VectorXd expected_jacobian = Eigen::VectorXd::Constant(30, .25);
      for (int row = 0; row < 2; ++row) {
        expected_jacobian.segment<6>(15 * row) += pose_jacobian.segment<6>(6 * row);
        for (int column = 0; column < 2; ++column)
          expected_hessian.block<6, 6>(15 * row, 15 * column) += pose_hessian.block<6, 6>(6 * row, 6 * column);
      }
      Eigen::MatrixXd hessian(30, 30);
      Eigen::VectorXd jacobian(30);
      const double actual = workspace.linearize(2, states, factor, hessian, jacobian, [&] {
        ++imu_calls;
        hessian.setIdentity();
        jacobian.setConstant(.25);
      });
      assert(imu_calls == 2);
      assert(hessian.allFinite() && jacobian.allFinite());
      assert((hessian - expected_hessian).norm() < 1e-10 * std::max(1., expected_hessian.norm()));
      assert((jacobian - expected_jacobian).norm() < 1e-10 * std::max(1., expected_jacobian.norm()));
      assert(std::abs(actual - expected_residual) < 1e-11);

      // A failed IMU task must settle the accepted LiDAR tasks before rethrow,
      // and the executor/workspace must remain reusable for the next batch.
      factor = make_factors(count);
      bool caught = false;
      try {
        workspace.evaluate_residual(states, factor, [] { throw std::runtime_error("IMU failure"); });
      } catch (const std::runtime_error &) { caught = true; }
      assert(caught);
      for (int i = 0; i < count; ++i) {
        assert(factor.world_moments(states, i).N == 48);
        assert((factor.world_moments(states, i).P - reference.world_moments(states, i).P).norm() < 1e-12);
      }
    }
  }
  std::cout << "LM range reduction, IMU completion, empty batches and exception settlement passed\n";
}
