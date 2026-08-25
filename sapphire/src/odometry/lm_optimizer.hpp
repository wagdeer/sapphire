#pragma once

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <vector>

#include "common.hpp"
#include "imu_factor.hpp"
#include "lidar_factor.hpp"
#include "parallel_executor.hpp"
#include "parameters.h"

namespace sapphire {

class LidarLmWorkspace final {
 public:
  explicit LidarLmWorkspace(ParallelExecutor &executor)
      : executor_(executor), hessians_(executor.thread_count()), jacobians_(executor.thread_count()), residuals_(executor.thread_count(), 0.0) {}

  double linearize(int window_size, const std::vector<StateGroup> &states, LidarFactor &factor, Eigen::MatrixXd &hessian, Eigen::VectorXd &jacobian,
                   const ParallelExecutor::ForegroundTask &foreground_task) {
    prepare(window_size * POSE_DOF);
    const size_t active_workers = executor_.parallel_for(
        factor.plvec_voxels.size(), kMinimumFactorsPerWorker,
        [&](size_t worker_index, size_t begin, size_t end) {
          factor.acc_evaluate2(states, static_cast<int>(begin), static_cast<int>(end), hessians_[worker_index], jacobians_[worker_index],
                               residuals_[worker_index]);
        },
        foreground_task);

    double residual = 0.0;
    for (size_t worker_index = 0; worker_index < active_workers; ++worker_index) {
      add_to_system(window_size, hessians_[worker_index], jacobians_[worker_index], hessian, jacobian);
      residual += residuals_[worker_index];
    }
    return residual;
  }

  double evaluate_residual(const std::vector<StateGroup> &states, LidarFactor &factor, const ParallelExecutor::ForegroundTask &foreground_task) {
    const size_t active_workers = executor_.parallel_for(
        factor.plvec_voxels.size(), kMinimumFactorsPerWorker,
        [&](size_t worker_index, size_t begin, size_t end) {
          factor.evaluate_only_residual(states, static_cast<int>(begin), static_cast<int>(end), residuals_[worker_index]);
        },
        foreground_task);

    double residual = 0.0;
    for (size_t worker_index = 0; worker_index < active_workers; ++worker_index) {
      residual += residuals_[worker_index];
    }
    return residual;
  }

 private:
  static constexpr size_t kMinimumFactorsPerWorker = 8;

  void prepare(int jacobian_size) {
    if (hessians_.front().rows() == jacobian_size) {
      return;
    }
    for (size_t worker_index = 0; worker_index < hessians_.size(); ++worker_index) {
      hessians_[worker_index].resize(jacobian_size, jacobian_size);
      jacobians_[worker_index].resize(jacobian_size);
    }
  }

  static void add_to_system(int window_size, const Eigen::MatrixXd &source_hessian, const Eigen::VectorXd &source_jacobian, Eigen::MatrixXd &hessian,
                            Eigen::VectorXd &jacobian) {
    for (int row = 0; row < window_size; ++row) {
      jacobian.block<POSE_DOF, 1>(row * STATE_DOF, 0) += source_jacobian.block<POSE_DOF, 1>(row * POSE_DOF, 0);
      for (int column = 0; column < window_size; ++column) {
        hessian.block<POSE_DOF, POSE_DOF>(row * STATE_DOF, column * STATE_DOF) +=
            source_hessian.block<POSE_DOF, POSE_DOF>(row * POSE_DOF, column * POSE_DOF);
      }
    }
  }

  ParallelExecutor &executor_;
  std::vector<Eigen::MatrixXd> hessians_;
  std::vector<Eigen::VectorXd> jacobians_;
  std::vector<double> residuals_;
};

class LI_BA_Optimizer {
 public:
  LI_BA_Optimizer(const LocalSubmapParameters &parameters, ParallelExecutor &executor)
      : imu_coefficient_(parameters.imu_coef), lidar_workspace_(executor) {}

  void damping_iter(std::vector<StateGroup> &states, LidarFactor &lidar_factor, std::deque<ImuFactor *> &imu_factors,
                    Eigen::MatrixXd *output_hessian) {
    const int window_size = lidar_factor.win_size;
    const int system_size = window_size * STATE_DOF;
    damping_.setIdentity(system_size, system_size);
    hessian_.resize(system_size, system_size);
    jacobian_.resize(system_size);
    increment_.resize(system_size);
    imu_hessian_.resize(2 * STATE_DOF, 2 * STATE_DOF);
    imu_jacobian_.resize(2 * STATE_DOF);
    output_hessian->resize(system_size, system_size);
    trial_states_ = states;

    double damping = 0.01;
    double damping_scale = 2.0;
    double current_residual = 0.0;
    bool needs_linearization = true;

    for (int iteration = 0; iteration < 3; ++iteration) {
      if (needs_linearization) {
        current_residual = linearize(window_size, states, lidar_factor, imu_factors);
        *output_hessian = hessian_;
      }

      hessian_.topRows(STATE_DOF).setZero();
      hessian_.leftCols(STATE_DOF).setZero();
      hessian_.block<STATE_DOF, STATE_DOF>(0, 0).setIdentity();
      jacobian_.head(STATE_DOF).setZero();
      damping_.diagonal() = hessian_.diagonal();
      increment_ = (hessian_ + damping * damping_).ldlt().solve(-jacobian_);

      for (int index = 0; index < window_size; ++index) {
        trial_states_[index].R = states[index].R * lie::SO3d::exp(increment_.block<3, 1>(STATE_DOF * index, 0)).R();
        trial_states_[index].p = states[index].p + increment_.block<3, 1>(STATE_DOF * index + 3, 0);
        trial_states_[index].v = states[index].v + increment_.block<3, 1>(STATE_DOF * index + 6, 0);
        trial_states_[index].bg = states[index].bg + increment_.block<3, 1>(STATE_DOF * index + 9, 0);
        trial_states_[index].ba = states[index].ba + increment_.block<3, 1>(STATE_DOF * index + 12, 0);
      }
      for (int index = 0; index < window_size - 1; ++index) {
        imu_factors[index]->update_state(increment_.block<STATE_DOF, 1>(STATE_DOF * index, 0));
      }

      const double predicted_reduction = 0.5 * increment_.dot(damping * damping_ * increment_ - jacobian_);
      const double trial_residual = evaluate_residual(window_size, trial_states_, lidar_factor, imu_factors);
      const double actual_reduction = current_residual - trial_residual;
      if (actual_reduction > 0.0) {
        states = trial_states_;
        const double gain = actual_reduction / predicted_reduction;
        damping *= std::max(1.0 / 3.0, 1.0 - std::pow(2.0 * gain - 1.0, 3));
        damping_scale = 2.0;
        needs_linearization = true;
      } else {
        damping *= damping_scale;
        damping_scale *= 2.0;
        needs_linearization = false;
        restore_imu_states(imu_factors, window_size);
      }
      if (current_residual != 0.0 && std::fabs(actual_reduction / current_residual) < 1e-6) {
        break;
      }
    }
  }

 private:
  double linearize(int window_size, const std::vector<StateGroup> &states, LidarFactor &lidar_factor, const std::deque<ImuFactor *> &imu_factors) {
    hessian_.setZero();
    jacobian_.setZero();
    double imu_residual = 0.0;
    const double lidar_residual = lidar_workspace_.linearize(window_size, states, lidar_factor, hessian_, jacobian_, [&] {
      for (int index = 0; index < window_size - 1; ++index) {
        imu_hessian_.setZero();
        imu_jacobian_.setZero();
        imu_residual += imu_factors[index]->give_evaluate(states[index], states[index + 1], imu_hessian_, imu_jacobian_, true);
        hessian_.block<2 * STATE_DOF, 2 * STATE_DOF>(index * STATE_DOF, index * STATE_DOF) += imu_hessian_;
        jacobian_.block<2 * STATE_DOF, 1>(index * STATE_DOF, 0) += imu_jacobian_;
      }
      hessian_ *= imu_coefficient_;
      jacobian_ *= imu_coefficient_;
      imu_residual *= imu_coefficient_ * 0.5;
    });
    return imu_residual + lidar_residual;
  }

  double evaluate_residual(int window_size, const std::vector<StateGroup> &states, LidarFactor &lidar_factor,
                           const std::deque<ImuFactor *> &imu_factors) {
    double imu_residual = 0.0;
    const double lidar_residual = lidar_workspace_.evaluate_residual(states, lidar_factor, [&] {
      for (int index = 0; index < window_size - 1; ++index) {
        imu_residual += imu_factors[index]->give_evaluate(states[index], states[index + 1], imu_hessian_, imu_jacobian_, false);
      }
      imu_residual *= imu_coefficient_ * 0.5;
    });
    return imu_residual + lidar_residual;
  }

  static void restore_imu_states(const std::deque<ImuFactor *> &imu_factors, int window_size) {
    for (int index = 0; index < window_size - 1; ++index) {
      imu_factors[index]->dbg = imu_factors[index]->dbg_buf;
      imu_factors[index]->dba = imu_factors[index]->dba_buf;
    }
  }

  const double imu_coefficient_;
  LidarLmWorkspace lidar_workspace_;
  Eigen::MatrixXd damping_;
  Eigen::MatrixXd hessian_;
  Eigen::VectorXd jacobian_;
  Eigen::VectorXd increment_;
  Eigen::MatrixXd imu_hessian_;
  Eigen::VectorXd imu_jacobian_;
  std::vector<StateGroup> trial_states_;
};

class LI_BA_OptimizerGravity {
 public:
  LI_BA_OptimizerGravity(const LocalSubmapParameters &parameters, ParallelExecutor &executor)
      : imu_coefficient_(parameters.imu_coef), lidar_workspace_(executor) {}

  void damping_iter(std::vector<StateGroup> &states, LidarFactor &lidar_factor, std::deque<ImuFactor *> &imu_factors, std::vector<double> &residuals,
                    Eigen::MatrixXd *output_hessian, int max_iterations = 2) {
    const int window_size = lidar_factor.win_size;
    const int system_size = window_size * STATE_DOF + 3;
    damping_.setIdentity(system_size, system_size);
    hessian_.resize(system_size, system_size);
    jacobian_.resize(system_size);
    increment_.resize(system_size);
    imu_hessian_.resize(2 * STATE_DOF + 3, 2 * STATE_DOF + 3);
    imu_jacobian_.resize(2 * STATE_DOF + 3);
    trial_states_ = states;

    double damping = 0.01;
    double damping_scale = 2.0;
    double current_residual = 0.0;
    double trial_residual = 0.0;
    bool needs_linearization = true;

    for (int iteration = 0; iteration < max_iterations; ++iteration) {
      if (needs_linearization) {
        current_residual = linearize(window_size, states, lidar_factor, imu_factors);
        *output_hessian = hessian_;
      }
      if (iteration == 0) {
        residuals.push_back(current_residual);
      }

      hessian_.topRows(POSE_DOF).setZero();
      hessian_.leftCols(POSE_DOF).setZero();
      hessian_.block<POSE_DOF, POSE_DOF>(0, 0).setIdentity();
      jacobian_.head(POSE_DOF).setZero();
      damping_.diagonal() = hessian_.diagonal();
      increment_ = (hessian_ + damping * damping_).ldlt().solve(-jacobian_);

      trial_states_[0].g = states[0].g + increment_.tail(3);
      for (int index = 0; index < window_size; ++index) {
        trial_states_[index].R = states[index].R * lie::SO3d::exp(increment_.block<3, 1>(STATE_DOF * index, 0)).R();
        trial_states_[index].p = states[index].p + increment_.block<3, 1>(STATE_DOF * index + 3, 0);
        trial_states_[index].v = states[index].v + increment_.block<3, 1>(STATE_DOF * index + 6, 0);
        trial_states_[index].bg = states[index].bg + increment_.block<3, 1>(STATE_DOF * index + 9, 0);
        trial_states_[index].ba = states[index].ba + increment_.block<3, 1>(STATE_DOF * index + 12, 0);
        trial_states_[index].g = trial_states_[0].g;
      }
      for (int index = 0; index < window_size - 1; ++index) {
        imu_factors[index]->update_state(increment_.block<STATE_DOF, 1>(STATE_DOF * index, 0));
      }

      const double predicted_reduction = 0.5 * increment_.dot(damping * damping_ * increment_ - jacobian_);
      trial_residual = evaluate_residual(window_size, trial_states_, lidar_factor, imu_factors);
      const double actual_reduction = current_residual - trial_residual;
      if (actual_reduction > 0.0) {
        states = trial_states_;
        const double gain = actual_reduction / predicted_reduction;
        damping *= std::max(1.0 / 3.0, 1.0 - std::pow(2.0 * gain - 1.0, 3));
        damping_scale = 2.0;
        needs_linearization = true;
      } else {
        damping *= damping_scale;
        damping_scale *= 2.0;
        needs_linearization = false;
        restore_imu_states(imu_factors, window_size);
      }
      if (current_residual != 0.0 && std::fabs(actual_reduction / current_residual) < 1e-6) {
        break;
      }
    }
    residuals.push_back(trial_residual);
  }

 private:
  double linearize(int window_size, const std::vector<StateGroup> &states, LidarFactor &lidar_factor, const std::deque<ImuFactor *> &imu_factors) {
    hessian_.setZero();
    jacobian_.setZero();
    const int gravity_offset = window_size * STATE_DOF;
    double imu_residual = 0.0;
    const double lidar_residual = lidar_workspace_.linearize(window_size, states, lidar_factor, hessian_, jacobian_, [&] {
      for (int index = 0; index < window_size - 1; ++index) {
        imu_hessian_.setZero();
        imu_jacobian_.setZero();
        imu_residual += imu_factors[index]->give_evaluate_g(states[index], states[index + 1], imu_hessian_, imu_jacobian_, true);
        hessian_.block<2 * STATE_DOF, 2 * STATE_DOF>(index * STATE_DOF, index * STATE_DOF) += imu_hessian_.block<2 * STATE_DOF, 2 * STATE_DOF>(0, 0);
        hessian_.block<2 * STATE_DOF, 3>(index * STATE_DOF, gravity_offset) += imu_hessian_.block<2 * STATE_DOF, 3>(0, 2 * STATE_DOF);
        hessian_.block<3, 2 * STATE_DOF>(gravity_offset, index * STATE_DOF) += imu_hessian_.block<3, 2 * STATE_DOF>(2 * STATE_DOF, 0);
        hessian_.block<3, 3>(gravity_offset, gravity_offset) += imu_hessian_.block<3, 3>(2 * STATE_DOF, 2 * STATE_DOF);
        jacobian_.block<2 * STATE_DOF, 1>(index * STATE_DOF, 0) += imu_jacobian_.head(2 * STATE_DOF);
        jacobian_.tail(3) += imu_jacobian_.tail(3);
      }
      hessian_ *= imu_coefficient_;
      jacobian_ *= imu_coefficient_;
      imu_residual *= imu_coefficient_ * 0.5;
    });
    return imu_residual + lidar_residual;
  }

  double evaluate_residual(int window_size, const std::vector<StateGroup> &states, LidarFactor &lidar_factor,
                           const std::deque<ImuFactor *> &imu_factors) {
    double imu_residual = 0.0;
    const double lidar_residual = lidar_workspace_.evaluate_residual(states, lidar_factor, [&] {
      for (int index = 0; index < window_size - 1; ++index) {
        imu_residual += imu_factors[index]->give_evaluate_g(states[index], states[index + 1], imu_hessian_, imu_jacobian_, false);
      }
      imu_residual *= imu_coefficient_ * 0.5;
    });
    return imu_residual + lidar_residual;
  }

  static void restore_imu_states(const std::deque<ImuFactor *> &imu_factors, int window_size) {
    for (int index = 0; index < window_size - 1; ++index) {
      imu_factors[index]->dbg = imu_factors[index]->dbg_buf;
      imu_factors[index]->dba = imu_factors[index]->dba_buf;
    }
  }

  const double imu_coefficient_;
  LidarLmWorkspace lidar_workspace_;
  Eigen::MatrixXd damping_;
  Eigen::MatrixXd hessian_;
  Eigen::VectorXd jacobian_;
  Eigen::VectorXd increment_;
  Eigen::MatrixXd imu_hessian_;
  Eigen::VectorXd imu_jacobian_;
  std::vector<StateGroup> trial_states_;
};

}  // namespace sapphire
