#pragma once

#include <Eigen/Eigenvalues>

#include "common/point.hpp"

namespace sapphire {

class LidarFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<PointCluster> sig_vecs;
  std::vector<std::vector<PointCluster>> plvec_voxels;
  std::vector<double> coeffs;
  int win_size;

  LidarFactor(int _w) : win_size(_w) {}

  void push_voxel(const std::vector<PointCluster> &vec_orig, const PointCluster &fix, double coe) {
    plvec_voxels.push_back(vec_orig);
    sig_vecs.push_back(fix);
    coeffs.push_back(coe);
  }

  void acc_evaluate2(const std::vector<StateGroup> &xs, int head, int end, Eigen::MatrixXd &Hess, Eigen::VectorXd &JacT, double &residual) const {
    Hess.setZero();
    JacT.setZero();
    residual = 0;
    std::vector<PointCluster> sig_tran(win_size);
    const int kk = 0;

    vvec<double, 3> viRiTuk(win_size);
    vmat<double, 3> viRiTukukT(win_size);

    std::vector<Eigen::Matrix<double, 3, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 6>>> Auk(win_size);
    Eigen::Matrix3d umumT;

    for (int a = head; a < end; a++) {
      const std::vector<PointCluster> &sig_orig = plvec_voxels[a];
      double coe = coeffs[a];

      const PointCluster aggregate = world_moments(xs, a);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(aggregate.cov());
      Eigen::Vector3d lmbd = solver.eigenvalues();
      Eigen::Matrix3d U = solver.eigenvectors();
      int NN = aggregate.N;
      Eigen::Vector3d vBar = aggregate.mean();

      Eigen::Vector3d u[3] = {U.col(0), U.col(1), U.col(2)};
      Eigen::Vector3d &uk = u[kk];
      Eigen::Matrix3d ukukT = uk * uk.transpose();
      umumT.setZero();
      for (int i = 0; i < 3; i++) {
        if (i != kk) {
          umumT += 2.0 / (lmbd[kk] - lmbd[i]) * u[i] * u[i].transpose();
        }
      }

      for (int i = 0; i < win_size; i++) {
        if (sig_orig[i].N != 0) {
          Eigen::Matrix3d Pi = sig_orig[i].P;
          Eigen::Vector3d vi = sig_orig[i].v;
          Eigen::Matrix3d Ri = xs[i].R;
          double ni = sig_orig[i].N;

          const Eigen::Matrix3d vihat = lie::SO3d::wedge(vi);
          Eigen::Vector3d RiTuk = Ri.transpose() * uk;
          const Eigen::Matrix3d RiTukhat = lie::SO3d::wedge(RiTuk);

          Eigen::Vector3d PiRiTuk = Pi * RiTuk;
          viRiTuk[i] = vihat * RiTuk;
          viRiTukukT[i] = viRiTuk[i] * uk.transpose();

          Eigen::Vector3d ti_v = xs[i].p - vBar;
          double ukTti_v = uk.dot(ti_v);

          Eigen::Matrix3d combo1 = lie::SO3d::wedge(PiRiTuk) + vihat * ukTti_v;
          Eigen::Vector3d combo2 = Ri * vi + ni * ti_v;
          Auk[i].block<3, 3>(0, 0) = (Ri * Pi + ti_v * vi.transpose()) * RiTukhat - Ri * combo1;
          Auk[i].block<3, 3>(0, 3) = combo2 * uk.transpose() + combo2.dot(uk) * I33<double>;
          Auk[i] /= NN;

          const Eigen::Matrix<double, 6, 1> &jjt = Auk[i].transpose() * uk;
          JacT.block<6, 1>(6 * i, 0) += coe * jjt;

          const Eigen::Matrix3d &HRt = 2.0 / NN * (1.0 - ni / NN) * viRiTukukT[i];
          Eigen::Matrix<double, 6, 6> Hb = Auk[i].transpose() * umumT * Auk[i];
          Hb.block<3, 3>(0, 0) += 2.0 / NN * (combo1 - RiTukhat * Pi) * RiTukhat - 2.0 / NN / NN * viRiTuk[i] * viRiTuk[i].transpose() -
                                  0.5 * lie::SO3d::wedge(jjt.block<3, 1>(0, 0));
          Hb.block<3, 3>(0, 3) += HRt;
          Hb.block<3, 3>(3, 0) += HRt.transpose();
          Hb.block<3, 3>(3, 3) += 2.0 / NN * (ni - ni * ni / NN) * ukukT;

          Hess.block<6, 6>(6 * i, 6 * i) += coe * Hb;
        }
      }

      for (int i = 0; i < win_size - 1; i++) {
        if (sig_orig[i].N != 0) {
          double ni = sig_orig[i].N;
          for (int j = i + 1; j < win_size; j++) {
            if (sig_orig[j].N != 0) {
              double nj = sig_orig[j].N;
              Eigen::Matrix<double, 6, 6> Hb = Auk[i].transpose() * umumT * Auk[j];
              Hb.block<3, 3>(0, 0) += -2.0 / NN / NN * viRiTuk[i] * viRiTuk[j].transpose();
              Hb.block<3, 3>(0, 3) += -2.0 * nj / NN / NN * viRiTukukT[i];
              Hb.block<3, 3>(3, 0) += -2.0 * ni / NN / NN * viRiTukukT[j].transpose();
              Hb.block<3, 3>(3, 3) += -2.0 * ni * nj / NN / NN * ukukT;

              Hess.block<6, 6>(6 * i, 6 * j) += coe * Hb;
            }
          }
        }
      }

      residual += coe * lmbd[kk];
    }

    for (int i = 1; i < win_size; i++) {
      for (int j = 0; j < i; j++) {
        Hess.block<6, 6>(6 * i, 6 * j) = Hess.block<6, 6>(6 * j, 6 * i).transpose();
      }
    }
  }

  // Pure evaluation: rejected trial poses must never leak into map geometry.
  void evaluate_only_residual(const std::vector<StateGroup> &xs, int head, int end, double &residual) const {
    residual = 0;
    for (int a = head; a < end; ++a) {
      const PointCluster aggregate = world_moments(xs, a);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(aggregate.cov());
      residual += coeffs[a] * solver.eigenvalues()[0];
    }
  }

  void clear() {
    sig_vecs.clear();
    plvec_voxels.clear();
    coeffs.clear();
  }

  // Sufficient statistics at an explicit pose argument; never a cached trial.
  PointCluster world_moments(const std::vector<StateGroup> &states, int factor) const {
    PointCluster sum = sig_vecs[factor];
    for (int i = 0; i < win_size; ++i) {
      if (!plvec_voxels[factor][i].N) continue;
      PointCluster transformed;
      transformed.transform(plvec_voxels[factor][i], states[i]);
      sum += transformed;
    }
    return sum;
  }
};

}  // namespace sapphire
