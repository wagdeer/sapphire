#pragma once

#include <Eigen/Eigenvalues>

#include "common.hpp"

namespace sapphire {

class LidarFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<PointCluster> sig_vecs;
  std::vector<std::vector<PointCluster>> plvec_voxels;
  std::vector<double> coeffs;
  vvec<double, 3> eig_values;
  vmat<double, 3> eig_vectors;
  std::vector<PointCluster> pcr_adds;
  int win_size;

  LidarFactor(int _w) : win_size(_w) {}

  void push_voxel(std::vector<PointCluster> &vec_orig, PointCluster &fix, double coe, Eigen::Vector3d &eig_value, Eigen::Matrix3d &eig_vector,
                  PointCluster &pcr_add) {
    plvec_voxels.push_back(vec_orig);
    sig_vecs.push_back(fix);
    coeffs.push_back(coe);
    eig_values.push_back(eig_value);
    eig_vectors.push_back(eig_vector);
    pcr_adds.push_back(pcr_add);
  }

  void acc_evaluate2(const std::vector<StateGroup> &xs, int head, int end, Eigen::MatrixXd &Hess, Eigen::VectorXd &JacT, double &residual) {
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
      std::vector<PointCluster> &sig_orig = plvec_voxels[a];
      double coe = coeffs[a];

      Eigen::Vector3d lmbd = eig_values[a];
      Eigen::Matrix3d U = eig_vectors[a];
      int NN = pcr_adds[a].N;
      Eigen::Vector3d vBar = pcr_adds[a].v / NN;

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

  void evaluate_only_residual(const std::vector<StateGroup> &xs, int head, int end, double &residual) {
    residual = 0;
    int kk = 0;

    PointCluster pcr;

    for (int a = head; a < end; a++) {
      const std::vector<PointCluster> &sig_orig = plvec_voxels[a];
      PointCluster sig = sig_vecs[a];

      for (int i = 0; i < win_size; i++) {
        if (sig_orig[i].N != 0) {
          pcr.transform(sig_orig[i], xs[i]);
          sig += pcr;
        }
      }

      Eigen::Vector3d vBar = sig.v / sig.N;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(sig.P / sig.N - vBar * vBar.transpose());
      Eigen::Vector3d lmbd = saes.eigenvalues();

      eig_values[a] = saes.eigenvalues();
      eig_vectors[a] = saes.eigenvectors();
      pcr_adds[a] = sig;

      residual += coeffs[a] * lmbd[kk];
    }
  }

  void clear() {
    sig_vecs.clear();
    plvec_voxels.clear();
    eig_values.clear();
    eig_vectors.clear();
    pcr_adds.clear();
    coeffs.clear();
  }

  ~LidarFactor() {}
};

}  // namespace sapphire