#pragma once

#include <array>
#include <vector>

#include "voxel_map.hpp"

namespace sapphire {

class ESKF {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  vvec<double, 3> pl_tree;
  EigenVectorAdapter kd_cloud;
  KDTree kd_map;

  static constexpr int NMATCH = 5;

  ESKF() : kd_cloud(pl_tree), kd_map(3, kd_cloud) {}

  void observe_kdtree(StateGroup &x_curr, PointCloud &pvec, double down_size_inv) {
    if (pl_tree.size() < 100) {
      for (const pointVar &pv : pvec) {
        pl_tree.push_back(x_curr.R * pv.pnt + x_curr.p);
      }
      kd_map.buildIndex();
      return;
    }

    const int num_max_iter = 4;
    StateGroup x_prop = x_curr;
    int psize = pvec.size();
    bool EKF_stop_flg = 0, flg_EKF_converged = 0;
    Eigen::Matrix<double, STATE_DOF, STATE_DOF> G, H_T_H, I_STATE;
    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();

    std::array<double, NMATCH> sqdis;
    std::array<size_t, NMATCH> nearInd;
    int rematch_num = 0;
    Eigen::Matrix<double, STATE_DOF, STATE_DOF> cov_inv = x_curr.cov.inverse();

    Eigen::Matrix<double, NMATCH, 1> b;
    b.setOnes();
    b *= -1.0f;

    std::vector<double> ds(psize, -1);
    vvec<double, 3> directs(psize);
    bool refind = true;

    for (int iterCount = 0; iterCount < num_max_iter; iterCount++) {
      Eigen::Matrix<double, POSE_DOF, POSE_DOF> HTH;
      HTH.setZero();
      Eigen::Matrix<double, POSE_DOF, 1> HTz;
      HTz.setZero();
      for (int i = 0; i < psize; i++) {
        pointVar &pv = pvec[i];
        Eigen::Matrix3d phat = lie::SO3d::wedge(pv.pnt);
        Eigen::Vector3d wld = x_curr.R * pv.pnt + x_curr.p;

        if (refind) {
          // 这里以后可以加个警告，但是理论上不应该存在knn连5个match都找不到情况，因为我们之前保证了kdtree的最低数据量是 >= 100的;
          (void)kd_map.knnSearch(wld.data(), NMATCH, nearInd.data(), sqdis.data());

          Eigen::Matrix<double, NMATCH, 3> A;
          for (int i = 0; i < NMATCH; i++) {
            A.row(i) = pl_tree[nearInd[i]].transpose();
          }
          Eigen::Vector3d direct = A.colPivHouseholderQr().solve(b);
          bool check_flag = false;
          for (int i = 0; i < NMATCH; i++) {
            if (fabs(direct.dot(A.row(i)) + 1.0) > 0.1) {
              check_flag = true;
            }
          }

          if (check_flag) {
            ds[i] = -1;
            continue;
          }

          double d = 1.0 / direct.norm();
          ds[i] = d;
          directs[i] = direct * d;
        }

        if (ds[i] >= 0) {
          double pd2 = directs[i].dot(wld) + ds[i];
          Eigen::Matrix<double, POSE_DOF, 1> jac_s;
          jac_s.head(3) = phat * x_curr.R.transpose() * directs[i];
          jac_s.tail(3) = directs[i];

          HTH += jac_s * jac_s.transpose();
          HTz += jac_s * (-pd2);
        }
      }

      H_T_H.block<POSE_DOF, POSE_DOF>(0, 0) = HTH;
      Eigen::Matrix<double, STATE_DOF, STATE_DOF> K_1 = (H_T_H + cov_inv / 1000).inverse();
      G.block<STATE_DOF, POSE_DOF>(0, 0) = K_1.block<STATE_DOF, POSE_DOF>(0, 0) * HTH;
      Eigen::Matrix<double, STATE_DOF, 1> vec = x_prop - x_curr;
      Eigen::Matrix<double, STATE_DOF, 1> solution =
          K_1.block<STATE_DOF, POSE_DOF>(0, 0) * HTz + vec - G.block<STATE_DOF, POSE_DOF>(0, 0) * vec.block<POSE_DOF, 1>(0, 0);

      x_curr += solution;
      Eigen::Vector3d rot_add = solution.block<3, 1>(0, 0);
      Eigen::Vector3d tra_add = solution.block<3, 1>(3, 0);

      refind = false;
      if ((rot_add.norm() * 57.3 < 0.01) && (tra_add.norm() * 100 < 0.015)) {
        refind = true;
        flg_EKF_converged = true;
        rematch_num++;
      }

      if (iterCount == num_max_iter - 2 && !flg_EKF_converged) {
        refind = true;
      }

      if (rematch_num >= 2 || (iterCount == num_max_iter - 1)) {
        x_curr.cov = (I_STATE - G) * x_curr.cov;
        EKF_stop_flg = true;
      }

      if (EKF_stop_flg) {
        break;
      }
    }

    for (const pointVar &pv : pvec) {
      pl_tree.push_back(x_curr.R * pv.pnt + x_curr.p);
    }
    down_sampling_voxel(pl_tree, down_size_inv);
    kd_map.buildIndex();
  }

  bool observe_voxelmap(StateGroup &x_curr, PointCloud &pvec, VoxelMap &voxel_map) {
    StateGroup x_prop = x_curr;

    const int num_max_iter = 4;
    bool EKF_stop_flg = 0, flg_EKF_converged = 0;
    Eigen::Matrix<double, STATE_DOF, STATE_DOF> G, H_T_H, I_STATE;
    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();
    int rematch_num = 0;
    int match_num = 0;

    int psize = pvec.size();
    std::vector<OctoTree *> octos;
    octos.resize(psize, nullptr);

    Eigen::Matrix3d nnt;
    Eigen::Matrix<double, STATE_DOF, STATE_DOF> cov_inv = x_curr.cov.inverse();
    for (int iterCount = 0; iterCount < num_max_iter; iterCount++) {
      Eigen::Matrix<double, POSE_DOF, POSE_DOF> HTH;
      HTH.setZero();
      Eigen::Matrix<double, POSE_DOF, 1> HTz;
      HTz.setZero();
      Eigen::Matrix3d rot_var = x_curr.cov.block<3, 3>(0, 0);
      Eigen::Matrix3d tsl_var = x_curr.cov.block<3, 3>(3, 3);
      match_num = 0;
      nnt.setZero();

      for (int i = 0; i < psize; i++) {
        pointVar &pv = pvec[i];
        Eigen::Matrix3d phat = lie::SO3d::wedge(pv.pnt);
        Eigen::Matrix3d var_world = x_curr.R * pv.var * x_curr.R.transpose() + phat * rot_var * phat.transpose() + tsl_var;
        Eigen::Vector3d wld = x_curr.R * pv.pnt + x_curr.p;

        double sigma_d = 0;
        Plane *pla = nullptr;
        int flag = 0;
        if (octos[i] != nullptr && octos[i]->inside(wld)) {
          double max_prob = 0;
          flag = octos[i]->match(wld, pla, max_prob, var_world, sigma_d, octos[i]);
        } else {
          flag = voxel_map.match(wld, pla, var_world, sigma_d, octos[i]);
        }

        if (flag) {
          Plane &pp = *pla;
          double R_inv = 1.0 / (0.0005 + sigma_d);
          double resi = pp.normal.dot(wld - pp.center);

          Eigen::Matrix<double, POSE_DOF, 1> jac;
          jac.head(3) = phat * x_curr.R.transpose() * pp.normal;
          jac.tail(3) = pp.normal;
          HTH += R_inv * jac * jac.transpose();
          HTz -= R_inv * jac * resi;
          nnt += pp.normal * pp.normal.transpose();
          match_num++;
        }
      }

      H_T_H.block<POSE_DOF, POSE_DOF>(0, 0) = HTH;
      Eigen::Matrix<double, STATE_DOF, STATE_DOF> K_1 = (H_T_H + cov_inv).inverse();
      G.block<STATE_DOF, POSE_DOF>(0, 0) = K_1.block<STATE_DOF, POSE_DOF>(0, 0) * HTH;
      Eigen::Matrix<double, STATE_DOF, 1> vec = x_prop - x_curr;
      Eigen::Matrix<double, STATE_DOF, 1> solution =
          K_1.block<STATE_DOF, POSE_DOF>(0, 0) * HTz + vec - G.block<STATE_DOF, POSE_DOF>(0, 0) * vec.block<POSE_DOF, 1>(0, 0);

      x_curr += solution;
      Eigen::Vector3d rot_add = solution.block<3, 1>(0, 0);
      Eigen::Vector3d tra_add = solution.block<3, 1>(3, 0);

      EKF_stop_flg = false;
      flg_EKF_converged = false;

      if ((rot_add.norm() * 57.3 < 0.01) && (tra_add.norm() * 100 < 0.015)) {
        flg_EKF_converged = true;
      }

      if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == num_max_iter - 2))) {
        rematch_num++;
      }

      if (rematch_num >= 2 || (iterCount == num_max_iter - 1)) {
        x_curr.cov = (I_STATE - G) * x_curr.cov;
        EKF_stop_flg = true;
      }

      if (EKF_stop_flg) {
        break;
      }
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(nnt);
    Eigen::Vector3d evalue = saes.eigenvalues();

    if (evalue[0] < 14) {
      return false;
    } else {
      return true;
    }
  }
};

}  // namespace sapphire