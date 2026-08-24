#pragma once

#include "common.hpp"
#include "imu_estimator.hpp"
#include "lm_optimizer.hpp"
#include "voxel_map.hpp"
#include "../thirdparty/lie/SEn3.hpp"

class Initialization
{
public:
  static Initialization &instance()
  {
    static Initialization inst;
    return inst;
  }

  void align_gravity(vector<StateGroup> &xs)
  {
    Eigen::Vector3d g0 = xs[0].g;
    Eigen::Vector3d n0 = g0 / g0.norm();
    Eigen::Vector3d n1(0, 0, 1);
    if(n0[2] < 0)
    {
      n1[2] = -1;
    }
    Eigen::Vector3d rotvec = n0.cross(n1);
    double rnorm = rotvec.norm();
    rotvec = rotvec / rnorm;

    Eigen::AngleAxisd angaxis(asin(rnorm), rotvec);
    Eigen::Matrix3d rot = angaxis.matrix();
    g0 = rot * g0;

    Eigen::Vector3d p0 = xs[0].p;
    for(int i=0; i<xs.size(); i++)
    {
      xs[i].p = rot * (xs[i].p - p0) + p0;
      xs[i].R = rot * xs[i].R;
      xs[i].v = rot * xs[i].v;
      xs[i].g = g0;
    }

  }

  void static_deskew(std::vector<LidarPoint> &pl, PointCloud &pvec, StateGroup xc, StateGroup xl, const deque<ImuMeas> &imus, double pcl_beg_time, StateGroup &extrin_para, double scale_gravity)
  {
    pointVar pv; pv.var.setIdentity();
    pvec.reserve(pl.size());

    if(pl.empty() || imus.size() < 2) return;

    using Gal3 = lie::Gal3d;
    const Eigen::Vector3d bg = xl.bg;
    const Eigen::Vector3d ba = xl.ba;
    const double reference_time = imus.back().timestamp;

    Gal3::IsometriesType end_isometries{xc.v, xc.p};
    Gal3 state(xc.R, end_isometries, 0.0);

    vector<Gal3StateGroup> states;
    states.reserve(imus.size());
    states.emplace_back(reference_time, state);

    for(size_t i=imus.size()-1; i>0; i--)
    {
      const ImuMeas &head = imus[i-1];
      const ImuMeas &tail = imus[i];
      const double head_time = head.timestamp;
      const double tail_time = tail.timestamp;

      Eigen::Vector3d angvel_avr = 0.5 * (head.gyro + tail.gyro);
      Eigen::Vector3d acc_avr = 0.5 * (head.accel + tail.accel);
      angvel_avr -= bg;
      acc_avr = acc_avr * scale_gravity - ba;

      const double dt = head_time - tail_time;
      Gal3::MatrixType gamma = Gal3::MatrixType::Identity();
      gamma.template block<3, 1>(0, 3) = xc.g * dt;
      gamma.template block<3, 1>(0, 4) = -0.5 * xc.g * dt * dt;
      gamma(3, 4) = -dt;

      Gal3::VectorType input;
      input << angvel_avr, acc_avr, Eigen::Vector3d::Zero(), 1.0;
      state = Gal3(gamma) * state * Gal3::exp(input * dt);
      states.emplace_back(head_time, state);
    }
    reverse(states.begin(), states.end());

    for(LidarPoint &point: pl)
    {
      const double point_stamp = pcl_beg_time + point.time_offset;
      const Eigen::Vector3d point_lidar(point.x, point.y, point.z);
      pv.pnt = ImuEstimator::deskew_by_point(states, point_stamp, point_lidar, extrin_para.R, extrin_para.p);
      pvec.push_back(pv);
    }
  }

  int motion_init(vector<MeasGroup> &measures, Eigen::MatrixXd *hess, LidarFactor &voxhess, vector<StateGroup> &x_buf, VoxelMap &voxel_map, vector<PointCloudPtr> &pvec_buf, int win_size, StateGroup &x_curr, deque<ImuFactor*> &imu_factor_buf, StateGroup &extrin_para, const InitializerParameters &parameters, const OdometryParameters &odometry, const LocalSubmapParameters &local_submap, double scale_gravity)
  {
    vvec<double, 3> pwld;
    int converge_flag = 0;
    vector<double> init_plane_thresholds(local_submap.plane_eigen_value_thre.size(), parameters.plane_eigen_value_thre_inv);

    double converge_thre = parameters.convergence_threshold;
    bool is_degrade = true;
    Eigen::Vector3d eigvalue; eigvalue.setZero();
    for(int iterCnt = 0; iterCnt < parameters.max_iterations; iterCnt++)
    {
      voxel_map.clear();

      for(int i=0; i<win_size; i++)
      {
        pwld.clear();
        pvec_buf[i]->clear();
        int l = i==0 ? i : i - 1;
        static_deskew(*measures[i].lidar_cloud, *pvec_buf[i], x_buf[i], x_buf[l], measures[i].imu_buf, measures[i].lidar_begin_time, extrin_para, scale_gravity);

        if(converge_flag == 1)
        {
          for(pointVar &pv: *pvec_buf[i])
          {
            calcMeasVar(pv.pnt, odometry.dept_err, odometry.beam_err, pv.var);
          }
          pvec_update(pvec_buf[i], x_buf[i], pwld);
        }
        else
        {
          for(pointVar &pv: *pvec_buf[i]) {
            pwld.push_back(x_buf[i].R * pv.pnt + x_buf[i].p);
          }
        }

        voxel_map.cut_voxel(pvec_buf[i], i, win_size, pwld);
      }

      voxhess.clear(); voxhess.win_size = win_size;
      const double eigen_threshold = converge_flag == 0
        ? parameters.min_eigen_value
        : odometry.min_eigen_value;
      const vector<double> &plane_thresholds = converge_flag == 0
        ? init_plane_thresholds
        : local_submap.plane_eigen_value_thre;
      voxel_map.recut(win_size, x_buf, voxhess, eigen_threshold, plane_thresholds);

      if(voxhess.plvec_voxels.size() < static_cast<size_t>(parameters.min_plane_factors))
      {
        break;
      }
      LI_BA_OptimizerGravity opt_lsv(local_submap);
      vector<double> resis;
      opt_lsv.damping_iter(x_buf, voxhess, imu_factor_buf, resis, hess, parameters.ba_iterations);
      Eigen::Matrix3d nnt; nnt.setZero();

      for(int i=0; i<win_size-1; i++)
      {
        delete imu_factor_buf[i];
      }
      imu_factor_buf.clear();

      for(int i=1; i<win_size; i++)
      {
        imu_factor_buf.push_back(new ImuFactor(x_buf[i-1].bg, x_buf[i-1].ba));
        imu_factor_buf.back()->push_imu(measures[i].imu_buf, scale_gravity, local_submap);
      }

      if(fabs(resis[0] - resis[1]) / resis[0] < converge_thre && iterCnt >= 2)
      {
        for(Eigen::Matrix3d &iter: voxhess.eig_vectors)
        {
          Eigen::Vector3d v3 = iter.col(0);
          nnt += v3 * v3.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(nnt);
        eigvalue = saes.eigenvalues();
        is_degrade = eigvalue[0] < parameters.degeneracy_threshold;

        converge_thre = parameters.refined_convergence_threshold;
        if(converge_flag == 0)
        {
          align_gravity(x_buf);
          converge_flag = 1;
          continue;
        }
        else
        {
          break;
        }
      }
    }

    x_curr = x_buf[win_size - 1];
    double gnm = x_curr.g.norm();
    if(is_degrade || gnm < 9.6 || gnm > 10.0)
    {
      converge_flag = 0;
    }
    if(converge_flag == 0)
    {
      voxel_map.clear();
    }

    return converge_flag;
  }

};
