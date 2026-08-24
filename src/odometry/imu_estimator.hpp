#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <spdlog/spdlog.h>
#include <vector>

#include "common.hpp"
#include "parameters.h"
#include "../thirdparty/lie/Gal3.hpp"

struct  Gal3StateGroup
{
  double stamp;
  lie::Gal3d state;

  Gal3StateGroup(double stamp_in, const lie::Gal3d &state_in) : stamp(stamp_in), state(state_in) {}
};

inline void gal3_interpolate(const vector<Gal3StateGroup> &states, double stamp, Eigen::Matrix3d &R_world_imu, Eigen::Vector3d &p_world_imu)
{
  const double point_stamp = std::clamp(stamp, states.front().stamp, states.back().stamp);
  auto upper = lower_bound(states.begin(), states.end(), point_stamp, [](const Gal3StateGroup &timed, double time) { return timed.stamp < time; });

  if(upper == states.begin())
  {
    R_world_imu = upper->state.R();
    p_world_imu = upper->state.p();
  }
  else if(upper == states.end())
  {
    R_world_imu = states.back().state.R();
    p_world_imu = states.back().state.p();
  }
  else
  {
    const Gal3StateGroup &right = *upper;
    const Gal3StateGroup &left = *(upper - 1);
    const double interval_dt = right.stamp - left.stamp;
    const double alpha = interval_dt > 0.0 ? (point_stamp - left.stamp) / interval_dt : 0.0;

    const lie::SO3d relative_rotation(left.state.R().transpose() * right.state.R());
    R_world_imu = left.state.R() * lie::SO3d::exp(alpha * lie::SO3d::log(relative_rotation)).R();

    const double a2 = alpha * alpha;
    const double a3 = a2 * alpha;
    const double h00 = 2.0 * a3 - 3.0 * a2 + 1.0;
    const double h10 = a3 - 2.0 * a2 + alpha;
    const double h01 = -2.0 * a3 + 3.0 * a2;
    const double h11 = a3 - a2;
    p_world_imu = h00 * left.state.p() + h10 * interval_dt * left.state.v() + h01 * right.state.p() + h11 * interval_dt * right.state.v();
  }
}

class ImuEstimator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool init_flag;
  double pcl_beg_time, pcl_end_time, last_pcl_end_time;
  int init_num;
  Eigen::Vector3d mean_acc, mean_gyr;
  ImuMeas last_imu;
  bool has_last_imu = false;
  int min_init_num = 30;
  Eigen::Vector3d angvel_last, acc_s_last;

  Eigen::Vector3d cov_acc, cov_gyr;
  Eigen::Vector3d cov_bias_gyr, cov_bias_acc;

  Eigen::Matrix3d Lid_rot_to_IMU;
  Eigen::Vector3d Lid_offset_to_IMU;

  double scale_gravity = 1.0;
  vector<StateGroup> imu_poses;

  ImuEstimator()
  {
    init_flag = false;
    init_num = 0;
    mean_acc.setZero(); mean_gyr.setZero();
    angvel_last.setZero(); acc_s_last.setZero();
  }

  void configure(const SensorParameters &sensor, const InitializerParameters &initializer, const OdometryParameters &odometry)
  {
    min_init_num = initializer.imu_init_samples;
    cov_gyr.setConstant(odometry.cov_gyr);
    cov_acc.setConstant(odometry.cov_acc);
    cov_bias_gyr.setConstant(odometry.rdw_gyr);
    cov_bias_acc.setConstant(odometry.rdw_acc);
    Lid_offset_to_IMU = Eigen::Map<const Eigen::Vector3d>(sensor.extrinsic_tran.data());
    Lid_rot_to_IMU = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(sensor.extrinsic_rota.data());
  }

  static Eigen::Vector3d deskew_by_point(const vector<Gal3StateGroup>& states, double stamp, const Eigen::Vector3d& point_lidar, const Eigen::Matrix3d& R_imu_lidar, const Eigen::Vector3d& p_imu_lidar)
  {
    Eigen::Matrix3d R_world_imu;
    Eigen::Vector3d p_world_imu;
    gal3_interpolate(states, stamp, R_world_imu, p_world_imu);
  
    const lie::Gal3d &reference_state = states.back().state;
    const Eigen::Vector3d point_imu = R_imu_lidar * point_lidar + p_imu_lidar;
    const Eigen::Vector3d point_world = R_world_imu * point_imu + p_world_imu;
    return reference_state.R().transpose() * (point_world - reference_state.p());
  }
  

  void deskew(StateGroup &xc, std::vector<LidarPoint> &pcl_in, deque<ImuMeas> &imus)
  {
    const StateGroup x_begin = xc;

    if(pcl_in.empty() || imus.size() < 2 || !has_last_imu) return;

    const double integration_begin = last_pcl_end_time;
    const double reference_time = pcl_end_time;
    if(!std::isfinite(integration_begin) || !std::isfinite(reference_time) || reference_time <= integration_begin) return;

    vector<const ImuMeas*> samples;
    samples.reserve(imus.size() + 1);
    auto append_sample = [&samples](const ImuMeas &sample)
    {
      if(samples.empty() || sample.timestamp > samples.back()->timestamp + 1e-12)
      {
        samples.push_back(&sample);
      }
    };
    append_sample(last_imu);
    for(const ImuMeas &sample: imus)
    {
      append_sample(sample);
    }

    if(samples.size() < 2 || samples.front()->timestamp > integration_begin) return;

    lie::Gal3d::IsometriesType initial_isometries{x_begin.v, x_begin.p};
    const lie::Gal3d initial_state(x_begin.R, initial_isometries, 0.0);
    lie::Gal3d preintegrated;

    auto recover_world_state = [&initial_state, &x_begin](const lie::Gal3d &delta) -> lie::Gal3d
    {
      const double dt = delta.s();
      lie::Gal3d::MatrixType gamma = lie::Gal3d::MatrixType::Identity();
      gamma.template block<3, 1>(0, 3) = x_begin.g * dt;
      gamma.template block<3, 1>(0, 4) = -0.5 * x_begin.g * dt * dt;
      gamma(3, 4) = -dt;
      return lie::Gal3d(gamma) * initial_state * delta;
    };

    vector<Gal3StateGroup> states;
    states.reserve(samples.size() + 1);
    states.emplace_back(integration_begin, initial_state);
    Eigen::Matrix<double, STATE_DOF, STATE_DOF> propagated_cov = x_begin.cov;

    double integrated_until = integration_begin;
    Eigen::Vector3d last_gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_accel = Eigen::Vector3d::Zero();
    bool has_last_input = false;
    for(size_t i = 0; i + 1 < samples.size(); i++)
    {
      const ImuMeas &imu0 = *samples[i];
      const ImuMeas &imu1 = *samples[i + 1];
      const double sample_begin = imu0.timestamp;
      const double sample_end = imu1.timestamp;
      if(sample_end <= integrated_until) continue;

      const double interval_begin = std::max(sample_begin, integrated_until);
      const double interval_end = std::min(sample_end, reference_time);
      if(interval_end <= interval_begin) continue;
      if(interval_begin > integrated_until + 1e-6) return;

      Eigen::Vector3d gyro = 0.5 * (imu0.gyro + imu1.gyro);
      Eigen::Vector3d accel = 0.5 * (imu0.accel + imu1.accel);
      gyro -= x_begin.bg;
      accel = accel * scale_gravity - x_begin.ba;
      last_gyro = gyro;
      last_accel = accel;
      has_last_input = true;

      const double dt = interval_end - interval_begin;
      const Eigen::Matrix3d R_world_imu = states.back().state.R();
      Eigen::Matrix<double, STATE_DOF, STATE_DOF> F_x = Eigen::Matrix<double, STATE_DOF, STATE_DOF>::Identity();
      Eigen::Matrix<double, STATE_DOF, STATE_DOF> cov_w = Eigen::Matrix<double, STATE_DOF, STATE_DOF>::Zero();
      F_x.block<3, 3>(0, 0) = lie::SO3d::exp(-gyro * dt).R();
      F_x.block<3, 3>(0, 9) = -I33<double> * dt;
      F_x.block<3, 3>(3, 6) = I33<double> * dt;
      F_x.block<3, 3>(6, 0) = -R_world_imu * lie::SO3d::wedge(accel) * dt;
      F_x.block<3, 3>(6, 12) = -R_world_imu * dt;
      cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt;
      cov_w.block<3, 3>(6, 6) = R_world_imu * cov_acc.asDiagonal() * R_world_imu.transpose() * dt * dt;
      cov_w.block<3, 3>(9, 9).diagonal() = cov_bias_gyr * dt * dt;
      cov_w.block<3, 3>(12, 12).diagonal() = cov_bias_acc * dt * dt;
      propagated_cov = F_x * propagated_cov * F_x.transpose() + cov_w;

      lie::Gal3d::VectorType input;
      input << gyro, accel, Eigen::Vector3d::Zero(), 1.0;
      preintegrated.multiplyRight(lie::Gal3d::exp(input * dt));
      integrated_until = interval_end;
      states.emplace_back(integrated_until, recover_world_state(preintegrated));

      if(integrated_until >= reference_time - 1e-12) break;
    }

    if(integrated_until < reference_time - 1e-12)
    {
      if(!has_last_input) return;

      const double dt = reference_time - integrated_until;
      const Eigen::Matrix3d R_world_imu = states.back().state.R();
      Eigen::Matrix<double, STATE_DOF, STATE_DOF> F_x = Eigen::Matrix<double, STATE_DOF, STATE_DOF>::Identity();
      Eigen::Matrix<double, STATE_DOF, STATE_DOF> cov_w = Eigen::Matrix<double, STATE_DOF, STATE_DOF>::Zero();
      F_x.block<3, 3>(0, 0) = lie::SO3d::exp(-last_gyro * dt).R();
      F_x.block<3, 3>(0, 9) = -I33<double> * dt;
      F_x.block<3, 3>(3, 6) = I33<double> * dt;
      F_x.block<3, 3>(6, 0) = -R_world_imu * lie::SO3d::wedge(last_accel) * dt;
      F_x.block<3, 3>(6, 12) = -R_world_imu * dt;
      cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt;
      cov_w.block<3, 3>(6, 6) = R_world_imu * cov_acc.asDiagonal() * R_world_imu.transpose() * dt * dt;
      cov_w.block<3, 3>(9, 9).diagonal() = cov_bias_gyr * dt * dt;
      cov_w.block<3, 3>(12, 12).diagonal() = cov_bias_acc * dt * dt;
      propagated_cov = F_x * propagated_cov * F_x.transpose() + cov_w;

      lie::Gal3d::VectorType input;
      input << last_gyro, last_accel, Eigen::Vector3d::Zero(), 1.0;
      preintegrated.multiplyRight(lie::Gal3d::exp(input * dt));
      integrated_until = reference_time;
      states.emplace_back(integrated_until, recover_world_state(preintegrated));
    }

    if(states.size() < 2 || integrated_until < reference_time - 1e-6) return;

    imu_poses.clear();
    imu_poses.reserve(states.size());
    for(const Gal3StateGroup &timed: states)
    {
      imu_poses.emplace_back(timed.stamp - pcl_beg_time, timed.state.R(), timed.state.p(), timed.state.v(), x_begin.bg, x_begin.ba, x_begin.g);
    }

    const lie::Gal3d &reference_state = states.back().state;

    for(LidarPoint &point: pcl_in)
    {
      const double point_stamp = pcl_beg_time + point.time_offset;
      const Eigen::Vector3d point_lidar(point.x, point.y, point.z);
      const Eigen::Vector3d point_imu_ref = deskew_by_point(states, point_stamp, point_lidar, Lid_rot_to_IMU, Lid_offset_to_IMU);
      const Eigen::Vector3d point_lidar_ref = Lid_rot_to_IMU.transpose() * (point_imu_ref - Lid_offset_to_IMU);

      point.x = point_lidar_ref.x();
      point.y = point_lidar_ref.y();
      point.z = point_lidar_ref.z();
    }

    xc.R = reference_state.R();
    xc.p = reference_state.p();
    xc.v = reference_state.v();
    xc.t = reference_time;
    xc.cov = propagated_cov;

    ImuMeas imu_begin = *samples.front();
    ImuMeas imu_end = *samples.back();
    imu_begin.timestamp = integration_begin;
    imu_end.timestamp = reference_time;
    last_imu = imus.back();
    has_last_imu = true;
    last_pcl_end_time = reference_time;
    imus.front() = std::move(imu_begin);
    imus.back() = std::move(imu_end);
  }

  void init(const deque<ImuMeas> &imus)
  {
    Eigen::Vector3d cur_acc, cur_gyr;
    for(const ImuMeas &imu: imus)
    {
      cur_acc = imu.accel;
      cur_gyr = imu.gyro;
      if(init_num != 0)
      {
        mean_acc += (cur_acc - mean_acc) / init_num; 
        mean_gyr += (cur_gyr - mean_gyr) / init_num;
      }
      else
      {
        mean_acc = cur_acc; // modify
        mean_gyr = cur_gyr;
        init_num = 1;
      }
      init_num++;
    }
    last_imu = imus.back();
    has_last_imu = true;
  }

  int process(StateGroup &x_curr, std::vector<LidarPoint> &pcl_in, deque<ImuMeas> &imus)
  {
    if(!init_flag)
    {
      init(imus);
      if(mean_acc.norm() < 2)  // for those who use gravity measured in grams instead of m/s^2
      {
        scale_gravity = G_m_s2;
      }
      spdlog::debug("scale_gravity: {} {} {}", scale_gravity, mean_acc.norm(), init_num);
      x_curr.g = -mean_acc * scale_gravity;
      if(init_num > min_init_num)
      {
        init_flag = true;
      }
      last_pcl_end_time = pcl_end_time;
      return 0;
    }
    deskew(x_curr, pcl_in, imus);
    return 1;
  }

};