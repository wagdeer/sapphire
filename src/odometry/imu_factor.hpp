#pragma once

#include "common.hpp"
#include "parameters.h"
#include <deque>

class ImuFactor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix3d R_delta;
  Eigen::Vector3d p_delta, v_delta;
  Eigen::Vector3d bg, ba;

  Eigen::Matrix3d R_bg;
  Eigen::Matrix3d p_bg, p_ba;
  Eigen::Matrix3d v_bg, v_ba;

  double dtime;

  Eigen::Vector3d dbg, dba;
  Eigen::Vector3d dbg_buf, dba_buf;

  Eigen::Matrix<double, STATE_DOF, STATE_DOF> cov;

  ImuFactor(const Eigen::Vector3d &bg1 = Eigen::Vector3d::Zero(), const Eigen::Vector3d &ba1 = Eigen::Vector3d::Zero())
  {
    bg = bg1; ba = ba1;
    R_delta.setIdentity();
    p_delta.setZero(); v_delta.setZero();

    R_bg.setZero();
    p_bg.setZero(); p_ba.setZero();
    v_bg.setZero(); v_ba.setZero();

    dtime = 0;

    dbg.setZero(); dba.setZero();
    dbg_buf.setZero(); dba_buf.setZero();

    cov.setZero();
  }

  void push_imu(
    const deque<ImuMeas> &imus,
    double scale_gravity,
    const LocalSubmapParameters &parameters)
  {
    Eigen::Matrix<double, 6, 1> noise_meas;
    Eigen::Matrix<double, 6, 1> noise_walk;
    noise_meas << parameters.cov_gyr, parameters.cov_gyr, parameters.cov_gyr,
      parameters.cov_acc, parameters.cov_acc, parameters.cov_acc;
    noise_walk << parameters.rdw_gyr, parameters.rdw_gyr, parameters.rdw_gyr,
      parameters.rdw_acc, parameters.rdw_acc, parameters.rdw_acc;

    Eigen::Vector3d cur_gyr, cur_acc;
    for(auto it_imu=imus.begin()+1; it_imu!=imus.end(); it_imu++)
    {
      const ImuMeas &imu1 = *(it_imu-1);
      const ImuMeas &imu2 = *it_imu;

      double dt = imu2.timestamp - imu1.timestamp;

      cur_gyr = 0.5 * (imu1.gyro + imu2.gyro);
      cur_acc = 0.5 * (imu1.accel + imu2.accel);

      cur_gyr = cur_gyr - bg;
      cur_acc = cur_acc * scale_gravity - ba;

      add_imu(cur_gyr, cur_acc, dt, noise_meas, noise_walk);
    }
  }

  void add_imu(
    Eigen::Vector3d &cur_gyr,
    Eigen::Vector3d &cur_acc,
    double dt,
    const Eigen::Matrix<double, 6, 1> &noise_meas,
    const Eigen::Matrix<double, 6, 1> &noise_walk)
  {
    dtime += dt;
    Eigen::Matrix3d R_inc = lie::SO3d::exp(cur_gyr * dt).R();
    Eigen::Matrix3d R_jr = lie::SO3d::rightJacobian(cur_gyr * dt);
    
    Eigen::Matrix3d R_dt = dt * R_delta;
    Eigen::Matrix3d R_dt2_2 = 0.5*dt*dt*R_delta;

    const Eigen::Matrix3d acc_skew = lie::SO3d::wedge(cur_acc);

    p_ba = p_ba + v_ba*dt - R_dt2_2;
    p_bg = p_bg + v_bg*dt - R_dt2_2*acc_skew*R_bg;
    v_ba = v_ba - R_dt;
    v_bg = v_bg - R_dt * acc_skew * R_bg;
    R_bg = R_inc.transpose() * R_bg - R_jr*dt;

    Eigen::Matrix<double, 9, 9> A;
    Eigen::Matrix<double, 9, 6> B;
    A.setIdentity(); B.setZero();
    
    A.block<3, 3>(0, 0) = R_inc.transpose();
    A.block<3, 3>(3, 0) = -R_dt2_2 * acc_skew;
    A.block<3, 3>(3, 6) = I33<double> * dt;
    A.block<3, 3>(6, 0) = -R_dt * acc_skew;

    B.block<3, 3>(0, 0) = R_jr * dt;
    B.block<3, 3>(3, 3) = R_dt2_2;
    B.block<3, 3>(6, 3) = R_dt;

    cov.block<9, 9>(0, 0) =
      A * cov.block<9, 9>(0, 0) * A.transpose() +
      B * noise_meas.asDiagonal() * B.transpose();
    cov.block<6, 6>(9, 9).diagonal() += noise_walk * dt;

    p_delta += v_delta*dt + R_dt2_2*cur_acc;
    v_delta += R_dt * cur_acc;
    R_delta = R_delta * R_inc;
  }

  double give_evaluate(StateGroup &st1, StateGroup &st2, Eigen::MatrixXd &jtj, Eigen::VectorXd &gg, bool jac_enable)
  {
    Eigen::Matrix<double, STATE_DOF, STATE_DOF> joca, jocb;
    Eigen::Matrix<double, STATE_DOF, 1> rr;
    joca.setZero(); jocb.setZero(); rr.setZero();

    Eigen::Matrix3d R_correct = R_delta * lie::SO3d::exp(R_bg * dbg).R();
    Eigen::Vector3d t_correct = p_delta + p_bg*dbg + p_ba*dba;
    Eigen::Vector3d v_correct = v_delta + v_bg*dbg + v_ba*dba;

    Eigen::Matrix3d res_r = R_correct.transpose() * st1.R.transpose() * st2.R;
    Eigen::Vector3d exp_v = st1.R.transpose() * (st2.v - st1.v - dtime*st1.g);
    Eigen::Vector3d res_v = exp_v - v_correct;
    Eigen::Vector3d exp_t = st1.R.transpose() * (st2.p - st1.p - st1.v*dtime - 0.5*dtime*dtime*st1.g);
    Eigen::Vector3d res_t = exp_t - t_correct;

    Eigen::Vector3d res_bg = st2.bg - st1.bg;
    Eigen::Vector3d res_ba = st2.ba - st1.ba;

    double b_wei = 1;

    rr.block<3, 1>(0, 0) = lie::SO3d::log(lie::SO3d(res_r));
    rr.block<3, 1>(3, 0) = res_t;
    rr.block<3, 1>(6, 0) = res_v;
    rr.block<3, 1>(9, 0) = res_bg*b_wei;
    rr.block<3, 1>(12, 0) = res_ba*b_wei;
    

    Eigen::Matrix<double, 15, 15> cov_inv = cov.inverse();

    if(jac_enable)
    {
      Eigen::Matrix3d JR_inv = lie::SO3d::invRightJacobian(lie::SO3d::log(lie::SO3d(res_r)));
      joca.block<3, 3>(0, 0) = -JR_inv * st2.R.transpose() * st1.R;
      jocb.block<3, 3>(0, 0) =  JR_inv;
      joca.block<3, 3>(0, 9) = -JR_inv * res_r.transpose() * lie::SO3d::rightJacobian(R_bg * dbg) * R_bg;

      joca.block<3, 3>(3, 0) = lie::SO3d::wedge(exp_t);
      joca.block<3, 3>(3, 3) = -st1.R.transpose();
      joca.block<3, 3>(3, 6) = -st1.R.transpose() * dtime;
      joca.block<3, 3>(3, 9) = -p_bg;
      joca.block<3, 3>(3, 12) = -p_ba;
      jocb.block<3, 3>(3, 3) = st1.R.transpose();
      
      joca.block<3, 3>(6, 0) = lie::SO3d::wedge(exp_v);
      joca.block<3, 3>(6, 6) = -st1.R.transpose();
      joca.block<3, 3>(6, 9) = -v_bg;
      joca.block<3, 3>(6, 12) = -v_ba;
      jocb.block<3, 3>(6, 6) = st1.R.transpose();

      joca.block<3, 3>(9, 9) = -I33<double>*b_wei;
      joca.block<3, 3>(12, 12) = -I33<double>*b_wei;
      jocb.block<3, 3>(9, 9) = I33<double>*b_wei;
      jocb.block<3, 3>(12, 12) = I33<double>*b_wei;

      Eigen::Matrix<double, STATE_DOF, 2*STATE_DOF> joc;
      joc.block<STATE_DOF, STATE_DOF>(0, 0) = joca;
      joc.block<STATE_DOF, STATE_DOF>(0, STATE_DOF) = jocb;

      jtj = joc.transpose() * cov_inv * joc;
      gg = joc.transpose() * cov_inv * rr;
    }

    return rr.dot(cov_inv * rr);
  }

  double give_evaluate_g(StateGroup &st1, StateGroup &st2, Eigen::MatrixXd &jtj, Eigen::VectorXd &gg, bool jac_enable)
  {
    Eigen::Matrix<double, STATE_DOF, STATE_DOF> joca, jocb;
    Eigen::Matrix<double, STATE_DOF, 1> rr;
    joca.setZero(); jocb.setZero(); rr.setZero();
    Eigen::Matrix<double, STATE_DOF, 3> jocg;
    jocg.setZero();

    Eigen::Matrix3d R_correct = R_delta * lie::SO3d::exp(R_bg * dbg).R();
    Eigen::Vector3d t_correct = p_delta + p_bg*dbg + p_ba*dba;
    Eigen::Vector3d v_correct = v_delta + v_bg*dbg + v_ba*dba;

    Eigen::Matrix3d res_r = R_correct.transpose() * st1.R.transpose() * st2.R;
    Eigen::Vector3d exp_v = st1.R.transpose() * (st2.v - st1.v - dtime*st1.g);
    Eigen::Vector3d res_v = exp_v - v_correct;
    Eigen::Vector3d exp_t = st1.R.transpose() * (st2.p - st1.p - st1.v*dtime - 0.5*dtime*dtime*st1.g);
    Eigen::Vector3d res_t = exp_t - t_correct;

    Eigen::Vector3d res_bg = st2.bg - st1.bg;
    Eigen::Vector3d res_ba = st2.ba - st1.ba;

    double b_wei = 1;

    rr.block<3, 1>(0, 0) = lie::SO3d::log(lie::SO3d(res_r));
    rr.block<3, 1>(3, 0) = res_t;
    rr.block<3, 1>(6, 0) = res_v;
    rr.block<3, 1>(9, 0) = res_bg*b_wei;
    rr.block<3, 1>(12, 0) = res_ba*b_wei;
    
    Eigen::Matrix<double, 15, 15> cov_inv = cov.inverse();

    if(jac_enable)
    {
      Eigen::Matrix3d JR_inv = lie::SO3d::invRightJacobian(lie::SO3d::log(lie::SO3d(res_r)));
      joca.block<3, 3>(0, 0) = -JR_inv * st2.R.transpose() * st1.R;
      jocb.block<3, 3>(0, 0) =  JR_inv;
      joca.block<3, 3>(0, 9) = -JR_inv * res_r.transpose() * lie::SO3d::rightJacobian(R_bg * dbg) * R_bg;

      joca.block<3, 3>(3, 0) = lie::SO3d::wedge(exp_t);
      joca.block<3, 3>(3, 3) = -st1.R.transpose();
      joca.block<3, 3>(3, 6) = -st1.R.transpose() * dtime;
      joca.block<3, 3>(3, 9) = -p_bg;
      joca.block<3, 3>(3, 12) = -p_ba;
      jocb.block<3, 3>(3, 3) = st1.R.transpose();
      
      joca.block<3, 3>(6, 0) = lie::SO3d::wedge(exp_v);
      joca.block<3, 3>(6, 6) = -st1.R.transpose();
      joca.block<3, 3>(6, 9) = -v_bg;
      joca.block<3, 3>(6, 12) = -v_ba;
      jocb.block<3, 3>(6, 6) = st1.R.transpose();

      joca.block<3, 3>(9, 9) = -I33<double>*b_wei;
      joca.block<3, 3>(12, 12) = -I33<double>*b_wei;
      jocb.block<3, 3>(9, 9) = I33<double>*b_wei;
      jocb.block<3, 3>(12, 12) = I33<double>*b_wei;

      jocg.block<3, 3>(3, 0) = st1.R.transpose() * (-0.5*dtime*dtime);
      jocg.block<3, 3>(6, 0) = st1.R.transpose() * (-dtime);

      Eigen::Matrix<double, STATE_DOF, 2*STATE_DOF+3> joc;
      joc.block<STATE_DOF, STATE_DOF>(0, 0) = joca;
      joc.block<STATE_DOF, STATE_DOF>(0, STATE_DOF) = jocb;
      joc.block<STATE_DOF, 3>(0, 2*STATE_DOF) = jocg;

      jtj = joc.transpose() * cov_inv * joc;
      gg = joc.transpose() * cov_inv * rr;
    }

    return rr.dot(cov_inv * rr);
  }

  void update_state(const Eigen::Matrix<double, STATE_DOF, 1> &dxi)
  {
    dbg_buf = dbg;
    dba_buf = dba;

    dbg += dxi.block<3, 1>(9, 0);
    dba += dxi.block<3, 1>(12, 0);
  }

  void merge(ImuFactor &imu2)
  {
    p_bg += v_bg * imu2.dtime + R_delta * (imu2.p_bg - lie::SO3d::wedge(imu2.p_delta) * R_bg);
    p_ba += v_ba*imu2.dtime + R_delta*imu2.p_ba;
    v_bg += R_delta * (imu2.v_bg - lie::SO3d::wedge(imu2.v_delta) * R_bg);
    v_ba += R_delta*imu2.v_ba;
    R_bg = imu2.R_delta.transpose()*R_bg + imu2.R_bg;
    
    Eigen::Matrix<double, STATE_DOF, STATE_DOF> Ai, Bi;
    Ai.setIdentity(); Bi.setIdentity();
    Ai.block<3, 3>(0, 0) = imu2.R_delta.transpose();
    Ai.block<3, 3>(3, 0) = -R_delta * lie::SO3d::wedge(imu2.p_delta);
    Ai.block<3, 3>(3, 6) = I33<double> * imu2.dtime;
    Ai.block<3, 3>(6, 0) = -R_delta * lie::SO3d::wedge(imu2.v_delta);

    Bi.block<3, 3>(3, 3) = R_delta;
    Bi.block<3, 3>(6, 6) = R_delta;
    cov = Ai*cov*Ai.transpose() + Bi*imu2.cov*Bi.transpose();

    p_delta += v_delta*imu2.dtime + R_delta*imu2.p_delta;
    v_delta += R_delta*imu2.v_delta;
    R_delta = R_delta * imu2.R_delta;

    dtime += imu2.dtime;
  }

};

