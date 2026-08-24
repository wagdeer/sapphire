#pragma once

#include "common.hpp"
#include "lidar_factor.hpp"
#include "imu_factor.hpp"
#include "parameters.h"
#include <thread>
#include <Eigen/Eigenvalues>

class LI_BA_Optimizer
{
public:
  int win_size, jac_leng, imu_leng;
  double imu_coef;
  int thread_num;

  explicit LI_BA_Optimizer(const LocalSubmapParameters &parameters)
    : imu_coef(parameters.imu_coef), thread_num(parameters.thread_num) {}

  void hess_plus(Eigen::MatrixXd &Hess, Eigen::VectorXd &JacT, Eigen::MatrixXd &hs, Eigen::VectorXd &js)
  {
    for(int i=0; i<win_size; i++)
    {
      JacT.block<POSE_DOF, 1>(i*STATE_DOF, 0) += js.block<POSE_DOF, 1>(i*POSE_DOF, 0);
      for(int j=0; j<win_size; j++)
      {
        Hess.block<POSE_DOF, POSE_DOF>(i*STATE_DOF, j*STATE_DOF) += hs.block<POSE_DOF, POSE_DOF>(i*POSE_DOF, j*POSE_DOF);
      }
    }
  }

  double divide_thread(vector<StateGroup> &x_stats, LidarFactor &voxhess, deque<ImuFactor*> &imus_factor, Eigen::MatrixXd &Hess, Eigen::VectorXd &JacT)
  {
    int thd_num = thread_num;
    double residual = 0;
    Hess.setZero(); JacT.setZero();
    vmat<double, -1> hessians(thd_num); 
    vvec<double, -1> jacobins(thd_num);
    vector<double> resis(thd_num, 0);

    for(int i=0; i<thd_num; i++)
    {
      hessians[i].resize(jac_leng, jac_leng);
      jacobins[i].resize(jac_leng);
    }

    int tthd_num = thd_num;
    int g_size = voxhess.plvec_voxels.size();
    if(g_size < tthd_num) tthd_num = 1;
    double part = 1.0 * g_size / tthd_num;

    vector<thread*> mthreads(tthd_num);
    for(int i=1; i<tthd_num; i++)
    {
      mthreads[i] = new thread(&LidarFactor::acc_evaluate2, &voxhess, x_stats, part*i, part * (i+1), ref(hessians[i]), ref(jacobins[i]), ref(resis[i]));
    }

    Eigen::MatrixXd jtj(2*STATE_DOF, 2*STATE_DOF);
    Eigen::VectorXd gg(2*STATE_DOF);

    for(int i=0; i<win_size-1; i++)
    {
      jtj.setZero(); gg.setZero();
      residual += imus_factor[i]->give_evaluate(x_stats[i], x_stats[i+1], jtj, gg, true);
      Hess.block<STATE_DOF*2, STATE_DOF*2>(i*STATE_DOF, i*STATE_DOF) += jtj;
      JacT.block<STATE_DOF*2, 1>(i*STATE_DOF, 0) += gg;
    }

    Eigen::Matrix<double, STATE_DOF, STATE_DOF> joc;
    Eigen::Matrix<double, STATE_DOF, 1> rr;
    joc.setIdentity(); rr.setZero();

    Hess *= imu_coef;
    JacT *= imu_coef;
    residual *= (imu_coef * 0.5);

    for(int i=0; i<tthd_num; i++)
    {
      if(i != 0) mthreads[i]->join();
      else
      {
        voxhess.acc_evaluate2(x_stats, 0, part, hessians[0], jacobins[0], resis[0]);
      }
      hess_plus(Hess, JacT, hessians[i], jacobins[i]);
      residual += resis[i];
      delete mthreads[i];
    }

    return residual;
  }

  double only_residual(vector<StateGroup> &x_stats, LidarFactor &voxhess, deque<ImuFactor*> &imus_factor)
  {
    double residual1 = 0, residual2 = 0;
    Eigen::MatrixXd jtj(2*STATE_DOF, 2*STATE_DOF);
    Eigen::VectorXd gg(2*STATE_DOF);

    int thd_num = thread_num;
    vector<double> residuals(thd_num, 0);
    int g_size = voxhess.plvec_voxels.size();
    if(g_size < thd_num)
    {
      thd_num = 1;
    }
    vector<thread*> mthreads(thd_num, nullptr);
    double part = 1.0 * g_size / thd_num;
    for(int i=1; i<thd_num; i++)
    {
      mthreads[i] = new thread(&LidarFactor::evaluate_only_residual, &voxhess, x_stats, part*i, part*(i+1), ref(residuals[i]));
    }

    for(int i=0; i<win_size-1; i++)
    {
      residual1 += imus_factor[i]->give_evaluate(x_stats[i], x_stats[i+1], jtj, gg, false);
    }
    residual1 *= (imu_coef * 0.5);

    for(int i=0; i<thd_num; i++)
    {
      if(i != 0) 
      {
        mthreads[i]->join(); delete mthreads[i];
      }
      else
      {
        voxhess.evaluate_only_residual(x_stats, part*i, part*(i+1), residuals[i]);
      }
      residual2 += residuals[i];
    }

    return (residual1 + residual2);
  }

  void damping_iter(vector<StateGroup> &x_stats, LidarFactor &voxhess, deque<ImuFactor*> &imus_factor, Eigen::MatrixXd* hess)
  {
    win_size = voxhess.win_size;
    jac_leng = win_size * 6;
    imu_leng = win_size * STATE_DOF;
    double u = 0.01, v = 2;
    Eigen::MatrixXd D(imu_leng, imu_leng), Hess(imu_leng, imu_leng);
    Eigen::VectorXd JacT(imu_leng), dxi(imu_leng);
    hess->resize(imu_leng, imu_leng);

    D.setIdentity();
    double residual1, residual2, q;
    bool is_calc_hess = true;
    vector<StateGroup> x_stats_temp = x_stats;

    double hesstime = 0;
    double resitime = 0;
  
    for(int i=0; i<3; i++)
    {
      if(is_calc_hess)
      {
        double tm = now_sec();
        residual1 = divide_thread(x_stats, voxhess, imus_factor, Hess, JacT);
        hesstime += now_sec() - tm;
        *hess = Hess;
      }
      Hess.topRows(STATE_DOF).setZero();
      Hess.leftCols(STATE_DOF).setZero();
      Hess.block<STATE_DOF, STATE_DOF>(0, 0).setIdentity();
      JacT.head(STATE_DOF).setZero();
      D.diagonal() = Hess.diagonal();
      dxi = (Hess + u*D).ldlt().solve(-JacT);
      for(int j=0; j<win_size; j++)
      {
        x_stats_temp[j].R = x_stats[j].R * lie::SO3d::exp(dxi.block<3, 1>(STATE_DOF*j, 0)).R();
        x_stats_temp[j].p = x_stats[j].p + dxi.block<3, 1>(STATE_DOF*j+3, 0);
        x_stats_temp[j].v = x_stats[j].v + dxi.block<3, 1>(STATE_DOF*j+6, 0);
        x_stats_temp[j].bg = x_stats[j].bg + dxi.block<3, 1>(STATE_DOF*j+9, 0);
        x_stats_temp[j].ba = x_stats[j].ba + dxi.block<3, 1>(STATE_DOF*j+12, 0);
      }
      for(int j=0; j<win_size-1; j++)
      {
        imus_factor[j]->update_state(dxi.block<STATE_DOF, 1>(STATE_DOF*j, 0));
      }
      double q1 = 0.5 * dxi.dot(u*D*dxi-JacT);
      double tl1 = now_sec();
      residual2 = only_residual(x_stats_temp, voxhess, imus_factor);
      double tl2 = now_sec();
      resitime += tl2 - tl1;
      q = (residual1-residual2);
      if(q > 0)
      {
        x_stats = x_stats_temp;
        double one_three = 1.0 / 3;
        q = q / q1;
        v = 2;
        q = 1 - pow(2*q-1, 3);
        u *= (q<one_three ? one_three:q);
        is_calc_hess = true;
      }
      else
      {
        u = u * v;
        v = 2 * v;
        is_calc_hess = false;
        for(int j=0; j<win_size-1; j++)
        {
          imus_factor[j]->dbg = imus_factor[j]->dbg_buf;
          imus_factor[j]->dba = imus_factor[j]->dba_buf;
        }
      }
      if(fabs((residual1-residual2)/residual1)<1e-6)
      {
        break;
      }
    }

  }

};

class LI_BA_OptimizerGravity
{
public:
  int win_size, jac_leng, imu_leng;
  double imu_coef;
  int thread_num;

  explicit LI_BA_OptimizerGravity(const LocalSubmapParameters &parameters)
    : imu_coef(parameters.imu_coef), thread_num(parameters.thread_num) {}

  void hess_plus(Eigen::MatrixXd &Hess, Eigen::VectorXd &JacT, Eigen::MatrixXd &hs, Eigen::VectorXd &js)
  {
    for(int i=0; i<win_size; i++)
    {
      JacT.block<POSE_DOF, 1>(i*STATE_DOF, 0) += js.block<POSE_DOF, 1>(i*POSE_DOF, 0);
      for(int j=0; j<win_size; j++)
      {
        Hess.block<POSE_DOF, POSE_DOF>(i*STATE_DOF, j*STATE_DOF) += hs.block<POSE_DOF, POSE_DOF>(i*POSE_DOF, j*POSE_DOF);
      }
    }
  }

  double divide_thread(vector<StateGroup> &x_stats, LidarFactor &voxhess, deque<ImuFactor*> &imus_factor, Eigen::MatrixXd &Hess, Eigen::VectorXd &JacT)
  {
    int thd_num = thread_num;
    double residual = 0;
    Hess.setZero(); JacT.setZero();
    vmat<double, -1> hessians(thd_num); 
    vvec<double, -1> jacobins(thd_num);
    vector<double> resis(thd_num, 0);

    for(int i=0; i<thd_num; i++)
    {
      hessians[i].resize(jac_leng, jac_leng);
      jacobins[i].resize(jac_leng);
    }

    int tthd_num = thd_num;
    int g_size = voxhess.plvec_voxels.size();
    if(g_size < tthd_num) tthd_num = 1;
    double part = 1.0 * g_size / tthd_num;

    vector<thread*> mthreads(tthd_num);
    for(int i=1; i<tthd_num; i++)
    {
      mthreads[i] = new thread(&LidarFactor::acc_evaluate2, &voxhess, x_stats, part*i, part * (i+1), ref(hessians[i]), ref(jacobins[i]), ref(resis[i]));
    }

    Eigen::MatrixXd jtj(2*STATE_DOF+3, 2*STATE_DOF+3);
    Eigen::VectorXd gg(2*STATE_DOF+3);

    for(int i=0; i<win_size-1; i++)
    {
      jtj.setZero(); gg.setZero();
      residual += imus_factor[i]->give_evaluate_g(x_stats[i], x_stats[i+1], jtj, gg, true);
      Hess.block<STATE_DOF*2, STATE_DOF*2>(i*STATE_DOF, i*STATE_DOF) += jtj.block<2*STATE_DOF, 2*STATE_DOF>(0, 0);
      Hess.block<STATE_DOF*2, 3>(i*STATE_DOF, imu_leng-3) += jtj.block<2*STATE_DOF, 3>(0, 2*STATE_DOF);
      Hess.block<3, STATE_DOF*2>(imu_leng-3, i*STATE_DOF) += jtj.block<3, 2*STATE_DOF>(2*STATE_DOF,0);
      Hess.block<3, 3>(imu_leng-3, imu_leng-3) += jtj.block<3, 3>(2*STATE_DOF, 2*STATE_DOF);

      JacT.block<STATE_DOF*2, 1>(i*STATE_DOF, 0) += gg.head(2*STATE_DOF);
      JacT.tail(3) += gg.tail(3);
    }

    Eigen::Matrix<double, STATE_DOF, STATE_DOF> joc;
    Eigen::Matrix<double, STATE_DOF, 1> rr;
    joc.setIdentity(); rr.setZero();

    Hess *= imu_coef;
    JacT *= imu_coef;
    residual *= (imu_coef * 0.5);

    for(int i=0; i<tthd_num; i++)
    {
      if(i != 0)
      {
        mthreads[i]->join();
      }
      else
      {
        voxhess.acc_evaluate2(x_stats, 0, part, hessians[0], jacobins[0], resis[0]);
      }
      hess_plus(Hess, JacT, hessians[i], jacobins[i]);
      residual += resis[i];
      delete mthreads[i];
    }

    return residual;
  }

  double only_residual(vector<StateGroup> &x_stats, LidarFactor &voxhess, deque<ImuFactor*> &imus_factor)
  {
    double residual1 = 0, residual2 = 0;
    Eigen::MatrixXd jtj(2*STATE_DOF, 2*STATE_DOF);
    Eigen::VectorXd gg(2*STATE_DOF);

    int thd_num = thread_num;
    vector<double> residuals(thd_num, 0);
    int g_size = voxhess.plvec_voxels.size();
    if(g_size < thd_num)
    {
      thd_num = 1;
    }
    vector<thread*> mthreads(thd_num, nullptr);
    double part = 1.0 * g_size / thd_num;
    for(int i=1; i<thd_num; i++)
    {
      mthreads[i] = new thread(&LidarFactor::evaluate_only_residual, &voxhess, x_stats, part*i, part*(i+1), ref(residuals[i]));
    }

    for(int i=0; i<win_size-1; i++)
    {
      residual1 += imus_factor[i]->give_evaluate_g(x_stats[i], x_stats[i+1], jtj, gg, false);
    }
    residual1 *= (imu_coef * 0.5);

    for(int i=0; i<thd_num; i++)
    {
      if(i != 0) 
      {
        mthreads[i]->join(); delete mthreads[i];
      }
      else 
      {
        voxhess.evaluate_only_residual(x_stats, part*i, part*(i+1), residuals[i]);
      }
      residual2 += residuals[i];
    }

    return (residual1 + residual2);
  }

  void damping_iter(vector<StateGroup> &x_stats, LidarFactor &voxhess, deque<ImuFactor*> &imus_factor, vector<double> &resis, Eigen::MatrixXd* hess, int max_iter = 2)
  {
    win_size = voxhess.win_size;
    jac_leng = win_size * 6;
    imu_leng = win_size * STATE_DOF + 3;
    double u = 0.01, v = 2;
    Eigen::MatrixXd D(imu_leng, imu_leng), Hess(imu_leng, imu_leng);
    Eigen::VectorXd JacT(imu_leng), dxi(imu_leng);

    D.setIdentity();
    double residual1, residual2, q;
    bool is_calc_hess = true;
    vector<StateGroup> x_stats_temp = x_stats;
    
    for(int i=0; i<max_iter; i++)
    {
      if(is_calc_hess)
      {
        residual1 = divide_thread(x_stats, voxhess, imus_factor, Hess, JacT);
        *hess = Hess;
      }

      if(i == 0)
      {
        resis.push_back(residual1);
      }

      Hess.topRows(6).setZero();
      Hess.leftCols(6).setZero();
      Hess.block<6, 6>(0, 0).setIdentity();
      JacT.head(6).setZero();

      D.diagonal() = Hess.diagonal();
      dxi = (Hess + u*D).ldlt().solve(-JacT);

      x_stats_temp[0].g += dxi.tail(3);
      for(int j=0; j<win_size; j++)
      {
        x_stats_temp[j].R = x_stats[j].R * lie::SO3d::exp(dxi.block<3, 1>(STATE_DOF*j, 0)).R();
        x_stats_temp[j].p = x_stats[j].p + dxi.block<3, 1>(STATE_DOF*j+3, 0);
        x_stats_temp[j].v = x_stats[j].v + dxi.block<3, 1>(STATE_DOF*j+6, 0);
        x_stats_temp[j].bg = x_stats[j].bg + dxi.block<3, 1>(STATE_DOF*j+9, 0);
        x_stats_temp[j].ba = x_stats[j].ba + dxi.block<3, 1>(STATE_DOF*j+12, 0);
        x_stats_temp[j].g = x_stats_temp[0].g;
      }

      for(int j=0; j<win_size-1; j++)
      {
        imus_factor[j]->update_state(dxi.block<STATE_DOF, 1>(STATE_DOF*j, 0));
      }
      
      double q1 = 0.5 * dxi.dot(u*D*dxi-JacT);
      residual2 = only_residual(x_stats_temp, voxhess, imus_factor);
      q = (residual1-residual2);

      if(q > 0)
      {
        x_stats = x_stats_temp;
        double one_three = 1.0 / 3;

        q = q / q1;
        v = 2;
        q = 1 - pow(2*q-1, 3);
        u *= (q<one_three ? one_three:q);
        is_calc_hess = true;
      }
      else
      {
        u = u * v;
        v = 2 * v;
        is_calc_hess = false;

        for(int j=0; j<win_size-1; j++)
        {
          imus_factor[j]->dbg = imus_factor[j]->dbg_buf;
          imus_factor[j]->dba = imus_factor[j]->dba_buf;
        }
      }

      if(fabs((residual1-residual2)/residual1)<1e-6)
      {
        break;
      }
    }
    resis.push_back(residual2);
    
  }

};

