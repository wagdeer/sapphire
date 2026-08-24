#pragma once

#include "../thirdparty/lie/SO3.hpp"

#include <Eigen/Core>

constexpr int POSE_DOF = 6;
constexpr int STATE_DOF = 15;
constexpr double G_m_s2 = 9.81;

struct StateGroup
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  double t;
  Eigen::Matrix3d R;
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d bg;
  Eigen::Vector3d ba;
  Eigen::Vector3d g;
  Eigen::Matrix<double, STATE_DOF, STATE_DOF> cov;
  
  StateGroup()
  {
    setZero();
  }

  StateGroup(double _t, const Eigen::Matrix3d &_R, const Eigen::Vector3d &_p, const Eigen::Vector3d &_v, const Eigen::Vector3d &_bg, const Eigen::Vector3d &_ba, const Eigen::Vector3d &_g = Eigen::Vector3d(0, 0, -G_m_s2)) 
    : t(_t), R(_R), p(_p), v(_v), bg(_bg), ba(_ba), g(_g) {}

  StateGroup &operator+=(const Eigen::Matrix<double, STATE_DOF, 1> &ist)
  {
    this->R = this->R * lie::SO3d::exp(ist.block<3, 1>(0, 0)).R();
    this->p += ist.block<3, 1>(3, 0);
    this->v += ist.block<3, 1>(6, 0);
    this->bg += ist.block<3, 1>(9, 0);
    this->ba += ist.block<3, 1>(12, 0);
    return *this;
  }

  Eigen::Matrix<double, STATE_DOF, 1> operator-(const StateGroup &b) 
  {
    Eigen::Matrix<double, STATE_DOF, 1> a;
    a.block<3, 1>(0, 0) = lie::SO3d::log(lie::SO3d(b.R.transpose() * this->R));
    a.block<3, 1>(3, 0) = this->p - b.p;
    a.block<3, 1>(6, 0) = this->v - b.v;
    a.block<3, 1>(9, 0) = this->bg - b.bg;
    a.block<3, 1>(12, 0) = this->ba - b.ba;
    return a;
  }

  StateGroup &operator=(const StateGroup &b)
  {
    this->R = b.R;
    this->p = b.p;
    this->v = b.v;
    this->bg = b.bg;
    this->ba = b.ba;
    this->g = b.g;
    this->t = b.t;
    this->cov = b.cov;
    return *this;
  }

  void setZero()
  {
    t = 0; R.setIdentity();
    p.setZero(); v.setZero();
    bg.setZero(); ba.setZero();
    cov.setIdentity(); 
    cov *= 0.0001;
    cov.block<6, 6>(9, 9) = Eigen::Matrix<double, 6, 6>::Identity() * 0.00001;
  }

};
