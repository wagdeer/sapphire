#pragma once

#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <sapphire/types.hpp>
#include <sapphire/utils.hpp>
#include <vector>

namespace sapphire {

using scalar = float;
using V3 = Eigen::Matrix<scalar, 3, 1>;
using V18 = Eigen::Matrix<scalar, 18, 1>;
using VV3 = std::vector<V3, Eigen::aligned_allocator<V3>>;
using M3 = Eigen::Matrix<scalar, 3, 3>;
using M4 = Eigen::Matrix<scalar, 4, 4>;
using M18 = Eigen::Matrix<scalar, 18, 18>;
using Quat = Eigen::Quaternion<scalar>;
using V3d = Eigen::Matrix<double, 3, 1>;
using M3d = Eigen::Matrix<double, 3, 3>;

struct FilterRotation {
  FilterRotation() = default;
  FilterRotation(const M3& rotation) : R_(rotation) {}

  static M3 hat(const V3& vector);
  static V3 vee(const M3& matrix);
  static FilterRotation Exp(const V3& rotation);
  static FilterRotation Exp(const V3& rate, const scalar& dt);
  M3 log(double* angle = nullptr) const noexcept;
  V3 log_vee() const noexcept;
  FilterRotation inverse() const noexcept;
  M3 R() const { return R_; }
  FilterRotation operator*(const FilterRotation& rhs) const { return M3(R_ * rhs.R_); }

  M3 R_ = M3::Identity();
};

struct FilterPose {
  FilterPose() = default;
  explicit FilterPose(const M4& transform);
  FilterPose(const M3& rotation, const V3& position);
  FilterPose(const FilterRotation& rotation, const V3& position) : FilterPose(rotation.R_, position) {}
  FilterPose operator*(const FilterPose& rhs) const noexcept;
  V3 operator*(const V3& point) const noexcept;

  M4 T_ = M4::Identity();
  M3 R_ = M3::Identity();
  V3 t_ = V3::Zero();
};

struct SysState {
  SysState() = default;

  explicit SysState(double time, const FilterRotation& R = FilterRotation(), const V3& t = V3::Zero(), const V3& v = V3::Zero(),
                    const V3& bg = V3::Zero(), const V3& ba = V3::Zero())
      : timestamp(time), R(R), p(t), v(v), bg(bg), ba(ba) {}

  double timestamp = 0;
  FilterRotation R;
  V3 p = V3::Zero();
  V3 v = V3::Zero();
  V3 bg = V3::Zero();
  V3 ba = V3::Zero();
};

struct NavState {
  NavState() = default;
  explicit NavState(double time, const FilterRotation& R = FilterRotation(), const V3& t = V3::Zero(), const V3& v = V3::Zero())
      : timestamp(time), R(R), p(t), v(v) {}

  double timestamp = 0;
  FilterRotation R;
  V3 p = V3::Zero();
  V3 v = V3::Zero();
};

struct DynamicState {
  DynamicState() = default;
  explicit DynamicState(double __time, const M3& __R, const V3& __p, const V3& __v, const V3& __w, const V3& __a)
      : time(__time), R(__R), p(__p), v(__v), w(__w), a(__a) {}

  double time = 0;
  M3 R = M3::Identity();
  V3 p = V3::Zero();
  V3 v = V3::Zero();
  V3 w = V3::Zero();
  V3 a = V3::Zero();
};

struct FilterImuSample {
  double secs = 0.0;
  V3 acc = V3::Zero();
  V3 gyr = V3::Zero();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class iEskf {
 public:
  using Ptr = std::shared_ptr<iEskf>;
  using STATE = Eigen::Matrix<scalar, 18, 1>;      // Flatten: R p v bg ba g.
  using STATE_DOF = Eigen::Matrix<scalar, 17, 1>;  // Flatten: R p v bg ba g_2.
  using NOISE = Eigen::Matrix<scalar, 12, 12>;
  using COV = Eigen::Matrix<scalar, 18, 18>;
  using F_X = Eigen::Matrix<scalar, 18, 18>;
  using F_W = Eigen::Matrix<scalar, 18, 12>;

  struct Options {
    Options() {}
    int num_iterations_ = 3;
    double quit_eps_ = 1e-6;

    double gyro_var_ = 1e-5;
    double acce_var_ = 1e-2;
    double bias_gyro_var_ = 1e-6;
    double bias_acce_var_ = 1e-4;
  };

  struct KFState {
    bool need_converge = true;
    FilterPose pose;
  };

  iEskf(Options option = Options()) : options_(option) { BuildNoise(option); }

  iEskf(Options options, const V3& init_bg, const V3& init_ba, const V3& gravity = V3(0, 0, -9.8)) : options_(options) {
    BuildNoise(options);
    bg_ = init_bg;
    ba_ = init_ba;
    g_ = gravity;
  }

  void SetInitialConditions(Options options, const V3& init_bg, const V3& init_ba, const float imu_scale = 1.0, const V3& gravity = V3(0, 0, -9.8));

  bool Predict(const FilterImuSample& imu);

  using ObsFunc = std::function<void(const KFState& kf_state, Mat_66<scalar>& HT_Vinv_H, Vec_6<scalar>& HT_Vinv_r)>;
  bool UpdateObserve(ObsFunc obs);

  double GetTime() const { return current_time_; }

  SysState GetSysState() const { return SysState(current_time_, R_, p_, v_, bg_, ba_); }

  NavState GetNavState() const { return NavState(current_time_, R_, p_, v_); }

  DynamicState GetDynamicState() const { return DynamicState(current_time_, R_.R_, p_, v_, body_omega_, global_acc_); }

  KFState GetKFState() const { return KFState{need_converge_, GetSE3()}; }

  COV GetCov() const { return P_; }

  FilterPose GetSE3() const { return FilterPose(R_, p_); }

  void SetObsTime(const double obs_time) { current_obs_time_ = obs_time; }
  void SetLastObsTime(const double obs_time) { last_obs_time_ = obs_time; }

  void SetX(const SysState& x);

  void SetCov(const COV& cov) { P_ = cov; }

  V3 GetGravity() const { return g_; }

  bool init_ = false;
  void StartTracking(const FilterImuSample& boundary);
  void SetTipMeasurement(const FilterImuSample& sample) { forward_last_imu_ = sample; }
  bool PredictTip(const FilterImuSample& imu, DynamicState& tip);

 private:
  void BuildNoise(const Options& options);
  void Update();

  bool need_converge_ = true;
  float imu_scale_ = 1.0;
  FilterImuSample last_imu_;
  double current_time_ = 0.0;
  double last_imu_time_ = -1.0;
  double last_obs_time_ = 0.0;
  double current_obs_time_ = 0.0;

  // nominal state
  FilterRotation R_;
  V3 p_ = V3::Zero();
  V3 v_ = V3::Zero();
  V3 bg_ = V3::Zero();
  V3 ba_ = V3::Zero();
  double gravity_magnitude_ = 9.8;
  V3 g_{0, 0, -(scalar)gravity_magnitude_};
  V3 global_acc_ = V3::Zero();
  V3 body_omega_ = V3::Zero();

  STATE dx_ = STATE::Zero();

  COV P_ = COV::Identity();

  NOISE Q_ = NOISE::Zero();

  Options options_;

  double forward_time_ = -1;
  FilterImuSample forward_last_imu_;
  FilterRotation fw_R_;
  V3 fw_p_ = V3::Zero();
  V3 fw_v_ = V3::Zero();
};

}  // namespace sapphire
