#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <deque>
#include <optional>
#include <sapphire/odometry/deskew.hpp>
#include <sapphire/odometry/detail/mean_only_gal3_integrator.hpp>
#include <sapphire/types.hpp>

namespace sapphire {

/// Scalar fixed point at a 0.1 s scan and R = (0.05 m)² that reproduces the
/// mixed-gain operating point K_p ≈ 0.85, K_v ≈ 0.8. See
/// docs/gal3-eskf-docking.md §11.
inline constexpr double kGal3SpecificForceModelDensity = 0.327;
inline constexpr double kGal3PositionModelDiffusion = 0.1053;

/// Result of one ESKF LiDAR correction at the scan reference time.
struct EskfUpdate {
  NavigationState state;
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  bool accepted = false;
  double mahalanobis = 0.0;
  double normalized_nis = 0.0;
  double mean_normalized_nis = 0.0;
  Eigen::Matrix<double, 6, 1> measurement_std = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> information_eigenvalues = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> pose_gain_diagonal = Eigen::Matrix<double, 6, 1>::Zero();
};

/// Error-state Kalman filter for IMU–LiDAR frontend fusion (Approach A).
///
/// Nominal mean uses Gal(3) preintegration (same as MeanOnlyGal3Integrator)
/// on both charts. The Euclidean chart keeps the 15D world-frame linearization
/// and the legacy gain switches (see docs/eskf_on_gal3.md). gal3_right keeps
/// that mean and replaces the error coordinates as a set.
class Eskf {
 public:
  using Mat15 = Eigen::Matrix<double, 15, 15>;
  using Mat6 = Eigen::Matrix<double, 6, 6>;
  using Vec15 = Eigen::Matrix<double, 15, 1>;
  using Vec6 = Eigen::Matrix<double, 6, 1>;

  Eskf(const EskfConfig& config, const ImuNoiseConfig& noise, const Eigen::Vector3d& gravity_world);

  /// Initialize nominal state, biases, and P0 at the given stamp.
  void initialize(const NavigationState& state, const Eigen::Vector3d& accel_bias, const Eigen::Vector3d& gyro_bias);

  /// Replace the correction baseline and clear the Gal(3) integrator.
  /// Covariance is set to P_ref (typically the posterior at the baseline).
  void setBaseline(const NavigationState& state, const Eigen::Vector3d& accel_bias, const Eigen::Vector3d& gyro_bias, const Mat15& P_ref);

  /// Propagate nominal state and covariance with one bias-corrected IMU sample.
  /// No-op if stamp is not newer than the current tip.
  void predict(const ImuData& imu);

  /// Rebuild mean/covariance from the baseline up to reference_stamp using
  /// buffered IMU, then apply a LiDAR pose measurement.
  ///
  /// On return, tip_state_/tipCovariance() hold the reference posterior, but
  /// the committed baseline is NOT updated. After optional IMU-buffer bias
  /// adjustment, the caller must setBaseline(...) + replayToLatest(...).
  ///
  /// On the Euclidean chart the effective measurement gain is controlled by
  /// three config flags — see the pose-injection priority table in
  /// EskfConfig (inject_full_pose > inject_directional_pose >
  /// velocity_correction_gain > Kalman rows). gal3_right does not enter
  /// those row replacements.
  ///
  /// @param prior_mean  deskew prior at reference_stamp (authoritative mean)
  /// @param T_world_imu_measured  GICP-corrected IMU pose
  /// @param hard_accepted  hard registration gates already passed
  /// @param imu_buffer  bias-corrected IMU buffer (oldest → newest)
  /// @param hessian  optional small_gicp information matrix [r|t]
  EskfUpdate correctAt(double reference_stamp, const NavigationState& prior_mean, const Isometry3d& T_world_imu_measured, bool hard_accepted,
                       const ImuBuffer& imu_buffer, const std::optional<Mat6>& hessian = std::nullopt);

  /// Replay buffered IMU samples newer than the current tip stamp.
  void replayToLatest(const ImuBuffer& imu_buffer);

  const NavigationState& tipState() const { return tip_state_; }
  Eigen::Vector3d accelBias() const { return accel_bias_; }
  Eigen::Vector3d gyroBias() const { return gyro_bias_; }
  const Mat15& tipCovariance() const { return P_tip_; }
  const Mat15& baselineCovariance() const { return P_baseline_; }
  bool initialized() const { return initialized_; }

  /// Build measurement covariance R assuming an identity correction frame.
  /// Primarily retained for simple callers and tests.
  Mat6 measurementCovariance(const std::optional<Mat6>& hessian) const;

  /// Build R in the ESKF innovation coordinates for a GICP correction
  /// Hessian evaluated between prior and measured IMU poses.
  Mat6 measurementCovariance(const std::optional<Mat6>& hessian, const Isometry3d& T_world_imu_prior, const Isometry3d& T_world_imu_measured,
                             Vec6* information_eigenvalues = nullptr, Mat6* directional_observability = nullptr) const;

  /// First-order map from small_gicp correction right perturbations
  /// [rotation, translation] to the Euclidean innovation (z_ω, z_p).
  static Mat6 registrationToInnovationJacobian(const Isometry3d& T_world_imu_prior, const Isometry3d& T_world_imu_measured);

  /// (z_ω, z_a) from Gal3::log(χ̄⁻¹ χ_z), χ_z = (R_m, v̄, p_m, 0).
  static Vec6 gal3RightInnovation(const Eigen::Matrix3d& R_nominal, const Eigen::Vector3d& v_nominal, const Eigen::Vector3d& p_nominal,
                                  const Eigen::Matrix3d& R_measured, const Eigen::Vector3d& p_measured);

  /// χ ← χ̄ · Gal3::exp(ω, α, a, 0). Does not touch biases.
  static void gal3RightRetract(Eigen::Matrix3d& rotation, Eigen::Vector3d& velocity, Eigen::Vector3d& position,
                               const Eigen::Matrix<double, 9, 1>& xi_nav);

  /// One IMU sample, one Φ. Argument of invAdjoint / rightJacobian is û = u·dt.
  static Mat15 gal3RightTransition(const Eigen::Vector3d& omega, const Eigen::Vector3d& accel, double dt);

  /// σ² arguments are continuous PSDs (diagonal entries of the 12×12 Qc).
  static Mat15 gal3RightProcessNoise(const Eigen::Vector3d& omega, const Eigen::Vector3d& accel, double dt, double sigma_g2, double sigma_a2,
                                     double sigma_ba2, double sigma_bg2);

  /// I₁₅ with the navigation block set to Gal3::rightJacobian(ω, α, a, 0).
  static Mat15 gal3RightResetJacobian(const Eigen::Matrix<double, 9, 1>& xi_nav);

  /// Central difference of (z_ω, z_a) w.r.t. a small_gicp right perturbation.
  static Mat6 gal3RightRegistrationJacobian(const Isometry3d& T_world_imu_prior, const Isometry3d& T_world_imu_measured);

 private:
  void recoverTipLocked(double stamp);
  void enforceGal3RightGains();
  void propagateCovariance(const Eigen::Matrix3d& R_world_imu, const Eigen::Vector3d& omega, const Eigen::Vector3d& accel, double dt, Mat15& P) const;
  void propagateCovarianceGal3Right(const Eigen::Vector3d& omega, const Eigen::Vector3d& accel, double dt, Mat15& P) const;
  Mat6 measurementCovarianceEuclidean(const std::optional<Mat6>& hessian, const Isometry3d& T_world_imu_prior, const Isometry3d& T_world_imu_measured,
                                      Vec6* information_eigenvalues, Mat6* directional_observability) const;
  Mat6 measurementCovarianceGal3Right(const std::optional<Mat6>& hessian, const Isometry3d& T_world_imu_prior, const Isometry3d& T_world_imu_measured,
                                      Vec6* information_eigenvalues, Mat6* directional_observability) const;
  EskfUpdate correctEuclidean(double reference_stamp, const Isometry3d& T_world_imu_measured, const std::optional<Mat6>& hessian,
                              double replay_rotation_error, double replay_position_error, double replay_velocity_error);
  EskfUpdate correctGal3Right(double reference_stamp, const Isometry3d& T_world_imu_measured, const std::optional<Mat6>& hessian,
                              double replay_rotation_error, double replay_position_error, double replay_velocity_error);
  Mat15 makeInitialCovariance() const;
  double biasRwAccel() const;
  double biasRwGyro() const;

  EskfConfig config_;
  ImuNoiseConfig noise_;
  Eigen::Vector3d gravity_world_ = Eigen::Vector3d::Zero();

  bool initialized_ = false;
  NavigationState baseline_state_;
  NavigationState tip_state_;
  Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
  Mat15 P_baseline_ = Mat15::Zero();
  Mat15 P_tip_ = Mat15::Zero();
  /// Precomputed IMU noise continuous covariance Qc (12×12 diagonal).
  /// Invariant across the filter lifetime; computed once in the constructor.
  Eigen::Matrix<double, 12, 12> precomputed_Qc_ = Eigen::Matrix<double, 12, 12>::Zero();
  detail::MeanOnlyGal3Integrator integrator_;
  std::size_t correction_count_ = 0;
  std::deque<double> normalized_nis_window_;
  double normalized_nis_sum_ = 0.0;
};

}  // namespace sapphire
