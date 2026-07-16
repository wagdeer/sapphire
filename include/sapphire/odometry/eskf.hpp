#pragma once

#include <sapphire/types.hpp>
#include <sapphire/odometry/deskew.hpp>
#include <sapphire/odometry/detail/mean_only_gal3_integrator.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <optional>

namespace sapphire {

/// Configuration for the Approach-A ESKF frontend fusion.
struct EskfConfig {
    /// Fallback ICP measurement std when Hessian is unused / invalid.
    /// Keep small: the current GICP frontend is already accurate without ESKF,
    /// so the filter should mostly follow ICP for pose.
    double sigma_rotation = 0.01;      // rad
    double sigma_translation = 0.01;   // m
    /// Inflate H_gicp^{-1} when use_hessian is enabled.
    double icp_covariance_scale = 25.0;
    /// If true, build R from small_gicp Hessian; otherwise diagonal fallback.
    bool use_hessian = false;
    /// χ² threshold for 6-DOF innovation. Set <= 0 to disable the gate.
    /// With high ICP trust / inject_full_pose, prefer hard registration gates
    /// only: a tight Mahalanobis gate rejects good ICP and opens an IMU-only
    /// death spiral within seconds.
    double mahalanobis_threshold = -1.0;
    /// If true, accepted updates set R/p exactly to the ICP pose. Velocity and
    /// biases still come from the Kalman update (bias optionally damped).
    bool inject_full_pose = true;
    /// Scale applied to Kalman bias increments (1 = full ESKF coupling).
    double bias_update_scale = 0.25;
    /// Optional position-innovation velocity correction [1/s]. This avoids
    /// differentiating noisy ICP poses while keeping scan-to-scan prediction
    /// responsive. Set 0 to use the unconstrained Kalman velocity rows.
    double velocity_correction_gain = 0.0;
    double accel_bias_max = 10.0;      // m/s²
    double gyro_bias_max = 0.5;        // rad/s
    /// Initial orientation / velocity / position std used in P0.
    double init_sigma_theta = 0.1;     // rad
    double init_sigma_velocity = 1.0;  // m/s
    double init_sigma_position = 1.0;  // m
};

/// Result of one ESKF LiDAR correction at the scan reference time.
struct EskfUpdate {
    NavigationState state;
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    bool accepted = false;
    double mahalanobis = 0.0;
};

/// Error-state Kalman filter for IMU–LiDAR frontend fusion (Approach A).
///
/// Nominal mean uses Gal(3) preintegration (same as MeanOnlyGal3Integrator).
/// Covariance uses the standard 15D linearized IMU error dynamics.
/// LiDAR pose updates use a right-multiplied SO(3) residual and world-frame
/// additive position residual (see docs/eskf_on_gal3.md).
class Eskf {
public:
    using Mat15 = Eigen::Matrix<double, 15, 15>;
    using Mat6 = Eigen::Matrix<double, 6, 6>;
    using Vec15 = Eigen::Matrix<double, 15, 1>;
    using Vec6 = Eigen::Matrix<double, 6, 1>;

    Eskf(const EskfConfig& config,
         const ImuNoiseConfig& noise,
         const Eigen::Vector3d& gravity_world);

    /// Initialize nominal state, biases, and P0 at the given stamp.
    void initialize(
        const NavigationState& state,
        const Eigen::Vector3d& accel_bias,
        const Eigen::Vector3d& gyro_bias);

    /// Replace the correction baseline and clear the Gal(3) integrator.
    /// Covariance is set to P_ref (typically the posterior at the baseline).
    void setBaseline(
        const NavigationState& state,
        const Eigen::Vector3d& accel_bias,
        const Eigen::Vector3d& gyro_bias,
        const Mat15& P_ref);

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
    /// @param prior_mean  deskew prior at reference_stamp (authoritative mean)
    /// @param T_world_imu_measured  GICP-corrected IMU pose
    /// @param hard_accepted  hard registration gates already passed
    /// @param imu_buffer  bias-corrected IMU buffer (oldest → newest)
    /// @param hessian  optional small_gicp information matrix [r|t]
    EskfUpdate correctAt(
        double reference_stamp,
        const NavigationState& prior_mean,
        const Isometry3d& T_world_imu_measured,
        bool hard_accepted,
        const ImuBuffer& imu_buffer,
        const std::optional<Mat6>& hessian = std::nullopt);

    /// Replay buffered IMU samples newer than the current tip stamp.
    void replayToLatest(const ImuBuffer& imu_buffer);

    const NavigationState& tipState() const { return tip_state_; }
    const NavigationState& baselineState() const { return baseline_state_; }
    Eigen::Vector3d accelBias() const { return accel_bias_; }
    Eigen::Vector3d gyroBias() const { return gyro_bias_; }
    const Mat15& tipCovariance() const { return P_tip_; }
    const Mat15& baselineCovariance() const { return P_baseline_; }
    bool initialized() const { return initialized_; }

    /// Build measurement covariance R (diagonal or Hessian-based).
    Mat6 measurementCovariance(const std::optional<Mat6>& hessian) const;

private:
    void recoverTipLocked(double stamp);
    void propagateCovariance(
        const Eigen::Matrix3d& R_world_imu,
        const Eigen::Vector3d& omega,
        const Eigen::Vector3d& accel,
        double dt,
        Mat15& P) const;
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
    detail::MeanOnlyGal3Integrator integrator_;
    std::size_t correction_count_ = 0;
};

}  // namespace sapphire
