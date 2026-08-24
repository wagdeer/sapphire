#pragma once

#include <sapphire/types.hpp>
#include <sapphire/odometry/deskew.hpp>
#include <sapphire/odometry/detail/mean_only_gal3_integrator.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <deque>
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
    /// Numerical and degeneracy controls for the GICP information matrix.
    double hessian_min_information = 1e-6;
    double hessian_max_condition = 1e4;
    double hessian_degenerate_sigma = 1.0;
    double hessian_max_sigma = 10.0;
    /// χ² threshold for 6-DOF innovation. Set <= 0 to disable the gate.
    /// With high ICP trust / inject_full_pose, prefer hard registration gates
    /// only: a tight Mahalanobis gate rejects good ICP and opens an IMU-only
    /// death spiral within seconds.
    double mahalanobis_threshold = -1.0;
    /// Pose-injection modes — interaction semantics
    /// ============================================
    /// correctAt() evaluates three gain-modifying flags in a fixed order.
    /// Because later blocks unconditionally overwrite rows of K_effective,
    /// the flags follow an explicit priority hierarchy:
    ///
    ///    inject_full_pose  >  inject_directional_pose  >  Kalman rows
    ///   (highest priority)                              (lowest priority)
    ///
    /// ── Priority table ─────────────────────────────────────────────────
    /// | velocity_correction_gain? | inject_directional? | inject_full? | Result                             |
    /// |---------------------------|---------------------|--------------|------------------------------------|
    /// | 0                         | false               | false        | Pure Kalman on all 15 states       |
    /// | >0                        | false               | false        | Kalman orientation + biases;       |
    /// |                           |                     |              | velocity via position-observer     |
    /// | any                       | true                | false        | Kalman velocity + biases; pose     |
    /// |                           |                     |              | rows projected by Hessian obs.     |
    /// | any                       | any                 | true         | Pose = ICP measurement (identity   |
    /// |                           |                     |              | gain); velocity/biases via Kalman  |
    /// |---------------------------|---------------------|--------------|------------------------------------|
    ///
    /// Note: inject_full_pose overwrites the rot/pos rows that may have been
    /// set by inject_directional_pose or the baseline Kalman gain.  Enabling
    /// both directional and full pose injection simultaneously is wasteful
    /// (directional rows are computed but immediately discarded).
    ///
    /// Note: inject_directional_pose requires use_hessian = true and a valid
    /// Hessian to produce meaningful observability weights.  Without those,
    /// directional_observability remains identity and the flag is a no-op.
    ///
    /// velocity_correction_gain also interacts with inject_directional_pose
    /// when use_hessian is enabled: the velocity observer is projected through
    /// position observability extracted from the Hessian, so weak directions
    /// receive proportional rather than full velocity updates.
    ///
    /// ── When to use each mode ──────────────────────────────────────────
    ///   • Pure Kalman (all false): highest theoretical accuracy when ICP
    ///     covariance is well-calibrated and biases are known.
    ///   • velocity_correction_gain > 0: recommended when ICP gives sharp
    ///     pose jumps that would otherwise mislead velocity differencing.
    ///     The observer acts as a low-pass on velocity while avoiding
    ///     Jacobian-based coupling that may be too weak in practice.
    ///   • inject_directional_pose: useful when ICP is only partially
    ///     observable (e.g. long corridors) — lets ICP correct the strong
    ///     eigen-directions while the IMU prior stabilizes the weak ones.
    ///   • inject_full_pose: simplest / most robust.  ICP owns pose; Kalman
    ///     only refines velocity, accel bias, and gyro bias.  This matches
    ///     the observer frontend's correction semantics and is the default.
    ///
    /// If true, accepted updates set R/p exactly to the ICP pose. Velocity and
    /// biases still come from the Kalman update (bias optionally damped).
    bool inject_full_pose = true;
    /// If true (and Hessian is enabled), accepted pose increments use the
    /// Hessian observability projector rather than the statistical Kalman
    /// pose rows. Strong directions follow GICP; weak directions retain IMU.
    bool inject_directional_pose = false;
    /// Scale applied to Kalman bias increments (1 = full ESKF coupling).
    double bias_update_scale = 0.25;
    /// Optional position-innovation velocity correction [1/s]. This avoids
    /// differentiating noisy ICP poses while keeping scan-to-scan prediction
    /// responsive. Set 0 to use the unconstrained Kalman velocity rows.
    /// See the pose-injection priority table above for interaction with
    /// inject_directional_pose / inject_full_pose.
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
    double normalized_nis = 0.0;
    double mean_normalized_nis = 0.0;
    Eigen::Matrix<double, 6, 1> measurement_std =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> information_eigenvalues =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> pose_gain_diagonal =
        Eigen::Matrix<double, 6, 1>::Zero();
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
    /// The effective measurement gain is controlled by three config flags —
    /// see the pose-injection priority table in EskfConfig for their
    /// interaction semantics (inject_full_pose > inject_directional_pose >
    /// velocity_correction_gain > Kalman rows).
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

    /// Build measurement covariance R assuming an identity correction frame.
    /// Primarily retained for simple callers and tests.
    Mat6 measurementCovariance(const std::optional<Mat6>& hessian) const;

    /// Build R in the ESKF innovation coordinates for a GICP correction
    /// Hessian evaluated between prior and measured IMU poses.
    Mat6 measurementCovariance(
        const std::optional<Mat6>& hessian,
        const Isometry3d& T_world_imu_prior,
        const Isometry3d& T_world_imu_measured,
        Vec6* information_eigenvalues = nullptr,
        Mat6* directional_observability = nullptr) const;

    /// First-order map from small_gicp correction right perturbations
    /// [rotation, translation] to the ESKF innovation coordinates.
    static Mat6 registrationToInnovationJacobian(
        const Isometry3d& T_world_imu_prior,
        const Isometry3d& T_world_imu_measured);

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
    /// Precomputed IMU noise continuous covariance Qc (12×12 diagonal).
    /// Invariant across the filter lifetime; computed once in the constructor.
    Eigen::Matrix<double, 12, 12> precomputed_Qc_ =
        Eigen::Matrix<double, 12, 12>::Zero();
    detail::MeanOnlyGal3Integrator integrator_;
    std::size_t correction_count_ = 0;
    std::deque<double> normalized_nis_window_;
    double normalized_nis_sum_ = 0.0;
};

}  // namespace sapphire
