#include <sapphire/odometry/eskf.hpp>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace sapphire {
namespace {

using SO3 = detail::MeanOnlyGal3Integrator::Gal3::SO3Type;

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
        -v.y(), v.x(), 0.0;
    return m;
}

Eigen::Vector3d so3Log(const Eigen::Matrix3d& R) {
    return SO3::log(SO3(R));
}

Eigen::Matrix3d so3Exp(const Eigen::Vector3d& omega) {
    return SO3::exp(omega).R();
}

}  // namespace

Eskf::Eskf(
    const EskfConfig& config,
    const ImuNoiseConfig& noise,
    const Eigen::Vector3d& gravity_world)
    : config_(config)
    , noise_(noise)
    , gravity_world_(gravity_world)
{
}

double Eskf::biasRwAccel() const {
    if (config_.bias_update_scale <= 0.0) {
        return 0.0;
    }
    return noise_.accel_bias_rw_sigma > 0.0
        ? noise_.accel_bias_rw_sigma
        : noise_.accel_random_walk;
}

double Eskf::biasRwGyro() const {
    if (config_.bias_update_scale <= 0.0) {
        return 0.0;
    }
    return noise_.gyro_bias_rw_sigma > 0.0
        ? noise_.gyro_bias_rw_sigma
        : noise_.gyro_random_walk;
}

Eskf::Mat15 Eskf::makeInitialCovariance() const {
    Mat15 P = Mat15::Zero();
    P.block<3, 3>(0, 0) =
        Eigen::Matrix3d::Identity()
        * config_.init_sigma_theta * config_.init_sigma_theta;
    P.block<3, 3>(3, 3) =
        Eigen::Matrix3d::Identity()
        * config_.init_sigma_velocity * config_.init_sigma_velocity;
    P.block<3, 3>(6, 6) =
        Eigen::Matrix3d::Identity()
        * config_.init_sigma_position * config_.init_sigma_position;
    const double ba0 = std::max(noise_.accel_bias_sigma, 1e-6);
    const double bg0 = std::max(noise_.gyro_bias_sigma, 1e-6);
    P.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * ba0 * ba0;
    P.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * bg0 * bg0;
    return P;
}

void Eskf::initialize(
    const NavigationState& state,
    const Eigen::Vector3d& accel_bias,
    const Eigen::Vector3d& gyro_bias)
{
    const Mat15 P0 = makeInitialCovariance();
    setBaseline(state, accel_bias, gyro_bias, P0);
    initialized_ = true;

    if (config_.bias_update_scale <= 0.0) {
        spdlog::info("[eskf] bias state fixed; bias random walk disabled");
    } else if (noise_.accel_bias_rw_sigma <= 0.0
               || noise_.gyro_bias_rw_sigma <= 0.0) {
        spdlog::info(
            "[eskf] using imu.noise.*_random_walk for bias RW "
            "(accel={:.3e}, gyro={:.3e}); set *_bias_rw_sigma to override",
            biasRwAccel(),
            biasRwGyro());
    }
}

void Eskf::setBaseline(
    const NavigationState& state,
    const Eigen::Vector3d& accel_bias,
    const Eigen::Vector3d& gyro_bias,
    const Mat15& P_ref)
{
    baseline_state_ = state;
    baseline_state_.valid = true;
    tip_state_ = baseline_state_;
    accel_bias_ = accel_bias;
    gyro_bias_ = gyro_bias;
    P_baseline_ = 0.5 * (P_ref + P_ref.transpose());
    P_tip_ = P_baseline_;
    integrator_.reset();
    initialized_ = true;
}

void Eskf::recoverTipLocked(double stamp) {
    using Gal3 = detail::MeanOnlyGal3Integrator::Gal3;
    Gal3::IsometriesType initial_isometries{
        baseline_state_.v_world,
        baseline_state_.T_world_imu.translation(),
    };
    const Gal3 initial_state(
        baseline_state_.T_world_imu.rotation(),
        initial_isometries,
        0.0);
    const Gal3 propagated = detail::recoverWorldState(
        integrator_.Upsilon(),
        initial_state,
        gravity_world_);

    tip_state_.stamp = stamp;
    tip_state_.T_world_imu = Isometry3d::Identity();
    tip_state_.T_world_imu.linear() = propagated.R();
    tip_state_.T_world_imu.translation() = propagated.p();
    tip_state_.v_world = propagated.v();
    tip_state_.valid = true;
}

void Eskf::propagateCovariance(
    const Eigen::Matrix3d& R_world_imu,
    const Eigen::Vector3d& omega,
    const Eigen::Vector3d& accel,
    double dt,
    Mat15& P) const
{
    if (dt <= 0.0) {
        return;
    }

    // Match MeanOnlyGal3Integrator: subdivide large gaps so the first-order
    // Φ = I + F dt discretization stays consistent with the mean path.
    constexpr double kMaxStepSec = 0.02;
    const int steps = std::max(
        1, static_cast<int>(std::ceil(dt / kMaxStepSec)));
    const double step_dt = dt / static_cast<double>(steps);

    Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Zero();
    F.block<3, 3>(0, 0) = -skew(omega);
    F.block<3, 3>(0, 12) = -Eigen::Matrix3d::Identity();
    F.block<3, 3>(3, 0) = -R_world_imu * skew(accel);
    F.block<3, 3>(3, 9) = -R_world_imu;
    F.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 15, 12> G = Eigen::Matrix<double, 15, 12>::Zero();
    G.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
    G.block<3, 3>(3, 3) = -R_world_imu;
    G.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();
    G.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 12, 12> Qc = Eigen::Matrix<double, 12, 12>::Zero();
    const double sg = noise_.gyro_noise_density;
    const double sa = noise_.accel_noise_density;
    const double sba = biasRwAccel();
    const double sbg = biasRwGyro();
    Qc.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * sg * sg;
    Qc.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * sa * sa;
    Qc.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * sba * sba;
    Qc.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * sbg * sbg;

    const Mat15 Phi = Mat15::Identity() + F * step_dt;
    const Mat15 Qd =
        Phi * G * Qc * G.transpose() * Phi.transpose() * step_dt;
    for (int step = 0; step < steps; ++step) {
        P = Phi * P * Phi.transpose() + Qd;
    }
    P = 0.5 * (P + P.transpose());
}

void Eskf::predict(const ImuData& imu) {
    if (!initialized_ || !tip_state_.valid || imu.stamp <= tip_state_.stamp) {
        return;
    }

    const double dt = imu.stamp - tip_state_.stamp;
    const Eigen::Matrix3d R_before = tip_state_.T_world_imu.rotation();
    const Eigen::Vector3d omega = imu.gyro;
    const Eigen::Vector3d accel = imu.accel;

    integrator_.integrate(imu, dt);
    recoverTipLocked(imu.stamp);
    propagateCovariance(R_before, omega, accel, dt, P_tip_);
}

void Eskf::replayToLatest(const ImuBuffer& imu_buffer) {
    for (const ImuData& imu : imu_buffer) {
        predict(imu);
    }
}

Eskf::Mat6 Eskf::measurementCovariance(
    const std::optional<Mat6>& hessian) const
{
    Mat6 R = Mat6::Zero();
    R.block<3, 3>(0, 0) =
        Eigen::Matrix3d::Identity()
        * config_.sigma_rotation * config_.sigma_rotation;
    R.block<3, 3>(3, 3) =
        Eigen::Matrix3d::Identity()
        * config_.sigma_translation * config_.sigma_translation;

    if (!config_.use_hessian || !hessian.has_value()) {
        return R;
    }

    Mat6 H = 0.5 * (*hessian + hessian->transpose());
    Eigen::SelfAdjointEigenSolver<Mat6> solver(H);
    if (solver.info() != Eigen::Success) {
        spdlog::warn("[eskf] Hessian eigensolve failed; using diagonal R");
        return R;
    }

    Eigen::Matrix<double, 6, 1> evals = solver.eigenvalues();
    constexpr double kMinEigen = 1e-4;
    for (int i = 0; i < 6; ++i) {
        evals(i) = std::max(evals(i), kMinEigen);
    }
    Mat6 H_inv =
        solver.eigenvectors()
        * evals.cwiseInverse().asDiagonal()
        * solver.eigenvectors().transpose();
    R = config_.icp_covariance_scale * H_inv;
    R = 0.5 * (R + R.transpose());
    return R;
}

EskfUpdate Eskf::correctAt(
    double reference_stamp,
    const NavigationState& prior_mean,
    const Isometry3d& T_world_imu_measured,
    bool hard_accepted,
    const ImuBuffer& imu_buffer,
    const std::optional<Mat6>& hessian)
{
    EskfUpdate update;
    update.state = prior_mean;
    update.state.stamp = reference_stamp;
    update.state.valid = true;
    update.accel_bias = accel_bias_;
    update.gyro_bias = gyro_bias_;
    update.accepted = false;

    if (!initialized_ || !baseline_state_.valid) {
        spdlog::warn("[eskf] correctAt called before initialize");
        return update;
    }

    // Rebuild covariance from the last correction baseline to the reference.
    integrator_.reset();
    tip_state_ = baseline_state_;
    P_tip_ = P_baseline_;
    const ImuData* next_imu = nullptr;
    for (const ImuData& imu : imu_buffer) {
        if (imu.stamp <= baseline_state_.stamp) {
            continue;
        }
        if (imu.stamp > reference_stamp) {
            next_imu = &imu;
            break;
        }
        predict(imu);
    }

    // The scan reference normally falls between two IMU samples. Deskew
    // interpolates its mean to that exact time, so propagate P through the
    // remaining partial interval with the same interval-end measurement
    // convention before replacing the mean with prior_mean.
    const double remaining_dt = reference_stamp - tip_state_.stamp;
    constexpr double kStampToleranceSec = 1e-9;
    if (remaining_dt > kStampToleranceSec) {
        if (next_imu != nullptr) {
            const Eigen::Matrix3d R_before =
                tip_state_.T_world_imu.rotation();
            integrator_.integrate(*next_imu, remaining_dt);
            recoverTipLocked(reference_stamp);
            propagateCovariance(
                R_before,
                next_imu->gyro,
                next_imu->accel,
                remaining_dt,
                P_tip_);
        } else {
            spdlog::warn(
                "[eskf] covariance not propagated to reference stamp: "
                "tip={:.6f}, reference={:.6f}",
                tip_state_.stamp,
                reference_stamp);
        }
    }

    const double replay_position_error =
        (prior_mean.T_world_imu.translation()
         - tip_state_.T_world_imu.translation()).norm();
    const double replay_velocity_error =
        (prior_mean.v_world - tip_state_.v_world).norm();
    const double replay_rotation_error =
        so3Log(
            tip_state_.T_world_imu.rotation().transpose()
            * prior_mean.T_world_imu.rotation()).norm();

    // Authoritative mean comes from deskew (matches GICP prior).
    tip_state_ = prior_mean;
    tip_state_.stamp = reference_stamp;
    tip_state_.valid = true;

    auto finishWithoutInjection = [&]() {
        tip_state_.stamp = reference_stamp;
        tip_state_.valid = true;
        update.state = tip_state_;
        update.accel_bias = accel_bias_;
        update.gyro_bias = gyro_bias_;
        update.accepted = false;
        return update;
    };

    if (!hard_accepted) {
        return finishWithoutInjection();
    }

    const Eigen::Matrix3d R_bar = tip_state_.T_world_imu.rotation();
    const Eigen::Vector3d p_bar = tip_state_.T_world_imu.translation();
    const Eigen::Vector3d v_bar = tip_state_.v_world;
    const Eigen::Matrix3d R_meas = T_world_imu_measured.rotation();
    const Eigen::Vector3d p_meas = T_world_imu_measured.translation();

    Vec6 nu = Vec6::Zero();
    nu.segment<3>(0) = so3Log(R_bar.transpose() * R_meas);
    nu.segment<3>(3) = p_meas - p_bar;

    Eigen::Matrix<double, 6, 15> H = Eigen::Matrix<double, 6, 15>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();

    const Mat6 R = measurementCovariance(hessian);
    const Mat6 S = H * P_tip_ * H.transpose() + R;
    Eigen::LDLT<Mat6> ldlt(S);
    if (ldlt.info() != Eigen::Success) {
        spdlog::warn("[eskf] innovation covariance factorization failed");
        return finishWithoutInjection();
    }

    const Vec6 Sinv_nu = ldlt.solve(nu);
    update.mahalanobis = nu.dot(Sinv_nu);
    // Optional diagnostic gate. Disabled by default (<=0): with high ICP trust
    // this gate previously rejected valid corrections and left the filter in
    // IMU-only open loop until VGICP also failed.
    if (config_.mahalanobis_threshold > 0.0
        && update.mahalanobis > config_.mahalanobis_threshold) {
        spdlog::warn(
            "[eskf] Mahalanobis rejection: d={:.2f} > {:.2f}",
            update.mahalanobis,
            config_.mahalanobis_threshold);
        return finishWithoutInjection();
    }

    const Eigen::Matrix<double, 15, 6> K =
        P_tip_ * H.transpose() * ldlt.solve(Mat6::Identity());
    Eigen::Matrix<double, 15, 6> K_effective = K;

    // Any deliberately damped state update must use the same effective gain
    // in the covariance update. Scaling dx alone makes P overconfident.
    const double bias_scale = std::clamp(config_.bias_update_scale, 0.0, 1.0);
    K_effective.block<3, 6>(9, 0) *= bias_scale;
    K_effective.block<3, 6>(12, 0) *= bias_scale;

    if (config_.velocity_correction_gain > 0.0) {
        // Position-only Kalman cross-covariance was too weak in the validated
        // Mid-360 sequence: a persistent ~0.2 m innovation changed velocity by
        // only ~0.05 m/s and produced a 10 Hz sawtooth. Use the same stable
        // innovation channel as the geometric observer, but include its actual
        // gain in the Joseph covariance update.
        constexpr double kMaxCorrectionDtSec = 0.2;
        const double correction_dt = std::clamp(
            reference_stamp - baseline_state_.stamp,
            0.0,
            kMaxCorrectionDtSec);
        K_effective.block<3, 6>(3, 0).setZero();
        K_effective.block<3, 3>(3, 3) =
            Eigen::Matrix3d::Identity()
            * correction_dt
            * config_.velocity_correction_gain;
    }

    if (config_.inject_full_pose) {
        // Pose is replaced by the accepted ICP measurement, i.e. its actual
        // measurement gain is identity. Encode that gain explicitly so the
        // Joseph update describes the state injection that really occurred.
        K_effective.block<3, 6>(0, 0).setZero();
        K_effective.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        K_effective.block<3, 6>(6, 0).setZero();
        K_effective.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity();
    }

    const Vec15 dx = K_effective * nu;

    tip_state_.stamp = reference_stamp;
    tip_state_.valid = true;
    tip_state_.v_world = v_bar + dx.segment<3>(3);
    if (config_.inject_full_pose) {
        // Match the working observer frontend: pose follows ICP when accepted.
        tip_state_.T_world_imu.linear() = R_meas;
        tip_state_.T_world_imu.translation() = p_meas;
    } else {
        tip_state_.T_world_imu.linear() = R_bar * so3Exp(dx.segment<3>(0));
        tip_state_.T_world_imu.translation() = p_bar + dx.segment<3>(6);
    }

    accel_bias_ += dx.segment<3>(9);
    gyro_bias_ += dx.segment<3>(12);
    accel_bias_ = accel_bias_.array()
        .min(config_.accel_bias_max)
        .max(-config_.accel_bias_max);
    gyro_bias_ = gyro_bias_.array()
        .min(config_.gyro_bias_max)
        .max(-config_.gyro_bias_max);

    // Joseph form + first-order reset Jacobian on orientation.
    const Mat15 I = Mat15::Identity();
    Mat15 P_upd =
        (I - K_effective * H)
            * P_tip_
            * (I - K_effective * H).transpose()
        + K_effective * R * K_effective.transpose();

    Mat15 Jr = Mat15::Identity();
    Jr.block<3, 3>(0, 0) =
        Eigen::Matrix3d::Identity() - 0.5 * skew(dx.segment<3>(0));
    P_tip_ = Jr * P_upd * Jr.transpose();
    P_tip_ = 0.5 * (P_tip_ + P_tip_.transpose());

    ++correction_count_;
    if (correction_count_ % 10 == 0 || dx.segment<3>(3).norm() > 0.25) {
        spdlog::warn(
            "[eskf] update #{}: nu_rot={:.4f}rad nu_pos={:.3f}m "
            "dv={:.3f}m/s replay_err(r,p,v)=({:.4f},{:.3f},{:.3f}) "
            "ba={:.4f} bg={:.5f} "
            "P(v,p,ba,bg)=({:.3e},{:.3e},{:.3e},{:.3e}) maha={:.2f}",
            correction_count_,
            nu.segment<3>(0).norm(),
            nu.segment<3>(3).norm(),
            dx.segment<3>(3).norm(),
            replay_rotation_error,
            replay_position_error,
            replay_velocity_error,
            accel_bias_.norm(),
            gyro_bias_.norm(),
            P_tip_.block<3, 3>(3, 3).trace(),
            P_tip_.block<3, 3>(6, 6).trace(),
            P_tip_.block<3, 3>(9, 9).trace(),
            P_tip_.block<3, 3>(12, 12).trace(),
            update.mahalanobis);
    }

    update.state = tip_state_;
    update.accel_bias = accel_bias_;
    update.gyro_bias = gyro_bias_;
    update.accepted = true;
    return update;
}

}  // namespace sapphire
