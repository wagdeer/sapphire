#include <sapphire/odometry/eskf.hpp>

#include <spdlog/spdlog.h>

#include <Eigen/LU>

#include <algorithm>
#include <cmath>

namespace sapphire {
namespace {

using SO3 = detail::MeanOnlyGal3Integrator::Gal3::SO3Type;

/// Binary search: find first index in buf where stamp > threshold.
/// buf must be sorted by stamp (oldest → newest).
inline std::size_t findFirstAfter(
    const ImuBuffer& buf, double threshold)
{
    if (buf.empty()) return 0;
    std::size_t lo = 0, hi = buf.size();
    while (lo < hi) {
        const std::size_t mid = lo + (hi - lo) / 2;
        if (buf[mid].stamp <= threshold) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo;
}

Eigen::Vector3d so3Log(const Eigen::Matrix3d& R) {
    return SO3::log(SO3(R));
}

Eigen::Matrix3d so3Exp(const Eigen::Vector3d& omega) {
    return SO3::exp(omega).R();
}

Eigen::Matrix3d so3RightJacobianInverse(const Eigen::Vector3d& phi) {
    return SO3::invRightJacobian(phi);
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
    return noise_.accel_bias_rw_sigma > 0.0
        ? noise_.accel_bias_rw_sigma
        : noise_.accel_random_walk;
}

double Eskf::biasRwGyro() const {
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
        spdlog::info(
            "[eskf] bias injection fixed; covariance random walk remains active");
    }
    if (noise_.accel_bias_rw_sigma <= 0.0
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
    F.block<3, 3>(0, 0) = -SO3::wedge(omega);
    F.block<3, 3>(0, 12) = -Eigen::Matrix3d::Identity();
    F.block<3, 3>(3, 0) = -R_world_imu * SO3::wedge(accel);
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
    const std::size_t start_idx = findFirstAfter(imu_buffer, tip_state_.stamp);
    for (std::size_t i = start_idx; i < imu_buffer.size(); ++i) {
        predict(imu_buffer[i]);
    }
}

Eskf::Mat6 Eskf::measurementCovariance(
    const std::optional<Mat6>& hessian) const
{
    return measurementCovariance(
        hessian,
        Isometry3d::Identity(),
        Isometry3d::Identity());
}

Eskf::Mat6 Eskf::registrationToInnovationJacobian(
    const Isometry3d& T_world_imu_prior,
    const Isometry3d& T_world_imu_measured)
{
    // small_gicp optimizes the global correction C with a right perturbation:
    // C' = C Exp(delta). The measured IMU pose is C * T_prior. Map delta to
    // [Log(R_prior^T R_measured), p_measured - p_prior].
    const Eigen::Matrix3d R_prior = T_world_imu_prior.rotation();
    const Eigen::Matrix3d R_measured = T_world_imu_measured.rotation();
    const Eigen::Matrix3d R_correction = R_measured * R_prior.transpose();
    const Eigen::Vector3d rotation_innovation =
        so3Log(R_prior.transpose() * R_measured);

    Mat6 J = Mat6::Zero();
    J.block<3, 3>(0, 0) =
        so3RightJacobianInverse(rotation_innovation) * R_prior.transpose();
    J.block<3, 3>(3, 0) =
        -R_correction * SO3::wedge(T_world_imu_prior.translation());
    J.block<3, 3>(3, 3) = R_correction;
    return J;
}

Eskf::Mat6 Eskf::measurementCovariance(
    const std::optional<Mat6>& hessian,
    const Isometry3d& T_world_imu_prior,
    const Isometry3d& T_world_imu_measured,
    Vec6* information_eigenvalues,
    Mat6* directional_observability) const
{
    if (directional_observability != nullptr) {
        *directional_observability = Mat6::Identity();
    }
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

    const Mat6 J = registrationToInnovationJacobian(
        T_world_imu_prior, T_world_imu_measured);
    const Mat6 J_inv = J.inverse();
    // Transform information before classifying eigen-directions. Clipping in
    // the correction coordinates and transforming afterwards produces a
    // non-orthogonal "projector" with gains outside [0, 1].
    Mat6 H =
        J_inv.transpose()
        * (0.5 * (*hessian + hessian->transpose()))
        * J_inv;
    H = 0.5 * (H + H.transpose());
    Eigen::SelfAdjointEigenSolver<Mat6> solver(H);
    if (solver.info() != Eigen::Success) {
        spdlog::warn("[eskf] Hessian eigensolve failed; using diagonal R");
        return R;
    }

    Eigen::Matrix<double, 6, 1> evals = solver.eigenvalues();
    if (information_eigenvalues != nullptr) {
        *information_eigenvalues = evals;
    }

    const double max_eigen =
        std::max(evals.maxCoeff(), config_.hessian_min_information);
    const double weak_threshold = std::max(
        config_.hessian_min_information,
        max_eigen / config_.hessian_max_condition);
    Eigen::Matrix<double, 6, 1> variances;
    Eigen::Matrix<double, 6, 1> observability_weights;
    for (int i = 0; i < 6; ++i) {
        const double information =
            std::max(evals(i), config_.hessian_min_information);
        double sigma =
            std::sqrt(config_.icp_covariance_scale / information);
        if (evals(i) < weak_threshold) {
            sigma = std::max(sigma, config_.hessian_degenerate_sigma);
        }
        sigma = std::min(sigma, config_.hessian_max_sigma);
        variances(i) = sigma * sigma;
        observability_weights(i) = std::clamp(
            evals(i) / weak_threshold, 0.0, 1.0);
    }

    const Mat6 innovation_covariance =
        solver.eigenvectors()
        * variances.asDiagonal()
        * solver.eigenvectors().transpose();
    if (directional_observability != nullptr) {
        *directional_observability =
            solver.eigenvectors()
            * observability_weights.asDiagonal()
            * solver.eigenvectors().transpose();
    }
    // The configured diagonal covariance is a physical noise floor. The
    // Hessian only contributes additional geometry-dependent uncertainty.
    R += innovation_covariance;
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
    const std::size_t start_idx = findFirstAfter(imu_buffer, baseline_state_.stamp);
    const ImuData* next_imu = nullptr;
    for (std::size_t i = start_idx; i < imu_buffer.size(); ++i) {
        const ImuData& imu = imu_buffer[i];
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

    Mat6 directional_observability = Mat6::Identity();
    const Mat6 R = measurementCovariance(
        hessian,
        prior_mean.T_world_imu,
        T_world_imu_measured,
        &update.information_eigenvalues,
        &directional_observability);
    update.measurement_std = R.diagonal().cwiseMax(0.0).cwiseSqrt();
    const Mat6 S = H * P_tip_ * H.transpose() + R;
    Eigen::LDLT<Mat6> ldlt(S);
    if (ldlt.info() != Eigen::Success) {
        spdlog::warn("[eskf] innovation covariance factorization failed");
        return finishWithoutInjection();
    }

    const Vec6 Sinv_nu = ldlt.solve(nu);
    update.mahalanobis = nu.dot(Sinv_nu);
    update.normalized_nis = update.mahalanobis / 6.0;
    constexpr std::size_t kNisWindowSize = 100;
    normalized_nis_window_.push_back(update.normalized_nis);
    normalized_nis_sum_ += update.normalized_nis;
    if (normalized_nis_window_.size() > kNisWindowSize) {
        normalized_nis_sum_ -= normalized_nis_window_.front();
        normalized_nis_window_.pop_front();
    }
    update.mean_normalized_nis =
        normalized_nis_sum_
        / static_cast<double>(normalized_nis_window_.size());
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
        Eigen::Matrix3d position_observability =
            Eigen::Matrix3d::Identity();
        if (config_.use_hessian && !config_.inject_full_pose) {
            // During directional bring-up, retain the stable velocity
            // observer but project it through Hessian observability only.
            // Unlike the Kalman gain, this projector does not decay merely
            // because repeated good measurements made P small.
            Eigen::Matrix3d raw_observability =
                0.5 * (
                    directional_observability.block<3, 3>(3, 3)
                    + directional_observability.block<3, 3>(3, 3).transpose());
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> obs_solver(
                raw_observability);
            if (obs_solver.info() == Eigen::Success) {
                Eigen::Vector3d weights =
                    obs_solver.eigenvalues().cwiseMax(0.0).cwiseMin(1.0);
                position_observability =
                    obs_solver.eigenvectors()
                    * weights.asDiagonal()
                    * obs_solver.eigenvectors().transpose();
            }
        }
        K_effective.block<3, 6>(3, 0).setZero();
        K_effective.block<3, 3>(3, 3) =
            correction_dt
            * config_.velocity_correction_gain
            * position_observability;
    }

    if (config_.inject_directional_pose
        && config_.use_hessian
        && hessian.has_value()) {
        // This bring-up mode expresses the intended degeneracy semantics
        // directly: GICP owns observable correction directions while the IMU
        // prior owns weak ones. Joseph below uses the same effective gain.
        K_effective.block<3, 6>(0, 0) =
            directional_observability.block<3, 6>(0, 0);
        K_effective.block<3, 6>(6, 0) =
            directional_observability.block<3, 6>(3, 0);
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
    update.pose_gain_diagonal.segment<3>(0) =
        K_effective.block<3, 3>(0, 0).diagonal();
    update.pose_gain_diagonal.segment<3>(3) =
        K_effective.block<3, 3>(6, 3).diagonal();

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
        lie::SO3d::invLeftJacobian(-dx.segment<3>(0));
    P_tip_ = Jr * P_upd * Jr.transpose();
    P_tip_ = 0.5 * (P_tip_ + P_tip_.transpose());

    ++correction_count_;
    if (correction_count_ % 10 == 0 || dx.segment<3>(3).norm() > 0.25) {
        spdlog::warn(
            "[eskf] update #{}: nu_rot={:.4f}rad nu_pos={:.3f}m "
            "dv={:.3f}m/s replay_err(r,p,v)=({:.4f},{:.3f},{:.3f}) "
            "ba={:.4f} bg={:.5f} "
            "P(v,p,ba,bg)=({:.3e},{:.3e},{:.3e},{:.3e}) "
            "NIS/6={:.2f} mean={:.2f}",
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
            update.normalized_nis,
            update.mean_normalized_nis);
        if (config_.use_hessian) {
            spdlog::warn(
                "[eskf] directional: info=[{:.2e},{:.2e}] "
                "std(r,p)=({:.3f},{:.3f},{:.3f};"
                "{:.3f},{:.3f},{:.3f}) Kpose=({:.2f},{:.2f},{:.2f};"
                "{:.2f},{:.2f},{:.2f})",
                update.information_eigenvalues.minCoeff(),
                update.information_eigenvalues.maxCoeff(),
                update.measurement_std(0),
                update.measurement_std(1),
                update.measurement_std(2),
                update.measurement_std(3),
                update.measurement_std(4),
                update.measurement_std(5),
                update.pose_gain_diagonal(0),
                update.pose_gain_diagonal(1),
                update.pose_gain_diagonal(2),
                update.pose_gain_diagonal(3),
                update.pose_gain_diagonal(4),
                update.pose_gain_diagonal(5));
        }
    }

    update.state = tip_state_;
    update.accel_bias = accel_bias_;
    update.gyro_bias = gyro_bias_;
    update.accepted = true;
    return update;
}

}  // namespace sapphire
