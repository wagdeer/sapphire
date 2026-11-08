#include <sapphire/config.hpp>
#include <sapphire/odometry/detail/mean_only_gal3_integrator.hpp>
#include <sapphire/odometry/eskf.hpp>

#include <Gal3.hpp>
#include <SO3.hpp>

#include <Eigen/Cholesky>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Vec9 = Eigen::Matrix<double, 9, 1>;
using Vec10 = Eigen::Matrix<double, 10, 1>;
using Vec15 = Eigen::Matrix<double, 15, 1>;
using Mat3 = Eigen::Matrix3d;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Mat9 = Eigen::Matrix<double, 9, 9>;
using Mat96 = Eigen::Matrix<double, 9, 6>;
using Mat15 = Eigen::Matrix<double, 15, 15>;
using Gal3 = lie::Gal3d;
using SO3 = lie::SO3d;

const Vec3 kGravity(0.0, 0.0, -9.80665);

[[noreturn]] void fail(const std::string& message) {
    throw std::runtime_error(message);
}

void expect(bool condition, const std::string& message) {
    if (!condition) {
        fail(message);
    }
}

double maxAbs(const Eigen::MatrixXd& matrix) {
    return matrix.cwiseAbs().maxCoeff();
}

Mat3 so3Exp(const Vec3& omega) {
    return SO3::exp(omega).R();
}

double degrees(double deg) {
    return deg * std::acos(-1.0) / 180.0;
}

sapphire::Isometry3d rightSe3Exp(const Vec6& delta) {
    sapphire::Isometry3d result = sapphire::Isometry3d::Identity();
    const Vec3 omega = delta.head<3>();
    result.linear() = so3Exp(omega);
    result.translation() = SO3::leftJacobian(omega) * delta.tail<3>();
    return result;
}

struct Nominal {
    Mat3 rotation = Mat3::Identity();
    Vec3 velocity = Vec3::Zero();
    Vec3 position = Vec3::Zero();
};

Gal3 groupOf(const Nominal& nominal) {
    return Gal3(
        nominal.rotation,
        Gal3::IsometriesType{nominal.velocity, nominal.position},
        0.0);
}

Nominal nominalFrom(const Gal3& group) {
    Nominal nominal;
    nominal.rotation = group.R();
    nominal.velocity = group.v();
    nominal.position = group.p();
    return nominal;
}

Vec6 innovationOf(const Nominal& nominal, const Mat3& R_m, const Vec3& p_m) {
    return sapphire::Eskf::gal3RightInnovation(
        nominal.rotation,
        nominal.velocity,
        nominal.position,
        R_m,
        p_m);
}

Vec10 logResidual(
    const Nominal& nominal, const Mat3& R_m, const Vec3& p_m)
{
    const Gal3 chi_z(
        R_m,
        Gal3::IsometriesType{nominal.velocity, p_m},
        0.0);
    return Gal3::log(groupOf(nominal).inv() * chi_z);
}

void expectMatchesLog(
    const Nominal& nominal,
    const Mat3& R_m,
    const Vec3& p_m,
    const std::string& label)
{
    const Vec10 xi = logResidual(nominal, R_m, p_m);
    const Vec6 z = innovationOf(nominal, R_m, p_m);
    expect(
        xi.segment<3>(3).lpNorm<Eigen::Infinity>() < 1e-12
            && std::abs(xi(9)) < 1e-12,
        label + ": alpha and tau slots must be ~0");
    expect(
        maxAbs(z.head<3>() - xi.head<3>()) < 1e-12
            && maxAbs(z.tail<3>() - xi.segment<3>(6)) < 1e-12,
        label + ": residual must match Gal3::log slots (ω, a)");
}

sapphire::EskfConfig pureGal3Config() {
    sapphire::EskfConfig config;
    config.chart = sapphire::EskfChart::Gal3Right;
    config.inject_full_pose = false;
    config.inject_directional_pose = false;
    config.velocity_correction_gain = 0.0;
    config.bias_update_scale = 1.0;
    config.mahalanobis_threshold = -1.0;
    return config;
}

void testResidualsMatchGal3Log() {
    Nominal nominal;
    nominal.rotation = so3Exp(Vec3(0.4, -0.25, 0.7));
    nominal.velocity = Vec3(0.5, -1.2, 0.3);
    nominal.position = Vec3(1.5, -2.0, 0.4);

    expectMatchesLog(
        nominal, nominal.rotation, nominal.position, "identity innovation");

    const Vec3 translation(0.35, -0.2, 0.15);
    const Vec3 p_translated =
        nominal.position + nominal.rotation * translation;
    expectMatchesLog(
        nominal, nominal.rotation, p_translated, "pure translation");
    const Vec6 translated =
        innovationOf(nominal, nominal.rotation, p_translated);
    expect(
        translated.head<3>().norm() < 1e-12
            && maxAbs(translated.tail<3>() - translation) < 1e-12,
        "pure translation residual must be (0, a) in the body chart");

    const Vec3 omega =
        degrees(20.0) * Vec3(0.2, -0.5, 0.8).normalized();
    const Vec3 a(0.22, -0.14, 0.08);
    const Mat3 Jl = SO3::leftJacobian(omega);
    const Mat3 R_m = nominal.rotation * so3Exp(omega);
    const Vec3 p_m = nominal.position + nominal.rotation * (Jl * a);
    expectMatchesLog(nominal, R_m, p_m, "20 degree rotation");
    const Vec6 rotated = innovationOf(nominal, R_m, p_m);
    expect(
        maxAbs(rotated.head<3>() - omega) < 1e-12
            && maxAbs(rotated.tail<3>() - a) < 1e-12,
        "20 degree residual must equal (ω, a)");
}

void testUnitGainRetractsOntoMeasurement() {
    Nominal nominal;
    nominal.rotation = so3Exp(Vec3(-0.3, 0.55, 1.1));
    nominal.velocity = Vec3(-0.4, 0.8, -0.2);
    nominal.position = Vec3(3.0, -1.5, 0.7);
    const Vec3 omega =
        degrees(20.0) * Vec3(1.0, 0.3, -0.6).normalized();
    const Vec3 a(0.31, -0.22, 0.17);
    const Mat3 Jl = SO3::leftJacobian(omega);
    const Mat3 R_m = nominal.rotation * so3Exp(omega);
    const Vec3 p_m = nominal.position + nominal.rotation * (Jl * a);

    const Vec6 z = innovationOf(nominal, R_m, p_m);
    expect(
        maxAbs(z.head<3>() - omega) < 1e-12
            && maxAbs(z.tail<3>() - a) < 1e-12,
        "unit-gain residual must equal the finite (ω, a)");

    Mat3 rotation = nominal.rotation;
    Vec3 velocity = nominal.velocity;
    Vec3 position = nominal.position;
    Vec9 xi;
    xi << z.head<3>(), Vec3::Zero(), z.tail<3>();
    sapphire::Eskf::gal3RightRetract(rotation, velocity, position, xi);
    expect(
        maxAbs(rotation - R_m) < 1e-12,
        "unit-gain retraction must recover R_m");
    expect(
        maxAbs(position - p_m) < 1e-12,
        "unit-gain retraction must recover p_m");
    expect(
        maxAbs(velocity - nominal.velocity) < 1e-12,
        "unit-gain retraction with α = 0 must keep velocity");
}

Mat9 finiteDifferenceNavigation(
    const Nominal& nominal,
    const Vec3& omega,
    const Vec3& accel,
    double dt)
{
    const Gal3 chi = groupOf(nominal);
    Vec10 input;
    input << omega, accel, Vec3::Zero(), 1.0;
    const Vec10 u_hat = input * dt;
    const Gal3 upsilon = Gal3::exp(u_hat);
    const Gal3 chi_plus =
        sapphire::detail::recoverWorldState(upsilon, chi, kGravity);

    auto navError = [](const Gal3& base, const Gal3& perturbed) -> Vec9 {
        return Gal3::log(base.inv() * perturbed).head<9>();
    };

    Mat9 numeric = Mat9::Zero();
    constexpr double epsilon = 1e-6;
    for (int column = 0; column < 9; ++column) {
        Vec10 direction = Vec10::Zero();
        direction(column) = 1.0;
        const auto evaluate = [&](double sign) {
            const Gal3 perturbed =
                chi * Gal3::exp(sign * epsilon * direction);
            const Gal3 perturbed_plus =
                sapphire::detail::recoverWorldState(
                    upsilon, perturbed, kGravity);
            return navError(chi_plus, perturbed_plus);
        };
        numeric.col(column) =
            (evaluate(1.0) - evaluate(-1.0)) / (2.0 * epsilon);
    }
    return numeric;
}

Mat96 finiteDifferenceBias(
    const Nominal& nominal,
    const Vec3& omega,
    const Vec3& accel,
    double dt)
{
    const Gal3 chi = groupOf(nominal);
    Vec10 input;
    input << omega, accel, Vec3::Zero(), 1.0;
    const Gal3 chi_plus = sapphire::detail::recoverWorldState(
        Gal3::exp(input * dt), chi, kGravity);

    auto navError = [](const Gal3& base, const Gal3& perturbed) -> Vec9 {
        return Gal3::log(base.inv() * perturbed).head<9>();
    };

    Mat96 numeric = Mat96::Zero();
    constexpr double epsilon = 1e-6;
    for (int column = 0; column < 6; ++column) {
        const auto evaluate = [&](double sign) {
            Vec3 gyro = omega;
            Vec3 specific = accel;
            if (column < 3) {
                specific(column) -= sign * epsilon;
            } else {
                gyro(column - 3) -= sign * epsilon;
            }
            Vec10 perturbed_input;
            perturbed_input << gyro, specific, Vec3::Zero(), 1.0;
            const Gal3 perturbed_plus =
                sapphire::detail::recoverWorldState(
                    Gal3::exp(perturbed_input * dt), chi, kGravity);
            return navError(chi_plus, perturbed_plus);
        };
        numeric.col(column) =
            (evaluate(1.0) - evaluate(-1.0)) / (2.0 * epsilon);
    }
    return numeric;
}

void testTransitionAgainstCentralDifference() {
    const Vec3 omega(0.35, -0.55, 0.8);
    const Vec3 accel(0.7, -0.25, 9.4);
    Nominal nominal;
    nominal.rotation = so3Exp(Vec3(0.45, -0.3, 0.9));
    nominal.velocity = Vec3(0.4, -0.8, 0.2);
    nominal.position = Vec3(2.0, -1.0, 0.5);

    double error_1ms = 0.0;
    for (const double dt : {0.001, 0.005, 0.02}) {
        Vec10 input;
        input << omega, accel, Vec3::Zero(), 1.0;
        const Vec10 u_hat = input * dt;
        const Gal3 upsilon = Gal3::exp(u_hat);
        const auto adjoint = upsilon.invAdjoint();
        const auto Jr = Gal3::rightJacobian(u_hat);

        Vec10 probe = Vec10::Zero();
        probe.head<9>() = Vec9::Ones();
        expect(
            std::abs((adjoint * probe)(9)) < 1e-12,
            "Adjoint time row must annihilate a zero time component");

        const Mat15 Phi =
            sapphire::Eskf::gal3RightTransition(omega, accel, dt);
        expect(
            maxAbs(Phi.block<9, 9>(0, 0) - adjoint.block<9, 9>(0, 0))
                < 1e-12,
            "navigation Φ must be invAdjoint without τ");
        expect(
            maxAbs(
                Phi.block<9, 3>(0, 9)
                + dt * Jr.block<9, 3>(0, 3))
                < 1e-12,
            "accel-bias columns must be -dt * rightJacobian");
        expect(
            maxAbs(
                Phi.block<9, 3>(0, 12)
                + dt * Jr.block<9, 3>(0, 0))
                < 1e-12,
            "gyro-bias columns must be -dt * rightJacobian");
        expect(
            maxAbs(Phi.block<6, 9>(9, 0)) < 1e-15
                && maxAbs(Phi.block<3, 3>(9, 9) - Mat3::Identity()) < 1e-15
                && maxAbs(Phi.block<3, 3>(12, 12) - Mat3::Identity())
                    < 1e-15
                && maxAbs(Phi.block<3, 3>(9, 12)) < 1e-15
                && maxAbs(Phi.block<3, 3>(12, 9)) < 1e-15,
            "bias rows must be [0 I] with no navigation feedback");

        const Mat9 nav_numeric =
            finiteDifferenceNavigation(nominal, omega, accel, dt);
        const Mat96 bias_numeric =
            finiteDifferenceBias(nominal, omega, accel, dt);
        const double nav_error = maxAbs(
            nav_numeric - adjoint.block<9, 9>(0, 0));
        const double bias_error = maxAbs(
            bias_numeric - Phi.block<9, 6>(0, 9));
        if (dt == 0.001) {
            error_1ms = std::max(nav_error, bias_error);
        }
        std::ostringstream message;
        message << "Φ central difference at dt=" << dt
                << " nav_error=" << nav_error
                << " bias_error=" << bias_error;
        expect(nav_error < 1e-7 && bias_error < 1e-7, message.str());
        if (dt == 0.02) {
            expect(
                std::max(nav_error, bias_error)
                    < 20.0 * error_1ms + 1e-8,
                "Φ error must not grow like dt^2");
        }
    }
}

void testYaw90RejectsWorldFrameBiasColumn() {
    const Vec3 omega(0.15, -0.35, 0.2);
    const Vec3 accel(1.6, -0.9, 8.7);
    const double dt = 0.02;
    Nominal nominal;
    nominal.rotation = so3Exp(Vec3(0.0, 0.0, degrees(90.0)));
    nominal.velocity = Vec3(0.2, -0.3, 0.1);
    nominal.position = Vec3(4.0, -2.0, 1.0);

    const Mat15 Phi =
        sapphire::Eskf::gal3RightTransition(omega, accel, dt);
    const Mat96 bias_numeric =
        finiteDifferenceBias(nominal, omega, accel, dt);
    const double bias_error =
        maxAbs(bias_numeric - Phi.block<9, 6>(0, 9));
    expect(
        bias_error < 1e-7,
        "yaw=90° bias columns must match the body-frame right Jacobian");

    const Mat3 world_frame = -dt * nominal.rotation;
    const double wrong =
        maxAbs(Phi.block<3, 3>(3, 9) - world_frame);
    expect(
        wrong > 0.5 * dt,
        "yaw=90° accel-bias column must not be -R̄ dt");
}

void testPredictUsesOneGal3Step() {
    sapphire::EskfConfig config = pureGal3Config();
    sapphire::ImuNoiseConfig noise;
    noise.accel_noise_density = 0.0;
    noise.gyro_noise_density = 0.0;
    noise.accel_random_walk = 0.0;
    noise.gyro_random_walk = 0.0;
    noise.accel_bias_rw_sigma = 0.0;
    noise.gyro_bias_rw_sigma = 0.0;
    sapphire::Eskf eskf(config, noise, kGravity);

    sapphire::NavigationState state;
    state.stamp = 0.0;
    state.valid = true;
    state.T_world_imu.linear() =
        so3Exp(Vec3(0.0, 0.0, degrees(90.0)));
    state.v_world = Vec3(0.2, -0.1, 0.05);
    state.T_world_imu.translation() = Vec3(1.0, 2.0, -0.5);
    eskf.setBaseline(
        state, Vec3::Zero(), Vec3::Zero(), Mat15::Identity());

    const Vec3 omega(0.2, -0.4, 0.15);
    const Vec3 accel(1.2, -0.7, 9.5);
    const double dt = 0.05;
    sapphire::ImuData imu;
    imu.stamp = dt;
    imu.gyro = omega;
    imu.accel = accel;
    eskf.predict(imu);

    const Mat15 Phi =
        sapphire::Eskf::gal3RightTransition(omega, accel, dt);
    const Mat15 expected = Phi * Phi.transpose();
    expect(
        maxAbs(eskf.tipCovariance() - 0.5 * (expected + expected.transpose()))
            < 1e-12,
        "a 50 ms sample must apply one gal3_right Φ, with no R̄ in P");

    sapphire::ImuNoiseConfig scaled;
    scaled.gyro_noise_density = 0.02;
    scaled.accel_noise_density = 0.03;
    scaled.gyro_random_walk = 0.004;
    scaled.accel_random_walk = 0.005;
    scaled.gyro_bias_rw_sigma = 0.004;
    scaled.accel_bias_rw_sigma = 0.005;
    sapphire::Eskf filter(pureGal3Config(), scaled, kGravity);
    filter.setBaseline(state, Vec3::Zero(), Vec3::Zero(), Mat15::Zero());
    filter.predict(imu);
    const Mat15 Q = sapphire::Eskf::gal3RightProcessNoise(
        omega,
        accel,
        dt,
        scaled.gyro_noise_density * scaled.gyro_noise_density,
        scaled.accel_noise_density * scaled.accel_noise_density,
        scaled.accel_bias_rw_sigma * scaled.accel_bias_rw_sigma,
        scaled.gyro_bias_rw_sigma * scaled.gyro_bias_rw_sigma);
    expect(
        maxAbs(filter.tipCovariance() - 0.5 * (Q + Q.transpose())) < 1e-12,
        "process noise must be the right-Jacobian Q, not Φ G Qc Gᵀ Φᵀ dt");
}

void testResetJacobian() {
    const Vec3 omega =
        0.3 * Vec3(1.0, -0.4, 0.25).normalized();
    const Vec3 alpha(0.04, -0.02, 0.03);
    const Vec3 a(0.28, -0.41, 0.19);
    Vec9 xi_nav;
    xi_nav << omega, alpha, a;
    const Mat15 reset = sapphire::Eskf::gal3RightResetJacobian(xi_nav);

    Vec10 xi = Vec10::Zero();
    xi.head<9>() = xi_nav;
    const Mat9 analytic = Gal3::rightJacobian(xi).block<9, 9>(0, 0);
    expect(
        maxAbs(reset.block<9, 9>(0, 0) - analytic) < 1e-12,
        "reset block must be Gal3::rightJacobian");
    expect(
        maxAbs(reset.block<6, 6>(9, 9) - Mat6::Identity()) < 1e-15
            && maxAbs(reset.block<6, 9>(9, 0)) < 1e-15,
        "bias reset rows must be [0 I]");
    expect(
        reset.block<3, 3>(6, 0).norm() > 1e-3,
        "reset coupling from ω into a must be visible at 0.3 rad");

    Mat9 numeric = Mat9::Zero();
    constexpr double epsilon = 1e-6;
    for (int column = 0; column < 9; ++column) {
        Vec10 direction = Vec10::Zero();
        direction(column) = 1.0;
        const auto evaluate = [&](double sign) -> Vec9 {
            const Vec10 perturbed = xi + sign * epsilon * direction;
            return Gal3::log(Gal3::exp(-xi) * Gal3::exp(perturbed)).head<9>();
        };
        numeric.col(column) =
            (evaluate(1.0) - evaluate(-1.0)) / (2.0 * epsilon);
    }
    const double error = maxAbs(numeric - reset.block<9, 9>(0, 0));
    std::ostringstream message;
    message << "reset central difference error " << error;
    expect(error < 1e-6, message.str());

    Mat9 so3_only = Mat9::Identity();
    so3_only.block<3, 3>(0, 0) = SO3::invRightJacobian(-omega);
    expect(
        maxAbs(so3_only - numeric) > 1e-3,
        "a 3x3 SO3 reset must not pass the 0.3 rad check");
}

void testLegacyGainsAreNotApplied() {
    sapphire::EskfConfig config = pureGal3Config();
    config.inject_full_pose = true;
    config.inject_directional_pose = true;
    config.velocity_correction_gain = 8.0;
    config.bias_update_scale = 0.0;
    config.use_hessian = true;
    config.sigma_rotation = 0.05;
    config.sigma_translation = 0.05;
    config.icp_covariance_scale = 1.0;
    config.hessian_max_condition = 100.0;
    config.hessian_degenerate_sigma = 2.0;
    sapphire::Eskf eskf(config, sapphire::ImuNoiseConfig{}, kGravity);

    sapphire::NavigationState state;
    state.stamp = 0.0;
    state.valid = true;
    state.T_world_imu.linear() =
        so3Exp(Vec3(0.2, -0.4, degrees(90.0)));
    state.T_world_imu.translation() = Vec3(1.0, -2.0, 0.4);
    state.v_world = Vec3(0.3, -0.2, 0.1);

    Mat15 P = Mat15::Zero();
    P.block<3, 3>(0, 0) = Mat3::Identity() * 0.04;
    P.block<3, 3>(3, 3) = Mat3::Identity() * 0.09;
    P.block<3, 3>(6, 6) = Mat3::Identity() * 0.04;
    P.block<3, 3>(9, 9) = Mat3::Identity() * 0.01;
    P.block<3, 3>(12, 12) = Mat3::Identity() * 0.0025;
    P.block<3, 3>(3, 6) = Mat3::Identity() * 0.02;
    P.block<3, 3>(6, 3) = Mat3::Identity() * 0.02;
    P.block<3, 3>(6, 9) = Mat3::Identity() * 0.004;
    P.block<3, 3>(9, 6) = Mat3::Identity() * 0.004;
    const Vec3 accel_bias(0.01, -0.02, 0.03);
    const Vec3 gyro_bias(0.001, 0.002, -0.001);
    eskf.setBaseline(state, accel_bias, gyro_bias, P);

    const Vec3 omega =
        degrees(20.0) * Vec3(0.3, -0.2, 0.9).normalized();
    const Vec3 a(0.12, -0.08, 0.05);
    const Mat3 Jl = SO3::leftJacobian(omega);
    sapphire::Isometry3d measured = sapphire::Isometry3d::Identity();
    measured.linear() = state.T_world_imu.rotation() * so3Exp(omega);
    measured.translation() =
        state.T_world_imu.translation()
        + state.T_world_imu.rotation() * (Jl * a);

    Mat6 information = Mat6::Identity() * 1e6;
    information(3, 3) = 1.0;
    const auto update = eskf.correctAt(
        0.1, state, measured, true, {}, information);
    expect(update.accepted, "gal3_right update must be accepted");

    const Vec6 z = innovationOf(
        Nominal{
            state.T_world_imu.rotation(),
            state.v_world,
            state.T_world_imu.translation()},
        measured.rotation(),
        measured.translation());
    Eigen::Matrix<double, 6, 15> H =
        Eigen::Matrix<double, 6, 15>::Zero();
    H.block<3, 3>(0, 0) = Mat3::Identity();
    H.block<3, 3>(3, 6) = Mat3::Identity();
    const Mat6 R = eskf.measurementCovariance(
        information, state.T_world_imu, measured);
    const Mat6 S = H * P * H.transpose() + R;
    Eigen::LDLT<Mat6> ldlt(S);
    expect(ldlt.info() == Eigen::Success, "synthetic S must factor");
    const Eigen::Matrix<double, 15, 6> K =
        P * H.transpose() * ldlt.solve(Mat6::Identity());
    const Vec15 dx = K * z;

    Mat3 rotation = state.T_world_imu.rotation();
    Vec3 velocity = state.v_world;
    Vec3 position = state.T_world_imu.translation();
    sapphire::Eskf::gal3RightRetract(rotation, velocity, position, dx.head<9>());
    const Vec3 expected_ba = accel_bias + dx.segment<3>(9);
    const Vec3 expected_bg = gyro_bias + dx.segment<3>(12);

    expect(
        maxAbs(update.state.T_world_imu.rotation() - rotation) < 1e-9
            && maxAbs(update.state.T_world_imu.translation() - position)
                < 1e-9
            && maxAbs(update.state.v_world - velocity) < 1e-9
            && maxAbs(update.accel_bias - expected_ba) < 1e-9
            && maxAbs(update.gyro_bias - expected_bg) < 1e-9,
        "gal3_right update must equal the pure Kalman retraction");
    expect(
        maxAbs(update.pose_gain_diagonal.head<3>() - K.block<3, 3>(0, 0).diagonal())
            < 1e-9
            && maxAbs(
                update.pose_gain_diagonal.tail<3>()
                - K.block<3, 3>(6, 3).diagonal())
                < 1e-9,
        "reported pose gain must be the Kalman diagonal");

    const Vec3 observer_velocity =
        state.v_world
        + 0.1 * 8.0
            * (measured.translation() - state.T_world_imu.translation());
    expect(
        (update.state.v_world - observer_velocity).norm() > 1e-3,
        "velocity_correction_gain must not replace the velocity rows");
    expect(
        (update.state.T_world_imu.translation() - measured.translation()).norm()
            > 1e-4
            || maxAbs(update.state.T_world_imu.rotation() - measured.rotation())
                > 1e-4,
        "inject_full_pose must not snap the pose to the measurement");
    expect(
        update.accel_bias.norm() > accel_bias.norm() * 0.5
            && (update.accel_bias - accel_bias).norm() > 1e-6,
        "bias_update_scale=0 must not zero the bias update");
}

void testMeasurementChart() {
    sapphire::Isometry3d prior = sapphire::Isometry3d::Identity();
    prior.linear() = so3Exp(Vec3(0.3, -0.2, 0.9));
    prior.translation() = Vec3(2.0, -1.0, 0.5);
    const Mat6 J_old =
        sapphire::Eskf::registrationToInnovationJacobian(prior, prior);
    const Mat6 J_z =
        sapphire::Eskf::gal3RightRegistrationJacobian(prior, prior);
    Mat6 body = Mat6::Zero();
    body.block<3, 3>(0, 0) = Mat3::Identity();
    body.block<3, 3>(3, 3) = prior.rotation().transpose();
    expect(
        maxAbs(J_z - body * J_old) < 1e-7,
        "zero innovation J_z must be blkdiag(I, R̄ᵀ) times the old Jacobian");

    const Vec3 omega =
        degrees(20.0) * Vec3(0.4, 0.2, -0.7).normalized();
    const Vec3 a(0.3, -0.25, 0.12);
    sapphire::Isometry3d measured = sapphire::Isometry3d::Identity();
    measured.linear() = prior.rotation() * so3Exp(omega);
    measured.translation() =
        prior.translation()
        + prior.rotation() * (SO3::leftJacobian(omega) * a);

    Mat6 numeric = Mat6::Zero();
    constexpr double epsilon = 1e-6;
    const sapphire::Isometry3d correction =
        measured * prior.inverse();
    for (int column = 0; column < 6; ++column) {
        Vec6 delta = Vec6::Zero();
        delta(column) = epsilon;
        const auto plus_pose = correction * rightSe3Exp(delta) * prior;
        delta(column) = -epsilon;
        const auto minus_pose = correction * rightSe3Exp(delta) * prior;
        const Vec6 plus = sapphire::Eskf::gal3RightInnovation(
            prior.rotation(),
            Vec3::Zero(),
            prior.translation(),
            plus_pose.rotation(),
            plus_pose.translation());
        const Vec6 minus = sapphire::Eskf::gal3RightInnovation(
            prior.rotation(),
            Vec3::Zero(),
            prior.translation(),
            minus_pose.rotation(),
            minus_pose.translation());
        numeric.col(column) = (plus - minus) / (2.0 * epsilon);
    }
    const Mat6 J_finite =
        sapphire::Eskf::gal3RightRegistrationJacobian(prior, measured);
    expect(
        maxAbs(J_finite - numeric) < 1e-7,
        "finite-innovation J_z must match an independent central difference");

    sapphire::EskfConfig config = pureGal3Config();
    config.use_hessian = false;
    config.sigma_rotation = 0.2;
    config.sigma_translation = 1.0;
    sapphire::Eskf eskf(config, sapphire::ImuNoiseConfig{}, kGravity);
    const Mat6 R = eskf.measurementCovariance(
        std::nullopt, prior, measured);
    const Mat6 J_old_finite =
        sapphire::Eskf::registrationToInnovationJacobian(prior, measured);
    const Mat6 D = J_finite * J_old_finite.inverse();
    Mat6 floor = Mat6::Zero();
    floor.block<3, 3>(0, 0) = Mat3::Identity() * 0.04;
    floor.block<3, 3>(3, 3) = Mat3::Identity();
    Mat6 expected = D * floor * D.transpose();
    expected = 0.5 * (expected + expected.transpose());
    expect(
        maxAbs(R - expected) < 1e-8,
        "measurement floor must be transported by the full D");
    Mat6 diagonal = floor;
    expect(
        maxAbs(R - diagonal) > 1e-3,
        "finite innovation must not keep a diagonal σ_t² floor in z");
}

std::filesystem::path writeConfig(const std::string& contents) {
    const auto path =
        std::filesystem::temp_directory_path()
        / "sapphire_gal3_eskf_chart.toml";
    std::ofstream stream(path);
    if (!stream) {
        fail("unable to create temporary config");
    }
    stream << contents;
    return path;
}

std::string minimalConfig(const std::string& chart_line) {
    return
        "[imu]\n"
        "gravity = 9.80665\n"
        "[imu.noise]\n"
        "accel_noise_density = 0.000196\n"
        "gyro_noise_density = 0.000152\n"
        "accel_random_walk = 0.000165\n"
        "gyro_random_walk = 0.0000655\n"
        "[odometry.eskf]\n"
        + chart_line +
        "[extrinsics.T_imu_lidar]\n"
        "translation = [0.0, 0.0, 0.0]\n"
        "rotation_xyzw = [0.0, 0.0, 0.0, 1.0]\n";
}

sapphire::ImuNoiseConfig handheldNoise() {
    sapphire::ImuNoiseConfig noise;
    noise.accel_noise_density = 0.001125;
    noise.gyro_noise_density = 0.0001884;
    noise.accel_random_walk = 0.000243;
    noise.gyro_random_walk = 0.00000452;
    noise.accel_bias_rw_sigma = 0.000243;
    noise.gyro_bias_rw_sigma = 0.00000452;
    return noise;
}

sapphire::EskfUpdate replayStaticScans(
    sapphire::Eskf& eskf,
    int scans)
{
    sapphire::NavigationState state;
    state.stamp = 0.0;
    state.valid = true;
    eskf.initialize(state, Vec3::Zero(), Vec3::Zero());

    sapphire::EskfUpdate update;
    constexpr double scan_dt = 0.1;
    constexpr double imu_dt = 0.005;
    const int imu_steps = static_cast<int>(std::lround(scan_dt / imu_dt));
    for (int scan = 0; scan < scans; ++scan) {
        const double stamp = (scan + 1) * scan_dt;
        sapphire::ImuBuffer buffer;
        for (int step = 1; step <= imu_steps; ++step) {
            sapphire::ImuData imu;
            imu.stamp = state.stamp + step * imu_dt;
            imu.accel = Vec3::Zero();
            imu.gyro = Vec3::Zero();
            buffer.push_back(imu);
        }
        update = eskf.correctAt(
            stamp,
            state,
            state.T_world_imu,
            true,
            buffer,
            std::nullopt);
        expect(update.accepted, "static gal3_right update must be accepted");
        eskf.setBaseline(
            update.state,
            update.accel_bias,
            update.gyro_bias,
            eskf.tipCovariance());
        state = update.state;
    }
    return update;
}

void testModelNoiseHoldsTheInjectionFixedPoint() {
    sapphire::EskfConfig collapsing = pureGal3Config();
    collapsing.sigma_rotation = 0.02;
    collapsing.sigma_translation = 0.05;
    collapsing.use_hessian = false;
    collapsing.init_sigma_velocity = 0.5;
    collapsing.init_sigma_position = 0.5;
    sapphire::Eskf collapsed(
        collapsing, handheldNoise(), kGravity);
    const auto collapsed_update = replayStaticScans(collapsed, 50);
    expect(
        collapsed_update.pose_gain_diagonal.tail<3>().maxCoeff() < 0.2,
        "datasheet Q must collapse the position gain below 0.2");

    sapphire::EskfConfig operating = collapsing;
    operating.specific_force_model_density =
        sapphire::kGal3SpecificForceModelDensity;
    operating.position_model_diffusion =
        sapphire::kGal3PositionModelDiffusion;
    sapphire::Eskf filter(operating, handheldNoise(), kGravity);
    const auto steady = replayStaticScans(filter, 50);
    const Vec3 position_gain = steady.pose_gain_diagonal.tail<3>();
    expect(
        position_gain.minCoeff() > 0.75
            && position_gain.maxCoeff() < 0.95,
        "model noise must hold position gain near 0.85");

    sapphire::NavigationState prior = steady.state;
    sapphire::Isometry3d measured = prior.T_world_imu;
    measured.translation().x() += 0.1;
    sapphire::ImuBuffer interval;
    constexpr double imu_dt = 0.005;
    for (int step = 1; step <= 20; ++step) {
        sapphire::ImuData imu;
        imu.stamp = prior.stamp + step * imu_dt;
        imu.accel = Vec3::Zero();
        imu.gyro = Vec3::Zero();
        interval.push_back(imu);
    }
    const auto kicked = filter.correctAt(
        prior.stamp + 0.1,
        prior,
        measured,
        true,
        interval,
        std::nullopt);
    const Vec3 velocity_correction =
        kicked.state.v_world - prior.v_world;
    expect(
        velocity_correction.x() > 0.06
            && velocity_correction.x() < 0.11
            && std::abs(velocity_correction.y()) < 1e-6
            && std::abs(velocity_correction.z()) < 1e-6,
        "a 0.1 m innovation must correct velocity by about 0.08 m/s");
    expect(
        (kicked.state.T_world_imu.translation() - measured.translation()).norm()
            < 0.03,
        "position gain near 0.85 must take most of a 0.1 m innovation");

    sapphire::EskfConfig position_only = collapsing;
    position_only.position_model_diffusion =
        sapphire::kGal3PositionModelDiffusion;
    sapphire::Eskf position_filter(
        position_only, handheldNoise(), kGravity);
    const auto position_steady = replayStaticScans(position_filter, 50);
    sapphire::NavigationState position_prior = position_steady.state;
    const auto position_kicked = position_filter.correctAt(
        position_prior.stamp + 0.1,
        position_prior,
        measured,
        true,
        interval,
        std::nullopt);
    const double position_only_speed =
        (position_kicked.state.v_world - position_prior.v_world).norm();
    expect(
        velocity_correction.norm() > 1.5 * position_only_speed,
        "position diffusion alone must not be asked to supply the velocity gain");
}

void testDegenerateAxisKeepsASmallGain() {
    sapphire::EskfConfig config = pureGal3Config();
    config.use_hessian = true;
    config.sigma_rotation = 0.02;
    config.sigma_translation = 0.05;
    config.icp_covariance_scale = 1.0;
    config.hessian_max_condition = 1.0e4;
    config.hessian_degenerate_sigma = 1.0;
    config.specific_force_model_density =
        sapphire::kGal3SpecificForceModelDensity;
    config.position_model_diffusion =
        sapphire::kGal3PositionModelDiffusion;
    sapphire::Eskf eskf(config, handheldNoise(), kGravity);

    sapphire::NavigationState state;
    state.stamp = 0.0;
    state.valid = true;
    Mat15 P = Mat15::Zero();
    P.block<3, 3>(0, 0) = Mat3::Identity() * 0.01;
    P.block<3, 3>(3, 3) = Mat3::Identity() * 0.01;
    P.block<3, 3>(6, 6) = Mat3::Identity() * 0.01;
    eskf.setBaseline(state, Vec3::Zero(), Vec3::Zero(), P);

    Mat6 information = Mat6::Identity() * 1.0e6;
    information(5, 5) = 1.0;
    const auto update = eskf.correctAt(
        0.0, state, state.T_world_imu, true, {}, information);
    expect(update.accepted, "degenerate-axis update must be accepted");
    const Vec3 gain = update.pose_gain_diagonal.tail<3>();
    expect(
        gain.x() > 0.6 && gain.y() > 0.6 && gain.z() < 0.05,
        "a weak translation axis must keep a much smaller Kalman gain");
}

void testChartConfig() {
    const auto absent = sapphire::loadConfig(
        writeConfig(minimalConfig("")));
    expect(
        absent.odometry.eskf.chart == "euclidean",
        "omitted chart must stay euclidean");

    const auto selected = sapphire::loadConfig(
        writeConfig(minimalConfig("chart = \"gal3_right\"\n")));
    expect(
        selected.odometry.eskf.chart == "gal3_right",
        "toml chart must parse gal3_right");

    bool rejected = false;
    try {
        sapphire::loadConfig(
            writeConfig(minimalConfig("chart = \"left_invariant\"\n")));
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    expect(rejected, "unknown chart must be rejected");

    const auto handheld = sapphire::loadConfig(
        std::filesystem::path(SAPPHIRE_SOURCE_DIR)
        / "cfg" / "sapphire_hilti_2022_handheld.toml");
    expect(handheld.odometry.fusion == "eskf", "handheld fusion stays eskf");
    expect(
        handheld.odometry.eskf.chart == "euclidean",
        "handheld chart must remain the euclidean default");
    expect(
        !handheld.odometry.eskf.inject_full_pose
            && handheld.odometry.eskf.inject_directional_pose
            &&         handheld.odometry.eskf.velocity_correction_gain == 8.0
            && handheld.odometry.eskf.bias_update_scale == 0.0,
        "handheld mixed gains must stay unchanged");
    expect(
        handheld.odometry.eskf.specific_force_model_density == 0.0
            && handheld.odometry.eskf.position_model_diffusion == 0.0,
        "handheld must not take the gal3_right model noise before acceptance");

    const auto tuned = sapphire::loadConfig(writeConfig(minimalConfig(
        "chart = \"gal3_right\"\n"
        "specific_force_model_density = 0.327\n"
        "position_model_diffusion = 0.1053\n")));
    expect(
        std::abs(
            tuned.odometry.eskf.specific_force_model_density - 0.327)
                < 1e-12
            && std::abs(
                tuned.odometry.eskf.position_model_diffusion - 0.1053)
                < 1e-12,
        "toml must accept the gal3_right model-noise terms");
}

}  // namespace

int main() {
    try {
        testResidualsMatchGal3Log();
        testUnitGainRetractsOntoMeasurement();
        testTransitionAgainstCentralDifference();
        testYaw90RejectsWorldFrameBiasColumn();
        testPredictUsesOneGal3Step();
        testResetJacobian();
        testLegacyGainsAreNotApplied();
        testMeasurementChart();
        testModelNoiseHoldsTheInjectionFixedPoint();
        testDegenerateAxisKeepsASmallGain();
        testChartConfig();
        std::cout << "All gal3_right ESKF tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "gal3_right ESKF test failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
