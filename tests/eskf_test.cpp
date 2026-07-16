#include <sapphire/odometry/eskf.hpp>

#include <Eigen/Eigenvalues>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

[[noreturn]] void fail(const std::string& message) {
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

void expect(bool condition, const std::string& message) {
    if (!condition) {
        fail(message);
    }
}

void expectNear(
    double actual, double expected, double tolerance,
    const std::string& message)
{
    if (std::abs(actual - expected) > tolerance) {
        fail(message + " (actual=" + std::to_string(actual)
             + " expected=" + std::to_string(expected) + ")");
    }
}

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
        -v.y(), v.x(), 0.0;
    return m;
}

Eigen::Matrix3d expSo3(const Eigen::Vector3d& omega) {
    const double angle = omega.norm();
    if (angle < 1e-12) {
        return Eigen::Matrix3d::Identity() + skew(omega);
    }
    return Eigen::AngleAxisd(angle, omega / angle).toRotationMatrix();
}

Eigen::Vector3d logSo3(const Eigen::Matrix3d& rotation) {
    const Eigen::AngleAxisd angle_axis(rotation);
    return angle_axis.axis() * angle_axis.angle();
}

sapphire::Isometry3d expSe3(
    const Eigen::Matrix<double, 6, 1>& delta)
{
    sapphire::Isometry3d result = sapphire::Isometry3d::Identity();
    const Eigen::Vector3d omega = delta.head<3>();
    const Eigen::Vector3d translation = delta.tail<3>();
    const double theta = omega.norm();
    const Eigen::Matrix3d Omega = skew(omega);
    Eigen::Matrix3d V = Eigen::Matrix3d::Identity();
    if (theta < 1e-8) {
        V += 0.5 * Omega + (1.0 / 6.0) * Omega * Omega;
    } else {
        V += (1.0 - std::cos(theta)) / (theta * theta) * Omega
            + (theta - std::sin(theta)) / (theta * theta * theta)
                * Omega * Omega;
    }
    result.linear() = expSo3(omega);
    result.translation() = V * translation;
    return result;
}

Eigen::Matrix<double, 6, 1> poseInnovation(
    const sapphire::Isometry3d& prior,
    const sapphire::Isometry3d& measured)
{
    Eigen::Matrix<double, 6, 1> innovation;
    innovation.head<3>() =
        logSo3(prior.rotation().transpose() * measured.rotation());
    innovation.tail<3>() = measured.translation() - prior.translation();
    return innovation;
}

sapphire::NavigationState makeState(
    double stamp,
    const Eigen::Vector3d& p = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d& v = Eigen::Vector3d::Zero())
{
    sapphire::NavigationState state;
    state.stamp = stamp;
    state.T_world_imu = sapphire::Isometry3d::Identity();
    state.T_world_imu.translation() = p;
    state.v_world = v;
    state.valid = true;
    return state;
}

sapphire::ImuBuffer makeStationaryBuffer(
    double start, double end, double step, double gravity)
{
    sapphire::ImuBuffer buffer;
    for (double t = start; t <= end + 1e-12; t += step) {
        sapphire::ImuData imu;
        imu.stamp = t;
        imu.accel = Eigen::Vector3d(0.0, 0.0, gravity);
        imu.gyro = Eigen::Vector3d::Zero();
        buffer.push_back(imu);
    }
    return buffer;
}

sapphire::Eskf makeFilter() {
    sapphire::EskfConfig config;
    config.sigma_rotation = 0.02;
    config.sigma_translation = 0.05;
    config.use_hessian = false;
    config.mahalanobis_threshold = 12.59;
    sapphire::ImuNoiseConfig noise;
    // Non-zero bias RW so covariance can grow.
    noise.accel_bias_rw_sigma = noise.accel_random_walk;
    noise.gyro_bias_rw_sigma = noise.gyro_random_walk;
    const Eigen::Vector3d gravity(0.0, 0.0, -9.80665);
    return sapphire::Eskf(config, noise, gravity);
}

void testPredictKeepsStationaryAndGrowsCovariance() {
    sapphire::Eskf eskf = makeFilter();
    eskf.initialize(makeState(0.0), Eigen::Vector3d::Zero(),
                    Eigen::Vector3d::Zero());
    const auto P0 = eskf.tipCovariance();

    sapphire::ImuData imu;
    imu.stamp = 0.01;
    imu.accel = Eigen::Vector3d(0.0, 0.0, 9.80665);
    imu.gyro.setZero();
    eskf.predict(imu);

    expectNear(eskf.tipState().T_world_imu.translation().norm(), 0.0, 1e-6,
               "stationary IMU predict must keep position");
    expectNear(eskf.tipState().v_world.norm(), 0.0, 1e-6,
               "stationary IMU predict must keep velocity");
    expect(eskf.tipCovariance()(6, 6) > P0(6, 6),
           "position covariance must grow during predict");
}

void testTranslationMeasurementUpdatesPosition() {
    sapphire::EskfConfig config;
    config.inject_full_pose = true;
    config.use_hessian = false;
    sapphire::ImuNoiseConfig noise;
    noise.accel_bias_rw_sigma = noise.accel_random_walk;
    noise.gyro_bias_rw_sigma = noise.gyro_random_walk;
    sapphire::Eskf eskf(
        config, noise, Eigen::Vector3d(0.0, 0.0, -9.80665));
    eskf.initialize(makeState(0.0), Eigen::Vector3d::Zero(),
                    Eigen::Vector3d::Zero());

    auto buffer = makeStationaryBuffer(0.0, 0.10, 0.01, 9.80665);
    const sapphire::NavigationState prior = makeState(0.10);
    sapphire::Isometry3d measurement = sapphire::Isometry3d::Identity();
    measurement.translation().x() = 0.2;

    const auto update = eskf.correctAt(
        0.10, prior, measurement, true, buffer, std::nullopt);

    expect(update.accepted, "small translation measurement must be accepted");
    expectNear(
        update.state.T_world_imu.translation().x(), 0.2, 1e-12,
        "inject_full_pose must adopt the ICP translation");
}

void testMahalanobisRejectsHugeInnovation() {
    sapphire::EskfConfig tight;
    tight.sigma_translation = 0.01;
    tight.init_sigma_position = 0.01;
    tight.mahalanobis_threshold = 12.59;
    sapphire::ImuNoiseConfig noise;
    noise.accel_bias_rw_sigma = noise.accel_random_walk;
    noise.gyro_bias_rw_sigma = noise.gyro_random_walk;
    sapphire::Eskf eskf(
        tight, noise, Eigen::Vector3d(0.0, 0.0, -9.80665));
    eskf.initialize(makeState(0.0), Eigen::Vector3d::Zero(),
                    Eigen::Vector3d::Zero());

    auto buffer = makeStationaryBuffer(0.0, 0.10, 0.01, 9.80665);
    sapphire::Isometry3d huge = sapphire::Isometry3d::Identity();
    huge.translation().x() = 5.0;
    const auto update = eskf.correctAt(
        0.10, makeState(0.10), huge, true, buffer, std::nullopt);

    expect(!update.accepted,
           "huge innovation must be rejected by Mahalanobis gate");
    expectNear(update.state.T_world_imu.translation().x(), 0.0, 1e-6,
               "rejected update must keep the IMU prior position");
}

void testHardRejectSkipsInjection() {
    sapphire::Eskf eskf = makeFilter();
    eskf.initialize(makeState(0.0), Eigen::Vector3d::Zero(),
                    Eigen::Vector3d::Zero());
    auto buffer = makeStationaryBuffer(0.0, 0.10, 0.01, 9.80665);
    sapphire::Isometry3d measurement = sapphire::Isometry3d::Identity();
    measurement.translation().x() = 0.2;

    const auto update = eskf.correctAt(
        0.10, makeState(0.10), measurement, false, buffer, std::nullopt);

    expect(!update.accepted, "hard-rejected measurement must not inject");
    expectNear(update.state.T_world_imu.translation().norm(), 0.0, 1e-9,
               "hard reject must keep prior pose");
    expect(update.accel_bias.isZero(1e-12),
           "hard reject must not change accelerometer bias");
}

void testRotationMeasurementUpdatesOrientation() {
    sapphire::EskfConfig config;
    config.inject_full_pose = true;
    config.use_hessian = false;
    sapphire::ImuNoiseConfig noise;
    noise.accel_bias_rw_sigma = noise.accel_random_walk;
    noise.gyro_bias_rw_sigma = noise.gyro_random_walk;
    sapphire::Eskf eskf(
        config, noise, Eigen::Vector3d(0.0, 0.0, -9.80665));
    eskf.initialize(makeState(0.0), Eigen::Vector3d::Zero(),
                    Eigen::Vector3d::Zero());

    auto buffer = makeStationaryBuffer(0.0, 0.10, 0.01, 9.80665);
    sapphire::Isometry3d measurement = sapphire::Isometry3d::Identity();
    measurement.linear() =
        Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    const auto update = eskf.correctAt(
        0.10, makeState(0.10), measurement, true, buffer, std::nullopt);
    expect(update.accepted, "rotation measurement must be accepted");

    const Eigen::AngleAxisd corrected(update.state.T_world_imu.rotation());
    expectNear(corrected.angle(), 0.1, 1e-9,
               "inject_full_pose must adopt the ICP orientation");
}

void testCorrectReducesPositionCovariance() {
    sapphire::Eskf eskf = makeFilter();
    eskf.initialize(makeState(0.0), Eigen::Vector3d::Zero(),
                    Eigen::Vector3d::Zero());
    auto buffer = makeStationaryBuffer(0.0, 0.10, 0.01, 9.80665);
    for (const auto& imu : buffer) {
        eskf.predict(imu);
    }
    const double p_before = eskf.tipCovariance().block<3, 3>(6, 6).trace();

    sapphire::Isometry3d measurement = sapphire::Isometry3d::Identity();
    const auto update = eskf.correctAt(
        0.10, makeState(0.10), measurement, true, buffer, std::nullopt);
    expect(update.accepted, "zero innovation measurement must be accepted");
    expect(eskf.tipCovariance().block<3, 3>(6, 6).trace() < p_before,
           "successful correction must reduce position covariance");
}

void testFullPoseInjectionUsesMeasurementCovariance() {
    sapphire::EskfConfig config;
    config.inject_full_pose = true;
    config.sigma_translation = 0.05;
    config.use_hessian = false;
    sapphire::ImuNoiseConfig noise;
    sapphire::Eskf eskf(
        config, noise, Eigen::Vector3d(0.0, 0.0, -9.80665));
    eskf.initialize(makeState(0.0), Eigen::Vector3d::Zero(),
                    Eigen::Vector3d::Zero());

    auto buffer = makeStationaryBuffer(0.0, 0.10, 0.01, 9.80665);
    sapphire::Isometry3d measurement = sapphire::Isometry3d::Identity();
    measurement.translation().x() = 0.1;
    const auto update = eskf.correctAt(
        0.10, makeState(0.10), measurement, true, buffer, std::nullopt);

    expect(update.accepted, "full-pose correction must be accepted");
    expectNear(
        eskf.tipCovariance()(6, 6),
        config.sigma_translation * config.sigma_translation,
        1e-10,
        "full-pose position covariance must match measurement covariance");
}

void testCovariancePropagatesToInterpolatedReferenceStamp() {
    sapphire::Eskf eskf = makeFilter();
    eskf.initialize(makeState(0.0), Eigen::Vector3d::Zero(),
                    Eigen::Vector3d::Zero());
    const auto P0 = eskf.tipCovariance();

    sapphire::ImuBuffer buffer;
    sapphire::ImuData start;
    start.stamp = 0.0;
    start.accel = Eigen::Vector3d(0.0, 0.0, 9.80665);
    start.gyro.setZero();
    buffer.push_back(start);
    sapphire::ImuData end = start;
    end.stamp = 0.10;
    buffer.push_back(end);

    const auto update = eskf.correctAt(
        0.05,
        makeState(0.05),
        sapphire::Isometry3d::Identity(),
        false,
        buffer,
        std::nullopt);

    expect(!update.accepted, "hard rejection must remain rejected");
    expectNear(update.state.stamp, 0.05, 1e-12,
               "update state must use the interpolated reference stamp");
    expect(eskf.tipCovariance()(6, 6) > P0(6, 6),
           "position covariance must grow through the partial IMU interval");
}

void testPositionInnovationVelocityCorrection() {
    sapphire::EskfConfig config;
    config.inject_full_pose = false;
    config.velocity_correction_gain = 8.0;
    config.use_hessian = false;
    sapphire::ImuNoiseConfig noise;
    sapphire::Eskf eskf(
        config, noise, Eigen::Vector3d(0.0, 0.0, -9.80665));
    eskf.initialize(makeState(0.0), Eigen::Vector3d::Zero(),
                    Eigen::Vector3d::Zero());

    const auto buffer = makeStationaryBuffer(0.0, 0.10, 0.01, 9.80665);
    sapphire::Isometry3d measurement = sapphire::Isometry3d::Identity();
    measurement.translation().x() = 0.2;
    const auto update = eskf.correctAt(
        0.10, makeState(0.10), measurement, true, buffer, std::nullopt);

    expect(update.accepted, "velocity correction update must be accepted");
    expectNear(
        update.state.v_world.x(),
        0.10 * config.velocity_correction_gain * 0.2,
        1e-12,
        "position innovation must apply the configured velocity gain");
}

void testPositionInnovationUpdatesAccelBias() {
    sapphire::EskfConfig config;
    config.init_sigma_position = 0.5;
    config.init_sigma_velocity = 0.5;
    config.sigma_translation = 0.05;
    config.use_hessian = false;
    sapphire::ImuNoiseConfig noise;
    noise.accel_bias_sigma = 0.1;
    noise.accel_bias_rw_sigma = noise.accel_random_walk;
    noise.gyro_bias_rw_sigma = noise.gyro_random_walk;
    sapphire::Eskf eskf(
        config, noise, Eigen::Vector3d(0.0, 0.0, -9.80665));
    eskf.initialize(makeState(0.0), Eigen::Vector3d::Zero(),
                    Eigen::Vector3d::Zero());

    // Propagate long enough for position–bias cross-covariance to form.
    auto buffer = makeStationaryBuffer(0.0, 0.50, 0.01, 9.80665);
    for (const auto& imu : buffer) {
        eskf.predict(imu);
    }
    eskf.setBaseline(
        eskf.tipState(), eskf.accelBias(), eskf.gyroBias(),
        eskf.tipCovariance());

    sapphire::Isometry3d measurement = sapphire::Isometry3d::Identity();
    measurement.translation().x() = 0.2;
    const auto update = eskf.correctAt(
        0.50, makeState(0.50), measurement, true, buffer, std::nullopt);

    expect(update.accepted, "bias-coupling measurement must be accepted");
    expect(std::abs(update.accel_bias.x()) > 1e-4,
           "position innovation after IMU propagation must update accel bias");
}

void testRegistrationJacobianMatchesFiniteDifference() {
    sapphire::Isometry3d prior = sapphire::Isometry3d::Identity();
    prior.linear() =
        (Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()))
            .toRotationMatrix();
    prior.translation() = Eigen::Vector3d(4.0, -2.0, 1.5);

    sapphire::Isometry3d correction = sapphire::Isometry3d::Identity();
    correction.linear() =
        Eigen::AngleAxisd(0.08, Eigen::Vector3d(1.0, 2.0, -1.0).normalized())
            .toRotationMatrix();
    correction.translation() = Eigen::Vector3d(0.1, -0.03, 0.04);
    const sapphire::Isometry3d measured = correction * prior;

    const auto analytic =
        sapphire::Eskf::registrationToInnovationJacobian(prior, measured);
    sapphire::Eskf::Mat6 numeric;
    constexpr double epsilon = 1e-7;
    for (int column = 0; column < 6; ++column) {
        Eigen::Matrix<double, 6, 1> delta =
            Eigen::Matrix<double, 6, 1>::Zero();
        delta(column) = epsilon;
        const auto plus =
            poseInnovation(prior, correction * expSe3(delta) * prior);
        delta(column) = -epsilon;
        const auto minus =
            poseInnovation(prior, correction * expSe3(delta) * prior);
        numeric.col(column) = (plus - minus) / (2.0 * epsilon);
    }

    expect(
        analytic.isApprox(numeric, 2e-7),
        "registration-to-innovation Jacobian must match finite differences");
}

void testDegenerateHessianInflatesWeakDirection() {
    sapphire::EskfConfig config;
    config.use_hessian = true;
    config.sigma_rotation = 1e-4;
    config.sigma_translation = 1e-4;
    config.icp_covariance_scale = 1.0;
    config.hessian_max_condition = 100.0;
    config.hessian_degenerate_sigma = 2.0;
    config.hessian_max_sigma = 5.0;
    sapphire::Eskf eskf(
        config,
        sapphire::ImuNoiseConfig{},
        Eigen::Vector3d(0.0, 0.0, -9.80665));

    sapphire::Eskf::Mat6 information =
        sapphire::Eskf::Mat6::Identity() * 1e6;
    information(3, 3) = 1.0;
    const auto covariance = eskf.measurementCovariance(information);

    expect(
        covariance(3, 3) > 1e5 * covariance(4, 4),
        "corridor-like weak translation axis must receive larger covariance");
    Eigen::SelfAdjointEigenSolver<sapphire::Eskf::Mat6> solver(covariance);
    expect(
        solver.info() == Eigen::Success
            && solver.eigenvalues().minCoeff() > 0.0,
        "directional measurement covariance must remain positive definite");
}

void testMixedDegenerateDirectionPreservesEigenvector() {
    sapphire::EskfConfig config;
    config.use_hessian = true;
    config.sigma_rotation = 1e-4;
    config.sigma_translation = 1e-4;
    config.icp_covariance_scale = 1.0;
    config.hessian_max_condition = 100.0;
    config.hessian_degenerate_sigma = 1.5;
    sapphire::Eskf eskf(
        config,
        sapphire::ImuNoiseConfig{},
        Eigen::Vector3d(0.0, 0.0, -9.80665));

    sapphire::Eskf::Mat6 basis = sapphire::Eskf::Mat6::Identity();
    const double inv_sqrt_two = 1.0 / std::sqrt(2.0);
    basis.col(3).setZero();
    basis.col(4).setZero();
    basis(3, 3) = inv_sqrt_two;
    basis(4, 3) = inv_sqrt_two;
    basis(3, 4) = -inv_sqrt_two;
    basis(4, 4) = inv_sqrt_two;
    Eigen::Matrix<double, 6, 1> eigenvalues =
        Eigen::Matrix<double, 6, 1>::Constant(1e6);
    eigenvalues(3) = 1.0;
    const sapphire::Eskf::Mat6 information =
        basis * eigenvalues.asDiagonal() * basis.transpose();
    const auto covariance = eskf.measurementCovariance(information);

    Eigen::Matrix<double, 6, 1> weak =
        Eigen::Matrix<double, 6, 1>::Zero();
    weak(3) = inv_sqrt_two;
    weak(4) = inv_sqrt_two;
    Eigen::Matrix<double, 6, 1> strong =
        Eigen::Matrix<double, 6, 1>::Zero();
    strong(3) = -inv_sqrt_two;
    strong(4) = inv_sqrt_two;
    expect(
        weak.dot(covariance * weak)
            > 1e5 * strong.dot(covariance * strong),
        "mixed weak Hessian eigenvector must be preserved in covariance");
}

void testPlanarHessianInflatesInPlaneAndYawDirections() {
    sapphire::EskfConfig config;
    config.use_hessian = true;
    config.sigma_rotation = 1e-4;
    config.sigma_translation = 1e-4;
    config.icp_covariance_scale = 1.0;
    config.hessian_max_condition = 100.0;
    config.hessian_degenerate_sigma = 2.0;
    sapphire::Eskf eskf(
        config,
        sapphire::ImuNoiseConfig{},
        Eigen::Vector3d(0.0, 0.0, -9.80665));

    sapphire::Eskf::Mat6 planar_information =
        sapphire::Eskf::Mat6::Identity() * 1e6;
    planar_information(2, 2) = 1.0;  // yaw around plane normal
    planar_information(3, 3) = 1.0;  // in-plane X
    planar_information(4, 4) = 1.0;  // in-plane Y
    const auto covariance =
        eskf.measurementCovariance(planar_information);

    expect(
        covariance(2, 2) > 1e5 * covariance(0, 0)
            && covariance(3, 3) > 1e5 * covariance(5, 5)
            && covariance(4, 4) > 1e5 * covariance(5, 5),
        "single-plane weak yaw and in-plane translation must be inflated");
}

void testDirectionalPoseUpdateTrustsOnlyObservableAxis() {
    sapphire::EskfConfig config;
    config.inject_full_pose = false;
    config.inject_directional_pose = true;
    config.use_hessian = true;
    config.sigma_rotation = 1e-4;
    config.sigma_translation = 1e-4;
    config.icp_covariance_scale = 1.0;
    config.hessian_max_condition = 100.0;
    config.hessian_degenerate_sigma = 2.0;
    config.init_sigma_position = 0.5;
    config.velocity_correction_gain = 8.0;
    sapphire::Eskf eskf(
        config,
        sapphire::ImuNoiseConfig{},
        Eigen::Vector3d(0.0, 0.0, -9.80665));
    eskf.initialize(
        makeState(0.0), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    sapphire::Eskf::Mat6 information =
        sapphire::Eskf::Mat6::Identity() * 1e6;
    information(3, 3) = 1.0;
    sapphire::Isometry3d measurement = sapphire::Isometry3d::Identity();
    measurement.translation() = Eigen::Vector3d(0.2, 0.2, 0.0);
    const auto update = eskf.correctAt(
        0.1,
        makeState(0.1),
        measurement,
        true,
        makeStationaryBuffer(0.0, 0.1, 0.01, 9.80665),
        information);

    expect(update.accepted, "directional Hessian update must be accepted");
    expect(
        update.state.T_world_imu.translation().y() > 0.19,
        "well-observed translation axis must follow GICP");
    expect(
        update.state.T_world_imu.translation().x() < 0.02,
        "degenerate translation axis must remain close to the IMU prior");
    expect(
        update.pose_gain_diagonal(3) < update.pose_gain_diagonal(4),
        "weak direction must have a smaller Kalman gain");
    expect(
        update.state.v_world.x() < 0.02
            && update.state.v_world.y() > 0.1,
        "velocity bridge must be projected through directional observability");
}

void testBiasRandomWalkGrowsWhenBiasInjectionDisabled() {
    sapphire::EskfConfig config;
    config.bias_update_scale = 0.0;
    sapphire::ImuNoiseConfig noise;
    noise.accel_bias_rw_sigma = 0.01;
    noise.gyro_bias_rw_sigma = 0.01;
    sapphire::Eskf eskf(
        config, noise, Eigen::Vector3d(0.0, 0.0, -9.80665));
    eskf.initialize(
        makeState(0.0), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    const double before = eskf.tipCovariance()(9, 9);

    sapphire::ImuData imu;
    imu.stamp = 1.0;
    imu.accel = Eigen::Vector3d(0.0, 0.0, 9.80665);
    imu.gyro.setZero();
    eskf.predict(imu);

    expect(
        eskf.tipCovariance()(9, 9) > before + 1e-6,
        "bias covariance random walk must not depend on bias injection scale");
}

}  // namespace

int main() {
    testPredictKeepsStationaryAndGrowsCovariance();
    testTranslationMeasurementUpdatesPosition();
    testMahalanobisRejectsHugeInnovation();
    testHardRejectSkipsInjection();
    testRotationMeasurementUpdatesOrientation();
    testCorrectReducesPositionCovariance();
    testFullPoseInjectionUsesMeasurementCovariance();
    testCovariancePropagatesToInterpolatedReferenceStamp();
    testPositionInnovationVelocityCorrection();
    testPositionInnovationUpdatesAccelBias();
    testRegistrationJacobianMatchesFiniteDifference();
    testDegenerateHessianInflatesWeakDirection();
    testMixedDegenerateDirectionPreservesEigenvector();
    testPlanarHessianInflatesInPlaneAndYawDirections();
    testDirectionalPoseUpdateTrustsOnlyObservableAxis();
    testBiasRandomWalkGrowsWhenBiasInjectionDisabled();
    std::cout << "All eskf tests passed\n";
    return EXIT_SUCCESS;
}
