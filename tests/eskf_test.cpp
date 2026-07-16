#include <sapphire/odometry/eskf.hpp>

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
    std::cout << "All eskf tests passed\n";
    return EXIT_SUCCESS;
}
