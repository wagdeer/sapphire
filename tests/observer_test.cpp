#include <sapphire/odometry/observer.hpp>

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
        fail(message);
    }
}

sapphire::NavigationState makePrior(double stamp = 1.0) {
    sapphire::NavigationState state;
    state.stamp = stamp;
    state.valid = true;
    return state;
}

void testTranslationUpdatesPositionVelocityAndAccelBias() {
    const sapphire::NavigationState prior = makePrior();
    sapphire::Isometry3d measurement = sapphire::Isometry3d::Identity();
    measurement.translation().x() = 1.0;
    const sapphire::Config::Odometry::Observer config;

    const auto update = sapphire::applyGeometricObserver(
        prior,
        measurement,
        0.0,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        config,
        true);

    expectNear(update.state.T_world_imu.translation().x(), 4.5, 1e-12,
               "position gain must feed translation innovation into state");
    expectNear(update.state.v_world.x(), 11.25, 1e-12,
               "velocity gain must feed translation innovation into velocity");
    expectNear(update.accel_bias.x(), -2.25, 1e-12,
               "translation innovation must update accelerometer bias");
    expect(update.gyro_bias.isZero(1e-12),
           "pure translation must not update gyroscope bias");
}

void testRotationUpdatesOrientationAndGyroBias() {
    const sapphire::NavigationState prior = makePrior(0.1);
    sapphire::Isometry3d measurement = sapphire::Isometry3d::Identity();
    measurement.linear() =
        Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const sapphire::Config::Odometry::Observer config;

    const auto update = sapphire::applyGeometricObserver(
        prior,
        measurement,
        0.0,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        config,
        true);

    const Eigen::AngleAxisd corrected_rotation(
        update.state.T_world_imu.rotation());
    expect(corrected_rotation.angle() > 0.0,
           "orientation gain must rotate state toward GICP measurement");
    expect(update.gyro_bias.z() < 0.0,
           "positive yaw innovation must update gyroscope bias");
    expect(update.accel_bias.isZero(1e-12),
           "pure rotation must not update accelerometer bias");
}

void testRejectedAndNonPositiveDtUpdatesAreNoOps() {
    sapphire::NavigationState prior = makePrior();
    prior.v_world = Eigen::Vector3d(1.0, 2.0, 3.0);
    sapphire::Isometry3d measurement = sapphire::Isometry3d::Identity();
    measurement.translation().setOnes();
    const Eigen::Vector3d accel_bias(0.1, 0.2, 0.3);
    const Eigen::Vector3d gyro_bias(0.01, 0.02, 0.03);
    const sapphire::Config::Odometry::Observer config;

    const auto rejected = sapphire::applyGeometricObserver(
        prior, measurement, 0.0, accel_bias, gyro_bias, config, false);
    expect(rejected.state.T_world_imu.matrix().isApprox(
               prior.T_world_imu.matrix(), 1e-12),
           "rejected registration must not change observer state");
    expect(rejected.accel_bias.isApprox(accel_bias, 1e-12)
           && rejected.gyro_bias.isApprox(gyro_bias, 1e-12),
           "rejected registration must not change biases");

    const auto zero_dt = sapphire::applyGeometricObserver(
        prior, measurement, prior.stamp, accel_bias, gyro_bias, config, true);
    expect(zero_dt.state.T_world_imu.matrix().isApprox(
               prior.T_world_imu.matrix(), 1e-12),
           "non-positive observer interval must be a no-op");
}

void testBiasLimitsAreEnforced() {
    const sapphire::NavigationState prior = makePrior();
    sapphire::Isometry3d measurement = sapphire::Isometry3d::Identity();
    measurement.translation().setConstant(100.0);
    sapphire::Config::Odometry::Observer config;
    config.accel_bias_max = 0.2;
    config.gyro_bias_max = 0.1;

    const auto update = sapphire::applyGeometricObserver(
        prior,
        measurement,
        0.0,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        config,
        true);
    expect(update.accel_bias.cwiseAbs().maxCoeff() <= 0.2,
           "accelerometer bias must respect configured limit");
}

}  // namespace

int main() {
    testTranslationUpdatesPositionVelocityAndAccelBias();
    testRotationUpdatesOrientationAndGyroBias();
    testRejectedAndNonPositiveDtUpdatesAreNoOps();
    testBiasLimitsAreEnforced();
    std::cout << "All observer tests passed\n";
    return EXIT_SUCCESS;
}
