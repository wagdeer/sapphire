#include <sapphire/config.hpp>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
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

void expectNear(double actual, double expected, const std::string& message) {
    if (std::abs(actual - expected) > 1e-12) {
        fail(message);
    }
}

std::filesystem::path writeConfig(
    const std::string& filename, const std::string& contents)
{
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / filename;
    std::ofstream stream(path);
    if (!stream) {
        fail("unable to create temporary config");
    }
    stream << contents;
    return path;
}

std::string validConfig(double gravity, const std::string& quaternion) {
    return
        "[imu]\n"
        "gravity = " + std::to_string(gravity) + "\n"
        "[imu.noise]\n"
        "accel_noise_density = 0.000196\n"
        "gyro_noise_density = 0.000152\n"
        "accel_random_walk = 0.000165\n"
        "gyro_random_walk = 0.0000655\n"
        "[deskew]\n"
        "time_offset = false\n"
        "[extrinsics.T_imu_lidar]\n"
        "translation = [0.1, -0.2, 0.3]\n"
        "rotation_xyzw = " + quaternion + "\n";
}

void testProjectConfig() {
    const auto config = sapphire::loadConfig(
        std::filesystem::path(SAPPHIRE_SOURCE_DIR)
        / "cfg" / "sapphire_mid360.toml");

    expectNear(config.imu.init.gravity_mag, 9.80665, "gravity value");
    expectNear(
        config.imu.noise.gyro_noise_density, 0.000152,
        "gyro noise value");
    expectNear(
        config.odometry.observer.position_gain, 4.5,
        "observer position gain");
    expectNear(
        config.odometry.observer.velocity_gain, 11.25,
        "observer velocity gain");
    expectNear(
        config.odometry.observer.orientation_gain, 4.0,
        "observer orientation gain");
    expectNear(
        config.odometry.observer.accel_bias_gain, 2.25,
        "observer accelerometer bias gain");
    expectNear(
        config.odometry.observer.gyro_bias_gain, 1.0,
        "observer gyroscope bias gain");
    expectNear(
        config.odometry.voxel_size, 0.25,
        "GICP source voxel size");
    expectNear(
        config.odometry.submap.splitting_distance, 1.5,
        "submap keyframe translation threshold");
    expectNear(
        config.odometry.submap.splitting_rotation,
        0.7853981633974483,
        "submap keyframe rotation threshold");
    expect(config.odometry.submap.max_keyframes == 10,
           "submap nearest-keyframe count");
    expectNear(
        config.odometry.submap.voxel_size, 0.25,
        "submap target voxel size");
    expect(!config.deskew.time_offset, "Mid-360 time offset must be disabled");
    expect(config.extrinsics.T_imu_lidar.matrix().isApprox(
               Eigen::Matrix4d::Identity(), 1e-12),
           "referenced DLIO extrinsic must be identity");
}

void testQuaternionNormalization() {
    const auto path = writeConfig(
        "sapphire_config_normalization.toml",
        validConfig(9.80665, "[0.0, 0.0, 0.0, 2.0]"));
    const auto config = sapphire::loadConfig(path);
    std::filesystem::remove(path);

    expect(config.extrinsics.T_imu_lidar.rotation().isApprox(
               Eigen::Matrix3d::Identity(), 1e-12),
           "non-unit quaternion must be normalized");
    expect(config.extrinsics.T_imu_lidar.translation().isApprox(
               Eigen::Vector3d(0.1, -0.2, 0.3), 1e-12),
           "translation must be parsed");
}

void testInvalidGravityRejected() {
    const auto path = writeConfig(
        "sapphire_config_bad_gravity.toml",
        validConfig(-9.80665, "[0.0, 0.0, 0.0, 1.0]"));
    bool threw = false;
    try {
        (void)sapphire::loadConfig(path);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    std::filesystem::remove(path);
    expect(threw, "negative gravity must be rejected");
}

void testMissingFieldRejected() {
    const auto path = writeConfig(
        "sapphire_config_missing_field.toml",
        "[imu]\ngravity = 9.80665\n");
    bool threw = false;
    try {
        (void)sapphire::loadConfig(path);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    std::filesystem::remove(path);
    expect(threw, "missing required fields must be rejected");
}

}  // namespace

int main() {
    testProjectConfig();
    testQuaternionNormalization();
    testInvalidGravityRejected();
    testMissingFieldRejected();
    std::cout << "All config tests passed\n";
    return EXIT_SUCCESS;
}
