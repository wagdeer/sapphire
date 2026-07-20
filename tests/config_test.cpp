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
    expect(config.imu.init.min_samples == 100, "IMU init sample count");
    expectNear(config.imu.init.timeout_sec, 5.0, "IMU init timeout");
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
        config.odometry.crop_box.min_x, -1.0,
        "crop box minimum X");
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
    expect(config.registration.type == "VGICP", "registration backend");
    expect(config.registration.gicp.max_iterations == 32,
           "GICP maximum iterations");
    expectNear(
        config.registration.gicp.max_correspondence_dist, 0.5,
        "GICP correspondence distance");
    expectNear(
        config.registration.vgicp.voxel_resolution, 0.5,
        "VGICP voxel resolution");
    expect(config.pgo.enabled, "PGO must be enabled in the Mid-360 profile");
    expectNear(
        config.pgo.keyframe_distance, 0.5,
        "PGO keyframe translation threshold");
    expectNear(
        config.pgo.loop_search_radius, 15.0,
        "PGO loop search radius");
    expect(config.pgo.map_frame_stride == 3,
           "PGO sparse-map frame stride");
    expectNear(
        config.pgo.map_voxel_size, 0.8,
        "PGO sparse-map voxel size");
    expect(config.pgo.occupancy.enabled,
           "occupancy grid must be enabled in Mid-360 profile");
    expectNear(
        config.pgo.occupancy.resolution, 0.1,
        "occupancy resolution");
    expectNear(
        config.pgo.occupancy.h_clearance, 2.0,
        "occupancy h_clearance");
    expectNear(
        config.pgo.occupancy.d_max, 3.0,
        "occupancy d_max");
    expect(!config.deskew.time_offset, "Mid-360 time offset must be disabled");
    expect(config.extrinsics.T_imu_lidar.matrix().isApprox(
               Eigen::Matrix4d::Identity(), 1e-12),
           "referenced DLIO extrinsic must be identity");
}

void testDirectionalEskfProfile() {
    const auto config = sapphire::loadConfig(
        std::filesystem::path(SAPPHIRE_SOURCE_DIR)
        / "cfg" / "sapphire_mid360_directional_eskf.toml");

    expect(config.odometry.fusion == "eskf",
           "directional profile must select ESKF fusion");
    expect(config.odometry.eskf.use_hessian,
           "directional profile must use the GICP Hessian");
    expect(!config.odometry.eskf.inject_full_pose,
           "directional profile must disable unconditional full-pose injection");
    expect(config.odometry.eskf.inject_directional_pose,
           "directional profile must use the Hessian observability pose gain");
    expectNear(
        config.odometry.eskf.velocity_correction_gain,
        8.0,
        "directional stage-one profile must retain stable velocity feedback");
    expectNear(
        config.odometry.eskf.bias_update_scale,
        0.0,
        "directional bring-up must initially freeze bias injection");
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
    testDirectionalEskfProfile();
    testQuaternionNormalization();
    testInvalidGravityRejected();
    testMissingFieldRejected();
    std::cout << "All config tests passed\n";
    return EXIT_SUCCESS;
}
