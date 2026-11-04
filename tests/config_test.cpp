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
        config.pgo.loop_search_radius, 30.0,
        "PGO loop search radius");
    expect(config.pgo.lidar_min_inliers == 64,
           "PGO loop minimum inliers");
    expect(config.pgo.lidar_max_iterations == 32,
           "PGO loop maximum iterations");
    expectNear(
        config.pgo.lidar_max_correspondence_m, 2.0,
        "PGO loop correspondence distance");
    expectNear(
        config.pgo.lidar_min_overlap, 0.5,
        "PGO loop minimum overlap");
    expectNear(
        config.pgo.lidar_max_fitness_m2, 0.5,
        "PGO loop maximum fitness");
    expectNear(
        config.pgo.lidar_translation_sigma_m, 0.2,
        "PGO loop translation sigma floor");
    expectNear(
        config.pgo.lidar_rotation_sigma_rad, 0.1,
        "PGO loop rotation sigma floor");
    expectNear(
        config.pgo.max_constraint_error_sigma, 3.0,
        "PGO loop residual rejection threshold");
    expect(config.pgo.map_frame_stride == 3,
           "PGO sparse-map frame stride");
    expectNear(
        config.pgo.map_voxel_size, 0.8,
        "PGO sparse-map voxel size");
    expectNear(
        config.pgo.source_voxel_size, 0.4,
        "PGO query voxel size");
    expectNear(
        config.pgo.source_travel_radius_m, 10.0,
        "PGO query travel window");
    expectNear(
        config.pgo.source_max_range_m, 20.0,
        "PGO query body-frame range gate");
    expect(config.pgo.source_max_frames == 256,
           "PGO query window frame cap");
    expect(config.sensor.lidar.time_field == "auto",
           "Mid-360 profile leaves time_field automatic");
    expect(config.sensor.lidar.time_unit == "auto",
           "Mid-360 profile leaves time_unit automatic");
    expect(config.sensor.lidar.time_ref == "auto",
           "Mid-360 profile leaves time_ref automatic");
    expectNear(config.sensor.lidar.max_scan_duration, 0.5,
               "Mid-360 profile uses the 0.5 s scan window");
    expect(!config.sensor.lidar.require_point_time,
           "Mid-360 profile keeps frames when time classification fails");
    expect(config.sensor.imu.accel_unit == "auto",
           "Mid-360 profile leaves accel_unit automatic");
    expect(!config.pgo.occupancy.enabled,
           "default Mid-360 profile keeps occupancy off");
    expectNear(
        config.pgo.occupancy.resolution, 0.1,
        "occupancy resolution");
    expectNear(
        config.pgo.occupancy.h_clearance, 2.0,
        "occupancy h_clearance");
    expectNear(
        config.pgo.occupancy.ground_margin, 0.5,
        "occupancy ground_margin");
    expectNear(
        config.pgo.occupancy.d_max, 6.0,
        "occupancy d_max");
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

void testSensorLidarDefaultsWithoutSection() {
    const auto path = writeConfig(
        "sapphire_config_no_sensor_lidar.toml",
        validConfig(9.80665, "[0.0, 0.0, 0.0, 1.0]"));
    const auto config = sapphire::loadConfig(path);
    std::filesystem::remove(path);

    expect(config.sensor.lidar.time_field == "auto",
           "missing sensor.lidar keeps time_field auto");
    expect(config.sensor.lidar.time_unit == "auto",
           "missing sensor.lidar keeps time_unit auto");
    expect(config.sensor.lidar.time_ref == "auto",
           "missing sensor.lidar keeps time_ref auto");
    expectNear(
        config.sensor.lidar.max_scan_duration, 0.5,
        "missing sensor.lidar keeps max_scan_duration at 0.5");
    expectNear(
        config.sensor.lidar.header_tolerance, 0.5,
        "missing sensor.lidar keeps header_tolerance at 0.5");
    expect(!config.sensor.lidar.require_point_time,
           "missing sensor.lidar does not require point time");
    expect(config.sensor.imu.accel_unit == "auto",
           "missing sensor.imu keeps accel_unit auto");
    expect(config.output.trajectory_dir.empty(),
           "missing output keeps the TUM dump disabled");
    expect(config.output.trajectory_file == "trajectory.tum",
           "missing output keeps the default TUM file name");
    expect(sapphire::trajectoryOutputPath(config).empty(),
           "disabled dump has no output path");
}

void testExplicitAccelUnit() {
    const auto path = writeConfig(
        "sapphire_config_accel_unit.toml",
        validConfig(9.80665, "[0.0, 0.0, 0.0, 1.0]")
            + "[sensor.imu]\naccel_unit = \"g\"\n");
    const auto config = sapphire::loadConfig(path);
    std::filesystem::remove(path);
    expect(config.sensor.imu.accel_unit == "g",
           "sensor.imu.accel_unit must override the default");
}

void testInvalidAccelUnitRejected() {
    const auto path = writeConfig(
        "sapphire_config_bad_accel_unit.toml",
        validConfig(9.80665, "[0.0, 0.0, 0.0, 1.0]")
            + "[sensor.imu]\naccel_unit = \"meters\"\n");
    bool threw = false;
    try {
        (void)sapphire::loadConfig(path);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    std::filesystem::remove(path);
    expect(threw, "unknown accel_unit must be rejected");
}

void testTrajectoryOutputPath() {
    const auto path = writeConfig(
        "sapphire_config_trajectory_output.toml",
        validConfig(9.80665, "[0.0, 0.0, 0.0, 1.0]")
            + "[output]\n"
              "trajectory_dir = \"/tmp/estimates\"\n"
              "trajectory_file = \"sapphire/S3E_Dormitory_1/alpha.tum\"\n");
    const auto config = sapphire::loadConfig(path);
    std::filesystem::remove(path);
    expect(config.output.trajectory_dir == "/tmp/estimates",
           "output.trajectory_dir must be parsed");
    expect(
        config.output.trajectory_file == "sapphire/S3E_Dormitory_1/alpha.tum",
        "output.trajectory_file must be parsed");
    expect(
        sapphire::trajectoryOutputPath(config)
            == std::filesystem::path(
                "/tmp/estimates/sapphire/S3E_Dormitory_1/alpha.tum"),
        "TUM path joins the configured directory and file");
}

void testTrajectoryFileMustStayRelative() {
    const auto path = writeConfig(
        "sapphire_config_bad_trajectory_file.toml",
        validConfig(9.80665, "[0.0, 0.0, 0.0, 1.0]")
            + "[output]\n"
              "trajectory_dir = \"/tmp/estimates\"\n"
              "trajectory_file = \"../alpha.tum\"\n");
    bool threw = false;
    try {
        (void)sapphire::loadConfig(path);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    std::filesystem::remove(path);
    expect(threw, "trajectory_file must not escape trajectory_dir");
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
    testSensorLidarDefaultsWithoutSection();
    testExplicitAccelUnit();
    testInvalidAccelUnitRejected();
    testTrajectoryOutputPath();
    testTrajectoryFileMustStayRelative();
    testProjectConfig();
    testQuaternionNormalization();
    testInvalidGravityRejected();
    testMissingFieldRejected();
    std::cout << "All config tests passed\n";
    return EXIT_SUCCESS;
}
