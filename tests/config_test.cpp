#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sapphire/config.hpp>
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

std::filesystem::path writeConfig(const std::string& filename, const std::string& contents) {
  const std::filesystem::path path = std::filesystem::temp_directory_path() / filename;
  std::ofstream stream(path);
  if (!stream) {
    fail("unable to create temporary config");
  }
  stream << contents;
  return path;
}

std::string validConfig(double gravity, const std::string& quaternion, const std::string& imu_noise = {}) {
  return "[sensor.imu]\n"
         "gravity = " +
         std::to_string(gravity) + "\n" + imu_noise +
         "[io]\n"
         "map_directory = \"method-b-test-map\"\n"
         "[extrinsics.T_imu_lidar]\n"
         "translation = [0.1, -0.2, 0.3]\n"
         "rotation_xyzw = " +
         quaternion + "\n";
}

void testQuaternionNormalization() {
  const auto path = writeConfig("sapphire_config_normalization.toml", validConfig(9.80665, "[0.0, 0.0, 0.0, 2.0]"));
  const auto config = sapphire::loadConfig(path);
  std::filesystem::remove(path);

  expect(config.extrinsics.T_imu_lidar.rotation().isApprox(Eigen::Matrix3d::Identity(), 1e-12), "non-unit quaternion must be normalized");
  expect(config.extrinsics.T_imu_lidar.translation().isApprox(Eigen::Vector3d(0.1, -0.2, 0.3), 1e-12), "translation must be parsed");
}

void testInvalidGravityRejected() {
  const auto path = writeConfig("sapphire_config_bad_gravity.toml", validConfig(-9.80665, "[0.0, 0.0, 0.0, 1.0]"));
  bool threw = false;
  try {
    (void)sapphire::loadConfig(path);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  std::filesystem::remove(path);
  expect(threw, "negative gravity must be rejected");
}

void testImuProcessNoise() {
  const auto check = [&](const std::string& noise, double gyro, double accel, double gyro_bias, double accel_bias) {
    const auto path = writeConfig("sapphire_config_imu_noise.toml", validConfig(9.80665, "[0.0, 0.0, 0.0, 1.0]", noise));
    const auto config = sapphire::loadConfig(path);
    std::filesystem::remove(path);
    expectNear(config.sensor.imu.noise.gyro_var, gyro, "gyro process noise");
    expectNear(config.sensor.imu.noise.accel_var, accel, "accel process noise");
    expectNear(config.sensor.imu.noise.gyro_bias_var, gyro_bias, "gyro bias process noise");
    expectNear(config.sensor.imu.noise.accel_bias_var, accel_bias, "accel bias process noise");
  };
  check("", 0.01, 0.01, 0.0001, 0.0001);
  check("gyro_var = 0.02\naccel_var = 0.03\ngyro_bias_var = 0.0004\naccel_bias_var = 0.0\n",
        0.02, 0.03, 0.0004, 0.0);
  for (const char* invalid : {"-0.01", "nan", "inf", "1e100"}) {
    const auto path = writeConfig("sapphire_config_bad_imu_noise.toml", validConfig(9.80665, "[0.0, 0.0, 0.0, 1.0]", std::string("accel_var = ") + invalid + "\n"));
    std::string error;
    try {
      (void)sapphire::loadConfig(path);
    } catch (const std::exception& e) {
      error = e.what();
    }
    std::filesystem::remove(path);
    expect(error.find("sensor.imu.accel_var") != std::string::npos, "invalid IMU process noise must be rejected by key");
  }
}

void testSensorLidarDefaultsWithoutSection() {
  const auto path = writeConfig("sapphire_config_no_sensor_lidar.toml", validConfig(9.80665, "[0.0, 0.0, 0.0, 1.0]"));
  const auto config = sapphire::loadConfig(path);
  std::filesystem::remove(path);

  expect(config.sensor.lidar.time_field == "auto", "missing sensor.lidar keeps time_field auto");
  expect(config.sensor.lidar.time_unit == "auto", "missing sensor.lidar keeps time_unit auto");
  expect(config.sensor.lidar.time_ref == "auto", "missing sensor.lidar keeps time_ref auto");
  expectNear(config.sensor.lidar.max_scan_duration, 0.5, "missing sensor.lidar keeps max_scan_duration at 0.5");
  expectNear(config.sensor.lidar.header_tolerance, 0.5, "missing sensor.lidar keeps header_tolerance at 0.5");
  expect(!config.sensor.lidar.require_point_time, "missing sensor.lidar does not require point time");
  expect(config.output.trajectory_dir.empty(), "missing output keeps the TUM dump disabled");
  expect(config.output.trajectory_file == "trajectory.tum", "missing output keeps the default TUM file name");
  expect(sapphire::trajectoryOutputPath(config).empty(), "disabled dump has no output path");
}

void testTrajectoryOutputPath() {
  const auto path = writeConfig("sapphire_config_trajectory_output.toml", validConfig(9.80665, "[0.0, 0.0, 0.0, 1.0]") +
                                                                              "[output]\n"
                                                                              "trajectory_dir = \"/tmp/estimates\"\n"
                                                                              "trajectory_file = \"sapphire/S3E_Dormitory_1/alpha.tum\"\n"
                                                                              "[pgo]\n"
                                                                              "enabled = true\n");
  const auto config = sapphire::loadConfig(path);
  std::filesystem::remove(path);
  expect(config.output.trajectory_dir == "/tmp/estimates", "output.trajectory_dir must be parsed");
  expect(config.output.trajectory_file == "sapphire/S3E_Dormitory_1/alpha.tum", "output.trajectory_file must be parsed");
  expect(sapphire::trajectoryOutputPath(config) == std::filesystem::path("/tmp/estimates/sapphire/S3E_Dormitory_1/alpha.tum"),
         "TUM path joins the configured directory and file");
}

void testPgoIsRequired() {
  const auto path = writeConfig("sapphire_config_trajectory_without_pgo.toml", validConfig(9.80665, "[0.0, 0.0, 0.0, 1.0]") +
                                                                                   "[pgo]\n"
                                                                                   "enabled = false\n");
  bool threw = false;
  std::string message;
  try {
    (void)sapphire::loadConfig(path);
  } catch (const std::invalid_argument& error) {
    threw = true;
    message = error.what();
  }
  std::filesystem::remove(path);
  expect(threw, "pgo.enabled = false must be rejected");
  expect(message.find("pgo.enabled") != std::string::npos,
         "the rejection must name pgo.enabled");
}

void testPgoParameters() {
  const auto base = validConfig(9.80665, "[0.0, 0.0, 0.0, 1.0]");
  const auto path = writeConfig("sapphire_config_pgo_parameters.toml", base + R"(
[submap]
min_voxel_num = 64
min_frames = 16
max_range_m = 42.0

[submap.ba]
min_frames = 6
min_planes = 18
max_iterations = 2

[pgo]
relinearize_threshold = 0.2
relinearize_skip = 2

[pgo.loop_closure]
enabled = false
local_max_candidates = 4
global_max_candidates = 11
fft_min_ambiguity = 1.3
global_min_overlap = 0.60
global_refine_updates = 0
)");
  const auto config = sapphire::loadConfig(path);
  std::filesystem::remove(path);
  expectNear(config.pgo.relinearize_threshold, 0.2, "iSAM2 relinearization threshold");
  expect(config.pgo.relinearize_skip == 2, "iSAM2 relinearization interval");
  expectNear(config.submap.max_range_m, 42.0, "sealed submap range");
  expect(config.submap.min_voxel_num == 64 && config.submap.min_frames == 16, "submap overrides");
  const auto& ba = config.submap.ba;
  expect(ba.min_frames == 6 && ba.min_planes == 18 && ba.max_iterations == 2, "submap BA overrides");
  const auto& loop = config.pgo.loop_closure;
  expect(!loop.enabled && loop.local_max_candidates == 4 && loop.global_max_candidates == 11 && loop.global_refine_updates == 0,
         "loop closure overrides");
  expectNear(loop.fft_min_ambiguity, 1.3, "FFT ambiguity threshold");
  expectNear(loop.global_min_overlap, 0.60, "global geometry overlap threshold");
}

void testShippedProfilesLoad() {
  const auto cfg = std::filesystem::path(SAPPHIRE_SOURCE_DIR) / "cfg" / "cfg_b";
  int count = 0;
  for (const auto& entry : std::filesystem::directory_iterator(cfg)) {
    if (entry.path().extension() != ".toml") continue;
    const auto config = sapphire::loadConfig(entry.path());
    expect(config.pgo.enabled, "Method B needs PGO");
    expect(!config.io.map_directory.empty(), "Method B needs a map directory");
    expectNear(config.odometry.voxel_size, 0.25, "registration voxel size");
    ++count;
  }
  expect(count == 5, "every shipped profile must load");
}

void testTrajectoryFileMustStayRelative() {
  const auto path = writeConfig("sapphire_config_bad_trajectory_file.toml", validConfig(9.80665, "[0.0, 0.0, 0.0, 1.0]") +
                                                                                "[output]\n"
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

void expectSchemaError(const std::string& contents, const std::string& filename, const std::string& needle, const std::string& path_in_message,
                       const std::string& detail) {
  const auto path = writeConfig(filename, contents);
  bool threw = false;
  std::string message;
  try {
    (void)sapphire::loadConfig(path);
  } catch (const std::runtime_error& error) {
    threw = true;
    message = error.what();
  }
  std::filesystem::remove(path);
  expect(threw, needle + " must be rejected");
  const auto at = contents.find(needle);
  expect(at != std::string::npos, "fixture must contain " + needle);
  int line = 1;
  for (size_t i = 0; i < at; ++i) {
    if (contents[i] == '\n') {
      ++line;
    }
  }
  expect(message.find(path_in_message) != std::string::npos, "the error must name " + path_in_message);
  expect(message.find(detail) != std::string::npos, "the error must say " + detail);
  expect(message.find("line " + std::to_string(line)) != std::string::npos, "the error must include the TOML line of " + needle);
}

void testWrongOptionalTypeRejected() {
  const std::string contents = validConfig(9.80665, "[0.0, 0.0, 0.0, 1.0]") +
                               "[pgo]\n"
                               "enabled = \"true\"\n";
  expectSchemaError(contents, "sapphire_config_pgo_enabled_string.toml", "enabled = \"true\"", "pgo.enabled", "boolean");
}

void testInvalidPgoParameters() {
  const auto base = validConfig(9.80665, "[0.0, 0.0, 0.0, 1.0]");
  const auto reject = [&](const char* section, const char* key) {
    const auto path = writeConfig("sapphire_config_invalid_pgo.toml", base + section);
    std::string error;
    try {
      (void)sapphire::loadConfig(path);
    } catch (const std::invalid_argument& e) {
      error = e.what();
    }
    std::filesystem::remove(path);
    expect(error.find(key) != std::string::npos, std::string(key) + " must be rejected");
  };
  reject("[pgo]\nrelinearize_skip = 0\n", "pgo.relinearize_skip");
  reject("[submap.ba]\nmax_iterations = 0\n", "submap.ba.max_iterations");
  reject("[submap]\nmin_frames = 0\n", "submap.min_frames");
  reject("[pgo.loop_closure]\nlocal_search_radius_m = 30.0\n", "pgo.loop_closure.local_departure_radius_m");
}

void testUnknownKeyRejected() {
  const std::string contents = validConfig(9.80665, "[0.0, 0.0, 0.0, 1.0]") +
                               "[pgo]\n"
                               "enable = true\n";
  expectSchemaError(contents, "sapphire_config_pgo_enable_typo.toml", "enable = true", "pgo.enable", "unknown TOML key");
}

void testLidarTimeSessionCopiesSchema() {
  sapphire::Config config;
  config.sensor.lidar.time_field = "timestamp";
  config.sensor.lidar.time_unit = "ns";
  config.sensor.lidar.time_ref = "relative";
  config.sensor.lidar.max_scan_duration = 0.2;
  config.sensor.lidar.header_tolerance = 0.25;
  config.sensor.lidar.require_point_time = true;

  const sapphire::sensor::LidarTimeSession session = sapphire::makeLidarTimeSession(config.sensor.lidar);
  expect(session.config().time_field == "timestamp", "session copies time_field");
  expect(session.config().time_unit == "ns", "session copies time_unit");
  expect(session.config().time_ref == "relative", "session copies time_ref");
  expectNear(session.config().max_scan_duration, 0.2, "session copies max_scan_duration");
  expectNear(session.config().header_tolerance, 0.25, "session copies header_tolerance");
  expect(session.requirePointTime(), "session copies require_point_time");
  expect(!session.locked(), "a new session is not locked");

  const sapphire::sensor::IngestConfig ingest;
  const sapphire::Config defaults;
  expect(ingest.time_field == defaults.sensor.lidar.time_field && ingest.time_unit == defaults.sensor.lidar.time_unit &&
             ingest.time_ref == defaults.sensor.lidar.time_ref,
         "ingest and schema time strings share defaults");
  expectNear(ingest.max_scan_duration, defaults.sensor.lidar.max_scan_duration, "ingest and schema share max_scan_duration");
  expectNear(ingest.header_tolerance, defaults.sensor.lidar.header_tolerance, "ingest and schema share header_tolerance");
}

void testMissingFieldRejected() {
  const auto path = writeConfig("sapphire_config_missing_field.toml", "[sensor.imu]\ngravity = 9.80665\n");
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
  testTrajectoryOutputPath();
  testPgoIsRequired();
  testPgoParameters();
  testInvalidPgoParameters();
  testShippedProfilesLoad();
  testTrajectoryFileMustStayRelative();
  testQuaternionNormalization();
  testInvalidGravityRejected();
  testImuProcessNoise();
  testWrongOptionalTypeRejected();
  testUnknownKeyRejected();
  testLidarTimeSessionCopiesSchema();
  testMissingFieldRejected();
  std::cout << "All config tests passed\n";
  return EXIT_SUCCESS;
}
