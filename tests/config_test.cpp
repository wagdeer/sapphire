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

std::string validConfig(double gravity, const std::string& quaternion) {
  return "[imu]\n"
         "gravity = " +
         std::to_string(gravity) +
         "\n"
         "[methodB]\n"
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

void testShippedProfilesLoad() {
  const auto cfg = std::filesystem::path(SAPPHIRE_SOURCE_DIR) / "cfg" / "cfg_b";
  int count = 0;
  for (const auto& entry : std::filesystem::directory_iterator(cfg)) {
    if (entry.path().extension() != ".toml") continue;
    const auto config = sapphire::loadConfig(entry.path());
    expect(config.pgo.enabled, "Method B needs PGO");
    expect(!config.methodB.map_directory.empty(), "Method B needs a map directory");
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
  const auto path = writeConfig("sapphire_config_missing_field.toml", "[imu]\ngravity = 9.80665\n");
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
  testShippedProfilesLoad();
  testTrajectoryFileMustStayRelative();
  testQuaternionNormalization();
  testInvalidGravityRejected();
  testWrongOptionalTypeRejected();
  testUnknownKeyRejected();
  testLidarTimeSessionCopiesSchema();
  testMissingFieldRejected();
  std::cout << "All config tests passed\n";
  return EXIT_SUCCESS;
}
