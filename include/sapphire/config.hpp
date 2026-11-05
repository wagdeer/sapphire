#pragma once

#include <sapphire/sensor/cloud_ingest.hpp>
#include <sapphire/types.hpp>

#include <filesystem>

namespace sapphire {

/// Load and validate a Sapphire TOML configuration file.
/// Throws std::runtime_error when the file cannot be parsed, a required field
/// is missing, a present field has the wrong type, or a key is not recognized.
/// Throws std::invalid_argument when a value fails validation.
Config loadConfig(const std::filesystem::path& path);

/// Validate a fully constructed configuration.
/// Throws std::invalid_argument when a value is invalid.
void validateConfig(const Config& config);

/// Join output.trajectory_dir and output.trajectory_file.
/// Empty when the dump is disabled.
std::filesystem::path trajectoryOutputPath(const Config& config);

/// Copy the schema lidar time fields into the runtime lock. This is the only
/// place those fields are transferred; ROS wrappers do not copy them again.
inline sensor::LidarTimeSession makeLidarTimeSession(
    const Config::Sensor::Lidar& lidar)
{
    sensor::IngestConfig ingest;
    ingest.time_field = lidar.time_field;
    ingest.time_unit = lidar.time_unit;
    ingest.time_ref = lidar.time_ref;
    ingest.max_scan_duration = lidar.max_scan_duration;
    ingest.header_tolerance = lidar.header_tolerance;
    return sensor::LidarTimeSession(ingest, lidar.require_point_time);
}

}  // namespace sapphire
