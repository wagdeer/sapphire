#pragma once
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string_view>

namespace sapphire {

enum class LidarType { Livox = 0, Robosense = 1, HesaiXt = 2, Velodyne = 3 };

inline LidarType parse_lidar_type(std::string_view name) {
  if (name == "livox") return LidarType::Livox;
  if (name == "robosense" || name == "airy") return LidarType::Robosense;
  if (name == "hesaixt" || name == "hesai") return LidarType::HesaiXt;
  if (name == "velodyne") return LidarType::Velodyne;
  throw std::invalid_argument("lidar_type must be livox, robosense/airy, hesaixt/hesai or velodyne");
}

inline const char* lidar_type_name(int type) {
  switch (static_cast<LidarType>(type)) {
    case LidarType::Livox: return "livox";
    case LidarType::Robosense: return "robosense";
    case LidarType::HesaiXt: return "hesaixt";
    case LidarType::Velodyne: return "velodyne";
  }
  throw std::invalid_argument("lidar_type must be 0, 1, 2 or 3");
}

// Decoded point values, not packed SDK/ROS/PCL layouts. XYZ is in metres.
// Livox CustomPoint field contract:
// https://github.com/Livox-SDK/livox_ros_driver2/blob/master/msg/CustomPoint.msg
struct LivoxPoint {
  static constexpr LidarType type = LidarType::Livox;
  float x = 0, y = 0, z = 0;
  std::uint32_t offset_time = 0;  // Nanoseconds from message base time.
  std::uint8_t reflectivity = 0, tag = 0, line = 0;
};

// XYZIRT value contract used by LIO-SAM's Velodyne input; time may be negative
// when the driver header marks the scan end rather than its start.
// https://github.com/TixiaoShan/LIO-SAM/blob/master/src/imageProjection.cpp
struct VelodynePoint {
  static constexpr LidarType type = LidarType::Velodyne;
  float x = 0, y = 0, z = 0, intensity = 0;
  std::uint16_t ring = 0;
  float time = 0;  // Seconds relative to the message header.
};

// ROS XYZIRT field contract (published intensity is FLOAT32):
// https://github.com/RoboSense-LiDAR/rslidar_sdk/blob/main/doc/howto/05_how_to_change_point_type.md
struct RobosensePoint {
  static constexpr LidarType type = LidarType::Robosense;
  float x = 0, y = 0, z = 0, intensity = 0;
  std::uint16_t ring = 0;
  double timestamp = 0;  // Absolute seconds on the IMU clock.
};

struct HesaiXtPoint {
  static constexpr LidarType type = LidarType::HesaiXt;
  float x = 0, y = 0, z = 0, intensity = 0;
  std::uint16_t ring = 0;
  double timestamp = 0;  // Absolute seconds on the IMU clock.
};

// Decoded multi-LiDAR input before calibration/filtering. Preserve double precision
// for wire formats with FLOAT64 coordinates and absolute acquisition timestamps.
struct TimedLidarPoint {
  double x = 0, y = 0, z = 0, intensity = 0;
  double timestamp = 0;
};

// Normalized estimator input; time_offset is seconds from the output scan start.
struct LidarPoint {
  float x = 0.0f, y = 0.0f, z = 0.0f, intensity = 0.0f;
  float time_offset = 0.0f;
  float operator[](size_t index) const { return index == 0 ? x : index == 1 ? y : z; }
};

}  // namespace sapphire
