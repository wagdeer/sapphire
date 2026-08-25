#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <vector>

#include "common.hpp"
#include "parameters.h"

namespace sapphire_ros {
namespace detail {

inline const sensor_msgs::msg::PointField *find_field(const sensor_msgs::msg::PointCloud2 &message, const char *name) {
  for (const sensor_msgs::msg::PointField &field : message.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

template <typename T>
T load_scalar(const std::uint8_t *source, bool source_big_endian) {
  std::array<std::uint8_t, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), source, sizeof(T));
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  constexpr bool kHostBigEndian = false;
#else
  constexpr bool kHostBigEndian = true;
#endif
  if (source_big_endian != kHostBigEndian) {
    std::reverse(bytes.begin(), bytes.end());
  }
  T value;
  std::memcpy(&value, bytes.data(), sizeof(T));
  return value;
}

inline bool read_numeric(const std::uint8_t *point, const sensor_msgs::msg::PointField *field, bool big_endian, double &value) {
  if (field == nullptr) {
    return false;
  }
  const std::uint8_t *source = point + field->offset;
  switch (field->datatype) {
    case sensor_msgs::msg::PointField::INT8:
      value = load_scalar<std::int8_t>(source, big_endian);
      return true;
    case sensor_msgs::msg::PointField::UINT8:
      value = load_scalar<std::uint8_t>(source, big_endian);
      return true;
    case sensor_msgs::msg::PointField::INT16:
      value = load_scalar<std::int16_t>(source, big_endian);
      return true;
    case sensor_msgs::msg::PointField::UINT16:
      value = load_scalar<std::uint16_t>(source, big_endian);
      return true;
    case sensor_msgs::msg::PointField::INT32:
      value = load_scalar<std::int32_t>(source, big_endian);
      return true;
    case sensor_msgs::msg::PointField::UINT32:
      value = load_scalar<std::uint32_t>(source, big_endian);
      return true;
    case sensor_msgs::msg::PointField::FLOAT32:
      value = load_scalar<float>(source, big_endian);
      return true;
    case sensor_msgs::msg::PointField::FLOAT64:
      value = load_scalar<double>(source, big_endian);
      return true;
    default:
      return false;
  }
}

}  // namespace detail

class LidarProcessor {
 public:
  explicit LidarProcessor(const sapphire::SensorParameters &parameters)
      : lidar_type_(parameters.lidar_type), point_filter_num_(parameters.point_filter_num), blind_squared_(parameters.blind_squared) {}

  bool process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &message, double &timestamp, std::vector<LidarPoint> &points) const {
    timestamp = static_cast<double>(message->header.stamp.sec) + static_cast<double>(message->header.stamp.nanosec) * 1e-9;
    if (lidar_type_ != 0 && lidar_type_ != 1) {
      return false;
    }
    const auto *x_field = detail::find_field(*message, "x");
    const auto *y_field = detail::find_field(*message, "y");
    const auto *z_field = detail::find_field(*message, "z");
    const auto *intensity_field = detail::find_field(*message, "intensity");
    const auto *time_field = detail::find_field(*message, "timestamp");
    if (time_field == nullptr) {
      time_field = detail::find_field(*message, "time");
    }
    if (x_field == nullptr || y_field == nullptr || z_field == nullptr || time_field == nullptr || message->point_step == 0) {
      return false;
    }

    const size_t point_count = static_cast<size_t>(message->width) * message->height;
    points.reserve(point_count / static_cast<size_t>(point_filter_num_) + 1);
    double first_time = 0.0;
    bool have_first_time = false;
    for (size_t index = 0; index < point_count; ++index) {
      if (index % static_cast<size_t>(point_filter_num_) != 0) {
        continue;
      }

      const size_t row = index / message->width;
      const size_t column = index % message->width;
      const size_t offset = row * message->row_step + column * message->point_step;
      if (offset + message->point_step > message->data.size()) {
        return false;
      }
      const std::uint8_t *source = message->data.data() + offset;
      double x;
      double y;
      double z;
      double intensity = 0.0;
      double point_time;
      if (!detail::read_numeric(source, x_field, message->is_bigendian, x) || !detail::read_numeric(source, y_field, message->is_bigendian, y) ||
          !detail::read_numeric(source, z_field, message->is_bigendian, z) ||
          !detail::read_numeric(source, time_field, message->is_bigendian, point_time)) {
        return false;
      }
      detail::read_numeric(source, intensity_field, message->is_bigendian, intensity);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(point_time)) {
        continue;
      }
      if (!have_first_time) {
        first_time = point_time;
        have_first_time = true;
        if (lidar_type_ == 1) {
          timestamp = first_time;
        }
      }

      LidarPoint point;
      point.x = static_cast<float>(x);
      point.y = static_cast<float>(y);
      point.z = static_cast<float>(z);
      point.intensity = static_cast<float>(intensity);
      const double time_scale = lidar_type_ == 0 ? 1e-9 : 1.0;
      point.time_offset = static_cast<float>((point_time - first_time) * time_scale);
      if (point.x * point.x + point.y * point.y + point.z * point.z > blind_squared_) {
        points.push_back(point);
      }
    }
    return true;
  }

 private:
  int lidar_type_;
  int point_filter_num_;
  double blind_squared_;
};

}  // namespace sapphire_ros
