#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <stdexcept>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <vector>

#include "common/common.hpp"
#include "parameters.h"
#include "frontend/common/lidar_preprocess.hpp"

namespace sapphire_ros {
namespace detail {

inline size_t scalar_size(std::uint8_t datatype) {
  using F = sensor_msgs::msg::PointField;
  switch (datatype) {
    case F::INT8: case F::UINT8: return 1;
    case F::INT16: case F::UINT16: return 2;
    case F::INT32: case F::UINT32: case F::FLOAT32: return 4;
    case F::FLOAT64: return 8;
    default: return 0;
  }
}

inline bool valid_scalar(const sensor_msgs::msg::PointField *field, std::uint32_t point_step) {
  return field && field->count == 1 && scalar_size(field->datatype) != 0 && field->offset <= point_step &&
         scalar_size(field->datatype) <= point_step - field->offset;
}

inline bool valid_layout(const sensor_msgs::msg::PointCloud2 &message) {
  return message.width != 0 && message.height != 0 && message.point_step != 0 &&
         static_cast<std::uint64_t>(message.width) * message.point_step <= message.row_step &&
         static_cast<std::uint64_t>(message.row_step) * message.height <= message.data.size();
}

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

// Shared with dual fusion: inspect every acquisition time before filtering.
inline bool inspect_absolute_times(const sensor_msgs::msg::PointCloud2 &m,
                                   const sensor_msgs::msg::PointField *time, double offset,
                                   double max_duration, double &begin, double &end, bool &monotonic) {
  if (!valid_layout(m) || !valid_scalar(time, m.point_step) ||
      time->datatype != sensor_msgs::msg::PointField::FLOAT64) return false;
  begin = std::numeric_limits<double>::infinity();
  end = -std::numeric_limits<double>::infinity();
  monotonic = true;
  double previous = -std::numeric_limits<double>::infinity();
  const size_t count = static_cast<size_t>(m.width) * m.height;
  for (size_t i = 0; i < count; ++i) {
    const auto *point = m.data.data() + (i / m.width) * m.row_step + (i % m.width) * m.point_step;
    const double t = load_scalar<double>(point + time->offset, m.is_bigendian) + offset;
    if (!std::isfinite(t) || t < 0.0 || t > 9e9) return false;
    if (t < previous) monotonic = false;
    previous = t;
    begin = std::min(begin, t);
    end = std::max(end, t);
  }
  return end >= begin && end - begin <= max_duration + 1e-6;
}

}  // namespace detail

// ROS wire decoding adapter; shared numeric filtering/timing lives in frontend/common.
class LidarProcessor {
 public:
  explicit LidarProcessor(const sapphire::SensorParameters &parameters) : processor_(parameters) {}

  bool process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &message, double &timestamp, std::vector<sapphire::LidarPoint> &points,
               std::optional<double> &end_timestamp) const {
    points.clear();
    end_timestamp.reset();
    timestamp = std::numeric_limits<double>::quiet_NaN();
    if (!message) return false;
    timestamp = static_cast<double>(message->header.stamp.sec) + static_cast<double>(message->header.stamp.nanosec) * 1e-9;
    const int lidar_type_ = processor_.lidarType();
    const auto *x_field = detail::find_field(*message, "x");
    const auto *y_field = detail::find_field(*message, "y");
    const auto *z_field = detail::find_field(*message, "z");
    const auto *intensity_field = detail::find_field(*message, "intensity");
    const auto *time_field = detail::find_field(*message, "timestamp");
    if (time_field == nullptr && lidar_type_ != 2) {
      time_field = detail::find_field(*message, "time");
    }
    if (!detail::valid_layout(*message) || !detail::valid_scalar(x_field, message->point_step) ||
        !detail::valid_scalar(y_field, message->point_step) || !detail::valid_scalar(z_field, message->point_step) ||
        !detail::valid_scalar(time_field, message->point_step) ||
        (intensity_field && !detail::valid_scalar(intensity_field, message->point_step))) {
      return false;
    }

    const double max_scan_duration_ = processor_.maxScanDuration();
    double scan_end = 0.0;
    if (lidar_type_ == 2) {
      bool monotonic;
      if (!detail::inspect_absolute_times(*message, time_field, 0.0, max_scan_duration_, timestamp, scan_end, monotonic) ||
          scan_end <= timestamp) return false;
    }
    const std::size_t count = static_cast<std::size_t>(message->width) * message->height;
    const auto read_point = [&](std::size_t index, double &x, double &y, double &z,
                                double &intensity, double &point_time) {
      const std::size_t offset = (index / message->width) * message->row_step +
                                 (index % message->width) * message->point_step;
      if (offset + message->point_step > message->data.size()) return false;
      const auto *source = message->data.data() + offset;
      if (!detail::read_numeric(source, x_field, message->is_bigendian, x) ||
          !detail::read_numeric(source, y_field, message->is_bigendian, y) ||
          !detail::read_numeric(source, z_field, message->is_bigendian, z) ||
          !detail::read_numeric(source, time_field, message->is_bigendian, point_time)) return false;
      detail::read_numeric(source, intensity_field, message->is_bigendian, intensity);
      return true;
    };
    const auto bounds = lidar_type_ == 2 ? std::make_optional(std::make_pair(timestamp, scan_end)) : std::nullopt;
    return processor_.process(timestamp, count, read_point, bounds, timestamp, points, end_timestamp);
  }

 private:
  sapphire::LidarProcessor processor_;
};

}  // namespace sapphire_ros
