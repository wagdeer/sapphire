#pragma once
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>
#include "common/common.hpp"
#include "parameters.h"

namespace sapphire {
// Shared sensor preprocessing, independent of ROS and the selected state estimator.
// The adapter validates its wire layout. read_point decodes one indexed sample;
// absolute_bounds includes ALL acquisition times, before decimation/range filtering.
class LidarProcessor {
 public:
  explicit LidarProcessor(const SensorParameters &parameters)
      : lidar_type_(parameters.lidar_type), point_filter_num_(parameters.point_filter_num), blind_squared_(parameters.blind_squared),
        max_scan_duration_(parameters.max_scan_duration) {
    if (lidar_type_ < 0 || lidar_type_ > 2 || point_filter_num_ <= 0 ||
        !std::isfinite(blind_squared_) || blind_squared_ < 0.0 ||
        !std::isfinite(max_scan_duration_) || max_scan_duration_ <= 0.0)
      throw std::invalid_argument("Invalid single LiDAR preprocessing parameters");
  }

  int lidarType() const { return lidar_type_; }
  double maxScanDuration() const { return max_scan_duration_; }

  template<class ReadPoint>
  bool process(double header_timestamp, std::size_t point_count, ReadPoint read_point,
               std::optional<std::pair<double, double>> absolute_bounds,
               double &timestamp, std::vector<LidarPoint> &points,
               std::optional<double> &end_timestamp) const {
    points.clear();
    end_timestamp.reset();
    timestamp = header_timestamp;
    double scan_end = 0.0;
    if (lidar_type_ == 2) {
      if (!absolute_bounds) return false;
      timestamp = absolute_bounds->first;
      scan_end = absolute_bounds->second;
      if (!std::isfinite(timestamp) || !std::isfinite(scan_end) || timestamp < 0.0 ||
          scan_end > 9e9 || scan_end <= timestamp ||
          scan_end - timestamp > max_scan_duration_ + 1e-6) return false;
    }
    points.reserve(point_count / static_cast<std::size_t>(point_filter_num_) + 1);
    double first_time = lidar_type_ == 2 ? timestamp : 0.0;
    bool have_first_time = lidar_type_ == 2;
    for (size_t index = 0; index < point_count; ++index) {
      if (index % static_cast<size_t>(point_filter_num_) != 0) {
        continue;
      }

      double x, y, z, point_time;
      double intensity = 0.0;
      if (!read_point(index, x, y, z, intensity, point_time)) return false;
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
      if (lidar_type_ == 2 && (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z) || !std::isfinite(point.intensity))) continue;
      const double time_scale = lidar_type_ == 0 ? 1e-9 : 1.0;
      point.time_offset = static_cast<float>((point_time - first_time) * time_scale);
      if (point.x * point.x + point.y * point.y + point.z * point.z > blind_squared_) {
        points.push_back(point);
      }
    }
    if (lidar_type_ == 2) {
      if (points.empty()) return false;
      end_timestamp = scan_end;
    }
    return true;
  }

 private:
  int lidar_type_;
  int point_filter_num_;
  double blind_squared_;
  double max_scan_duration_;
};

}  // namespace sapphire
