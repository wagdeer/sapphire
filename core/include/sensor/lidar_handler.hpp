#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>
#include "sensor/lidar_type.hpp"
#include "parameters.hpp"

namespace sapphire {

class LidarHandler {
 public:
  explicit LidarHandler(const SensorParameters &parameters)
      : lidar_type_(parameters.lidar_type), point_filter_num_(parameters.point_filter_num), blind_squared_(parameters.blind_squared),
        max_scan_duration_(parameters.max_scan_duration) {
    if (lidar_type_ < 0 || lidar_type_ > 3 || point_filter_num_ <= 0 ||
        !std::isfinite(blind_squared_) || blind_squared_ < 0.0 ||
        !std::isfinite(max_scan_duration_) || max_scan_duration_ <= 0.0)
      throw std::invalid_argument("Invalid single LiDAR preprocessing parameters");
  }

  int lidarType() const { return lidar_type_; }
  double maxScanDuration() const { return max_scan_duration_; }

  // Native decoded input. Determine scan bounds before range filtering or thinning.
  template<class RawPoint>
  bool process(double header_timestamp, const std::vector<RawPoint> &raw,
               double &timestamp, std::vector<LidarPoint> &points,
               std::optional<double> &end_timestamp) const {
    points.clear();
    end_timestamp.reset();
    timestamp = header_timestamp;
    if (static_cast<int>(RawPoint::type) != lidar_type_ || raw.empty()) return false;
    const auto acquisition_time = [header_timestamp](const RawPoint &p) {
      if constexpr (RawPoint::type == LidarType::Livox)
        return header_timestamp + p.offset_time * 1e-9;
      else if constexpr (RawPoint::type == LidarType::Velodyne)
        return header_timestamp + static_cast<double>(p.time);
      else
        return p.timestamp;
    };
    double begin = std::numeric_limits<double>::infinity();
    double end = -std::numeric_limits<double>::infinity();
    for (const auto &p : raw) {
      const double time = acquisition_time(p);
      if (!std::isfinite(time) || time < 0.0 || time > 9e9) return false;
      begin = std::min(begin, time);
      end = std::max(end, time);
    }
    const auto read = [&](std::size_t i, double &x, double &y, double &z,
                          double &intensity, double &time) {
      const auto &p = raw[i];
      x = p.x; y = p.y; z = p.z;
      if constexpr (RawPoint::type == LidarType::Livox) intensity = p.reflectivity;
      else intensity = p.intensity;
      time = acquisition_time(p);
      return true;
    };
    return process(header_timestamp, raw.size(), read, std::make_pair(begin, end),
                   timestamp, points, end_timestamp);
  }

  template<class ReadPoint>
  bool process(double header_timestamp, std::size_t point_count, ReadPoint read_point,
               std::optional<std::pair<double, double>> absolute_bounds,
               double &timestamp, std::vector<LidarPoint> &points,
               std::optional<double> &end_timestamp) const {
    points.clear();
    end_timestamp.reset();
    timestamp = header_timestamp;
    double scan_end = 0.0;
    const bool bounded = absolute_bounds.has_value();
    if (lidar_type_ != 0 && !bounded) return false;
    if (bounded) {
      timestamp = absolute_bounds->first;
      scan_end = absolute_bounds->second;
      if (!std::isfinite(timestamp) || !std::isfinite(scan_end) || timestamp < 0.0 ||
          scan_end > 9e9 || scan_end <= timestamp ||
          scan_end - timestamp > max_scan_duration_ + 1e-6) return false;
    }
    points.reserve(point_count / static_cast<std::size_t>(point_filter_num_) + 1);
    double first_time = bounded ? timestamp : 0.0;
    bool have_first_time = bounded;
    for (size_t index = 0; index < point_count; ++index) {
      if (index % static_cast<size_t>(point_filter_num_) != 0) {
        continue;
      }

      double x, y, z, point_time;
      double intensity = 0.0;
      if (!read_point(index, x, y, z, intensity, point_time)) {
        points.clear();
        return false;
      }
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(point_time)) {
        continue;
      }
      if (!have_first_time) {
        first_time = point_time;
        have_first_time = true;
      }

      LidarPoint point;
      point.x = static_cast<float>(x);
      point.y = static_cast<float>(y);
      point.z = static_cast<float>(z);
      point.intensity = static_cast<float>(intensity);
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z) || !std::isfinite(point.intensity)) continue;
      const double time_scale = !bounded && lidar_type_ == 0 ? 1e-9 : 1.0;
      point.time_offset = static_cast<float>((point_time - first_time) * time_scale);
      if (point.x * point.x + point.y * point.y + point.z * point.z > blind_squared_) {
        points.push_back(point);
      }
    }
    if (bounded) {
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
