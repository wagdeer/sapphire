#include "frontend/common/synchronizer.hpp"
#include <cmath>
#include <spdlog/spdlog.h>

namespace sapphire {
bool Synchronizer::push_imu(ImuMeas imu) {
  if (!std::isfinite(imu.timestamp) || !imu.gyro.allFinite() || !imu.accel.allFinite()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!accepting_ || imu_buf_.size() >= imu_capacity_ || imu.timestamp <= last_imu_time_) {
    return false;
  }
  last_imu_time_ = imu.timestamp;
  imu_buf_.push_back(std::move(imu));
  return true;
}

bool Synchronizer::push_lidar(double timestamp, std::vector<LidarPoint> &&cloud, std::optional<double> end_timestamp) {
  if (!std::isfinite(timestamp) || cloud.empty() || cloud.capacity() > 16 * 1024 * 1024 / sizeof(LidarPoint)) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!accepting_ || lidar_buf_.size() >= 2 || timestamp <= last_lidar_time_) return false;
  for (const LidarPoint &point : cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) || !std::isfinite(point.intensity) ||
        !std::isfinite(point.time_offset) || point.time_offset < 0.0F) {
      return false;
    }
  }
  if (end_timestamp && (!std::isfinite(*end_timestamp) || *end_timestamp < timestamp || *end_timestamp - timestamp > max_scan_duration_ + 1e-6)) {
    return false;
  }
  std::stable_sort(cloud.begin(), cloud.end(), [](const LidarPoint &left, const LidarPoint &right) { return left.time_offset < right.time_offset; });
  if (end_timestamp && timestamp + cloud.back().time_offset > *end_timestamp + 1e-6) {
    return false;
  }
  const auto clipped = std::upper_bound(cloud.begin(), cloud.end(), static_cast<float>(max_scan_duration_),
                                        [](float limit, const LidarPoint &point) { return limit < point.time_offset; });
  cloud.erase(clipped, cloud.end());
  if (cloud.empty()) {
    return false;
  }

  last_lidar_time_ = timestamp;
  lidar_time_buf_.push_back(timestamp);
  lidar_end_time_buf_.push_back(end_timestamp.value_or(timestamp + cloud.back().time_offset));
  lidar_buf_.push_back(std::make_shared<std::vector<LidarPoint>>(std::move(cloud)));
  return true;
}

bool Synchronizer::sync_packages(MeasGroup &measures) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!pending_.lidar_cloud) {
    if (lidar_buf_.empty()) {
      return false;
    }
    pending_.lidar_cloud = std::move(lidar_buf_.front());
    pending_.lidar_begin_time = lidar_time_buf_.front();
    lidar_buf_.pop_front();
    lidar_time_buf_.pop_front();
    pending_.lidar_end_time = lidar_end_time_buf_.front();
    lidar_end_time_buf_.pop_front();
  }
  if (imu_buf_.empty() || imu_buf_.back().timestamp <= pending_.lidar_end_time) {
    return false;
  }

  while (!imu_buf_.empty()) {
    if (imu_buf_.front().timestamp > pending_.lidar_end_time) {
      break;
    }
    pending_.imu_buf.push_back(std::move(imu_buf_.front()));
    imu_buf_.pop_front();
  }
  measures = std::move(pending_);
  pending_.clear();
  if (measures.imu_buf.size() < 5) {
    continuity_lost_ = true;
    ++insufficient_imu_scans_;
    if (insufficient_imu_scans_ == 1 || insufficient_imu_scans_ % 100 == 0) {
      spdlog::warn(
          "LiDAR update skipped: only {} new IMU samples (need >=5), skipped total={}. "
          "Check IMU delivery/clock or reduce lidar.lidar_merge_hz; configured imu_rate_hz must match the actual stream.",
          measures.imu_buf.size(), insufficient_imu_scans_);
    }
    return false;
  }
  return true;
}

void Synchronizer::stop_accepting() {
  std::lock_guard<std::mutex> lock(mutex_);
  accepting_ = false;
}


}
