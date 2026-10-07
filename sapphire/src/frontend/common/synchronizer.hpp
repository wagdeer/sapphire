#pragma once
#include <mutex>
#include <optional>
#include "common/common.hpp"

namespace sapphire {
class Synchronizer {
 public:
  explicit Synchronizer(double max_scan_duration = 0.11, std::size_t imu_capacity = 4096) : max_scan_duration_(max_scan_duration), imu_capacity_(imu_capacity) {
    if (!imu_capacity || !std::isfinite(max_scan_duration) || max_scan_duration <= 0.0) {
      throw std::invalid_argument("max_scan_duration must be finite and positive");
    }
  }
  bool push_imu(ImuMeas imu);
  bool push_lidar(double timestamp, std::vector<LidarPoint> &&cloud, std::optional<double> end_timestamp = std::nullopt);
  bool push_lidar(double timestamp, const std::vector<LidarPoint> &cloud, std::optional<double> end_timestamp = std::nullopt) {
    if (cloud.size() > 16 * 1024 * 1024 / sizeof(LidarPoint)) return false;
    auto owned = cloud; return push_lidar(timestamp, std::move(owned), end_timestamp);
  }
  bool sync_packages(MeasGroup &measures);
  void stop_accepting();
  bool takeContinuityLoss() {
    std::lock_guard<std::mutex> lock(mutex_); const bool lost = continuity_lost_; continuity_lost_ = false; return lost;
  }

 private:
  std::mutex mutex_;
  std::deque<ImuMeas> imu_buf_;
  std::deque<std::shared_ptr<std::vector<LidarPoint>>> lidar_buf_;
  std::deque<double> lidar_time_buf_;
  std::deque<double> lidar_end_time_buf_;
  MeasGroup pending_;
  double last_imu_time_ = -std::numeric_limits<double>::infinity();
  double last_lidar_time_ = -std::numeric_limits<double>::infinity();
  bool accepting_ = true, continuity_lost_ = false;
  double max_scan_duration_;
  size_t insufficient_imu_scans_ = 0;
  std::size_t imu_capacity_;
};

}
