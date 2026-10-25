#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>

#include <Eigen/Core>
#include <deque>
#include <memory>
#include <vector>
#include "sensor/lidar_type.hpp"

namespace sapphire {

struct alignas(16) ImuMeas {
  double timestamp;
  Eigen::Vector3d gyro;
  Eigen::Vector3d accel;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct alignas(16) MeasGroup {
  double lidar_begin_time = -1.0;
  double lidar_end_time = -1.0;
  std::deque<ImuMeas> imu_buf;
  std::shared_ptr<std::vector<LidarPoint>> lidar_cloud;
  void clear() {
    lidar_begin_time = -1.0;
    lidar_end_time = -1.0;
    imu_buf.clear();
    lidar_cloud.reset();
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class Synchronizer {
 public:
  explicit Synchronizer(double max_scan_duration = 0.11, std::size_t imu_capacity = 4096);
  bool push_imu(ImuMeas imu);
  bool push_lidar(double timestamp, std::vector<LidarPoint> &&cloud, std::optional<double> end_timestamp = std::nullopt);
  bool push_lidar(double timestamp, const std::vector<LidarPoint> &cloud, std::optional<double> end_timestamp = std::nullopt) {
    if (cloud.size() > 16 * 1024 * 1024 / sizeof(LidarPoint)) return false;
    auto owned = cloud;
    return push_lidar(timestamp, std::move(owned), end_timestamp);
  }
  bool sync_packages(MeasGroup &measures);
  void stop_accepting();
  bool takeContinuityLoss() {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool lost = continuity_lost_;
    continuity_lost_ = false;
    return lost;
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

}  // namespace sapphire
