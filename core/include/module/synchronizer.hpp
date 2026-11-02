#pragma once
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <deque>
#include <memory>
#include <vector>
#include <Eigen/Core>
#include "sensor/lidar_type.hpp"
#include "common/recovery.hpp"

namespace sapphire {
struct alignas(16) ImuMeas {
  double timestamp;
  Eigen::Vector3d gyro, accel;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
struct alignas(16) MeasGroup {
  double lidar_begin_time = -1.0, lidar_end_time = -1.0;
  std::uint64_t sequence = 0, generation = 0;
  std::deque<ImuMeas> imu_buf;
  std::shared_ptr<std::vector<LidarPoint>> lidar_cloud;
  void clear() { *this = MeasGroup{}; }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
// Waiting never consumes a scan. Invalid consumes one scan and retains its ticket
// for an explicit terminal acknowledgement by the frontend owner.
struct SyncOutcome {
  enum class Kind { Waiting, Ready, Invalid };
  Kind kind = Kind::Waiting;
  RecoveryReason reason = RecoveryReason::None;
};
class Synchronizer {
 public:
  explicit Synchronizer(double max_scan_duration = .11, std::size_t imu_capacity = 4096,
                        double max_imu_gap = .05, double max_lidar_gap = .5);
  bool push_imu(ImuMeas, RecoveryReason* reason = nullptr);
  bool push_lidar(double, std::vector<LidarPoint>&&, std::optional<double> = std::nullopt,
                  std::uint64_t sequence = 0, std::uint64_t generation = 0, RecoveryReason* reason = nullptr);
  bool push_lidar(double stamp, const std::vector<LidarPoint>& cloud, std::optional<double> end = std::nullopt) {
    if (cloud.size() > 16 * 1024 * 1024 / sizeof(LidarPoint)) return false;
    auto owned = cloud;
    return push_lidar(stamp, std::move(owned), end);
  }
  SyncOutcome sync_packages(MeasGroup&);
  // Clears clock baselines and returns every abandoned scan, including pending.
  // It never reopens a stopped synchronizer.
  std::deque<MeasGroup> reset();
  void stop_accepting();
 private:
  std::mutex mutex_;
  std::deque<ImuMeas> imu_buf_;
  std::deque<MeasGroup> lidar_buf_;
  MeasGroup pending_;
  double last_imu_time_ = -std::numeric_limits<double>::infinity();
  double last_lidar_time_ = -std::numeric_limits<double>::infinity();
  double last_used_imu_ = -std::numeric_limits<double>::infinity();
  double last_scan_end_ = -std::numeric_limits<double>::infinity();
  bool accepting_ = true;
  double max_scan_duration_, max_imu_gap_, max_lidar_gap_;
  std::size_t imu_capacity_;
  // Admission is bounded by both count and retained bytes, including pending.
  static constexpr std::size_t max_lidar_scans_ = 32, max_lidar_bytes_ = 16 * 1024 * 1024;
  std::size_t lidar_bytes_ = 0;
};
}  // namespace sapphire
