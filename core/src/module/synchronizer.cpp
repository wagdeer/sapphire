#include "module/synchronizer.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace sapphire {
Synchronizer::Synchronizer(double duration, std::size_t capacity, double imu_gap, double lidar_gap)
    : max_scan_duration_(duration), max_imu_gap_(imu_gap), max_lidar_gap_(lidar_gap), imu_capacity_(capacity) {
  if (!std::isfinite(duration) || duration <= 0 || !capacity || !std::isfinite(imu_gap) || imu_gap <= 0 ||
      !std::isfinite(lidar_gap) || lidar_gap <= 0) throw std::invalid_argument("Invalid synchronization policy");
}
bool Synchronizer::push_imu(ImuMeas imu, RecoveryReason* reason) {
  const auto reject = [&](RecoveryReason why) { if (reason) *reason = why; return false; };
  if (reason) *reason = RecoveryReason::None;
  if (!std::isfinite(imu.timestamp) || imu.timestamp < 0 || !imu.gyro.allFinite() || !imu.accel.allFinite())
    return reject(RecoveryReason::InvalidImu);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!accepting_) return reject(RecoveryReason::InputUnavailable);
  if (imu.timestamp <= last_imu_time_) return reject(RecoveryReason::ClockDiscontinuity);
  if (imu_buf_.size() >= imu_capacity_) return reject(RecoveryReason::CapacityExceeded);
  last_imu_time_ = imu.timestamp;
  imu_buf_.push_back(std::move(imu));
  return true;
}
bool Synchronizer::push_lidar(double timestamp, std::vector<LidarPoint>&& cloud, std::optional<double> end,
                              std::uint64_t sequence, std::uint64_t generation, RecoveryReason* reason) {
  const auto reject = [&](RecoveryReason why) { if (reason) *reason = why; return false; };
  if (reason) *reason = RecoveryReason::None;
  if (!std::isfinite(timestamp) || timestamp < 0 || cloud.empty() || cloud.capacity() > 16 * 1024 * 1024 / sizeof(LidarPoint))
    return reject(RecoveryReason::InvalidLidar);
  for (const auto& p : cloud)
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(p.intensity) ||
        !std::isfinite(p.time_offset) || p.time_offset < 0 || p.time_offset > max_scan_duration_ + 1e-6)
      return reject(RecoveryReason::InvalidLidar);
  std::stable_sort(cloud.begin(), cloud.end(), [](const LidarPoint& a, const LidarPoint& b) { return a.time_offset < b.time_offset; });
  const double finish = end.value_or(timestamp + cloud.back().time_offset);
  if (!std::isfinite(finish) || finish < timestamp || finish - timestamp > max_scan_duration_ + 1e-6 ||
      timestamp + cloud.back().time_offset > finish + 1e-6) return reject(RecoveryReason::InvalidLidar);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!accepting_) return reject(RecoveryReason::InputUnavailable);
  if (timestamp <= last_lidar_time_) return reject(RecoveryReason::ClockDiscontinuity);
  const auto bytes = cloud.capacity() * sizeof(LidarPoint);
  if (lidar_buf_.size() + bool(pending_.lidar_cloud) >= max_lidar_scans_ || bytes > max_lidar_bytes_ - lidar_bytes_)
    return reject(RecoveryReason::CapacityExceeded);
  MeasGroup scan;
  scan.lidar_begin_time = timestamp; scan.lidar_end_time = finish;
  scan.sequence = sequence; scan.generation = generation;
  scan.lidar_cloud = std::make_shared<std::vector<LidarPoint>>(std::move(cloud));
  last_lidar_time_ = timestamp;
  lidar_bytes_ += bytes;
  lidar_buf_.push_back(std::move(scan));
  return true;
}
SyncOutcome Synchronizer::sync_packages(MeasGroup& measures) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!pending_.lidar_cloud) {
    if (lidar_buf_.empty()) return {};
    pending_ = std::move(lidar_buf_.front()); lidar_buf_.pop_front();
  }
  if (imu_buf_.empty() || imu_buf_.back().timestamp <= pending_.lidar_end_time) return {};
  while (!imu_buf_.empty() && imu_buf_.front().timestamp <= pending_.lidar_end_time) {
    pending_.imu_buf.push_back(std::move(imu_buf_.front())); imu_buf_.pop_front();
  }
  lidar_bytes_ -= pending_.lidar_cloud->capacity() * sizeof(LidarPoint);
  measures = std::move(pending_); pending_.clear();
  const auto invalid = [](RecoveryReason why) { return SyncOutcome{SyncOutcome::Kind::Invalid, why}; };
  if (measures.imu_buf.size() < 5) return invalid(RecoveryReason::ImuCoverage);
  if (std::isfinite(last_scan_end_) && measures.lidar_begin_time - last_scan_end_ > max_lidar_gap_)
    return invalid(RecoveryReason::LidarGap);
  double previous = last_used_imu_;
  for (const auto& imu : measures.imu_buf) {
    if (std::isfinite(previous) && imu.timestamp - previous > max_imu_gap_) return invalid(RecoveryReason::ImuGap);
    previous = imu.timestamp;
  }
  if (measures.imu_buf.front().timestamp > measures.lidar_begin_time + max_imu_gap_ ||
      measures.lidar_end_time - previous > max_imu_gap_ || imu_buf_.front().timestamp - previous > max_imu_gap_)
    return invalid(RecoveryReason::ImuCoverage);
  last_used_imu_ = previous; last_scan_end_ = measures.lidar_end_time;
  return {SyncOutcome::Kind::Ready, RecoveryReason::None};
}
std::deque<MeasGroup> Synchronizer::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::deque<MeasGroup> dropped;
  if (pending_.lidar_cloud) dropped.push_back(std::move(pending_));
  pending_.clear();
  while (!lidar_buf_.empty()) { dropped.push_back(std::move(lidar_buf_.front())); lidar_buf_.pop_front(); }
  imu_buf_.clear();
  lidar_bytes_ = 0;
  last_imu_time_ = last_lidar_time_ = last_used_imu_ = last_scan_end_ = -std::numeric_limits<double>::infinity();
  return dropped;
}
void Synchronizer::stop_accepting() { std::lock_guard<std::mutex> lock(mutex_); accepting_ = false; }
}  // namespace sapphire
