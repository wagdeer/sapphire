#include "frontend/common/synchronizer.hpp"
#include "frontend/common/image_preprocess.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <spdlog/spdlog.h>

namespace sapphire {
namespace {
constexpr double stereo_tolerance = .005;
std::size_t imageBytes(const ImageMeas &image) { return image.gray.total() * image.gray.elemSize(); }
double groupStamp(const std::deque<ImageMeas> &images) {
  return std::max(images.front().timestamp, images.back().timestamp);
}
}  // namespace

Synchronizer::Synchronizer(double max_scan_duration, std::size_t imu_capacity, ImageSyncParameters images)
    : image_config_(std::move(images)), last_image_(image_config_.cameras.size(), -std::numeric_limits<double>::infinity()),
      last_sampled_(last_image_), max_scan_duration_(max_scan_duration), imu_capacity_(imu_capacity) {
  if (!imu_capacity || !std::isfinite(max_scan_duration) || max_scan_duration <= 0)
    throw std::invalid_argument("Invalid LiDAR/IMU synchronization limits");
  const auto &p = image_config_;
  if (p.cameras.size() > 4 || (p.stereo ? p.cameras.size() < 2 : p.cameras.size() > 1) ||
      !std::isfinite(p.interval) || p.interval <= 0 || !p.capacity || (p.stereo && p.capacity < 2) || !p.byte_budget)
    throw std::invalid_argument("Invalid image synchronization policy");
  // Camera dimensions/calibration are normalized once at configuration setup.

}

bool Synchronizer::push_image(ImageMeas image) {
  // Independent lock: image conversion/ownership never holds the IMU/LiDAR lock.
  std::lock_guard<std::mutex> lock(image_mutex_);
  const auto id = image.camera_id;
  if (!image_accepting_ || id >= image_config_.cameras.size()) return false;
  const auto &camera = image_config_.cameras[id];
  const double stamp = image.timestamp + camera.time_offset;
  const std::size_t bytes = std::size_t(camera.width) * camera.height;
  if (!std::isfinite(stamp) || stamp <= last_image_[id] ||
      bytes > image_config_.byte_budget) return false;
  if (!preprocessImage(image, camera)) return false;
  last_image_[id] = stamp;
  if (image_config_.stereo && id < 2) {
    stereo_pending_[id].push_back(std::move(image));
    ++image_stats_.frames; image_stats_.bytes += bytes;
    collect_stereo();
  } else {
    if (stamp - last_sampled_[id] < image_config_.interval) { ++image_stats_.sampled; return true; }
    last_sampled_[id] = stamp;
    std::deque<ImageMeas> group;
    group.push_back(std::move(image));
    const auto pos = std::upper_bound(ready_images_.begin(), ready_images_.end(), stamp,
        [](double time, const auto &value) { return time < groupStamp(value); });
    ready_images_.insert(pos, std::move(group));
    ++image_stats_.frames; image_stats_.bytes += bytes;
  }
  bound_images();
  return true;
}

void Synchronizer::collect_stereo() {
  while (!stereo_pending_[0].empty() && !stereo_pending_[1].empty()) {
    const double left = stereo_pending_[0].front().timestamp, right = stereo_pending_[1].front().timestamp;
    if (std::abs(left - right) > stereo_tolerance) {
      auto &queue = stereo_pending_[left < right ? 0 : 1];
      --image_stats_.frames; image_stats_.bytes -= imageBytes(queue.front());
      ++image_stats_.unmatched; queue.pop_front();
      continue;
    }
    const double stamp = std::max(left, right);
    std::deque<ImageMeas> group;
    for (auto &queue : stereo_pending_) { group.push_back(std::move(queue.front())); queue.pop_front(); }
    if (stamp - last_sampled_[0] < image_config_.interval) {
      image_stats_.frames -= 2;
      for (const auto &image : group) image_stats_.bytes -= imageBytes(image);
      image_stats_.sampled += 2;
      continue;
    }
    last_sampled_[0] = last_sampled_[1] = stamp;
    const auto pos = std::upper_bound(ready_images_.begin(), ready_images_.end(), stamp,
        [](double time, const auto &value) { return time < groupStamp(value); });
    ready_images_.insert(pos, std::move(group));
  }
}

void Synchronizer::bound_images() {
  while (image_stats_.frames > image_config_.capacity || image_stats_.bytes > image_config_.byte_budget) {
    double oldest = ready_images_.empty() ? std::numeric_limits<double>::infinity() : groupStamp(ready_images_.front());
    int pending = -1;
    for (int id = 0; id < 2; ++id) if (!stereo_pending_[id].empty() && stereo_pending_[id].front().timestamp < oldest) {
      oldest = stereo_pending_[id].front().timestamp; pending = id;
    }
    if (pending >= 0) {
      auto &queue = stereo_pending_[pending];
      --image_stats_.frames; image_stats_.bytes -= imageBytes(queue.front()); ++image_stats_.evicted;
      queue.pop_front();
    } else {
      for (const auto &image : ready_images_.front()) {
        --image_stats_.frames; image_stats_.bytes -= imageBytes(image); ++image_stats_.evicted;
      }
      ready_images_.pop_front();
    }
  }
}

std::deque<ImageMeas> Synchronizer::take_images() {
  std::lock_guard<std::mutex> lock(image_mutex_);
  std::deque<ImageMeas> result;
  for (auto &group : ready_images_) for (auto &image : group) {
    --image_stats_.frames;
    image_stats_.bytes -= imageBytes(image);
    result.push_back(std::move(image));
  }
  ready_images_.clear();
  return result;
}

void Synchronizer::clear_images() {
  std::lock_guard<std::mutex> lock(image_mutex_);
  ready_images_.clear();
  for (auto &queue : stereo_pending_) queue.clear();
  image_stats_.frames = image_stats_.bytes = 0;
  std::fill(last_image_.begin(), last_image_.end(), -std::numeric_limits<double>::infinity());
  last_sampled_ = last_image_;
}

Synchronizer::ImageStats Synchronizer::image_stats() const {
  std::lock_guard<std::mutex> lock(image_mutex_);
  return image_stats_;
}

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
  { std::lock_guard<std::mutex> lock(mutex_); accepting_ = false; }
  { std::lock_guard<std::mutex> lock(image_mutex_); image_accepting_ = false; }
}


}
