#pragma once
#include <array>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include "frontend/common/measurement.hpp"
#include "app/parameters.h"

namespace sapphire {
// Sensor input policy, independent of backend keyframe/retrieval settings.
struct ImageSyncParameters {
  std::vector<CameraParameters> cameras; // setup-validated, uniform algorithm dimensions
  bool stereo = false; // pair only IDs 0/1; IDs >=2 remain independent
  double interval = .05;
  std::size_t capacity = 64;
  std::size_t byte_budget = 32 * 1024 * 1024;
};

class Synchronizer {
 public:
  explicit Synchronizer(double max_scan_duration = 0.11, std::size_t imu_capacity = 4096,
                        ImageSyncParameters images = {});
  bool push_image(ImageMeas image);
  // Drain ready observations by delivery, independently of the LiDAR clock.
  // Primary stereo members retain their camera-local stamps and leave together.
  std::deque<ImageMeas> take_images();
  void clear_images();
  struct ImageStats {
    std::size_t frames = 0, bytes = 0;
    std::uint64_t evicted = 0, sampled = 0, unmatched = 0;
  };
  ImageStats image_stats() const;
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
  void collect_stereo(); // image_mutex_ held
  void bound_images();
  mutable std::mutex image_mutex_;
  ImageSyncParameters image_config_;
  std::vector<double> last_image_, last_sampled_;
  std::array<std::deque<ImageMeas>, 2> stereo_pending_;
  std::deque<std::deque<ImageMeas>> ready_images_; // one mono image or one complete stereo pair
  ImageStats image_stats_;
  bool image_accepting_ = true;
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
