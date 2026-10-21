#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <deque>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <utility>
#include <vector>

namespace sapphire {

struct LidarPoint {
  float x = 0.0f, y = 0.0f, z = 0.0f, intensity = 0.0f;
  float time_offset = 0.0f;
  float operator[](size_t index) const { return index == 0 ? x : index == 1 ? y : z; }
};

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

// Borrowed camera input. Synchronizer validates and owns pixels at admission.
struct ImageMeas {
  double timestamp = -1.0;
  cv::Mat gray;
  std::size_t camera_id = 0;
  std::shared_ptr<const void> owner;

  ImageMeas(double stamp, cv::Mat image, std::shared_ptr<const void> image_owner = {})
      : timestamp(stamp), gray(std::move(image)), owner(std::move(image_owner)) {}

  ImageMeas(const ImageMeas &) = delete;
  ImageMeas &operator=(const ImageMeas &) = delete;
  ImageMeas(ImageMeas &&) noexcept = default;
  ImageMeas &operator=(ImageMeas &&) noexcept = default;
};

}  // namespace sapphire
