#include <Eigen/Cholesky>
#include <optional>
#include <stdexcept>

#include "backend/sensor/lidar/geometry.hpp"
#pragma once

#include <backend/common/env_frame.hpp>
#include <backend/common/state_group.hpp>
#include <backend/common/visual_frame.hpp>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <opencv2/core/core.hpp>
#include <utility>
#include <vector>

namespace sapphire::backend {

struct LocalGrid;

class SensorData {
 public:
  // Transient physical camera IDs in image-layout order; archive calibration stays self-contained.
  std::vector<int64_t> visual_camera_ids;

  // Frozen Gaussian geometry in this observation/node base frame.
  GaussianCloudPtr lidar_cloud;

  SensorData();

  // Transient image-space exclusion masks; generated features, not masks, are archived.
  void set_feature_masks(cv::Mat left, cv::Mat right = {}) {
    feature_mask_ = std::move(left);
    stereo_mask_ = std::move(right);
  }

  const cv::Mat& feature_mask() const { return feature_mask_; }

  const cv::Mat& stereo_mask() const { return stereo_mask_; }

  // Transient rectified right image, never part of the map archive.
  void set_stereo_right(cv::Mat image) { stereo_right_ = std::move(image); }

  const cv::Mat& stereo_right() const { return stereo_right_; }

  SensorData(const cv::Mat& image, const camera::Perspective& perspective, int id = 0, double stamp = 0.0, const cv::Mat& userData = cv::Mat());

  SensorData(const cv::Mat& rgb, const cv::Mat& depth, const camera::Perspective& perspective, int id = 0, double stamp = 0.0,
             const cv::Mat& userData = cv::Mat());

  // Runtime visual frame with its calibration list; depth is optional.
  SensorData(const cv::Mat& image, const cv::Mat& depth, const std::vector<camera::Perspective>& cameras, int id = 0, double stamp = 0.0,
             const cv::Mat& userData = {});

  SensorData(const SensorData&) = default;
  SensorData(SensorData&&) noexcept = default;
  SensorData& operator=(const SensorData&) = default;
  SensorData& operator=(SensorData&&) noexcept = default;

  bool isValid() const;

  int id() const { return _id; }

  void setId(int id) { _id = id; }

  double stamp() const { return _stamp; }

  void setStamp(double stamp) { _stamp = stamp; }

  const cv::Mat& imageRaw() const { return _imageRaw; }
  cv::Mat imageRaw(std::size_t cameraIndex) const;

  const cv::Mat& depthRaw() const { return _depthRaw; }
  cv::Mat depthRaw(std::size_t cameraIndex) const;

  // Update one runtime observation. Image and depth may arrive independently;
  // the optional stereo-right image is transient. This is not an N-RGB-D input.
  // Raw depth is CV_16UC1 millimeters; input conversion belongs to preprocess.
  void setImageData(const cv::Mat& image, const cv::Mat& depth, const camera::Perspective& camera, bool clearPreviousData = true);
  void setImageData(const cv::Mat& image, const cv::Mat& depth, std::vector<camera::Perspective> cameras, bool clearPreviousData = true);

  void setPerspective(const camera::Perspective& model);

  void setPerspectives(const std::vector<camera::Perspective>& models);

  const std::vector<camera::Perspective>& perspectives() const { return _visualFrame.perspectives(); }

  VisualFrame& visualFrame() { return _visualFrame; }
  const VisualFrame& visualFrame() const { return _visualFrame; }

  void setUserData(const cv::Mat& userData) { _userDataRaw = userData; }
  const cv::Mat& userDataRaw() const { return _userDataRaw; }

  // Optional navigation result; LocalGrid owns its geometry and metadata.
  void setLocalGrid(std::shared_ptr<const LocalGrid> grid) { _localGrid = std::move(grid); }
  const std::shared_ptr<const LocalGrid>& localGrid() const noexcept { return _localGrid; }

  void addGlobalDescriptor(const GlobalDescriptor& descriptor) { _globalDescriptors.push_back(descriptor); }

  void setGlobalDescriptors(const std::vector<GlobalDescriptor>& descriptors) { _globalDescriptors = descriptors; }

  void clearGlobalDescriptors() { _globalDescriptors.clear(); }

  const std::vector<GlobalDescriptor>& globalDescriptors() const { return _globalDescriptors; }

  void setGlobalPose(const StateGroupf& pose, const std::optional<Mat66_t<double>>& covariance) {
    if (covariance &&
        (!covariance->allFinite() || !covariance->isApprox(covariance->transpose(), 1e-8) || covariance->llt().info() != Eigen::Success))
      throw std::invalid_argument("Global pose covariance must be finite, symmetric and positive definite");
    globalPose_ = pose;
    globalPoseCovariance_ = covariance;
  }

  const StateGroupf& globalPose() const { return globalPose_; }

  const std::optional<Mat66_t<double>>& globalPoseCovariance() const { return globalPoseCovariance_; }

  void setEnvSensors(const EnvSensors& sensors) { _envSensors = sensors; }

  void addEnvSensor(const EnvSensor& sensor) { _envSensors.insert(std::make_pair(sensor.type(), sensor)); }

  const EnvSensors& envSensors() const { return _envSensors; }

  unsigned long getMemoryUsed() const;
  void clearRawData(bool images = true, bool userData = true, bool occupancyGrid = true);

 private:
  int _id = 0;          ///< Unique sensor data ID (0 if invalid)
  double _stamp = 0.0;  ///< Timestamp in seconds

  cv::Mat feature_mask_, stereo_mask_;
  cv::Mat stereo_right_;
  cv::Mat _imageRaw;  // CV_8UC1 grayscale, including single-row images.
  cv::Mat _depthRaw;  // CV_16UC1 millimeters.
  cv::Mat _userDataRaw;
  VisualFrame _visualFrame;
  std::shared_ptr<const LocalGrid> _localGrid;

  EnvSensors _envSensors;

  std::vector<GlobalDescriptor> _globalDescriptors;

  StateGroupf globalPose_;
  std::optional<Mat66_t<double>> globalPoseCovariance_;
};

}  // namespace sapphire::backend
