#include "backend/common/sensor_data.hpp"

#include <algorithm>
#include <backend/tools/logging.hpp>
#include <utility>

#include "backend/common/state_group.hpp"
#include "backend/maplayer/navimap/local_grid.hpp"
#include "backend/sensor/camera/utils.hpp"

namespace sapphire::backend {

SensorData::SensorData() = default;

// Mono constructor
SensorData::SensorData(const cv::Mat& image, const camera::Perspective& perspective, int id, double stamp, const cv::Mat& userData)
    : SensorData(image, cv::Mat{}, perspective, id, stamp, userData) {}

SensorData::SensorData(const cv::Mat& rgb, const cv::Mat& depth, const camera::Perspective& perspective, int id, double stamp,
                       const cv::Mat& userData)
    : _id(id), _stamp(stamp) {
  setImageData(rgb, depth, perspective);
  setUserData(userData);
}

SensorData::SensorData(const cv::Mat& rgb, const cv::Mat& depth, const std::vector<camera::Perspective>& perspectives, int id, double stamp,
                       const cv::Mat& userData)
    : _id(id), _stamp(stamp) {
  setImageData(rgb, depth, perspectives);
  setUserData(userData);
}

cv::Mat SensorData::imageRaw(std::size_t cameraIndex) const {
  if (_imageRaw.empty()) {
    return cv::Mat();
  }
  return _imageRaw(camera::cameraImageRois(perspectives()).at(cameraIndex));
}

cv::Mat SensorData::depthRaw(std::size_t cameraIndex) const {
  if (_depthRaw.empty()) {
    return cv::Mat();
  }
  return _depthRaw(camera::cameraImageRois(perspectives()).at(cameraIndex));
}

void SensorData::setPerspective(const camera::Perspective& model) { _visualFrame.setPerspective(model); }

void SensorData::setPerspectives(const std::vector<camera::Perspective>& models) { _visualFrame.setPerspectives(models); }

void SensorData::setImageData(const cv::Mat& image, const cv::Mat& depth, const camera::Perspective& camera, bool clearPreviousData) {
  setImageData(image, depth, std::vector<camera::Perspective>{camera}, clearPreviousData);
}

void SensorData::setImageData(const cv::Mat& rgb, const cv::Mat& depth, std::vector<camera::Perspective> models, bool clearPreviousData) {
  const cv::Mat nextImage = !clearPreviousData && rgb.empty() ? _imageRaw : rgb;
  const cv::Mat nextDepth = !clearPreviousData && depth.empty() ? _depthRaw : depth;
  const cv::Mat* reference = !nextImage.empty() ? &nextImage : !nextDepth.empty() ? &nextDepth : nullptr;
  EXT_ASSERT_MSG(!reference || !models.empty(), "Camera models are required for image/depth observations.");
  if (reference && std::all_of(models.begin(), models.end(), [](const auto& model) { return model.imageSize() == cv::Size(); }) &&
      reference->cols % models.size() == 0) {
    const cv::Size size(reference->cols / static_cast<int>(models.size()), reference->rows);
    for (auto& model : models) model.setImageSize(size);
  }
  if (!nextImage.empty()) {
    EXT_ASSERT_MSG(nextImage.type() == CV_8UC1, "SensorData accepts only CV_8UC1 grayscale images.");
    camera::validateCameraMosaic(nextImage, models, "Grayscale");
  }
  if (!nextDepth.empty()) {
    EXT_ASSERT_MSG(nextDepth.type() == CV_16UC1, "SensorData depth must be CV_16UC1 millimeters; normalize input with prepareSensorData.");
    camera::validateCameraMosaic(nextDepth, models, "Depth");
    EXT_ASSERT_MSG(nextImage.empty() || nextImage.size() == nextDepth.size(), "Image and registered depth must have the same canvas size.");
  }
  if (clearPreviousData) {
    stereo_right_.release();
    feature_mask_.release();
    stereo_mask_.release();
  }
  _imageRaw = nextImage;
  _depthRaw = nextDepth;
  _visualFrame.setPerspectives(std::move(models));
  _visualFrame.setImageBoundary(reference ? reference->size() : cv::Size());
}

bool SensorData::isValid() const {
  return (lidar_cloud && !lidar_cloud->empty()) || _id != 0 || _stamp != 0 || !_imageRaw.empty() || !_depthRaw.empty() || !stereo_right_.empty() ||
         !_userDataRaw.empty() || !perspectives().empty() || !_visualFrame.keypoints().empty() || !_visualFrame.descriptors().empty() || _localGrid ||
         !_envSensors.empty() || !_globalDescriptors.empty() || !globalPose_.isNull();
}

unsigned long SensorData::getMemoryUsed() const {
  const auto bytes = [](const cv::Mat& m) { return m.total() * m.elemSize(); };
  unsigned long total = (lidar_cloud ? lidar_cloud->capacity() * sizeof(GaussianPoint) : 0) + sizeof(SensorData) + bytes(_imageRaw) +
                        bytes(_depthRaw) + bytes(stereo_right_) + bytes(feature_mask_) + bytes(stereo_mask_) + bytes(_userDataRaw) +
                        _visualFrame.getMemoryUsed() - sizeof(VisualFrame) + _envSensors.size() * sizeof(EnvSensors::value_type) +
                        _globalDescriptors.capacity() * sizeof(GlobalDescriptor);
  total += visual_camera_ids.capacity() * sizeof(int64_t);
  for (const auto& descriptor : _globalDescriptors) total += bytes(descriptor.data()) + bytes(descriptor.info());
  if (_localGrid) total += _localGrid->memoryUsed();
  return total;
}

void SensorData::clearRawData(bool images, bool userData, bool occupancyGrid) {
  if (images) {
    stereo_right_.release();
    feature_mask_.release();
    stereo_mask_.release();
    visual_camera_ids.clear();
    _imageRaw.release();
    _depthRaw.release();
    _visualFrame.setImageBoundary({});
  }
  if (userData) _userDataRaw.release();
  if (occupancyGrid) _localGrid.reset();
}

}  // namespace sapphire::backend
