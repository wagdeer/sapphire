#pragma once

#include <backend/common/common_type.hpp>

#include <algorithm>
#include "backend/sensor/camera/kb4.hpp"
#include <backend/sensor/camera/perspective.hpp>
#include <backend/tools/logging.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace sapphire::backend::camera {

void makeRectificationMaps(const KB4& source, const Mat33_t<double>& rectifiedFromSource,
                           const Mat33_t<double>& rectifiedIntrinsics, cv::Size size, cv::Mat& map1, cv::Mat& map2);

inline std::vector<cv::Rect> cameraImageRois(const std::vector<Perspective>& models) {
  std::vector<cv::Rect> rois;
  rois.reserve(models.size());
  int offsetX = 0;
  for (std::size_t i = 0; i < models.size(); ++i) {
    const cv::Size size = models[i].imageSize();
    EXT_ASSERT_MSG(size.width > 0 && size.height > 0, "{}",
                   fmt::format("Camera model {} has no valid image size ({}x{}).", static_cast<int>(i), size.width, size.height));
    rois.emplace_back(offsetX, 0, size.width, size.height);
    offsetX += size.width;
  }
  return rois;
}

inline cv::Size cameraMosaicSize(const std::vector<Perspective>& models) {
  int width = 0;
  int height = 0;
  for (const Perspective& model : models) {
    const cv::Size size = model.imageSize();
    EXT_ASSERT_MSG(size.width > 0 && size.height > 0, "All camera models must define a positive image size.");
    width += size.width;
    height = std::max(height, size.height);
  }
  return cv::Size(width, height);
}

inline void validateCameraMosaic(const cv::Mat& mosaic, const std::vector<Perspective>& models, const char* name) {
  if (mosaic.empty()) {
    return;
  }
  const cv::Size expected = cameraMosaicSize(models);
  EXT_ASSERT_MSG(mosaic.size() == expected, "{}",
                 fmt::format("{} mosaic size is {}x{}, expected {}x{} from camera models.", name, mosaic.cols, mosaic.rows, expected.width, expected.height));
}

inline int cameraIndexForPoint(const cv::Point2f& point, const std::vector<cv::Rect>& rois) {
  for (std::size_t i = 0; i < rois.size(); ++i) {
    if (point.x >= rois[i].x && point.x < rois[i].x + rois[i].width && point.y >= rois[i].y && point.y < rois[i].y + rois[i].height) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

inline std::vector<cv::Mat> splitCameraMosaic(const cv::Mat& mosaic, const std::vector<Perspective>& models) {
  std::vector<cv::Mat> images;
  if (mosaic.empty()) {
    return images;
  }
  validateCameraMosaic(mosaic, models, "Camera");
  const std::vector<cv::Rect> rois = cameraImageRois(models);
  images.reserve(rois.size());
  for (const cv::Rect& roi : rois) {
    images.push_back(mosaic(roi));
  }
  return images;
}

inline cv::Mat makeCameraMosaic(const std::vector<cv::Mat>& images) {
  if (images.empty()) {
    return cv::Mat();
  }
  const int type = images.front().type();
  int width = 0;
  int height = 0;
  for (const cv::Mat& image : images) {
    EXT_ASSERT_MSG(!image.empty(), "Camera mosaic cannot contain an empty image.");
    EXT_ASSERT_MSG(image.type() == type, "All images in a camera mosaic must have the same OpenCV type.");
    width += image.cols;
    height = std::max(height, image.rows);
  }
  cv::Mat mosaic = cv::Mat::zeros(height, width, type);
  int offsetX = 0;
  for (const cv::Mat& image : images) {
    image.copyTo(mosaic(cv::Rect(offsetX, 0, image.cols, image.rows)));
    offsetX += image.cols;
  }
  return mosaic;
}

cv::Rect imageRoi(const cv::Size& size, const std::vector<float>& ratios);

inline cv::Rect imageRoi(const cv::Mat& image, const std::vector<float>& ratios) { return imageRoi(image.size(), ratios); }

cv::Mat decimateImage(const cv::Mat& image, int factor);

float sampleDepth(const cv::Mat& depth, float x, float y, bool smoothing, float error_ratio = 0.02f, bool estimate_missing = false);

cv::Mat depthToMillimeters(const cv::Mat& depth);

// Binary exclusion mask: 0 is valid, 255 is excluded. Empty means no mask.
cv::Mat loadImageMask(const std::string& path);

void validateImageMask(const cv::Mat& mask, cv::Size size = {});
// Conservative area reduction; any excluded source contribution stays excluded.
cv::Mat resizeImageMask(const cv::Mat& mask, cv::Size size);

}  // namespace sapphire::backend::camera
