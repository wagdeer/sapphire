#include "backend/visual/feature/visual_features.hpp"
#include "common/camera/camera.hpp"
#include <opencv2/features2d.hpp>

namespace sapphire {
VisualFrame extractLoopFeatures(const ImageMeas &image, const VisualLoopParameters &config) {
  VisualFrame result(image.timestamp, image.camera_id);
  const auto &camera = config.camera(image.camera_id);
  if (!config.enabled || image.gray.cols != camera.width || image.gray.rows != camera.height) return result;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  cv::ORB::create(config.max_features)->detectAndCompute(image.gray, cv::noArray(), keypoints, descriptors);
  std::vector<cv::Point2f> pixels;
  for (const auto &point : keypoints) pixels.push_back(point.pt);
  if (!valid_camera_distortion(camera)) throw std::invalid_argument("Invalid visual distortion model");
  if (!pixels.empty() && !camera.distortion.empty()) {
    std::vector<cv::Point2f> rectified;
    CameraModel(camera).undistortPixels( pixels, rectified, true);
    pixels = std::move(rectified);
  }
  std::vector<VisualPoint> points;
  cv::Mat valid_descriptors;
  for (std::size_t i = 0; i < pixels.size() && points.size() < std::size_t(config.max_features); ++i) {
    const auto &p = pixels[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0 || p.y < 0 || p.x >= camera.width || p.y >= camera.height) continue;
    points.emplace_back(Eigen::Vector2f(p.x, p.y), keypoints[i].octave, keypoints[i].response);
    valid_descriptors.push_back(descriptors.row(static_cast<int>(i)));
  }
  result.update_features(std::move(points), std::move(valid_descriptors));
  return result;
}

}
