#pragma once
#include <opencv2/core.hpp>
#include <vector>

namespace sapphire::backend::feature {
class FastOrb;

// Intensity-centroid orientation and rotated BRIEF-256 description. Detection
// owns corner selection/scoring; the borrowed FastOrb owns image preparation.
class OrbDescriptor {
 public:
  static constexpr int patch_size = 31;
  static constexpr int support_radius = 21;  // Rotated sampling plus Gaussian support.

  explicit OrbDescriptor(FastOrb& orb) : orb_(orb) {}
  // Unblurred pyramid-level coordinates; callers must validate patch support.
  static float orientation(const cv::Mat& image, const cv::Point2f& point);

  // Filters support and sets orientations. With noArray(), skip descriptor bytes.
  void compute(const cv::_InputArray& image, std::vector<cv::KeyPoint>& keypoints, const cv::_OutputArray& descriptors, const cv::Mat& mask = {},
               cv::Mat* storage = nullptr);

 private:
  // Uses a blurred level and its local keypoint; always writes exactly 32 bytes.
  static void write_descriptor(const cv::KeyPoint& keypoint, const cv::Mat& image, uchar* descriptor);

  FastOrb& orb_;
  std::vector<cv::KeyPoint> filtered_keypoints_, local_keypoints_;
  std::vector<unsigned int> descriptor_levels_;
  std::vector<bool> populated_levels_;
};
}  // namespace sapphire::backend::feature
