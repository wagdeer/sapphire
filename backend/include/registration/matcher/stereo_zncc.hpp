#pragma once
#include <array>
#include <backend/sensor/camera/perspective.hpp>
#include <opencv2/core.hpp>
#include <vector>
namespace sapphire::backend {
// Coarse-to-fine epipolar ZNCC; no right-image detector or descriptor.
class StereoZncc {
 public:
  std::vector<cv::Point3f> match(const cv::Mat& left, const cv::Mat& right, const std::vector<cv::KeyPoint>& points, const camera::Perspective& camera,
                                 const cv::Mat& left_mask = {}, const cv::Mat& right_mask = {});

 private:
  std::array<cv::Mat, 3> epipolar_left_, epipolar_right_, epipolar_left_mask_, epipolar_right_mask_;
  cv::Mat epipolar_mask_workspace_;
  std::vector<float> epipolar_scores_;
};
}  // namespace sapphire::backend
