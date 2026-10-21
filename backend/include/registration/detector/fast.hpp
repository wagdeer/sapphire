#pragma once
#include <array>
#include <cstdint>
#include <opencv2/core.hpp>
#include <vector>

namespace sapphire::backend::feature {
class FastOrb;

class FastDetector {
 public:
  explicit FastDetector(FastOrb& orb);
  void detect(const cv::_InputArray& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask = {});

 private:
  FastOrb& orb_;

  struct LevelWorkspace {
    std::vector<std::vector<cv::KeyPoint>> cell_keypoints;
    std::vector<cv::Rect> cell_bounds;
    std::vector<int> cell_to_retain;
    std::vector<int> cell_total;
    std::vector<std::uint8_t> cell_exhausted;
    std::vector<cv::KeyPoint> harris_raw_keypoints;
    std::vector<cv::KeyPoint> harris_fallback_keypoints;
    cv::Mat harris_fast_scores;
    std::array<std::vector<std::uint8_t>, 3> primary_fast_score_rows;
    std::array<std::vector<int>, 3> primary_fast_corner_positions;
  };

  void compute_harris_keypoints(std::vector<std::vector<cv::KeyPoint>>& keypoints) const;
  void compute_orientations(std::vector<std::vector<cv::KeyPoint>>& keypoints) const;
  void correct_keypoint_scale(std::vector<cv::KeyPoint>& keypoints, unsigned int level) const;

  std::vector<unsigned int> features_per_level_;
  mutable std::vector<LevelWorkspace> level_workspaces_;
  std::vector<std::vector<cv::KeyPoint>> all_keypoints_workspace_;
};
}  // namespace sapphire::backend::feature
