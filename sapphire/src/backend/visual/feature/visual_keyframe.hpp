#pragma once
#include <opencv2/features2d.hpp>
#include <unordered_set>

#include "common/visual_frame.hpp"
#include "parameters.h"

namespace sapphire {
// Fill under-covered image cells; no descriptors or geometric depth.
void replenishVisualCorners(const cv::Mat &image, std::vector<cv::Point2f> &points, int limit);

// Mapping-worker owned, one selector per camera. Read-only original LIO poses select frames;
// neither temporal LK nor the selector produces depth or an odometry estimate.
class VisualKeyframeSelector {
 public:
  struct Decision {
    bool selected = false, seed = false, usable = false, visual = false, translation = false, rotation = false, gap = false;
    std::size_t tracks = 0, shared = 0;
    double coverage = 0, renewal = 0, translation_m = -1, rotation_deg = -1, flow_ms = 0;
  };
  explicit VisualKeyframeSelector(const VisualLoopParameters &config, std::size_t camera_id = 0) : config_(config), camera_id_(camera_id) {
    if (camera_id > 1) throw std::invalid_argument("Unknown keyframe camera");
  }
  Decision evaluate(const ImageMeas &image, const std::optional<Eigen::Isometry3d> &pose);
  void accept();  // Only after the selected observation has been prepared.
  void reset();
  void beginSubmap(); // Reset representative reference, retain temporal tracking.

 private:
  VisualLoopParameters config_;
  std::size_t camera_id_;
  std::vector<cv::Point2f> points_;
  std::vector<std::uint64_t> ids_;
  std::vector<cv::Mat> pyramid_;
  std::unordered_set<std::uint64_t> reference_;
  std::optional<Eigen::Isometry3d> reference_pose_, current_pose_;
  double last_time_ = -1, reference_time_ = -1;
  std::uint64_t next_id_ = 1;
  bool selected_ = false;
};
VisualFrame makeImageAttribute(const ImageMeas &image, const CameraParameters &camera, const Eigen::Isometry3d &T_odom_camera);
}  // namespace sapphire
