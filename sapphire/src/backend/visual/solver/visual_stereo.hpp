#pragma once
#include <array>
#include <deque>

#include "backend/visual/feature/visual_features.hpp"
#include "backend/visual/feature/visual_keyframe.hpp"

namespace sapphire {
// Camera-local geometry from calibrated simultaneous rays, independent of odometry.
std::optional<VisualGeometry> triangulateStereo(const Eigen::Vector2d &left, const Eigen::Vector2d &right, const Eigen::Isometry3d &T_left_right,
                                                const CameraParameters &left_camera, const CameraParameters &right_camera);

// Sole owner: mapping worker. Bounded pairing and local descriptor reference only.
// Optional lightweight flow selects keyframes only. No temporal triangulation,
// global covisibility or odometry updates.
class StereoVisualProcessor {
 public:
  struct Result {
    std::array<VisualFrame, 2> frames;
    std::size_t matches = 0, metric = 0;
    bool usable = false, submap_candidate = false;
    bool keyframe = true;
    VisualKeyframeSelector::Decision selection;
    double total_ms = 0, descriptor_ms = 0, stereo_ms = 0;
  };
  explicit StereoVisualProcessor(VisualLoopParameters config);
  std::optional<Result> push(ImageMeas image, std::optional<Eigen::Isometry3d> pose = {});
  void beginSubmap();
  void reset();
  std::size_t dropped() const { return dropped_; }
  std::size_t pending() const { return pending_[0].size() + pending_[1].size(); }

 private:
  struct Pending {
    ImageMeas image;
    std::optional<Eigen::Isometry3d> pose;
  };
  Result process(Pending left, Pending right);
  VisualLoopParameters config_;
  VisualKeyframeSelector selector_;
  Eigen::Isometry3d T_left_right_;
  std::array<Eigen::Matrix3d, 2> rectification_;
  std::array<cv::Mat, 2> map_x_, map_y_, valid_patch_;
  std::array<double, 4> rectified_intrinsics_{};
  double disparity_sign_ = 1;
  void prepareRectification();
  std::array<std::deque<Pending>, 2> pending_;
  std::array<double, 2> last_{{-1, -1}};
  std::array<std::optional<VisualFrame>, 2> reference_;
  std::uint64_t next_id_ = 1;
  std::size_t dropped_ = 0;
  double novelty_since_ = -1;
};
}  // namespace sapphire
