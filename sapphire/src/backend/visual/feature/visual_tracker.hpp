#pragma once

#include <array>
#include <memory>
#include <optional>

#include "common/visual_frame.hpp"
#include "parameters.h"

namespace sapphire {

struct VisualTrackingParameters {
  int max_features = 600;
  int grid_cols = 8, grid_rows = 6;
  int pyramid_levels = 3;
  int min_tracks = 40;
  int mature_age = 3;
  double max_gap_s = .3;
  double min_feature_distance_px = 5;
  double max_forward_backward_px = 1;
  double max_patch_error = 30;
  double min_coverage = .35;
  double renewal_threshold = .5;
  double min_keyframe_interval_s = .5;
  double observation_interval_s = 1;
  double observation_displacement_px = 8;
  double min_submap_interval_s = 1;
  double submap_displacement_px = 80;
  double min_baseline_m = .1;
  double min_parallax_rad = .017453292519943295;
  double max_reprojection_px = 2;
  double max_depth_m = 80;
  int max_anchors = 3;
  int max_match_visits = 16384;
  int min_anchor_matches = 20;
  double projection_radius_px = 6;
  double max_anchor_reprojection_px = 3;
  int max_hamming_distance = 64;
  double match_ratio = .8;
  double min_affiliated_fraction = .45;
  double novelty_duration_s = .3;
  double recovery_duration_s = 1;
};

struct VisualRayObservation {
  double timestamp = -1;
  Eigen::Isometry3d T_odom_camera = Eigen::Isometry3d::Identity();
  Eigen::Vector2d normalized_pixel = Eigen::Vector2d::Zero();
};

// Three independent times from one optical camera; known metric poses are input
// only. Returns current-camera geometry, checked against all three observations.
std::optional<VisualGeometry> triangulateVisualTrack(const std::array<VisualRayObservation, 3> &observations,
    double fx, double fy, const VisualTrackingParameters &parameters);

// Original metric body poses only; never extrapolates. T_body_camera includes
// the calibrated optical-camera extrinsic, not a point-cloud depth association.
std::optional<Eigen::Isometry3d> interpolateVisualCameraPose(double image_time,
    double before_time, const Eigen::Isometry3d &before_body,
    double after_time, const Eigen::Isometry3d &after_body,
    const Eigen::Isometry3d &T_body_camera, double max_gap_s = .25);

struct VisualTrackingStats {
  std::size_t tracks = 0, mature_tracks = 0, new_tracks = 0, flow_rejected = 0;
  std::size_t triangulated = 0, reference_tracks = 0, shared_tracks = 0;
  double coverage = 0, all_point_coverage = 0, reference_retention = 0, renewal = 0, spatial_retention = 0;
  double reference_displacement_px = 0;
  double tracking_ms = 0, descriptor_ms = 0;
  bool reset_by_gap = false, reference_lost = false, usable = false, submap_candidate = false;
  bool raw_submap_candidate = false, affiliated = false, internal_keyframe = false, recovering = false, match_budget_exhausted = false;
  std::size_t anchors = 0, anchor_points = 0, projection_tests = 0, projected = 0, match_visits = 0;
  std::size_t associated = 0, reacquired = 0, best_anchor_matches = 0;
  double affiliation_coverage = 0, unexplained_fraction = 0, affiliation_ms = 0;
  std::uint64_t reference_anchor = 0;
};

struct VisualAssociation {
  std::uint64_t track_id = 0;  // current uninterrupted temporal segment
  std::uint64_t anchor_id = 0;
  std::uint64_t landmark_track_id = 0;  // feature identity in that retained anchor
};

// One camera, one mapping-worker owner. Images/pyramids/tracks are bounded;
// no thread, odometry estimator, scene archive or independent descriptor detector.
class VisualTracker final {
 public:
  struct Result {
    VisualTrackingStats stats;
    std::optional<VisualFrame> keyframe;
    std::vector<VisualAssociation> associations;
  };
  VisualTracker(CameraParameters camera, std::size_t camera_id, VisualTrackingParameters parameters = {});
  ~VisualTracker();
  VisualTracker(const VisualTracker &) = delete;
  VisualTracker &operator=(const VisualTracker &) = delete;
  Result process(const ImageMeas &image, std::optional<Eigen::Isometry3d> T_odom_camera = {});
  // Call only when the owner accepts a submap boundary. Intermediate descriptor
  // observations do not move the submap overlap reference.
  void acceptSubmapReference();
  // Actual producer boundary, including resource/tail cuts without a current
  // described image. Clear local coverage, retain continuous tracks/geometry
  // history; the next usable observation seeds the new submap's anchors.
  void beginSubmap();
  // Detached, bounded representative observations, including geometry learned
  // later by continuous tracks. Export before accepting a new submap reference.
  // Geometry remains in each observation's optical camera frame, not submap XYZ.
  std::vector<VisualFrame> copySubmapEvidence() const;
  void reset();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace sapphire
