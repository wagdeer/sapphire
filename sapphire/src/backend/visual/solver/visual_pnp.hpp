#pragma once

#include <Eigen/Geometry>
#include <optional>

#include "backend/visual/feature/scene_features.hpp"
#include "parameters.h"

namespace sapphire {

struct VisualPnpParameters {
  std::size_t max_pairs = 1024;
  int ransac_iterations = 200;
  int refinement_iterations = 10;
  int min_inliers = 20;
  double min_inlier_fraction = .35;
  double max_reprojection_px = 3;
  int min_occupied_cells = 5;  // fixed 4 x 3 grid, counted after refinement
  double min_second_axis_ratio = .001;  // rejects collinear support; planes allowed
};

enum class VisualPnpStatus { seed, pair_budget, insufficient_geometry, no_consensus, insufficient_support };
struct VisualPnpResult {
  VisualPnpStatus status = VisualPnpStatus::insufficient_geometry;
  // Maps query-submap anchor to target-submap anchor; metres. Present only for a
  // seed, never an accepted loop. Independent LiDAR verification is mandatory.
  std::optional<Eigen::Isometry3d> T_target_query;
  std::vector<mapping::scene::Match> inliers;
  std::size_t metric_pairs = 0;
  int occupied_cells = 0;
  double rms_reprojection_px = 0;
};

// Pure bounded candidate-pair computation, with no graph/archive ownership.
// Matches index these exact FeatureMap/FeatureBlock arrays (including any prior
// query quantization sort). Query pixels are already rectified with camera.K;
// camera.distortion is intentionally NOT reapplied. Target XYZ must originate
// from visual triangulation in the target anchor. T_query_camera includes the
// query image-time camera pose relative to its submap anchor, not just extrinsics.
// Invalid contracts throw; insufficient evidence returns a non-seed status.
VisualPnpResult estimateVisualPnpSeed(const mapping::scene::FeatureMap &target,
    const features::FeatureBlock &query, const std::vector<mapping::scene::Match> &matches,
    const CameraParameters &camera, const Eigen::Isometry3d &T_query_camera,
    const VisualPnpParameters &parameters = {});

}  // namespace sapphire
