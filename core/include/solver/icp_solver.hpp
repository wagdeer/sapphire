#pragma once
#include <optional>
#include <string>

#include "common/se3_state.hpp"
#include <parameters.hpp>
#include "common/common_type.hpp"

namespace sapphire::lidar {
struct GaussianMatch { std::size_t target, query; };
using GaussianMatches = std::vector<GaussianMatch>;
struct Alignment {
  SE3f T_target_query;
  SE3f fitted_pose; // Diagnostic converged/support-qualified hypothesis, NEVER an accepted constraint.
  GaussianMatches matches;
  std::optional<Mat66_t<double>> covariance;  // query-local right tangent [translation, rotation]
  std::size_t inliers = 0;
  double fitness = 0, distribution_error = 0;
  double pyramid_ms = 0, bbs_ms = 0, gicp_ms = 0, metric_ms = 0;
  std::size_t bbs_roots = 0, bbs_expanded = 0, bbs_pruned = 0, bbs_queue_peak = 0, device_peak_bytes = 0;
  int iterations = 0;
  std::string failure;
};
Alignment align(const GaussianCloudPtr& target, const GaussianCloudPtr& query, const SE3f& T_target_query, const RegistrationParams& config);
// Same nearest-Gaussian/distance correspondence rule used by GICP.
GaussianMatches match(const GaussianCloudPtr& target, const GaussianCloudPtr& query, const SE3f&, double max_distance_m);
// Fixed-pose geometric verification and covariance; does not move the pose.
Alignment evaluate(const GaussianCloudPtr& target, const GaussianCloudPtr& query, const SE3f&, const RegistrationParams&);
}  // namespace sapphire::lidar
