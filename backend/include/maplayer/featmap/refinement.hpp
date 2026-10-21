#pragma once
#include <backend/registration/matcher/match.hpp>
#include <optional>
#include <backend/common/common_type.hpp>
#include "backend/maplayer/featmap/observations.hpp"
#include "backend/solver/ba_solver.hpp"

namespace sapphire::backend::featmap {
struct NodeBundleResult {
  solver::BASolver::Status status = solver::BASolver::Status::InvalidProblem;
  StateGroupf pose;
  std::optional<Mat66_t<double>> covariance;
  std::map<uint64_t, cv::Point3f> points;
  std::map<uint64_t, Mat33_t<double>> point_covariances;
  std::map<int, StateGroupf> observation_poses;
  std::vector<int> inliers;
  std::vector<std::pair<int, uint64_t>> rejected_observations;
  size_t views = 0, measurements = 0;
  size_t optimized_views = 0, fixed_views = 0, optimized_points = 0;
  size_t deferred_depths = 0;  // Valid pixel tracks without sufficiently constrained finite depth.
  double elapsed_ms = 0;
  std::string failure;
};
struct NodeBundleOptions {
  int max_iterations = 20;
  bool profile = false;
  solver::BADiagnostics* diagnostics = nullptr;
  // Historical-window diagnostics only; production updates keep history fixed.
  unsigned int max_optimized_views = 8;
  unsigned int max_points = 512;
};

// Online fusion optimizes the query relative pose and its associated points.
// joint_history is an explicit historical-window diagnostic, not the online policy.
// query_from_members attaches a source node's other captures to the same query
// pose with immutable local transforms, preserving the node's internal geometry.
// An independent fixed query pose may carry uncertainty. Points are optimized
// conditionally at that pose; their marginals include its first-order propagated
// uncertainty. This optional solve never refines or creates a graph pose factor.
NodeBundleResult refine_node(const LandmarkMap& map, const NodeObservations& history, const std::map<int, StateGroupf>& boundaries,
                                 const bow::FeatureBlock& query, const camera::Perspective& camera, const std::vector<Match>& matches,
                                 const StateGroupf& initial, const std::vector<double>& variances, size_t minimum, bool fixed_query = false,
                                 bool joint_history = false, int reference_node = 0, const NodeBundleOptions& options = {},
                                 const Mat66_t<double>* fixed_query_covariance = nullptr,
                                 const std::map<int, StateGroupf>& query_from_members = {});
// Original historical pixels + current pixels. Point variables are nuisance
// variables and are eliminated for the returned relative-pose covariance.
// No optimized points are written into either map or query.
solver::PoseResult refine_stereo_pose(const featmap::LandmarkMap& map, const bow::FeatureBlock& query, const camera::Perspective& camera,
                                      const std::vector<featmap::Match>& matches, const StateGroupf& initial, const std::vector<double>& pixel_variances,
                                      size_t min_inliers);

}  // namespace sapphire::backend::featmap
