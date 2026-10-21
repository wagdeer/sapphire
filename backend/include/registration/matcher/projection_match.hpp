#pragma once
#include <backend/maplayer/featmap/landmark_map.hpp>
#include <backend/registration/matcher/match.hpp>

namespace sapphire::backend::featmap {
// Verified-pose association policy, separate from initial descriptor matching.
// Callers specialize these bounds for recovery or duplicate fusion.
struct ProjectionOptions {
  bool detect_duplicates = false;
  float radius = 7.5f;     // Bounded primary-image search window.
  int max_distance = 100;  // Hamming projection threshold; L2 uses UnitL2MatchPolicy.
  uint64_t max_comparisons = 20000;
  uint64_t max_spatial_visits = 20000;
  uint32_t max_projected_points = 2048;
  uint32_t min_seed_matches = 6;  // Zero when an independent measurement already verified the pose.
};

// Borrow original stereo measurements from two captures; no map conversion or
// vocabulary lookup. Match::landmark indexes reference rows here. Fixed raw
// relative pose only; unique left-image associations also expose conflicting depth.
// Incomplete searches return no pairs.
MatchResult match_fixed_pose(const VisualFeatures& reference, const camera::Perspective& reference_camera, const VisualFeatures& query,
                             const camera::Perspective& query_camera, const StateGroupf& query_from_reference, const std::vector<double>& pixel_variances,
                             const MatchOptions& options);

// Requires a geometrically verified pose and unique PnP seed associations. Returns
// ONLY provisional new associations, never appearance updates or a loop decision.
// Monocular query; query_from_map maps map coordinates into the query body frame.
MatchResult project_unmatched(const LandmarkMap& map, const VisualFrame& query, const std::shared_ptr<const bow::FeatureBlock>& query_features,
                              const StateGroupf& query_from_map, const std::vector<Match>& verified_seeds, const ProjectionOptions& options = {});

}  // namespace sapphire::backend::featmap
