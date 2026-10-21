#pragma once
#include <backend/registration/matcher/projection_match.hpp>
#include <map>
#include <stdexcept>
#include "backend/maplayer/featmap/update.hpp"

namespace sapphire::backend::featmap {
// Expected candidate rejection. Infrastructure and invariant failures keep
// their original exceptions and must reach the task's failure boundary.
class MergeRejected : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};
struct MergeStats {
  // Valid new spatial evidence before the feature-bank capacity limit.
  uint32_t eligible = 0;
  uint32_t added = 0;
  uint32_t retired = 0;
  uint32_t blocked = 0;
};
struct MergeDraft {
  NodeUpdate geometry;
  std::shared_ptr<const LandmarkMap> features;
  MergeStats stats;
  std::map<uint64_t,uint32_t> inserted_rows; // New landmark identity -> real query feature row.
  MatchResult projection;
};
// Failed initial matches suppress NEW point insertion until re-associated;
// they never blacklist a feature from projection matching to an existing point.
// Only final current-frame inliers survive from the old node. Other old visual
// points are retired; valid unmatched current 3D points fill remaining capacity.
MergeDraft prepare_merge(std::shared_ptr<const LandmarkMap> target, const VisualFrame& query,
    const std::shared_ptr<const bow::FeatureBlock>& query_features, const StateGroupf& query_from_target,
                         const std::vector<Match>& inliers, const std::vector<Match>& projected, const std::vector<uint32_t>& rejected_rows,
                         const std::vector<float>& quality, const GeometryOptions& geometry = {}, const AppearanceOptions& appearance = {},
                         uint64_t identity_floor = 0);
// Revisit maintenance preserves unseen points when capacity permits. At capacity,
// verified old points and new points are equally active. Unseen points gain a
// miss and are replaced first, regardless of their historical support.
MergeDraft prepare_refresh(std::shared_ptr<const LandmarkMap> target, const VisualFrame& query,
    const std::shared_ptr<const bow::FeatureBlock>& query_features, const StateGroupf& query_from_target,
                         const std::vector<Match>& inliers, const std::vector<uint32_t>& rejected_rows,
                         const std::vector<float>& quality,
                         const GeometryOptions& geometry = {}, const AppearanceOptions& appearance = {}, uint64_t identity_floor = 0);
// Shared online/offline path: bounded projection recovery followed by geometry
// and content maintenance. Exhaustion discards incomplete supplements and skips
// new-point insertion, without overruling independently sufficient seed support.
MergeDraft prepare_loop_merge(std::shared_ptr<const LandmarkMap> target, const VisualFrame& query,
    const std::shared_ptr<const bow::FeatureBlock>& query_features, const StateGroupf& query_from_target,
                              const std::vector<Match>& inliers, const std::vector<uint32_t>& rejected_rows, const std::vector<float>& quality,
                              const GeometryOptions& geometry = {}, const AppearanceOptions& appearance = {}, const ProjectionOptions& projection = {});
}  // namespace sapphire::backend::featmap
