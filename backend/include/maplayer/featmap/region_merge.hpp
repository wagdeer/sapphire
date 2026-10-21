#pragma once
#include "backend/maplayer/featmap/refinement.hpp"

namespace sapphire::backend::featmap {
struct RegionMergeResult {
  std::shared_ptr<const LandmarkMap> features;
  NodeObservations bundle;
  // Source-region local point identity -> retained-region local identity.
  // Original (observer node, feature_id) provenance is never renumbered.
  std::map<uint64_t, uint64_t> source_to_retained;
  sapphire::backend::featmap::NodeBundleResult refinement;
  int optimization_observer = 0;  // Source Node datum; internal captures stay fixed.
};
// Descriptor proposals index the exact two bank snapshots (feature = source
// point row). Construct hypotheses with valid pixel summaries; joint BA accepts
// the associations before any ownership transfer.
std::map<uint64_t, uint64_t> supported_region_correspondences(const LandmarkMap& target, const NodeObservations& target_history, const LandmarkMap& source,
                                                              const NodeObservations& source_history, const StateGroupf& target_from_source,
                                                              const std::vector<Match>& proposals);
// Merge two independently maintained regions. Both inputs remain unchanged on
// failure. Correspondences are persistent source -> target point identities,
// checked against both pixel summaries before any ownership transfer.
// This merges content, not graph variables or global pose constraints.
RegionMergeResult merge_node_regions(const LandmarkMap& target, const NodeObservations& target_history, const LandmarkMap& source,
                                       const NodeObservations& source_history, const StateGroupf& target_from_source,
                                       const std::map<uint64_t, uint64_t>& correspondences, const camera::Perspective& source_camera, size_t minimum_inliers,
                                       const sapphire::backend::featmap::NodeBundleOptions& options = {});
}  // namespace sapphire::backend::featmap
