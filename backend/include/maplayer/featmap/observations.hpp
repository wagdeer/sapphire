#pragma once
#include <backend/maplayer/featmap/landmark_map.hpp>
#include <limits>
#include <map>
#include <set>

namespace sapphire::backend::featmap {
// Historical measurements owned by one Node. Shared with BA and persistence
// as a data snapshot; this does not maintain a frame-to-frame graph.
struct NodeObservations {
  // node identifies an original SUBMAP ANCHOR (its selected keyframe), not an
  // ordinary input frame. Pixels are immutable evidence attached to that anchor.
  // Camera transforms are body_from_camera, never a fitted submap alignment.
  struct Observation {
    int node = 0;
    uint64_t landmark = 0;
    camera::StereoObservation stereo;
    Appearance appearance;
    // Stable original feature identity, never a row in a sorted descriptor bank.
    // Legacy archives did not store this provenance.
    uint32_t feature_id = std::numeric_limits<uint32_t>::max();
  };
  // Temporary lookups into observations; no measurements are copied here.
  struct Index {
    std::map<int, std::map<uint64_t, size_t>> frames;     // Capture -> landmark -> measurement row.
    std::map<uint64_t, std::map<int, size_t>> landmarks;  // Landmark -> capture -> measurement row.
  };

  struct Reliability {
    bool confirmed = false;
    // Last eight eligible revisits in processing order, not numeric node order.
    // A hit means a surviving geometric association, not a descriptor proposal.
    std::vector<std::pair<int, bool>> trials;
  };
  std::map<uint64_t, Reliability> reliability;
  // Capture placement survives pixel pruning; so must exactly-once maintenance.
  std::set<int> maintenance_captures;
  std::vector<Observation> observations;

  std::map<uint64_t, uint32_t> support;  // Derived CURRENT distinct observers; never confidence or lifetime count.
  uint64_t landmark_high_water = 0;      // Deleted/replaced identities must never be reallocated.
  // Node-local camera estimates, committed with the point geometry. These are
  // not raw odometry or independent constraints for the global pose graph.
  // Includes captures whose pixels were pruned. This immutable content snapshot
  // is the authority for internal placement; Topograph indexes it for graph projection.
  int reference_node = 0;
  std::map<int, StateGroupf> observation_poses;
  // Same structural/size contract as encode(), without serializing small drafts.
  void validate() const;
  size_t memory_size() const;
  cv::Mat encode() const;
  static NodeObservations decode(const cv::Mat& bytes);
  Index index() const;
  void refresh_support();
  void reject_observations(const std::vector<std::pair<int, uint64_t>>& rejected);
  // Cull only redundant historical sources. Persistent history is independent
  // of the BA active-window budget; nonredundant sources are never evicted to
  // make a local solve fit. Raw source nodes and global constraints are untouched.
  void prune_redundant_views(int anchor, const LandmarkMap& features, float pyramid_scale = 1.2f, const std::set<int>& protected_observers = {});
};
}  // namespace sapphire::backend::featmap
