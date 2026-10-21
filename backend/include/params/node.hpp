#pragma once

#include <backend/common/common_type.hpp>

namespace sapphire::backend {

// Optimization budgets for one node refinement; observations carry their own noise.
struct LocalBAParams {
  int max_iterations = 20;
  unsigned int max_optimized_points = 512;

  LocalBAParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
};

// Persistent backend node: one or more fused observations, not a single image frame.
struct NodeParams {
  bool generate_node_ids = true;
  bool ignore_bad_frames = false;
  float min_rotation_rad = 0.1;
  float min_translation_m = 0.1;

  LocalBAParams local_ba;
  bool profile_enabled = false;
  unsigned int merge_max_comparisons = 1000000;
  bool merge_previous = true;
  bool refresh_enabled = false;
  bool rehearsal_ignore_moving = false;
  float rehearsal_similarity = 0.6;
  bool rehearsal_use_new_id = false;

  NodeParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
};

}  // namespace sapphire::backend
