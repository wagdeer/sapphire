#pragma once

#include <backend/common/common_type.hpp>
#include <string>

namespace sapphire::backend {

struct RelocalizerParams {
  bool identity_initial_guess = false;
  bool invert_registration = false;
  bool limit_covariance_by_odometry = false;
  float local_radius_m = 10;
  unsigned int max_local_retrieved = 2;
  float max_loop_distance_m = 0.0;
  unsigned int max_retrieved = 2;
  float min_known_probability = 0.11;
  float min_known_probability_ratio = 0;
  float min_unlinked_known_probability = 0.05;
  bool spatial_enabled = true;
  unsigned int spatial_max_candidates = 5;
  float spatial_radius_m = 1;
  float spatial_max_view_angle_deg = 45;
  unsigned int spatial_max_graph_hops = 49;
  int virtual_place_likelihood_method = 0;
  bool visits_enabled = false;
  int visits_max_entries = 4096;
  float visits_max_gap_s = 5;

  RelocalizerParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
};

}  // namespace sapphire::backend
