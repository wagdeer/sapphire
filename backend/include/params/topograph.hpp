#pragma once

#include <backend/common/common_type.hpp>
#include <vector>
#include <string>

namespace sapphire::backend {

struct TopographParams {
  bool metric_enabled = true;
  float new_session_odometry_jump_m = 0;
  bool start_session_on_good_frame = false;
  bool use_previous_map = true;

  bool automatic_map_labels = true;
  bool belief_rebuild_prediction_each_update = false;
  std::string belief_transition_weights =
      "0.1 0.36 0.30 0.16 0.062 0.0151 0.00255 0.000324 2.5e-05 1.3e-06 4.8e-08 1.2e-09 1.9e-11 2.2e-13 1.7e-15 8.5e-18 2.9e-20 6.9e-23";
  float belief_virtual_place_prior_threshold = 0.9;
  bool ignore_odometry_cross_covariance = true;
  // Establish component datums when building a map; loading never invents missing datums.
  bool persistent_root_frames = false;
  bool reduce_graph = false;
  bool use_odometry_gravity = false;

  TopographParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
  std::vector<double> transitionWeights() const;
};

}  // namespace sapphire::backend
