#pragma once

#include <backend/common/common_type.hpp>

namespace sapphire::backend {

struct PGOParams {
  bool anchor_at_last_node = false;
  bool gnc_enabled = false;
  float gravity_sigma = 0.3;
  bool ignore_constraint_variance = false;
  bool ignore_priors = true;
  bool incremental_enabled = true;
  bool isam2_adaptive_reorder_enabled = true;
  double isam2_relinearize_rotation_rad = 0.01;
  int isam2_relinearize_skip = 1;
  double isam2_relinearize_translation_m = 0.05;
  double isam2_reorder_growth_ratio = 2.0;
  double isam2_wildfire_threshold = 0.001;
  float max_constraint_error_sigma = 3.0;
  int max_iterations = 20;
  double min_error_improvement = 0.0;
  int optimizer = 1;
  float repair_radius_m = 0.0;

  PGOParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
};

}  // namespace sapphire::backend
