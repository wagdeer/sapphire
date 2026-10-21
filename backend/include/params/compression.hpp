#pragma once

#include <backend/common/common_type.hpp>
#include <string>

namespace sapphire::backend {

struct CompressionParams {
  float detect_angle_deg = 30;
  int detect_iterations = 1;
  float detect_max_radius_m = 1;
  float detect_min_radius_m = 0;
  std::string detect_scope = "both";
  int detect_session_id = -2;
  bool merge_keep_latest = false;
  bool merge_keep_linked = false;
  float merge_radius_m = 1;

  CompressionParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
};

}  // namespace sapphire::backend
