#pragma once

#include <backend/common/common_type.hpp>

namespace sapphire::backend {

struct MemoryParams {
  bool eviction_order_by_id = false;
  int feature_cache_nodes = 32;
  bool load_all_nodes_on_start = false;
  bool predictive_prefetch_enabled = false;
  bool preload_visual_features = false;
  float protected_local_ratio = 0.25;
  float protected_recent_ratio = 0.2;
  unsigned int recent_nodes = 10;
  unsigned int resident_budget_bytes = 0;
  int resident_budget_nodes = 0;
  float update_budget_ms = 0;

  MemoryParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
};

}  // namespace sapphire::backend
