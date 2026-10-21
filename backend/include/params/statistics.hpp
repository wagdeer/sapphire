#pragma once

#include <backend/common/common_type.hpp>

namespace sapphire::backend {

struct StatisticsParams {
  bool buffer_logs = true;
  bool collect_memory_usage = false;
  bool log_enabled = false;
  bool log_headers = true;
  bool publish_enabled = true;
  bool save_resident_nodes = false;

  StatisticsParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
};

}  // namespace sapphire::backend
