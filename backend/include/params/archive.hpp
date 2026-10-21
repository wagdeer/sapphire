#pragma once

#include <backend/common/common_type.hpp>

namespace sapphire::backend {

struct ArchiveParams {
  bool keep_images_in_memory = false;
  bool keep_intermediate_data = false;
  bool keep_sensor_data = true;
  bool keep_unlinked_nodes = true;
  bool parallel_compression = true;

  ArchiveParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
};

}  // namespace sapphire::backend
