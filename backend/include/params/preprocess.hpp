#pragma once

#include <backend/common/common_type.hpp>

namespace sapphire::backend {

struct PreprocessParams {
  unsigned int feature_image_decimation = 1;
  unsigned int storage_image_decimation = 1;
  int stereo_image_width = 0;
  int stereo_image_height = 0;

  PreprocessParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
};

}  // namespace sapphire::backend
