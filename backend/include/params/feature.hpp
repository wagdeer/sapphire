#pragma once

#include <backend/common/common_type.hpp>

namespace sapphire::backend {

struct PreprocessParams;

struct FeatureParams {
  // One selector owns both local descriptors and global retrieval.
  // 0: FAST + ORB + FBoW; 1: FAST + XFeat + SigLIP2 cosine FAISS.
  bool enabled = true;
  int method = 0;
  bool uses_xfeat() const { return enabled && method == 1; }
  bool fast_secondary_detection_enabled = true;
  int fast_secondary_threshold = 7;
  int fast_threshold = 20;
  int max_features = 500;
  float orb_harris_k = 0.04;
  int orb_pyramid_levels = 8;
  float orb_pyramid_scale = 1.2;

  FeatureParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
  // Input geometry required by the deployed descriptor models.
  void validate(const PreprocessParams& preprocess) const;
};

}  // namespace sapphire::backend
