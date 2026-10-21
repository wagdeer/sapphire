#pragma once
#include <backend/maplayer/featmap/landmark_map.hpp>
#include <backend/registration/matcher/match.hpp>
#include <memory>
#include <string>

// Appearance proposals only. Pose estimation and geometric admission belong
// to the system/solver layer; a descriptor match alone never verifies a loop.
namespace sapphire::backend {
// Exact Hamming 2-NN by exhaustive pair scan, with ratio and mutual-match gates.
// Owner-thread reusable buffers; no FAISS index or quadratic distance matrix.
class BinaryKnn final {
 public:
  std::vector<featmap::Match> match(const featmap::LandmarkMap&, const bow::FeatureBlock&, int max_distance, float ratio);
  size_t buffer_bytes() const;

 private:
  std::vector<uint8_t> match_source_, match_target_;
  std::vector<uint32_t> match_rows_;
  std::vector<int32_t> match_distances_;
  std::vector<int64_t> match_labels_;
};
}  // namespace sapphire::backend

namespace sapphire::backend::bow {
// Vocabulary-grouped frame matching, greedy target uniqueness in source order.
// Requires source 3D; rejects the whole request if its comparison bound exceeds
// the budget. Preserves the frame pipeline's singleton and ratio equality rules.
MatchResult bow_knn(const FeatureBlock& source, const FeatureBlock& target, const MatchOptions& options);
}  // namespace sapphire::backend::bow

namespace sapphire::backend::featmap {
// Vocabulary-grouped Hamming 2-NN with mutual, unambiguous association.
// Budget exhaustion discards partial results.
MatchResult bow_knn(const LandmarkMap& map, const bow::FeatureBlock& query, const MatchOptions& options);

// Exhaustive normalized L2; unique mutual neighbors, no vocabulary grouping.
// Checks the comparison bound before search; tied minima are rejected.
MatchResult l2_knn(const LandmarkMap& map, const bow::FeatureBlock& query, const MatchOptions& options);
}  // namespace sapphire::backend::featmap

namespace sapphire::backend::feature {
// RKNN computes the 512-slot descriptor similarity matrix; mutual nearest
// neighbors select correspondences. Identity binds inputs to the XFeat model.
class XFeatKnn final {
 public:
  XFeatKnn(const std::string& matcher_model, uint64_t descriptor_identity, int core);
  ~XFeatKnn();
  featmap::MatchResult match(const featmap::LandmarkMap&, const bow::FeatureBlock&, const MatchOptions&);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace sapphire::backend::feature
