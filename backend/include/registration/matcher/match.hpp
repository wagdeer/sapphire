#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace sapphire::backend {

struct MatchGate {
  float ratio;
  float max_distance;  // Inclusive, in the public metric's units.
};

// Current normalized-vector pipeline policy, shared by CPU, NPU refinement,
// projection and external-proposal admission. This is not a property of FP32
// storage or of a descriptor having been learned. Keep the strict 0.7 boundary.
struct UnitL2MatchPolicy {
  static constexpr float max_distance = .7f;
  static MatchGate gate() { return {1.f, std::nextafter(max_distance, 0.f)}; }
};

inline MatchGate hamming_match_gate(float ratio, int max_distance) {
  if (!std::isfinite(ratio) || ratio <= 0 || ratio > 1 || max_distance < 0 || max_distance > 256)
    throw std::invalid_argument("Invalid Hamming match thresholds");
  return {ratio, float(max_distance)};
}

// Per-call limits supplied by the caller, independent of parameter loading.
// Ratio and distance are Hamming gates; normalized L2 uses UnitL2MatchPolicy.
struct MatchOptions {
  constexpr MatchOptions(float ratio, int max_distance, uint64_t max_comparisons, uint32_t min_matches)
      : ratio(ratio), max_distance(max_distance), max_comparisons(max_comparisons), min_matches(min_matches) {}

  float ratio;
  int max_distance;
  uint64_t max_comparisons;
  uint32_t min_matches;
};

}  // namespace sapphire::backend

namespace sapphire::backend::bow {

// Frame-to-frame correspondences retain source/target feature-row identities.
struct MatchPair {
  uint32_t source;
  uint32_t target;
  bool operator==(const MatchPair &other) const { return source == other.source && target == other.target; }
};

enum class MatchStatus { success, insufficient_matches, comparison_budget, vocabulary_mismatch };
struct MatchResult {
  MatchStatus status = MatchStatus::insufficient_matches;
  uint64_t comparison_bound = 0;
  uint64_t comparisons = 0;
  std::vector<MatchPair> pairs;
};

}  // namespace sapphire::backend::bow

namespace sapphire::backend::featmap {

struct Match {
  uint32_t landmark;  // LandmarkMap row, or reference feature row for fixed-frame matching; never a persistent ID.
  uint32_t feature;   // Row in the query FeatureBlock, not its pre-sort feature ID.
  float distance;     // Hamming for binary; Euclidean L2 for floating descriptors (never a squared search score).
};

enum class MatchStatus { success, insufficient_matches, budget_exceeded, vocabulary_mismatch };

struct MatchResult {
  MatchStatus status = MatchStatus::insufficient_matches;
  std::vector<Match> pairs;
  uint64_t comparisons = 0;
  uint64_t spatial_visits = 0;
  uint32_t projected_points = 0;
  uint32_t inconsistent_depth = 0;
};

}  // namespace sapphire::backend::featmap
