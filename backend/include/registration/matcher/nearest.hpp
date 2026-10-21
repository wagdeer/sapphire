#pragma once
#include <algorithm>
#include <cmath>
#include <backend/registration/descriptor/distance.hpp>
#include <backend/registration/matcher/match.hpp>
#include <limits>
#include <opencv2/core.hpp>
#include <stdexcept>
namespace sapphire::backend::featmap::detail {
constexpr uint32_t invalid_index = std::numeric_limits<uint32_t>::max();
// Resolve the pipeline policy once on entry, outside candidate search loops.
template <class Format>
MatchGate descriptor_match_gate(float hamming_ratio, int hamming_max_distance) {
  if constexpr (descriptor_traits<Format>::metric == DistanceMetric::hamming)
    return hamming_match_gate(hamming_ratio, hamming_max_distance);
  else
    return UnitL2MatchPolicy::gate();
}

inline bool valid_pixel(const cv::KeyPoint& keypoint) { return std::isfinite(keypoint.pt.x) && std::isfinite(keypoint.pt.y); }

struct Neighbor {
  uint32_t id = invalid_index;
  float distance = std::numeric_limits<float>::infinity();
};

struct Nearest {
  Neighbor first, second;

  void add(uint32_t id, float distance) noexcept {
    Neighbor candidate{id, distance};
    if (id == first.id) {
      if (distance < first.distance) first = candidate;
      return;
    }
    if (id == second.id) {
      if (distance < second.distance) second = candidate;
      if (second.distance < first.distance) std::swap(first, second);
      return;
    }
    if (distance < first.distance) {
      second = first;
      first = candidate;
    } else if (distance < second.distance) {
      second = candidate;
    }
  }

  bool accepts(float ratio, float max_distance, bool allow_singleton) const noexcept {
    if (first.id == invalid_index || first.distance > max_distance) return false;
    if (second.id == invalid_index) return allow_singleton;
    return first.distance < second.distance && first.distance <= ratio * second.distance;
  }

  bool accepts(MatchGate gate, bool allow_singleton = true) const noexcept {
    return accepts(gate.ratio, gate.max_distance, allow_singleton);
  }
};

// Fixed-capacity result set over borrowed descriptor rows, independent of the
// candidate traversal (exhaustive, projection grid or NPU shortlist). The score
// stays private: consumers can only obtain distances in public metric units.
template <class Format>
class DescriptorNearest {
 public:
  void add(uint32_t id, float score) noexcept { scores_.add(id, score); }

  Nearest distances() const noexcept {
    // Keep the original float-space gate and tie semantics. Comparing against
    // a squared threshold alone would change decisions at rounding boundaries.
    auto result = scores_;
    using Distance = feature::descriptor_distance<Format>;
    result.first.distance = Distance::distance_from_score(result.first.distance);
    result.second.distance = Distance::distance_from_score(result.second.distance);
    return result;
  }

 private:
  Nearest scores_;
};

inline void finish(MatchResult& result, uint32_t min_matches) {
  result.status = result.pairs.size() >= min_matches ? MatchStatus::success : MatchStatus::insufficient_matches;
}

inline MatchResult budget_result(MatchResult result) {
  result.pairs.clear();
  result.status = MatchStatus::budget_exceeded;
  return result;
}
}  // namespace sapphire::backend::featmap::detail
