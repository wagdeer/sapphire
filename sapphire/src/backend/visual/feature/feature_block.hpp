#pragma once

#include <cstdint>
#include <cstring>
#if defined(__aarch64__)
#include <arm_neon.h>
#endif
#include <memory>
#include <opencv2/core.hpp>
#include <vector>

namespace sapphire::features {

// Unaligned input is supported on both x86 and AArch64.
struct DescriptorQuery {
#if defined(__aarch64__)
  uint8x16_t low, high;
  explicit DescriptorQuery(const uint8_t* data) : low(vld1q_u8(data)), high(vld1q_u8(data + 16)) {}
  int distance(const uint8_t* data) const {
    return vaddvq_u8(vcntq_u8(veorq_u8(low, vld1q_u8(data)))) + vaddvq_u8(vcntq_u8(veorq_u8(high, vld1q_u8(data + 16))));
  }
#else
  uint64_t words[4];
  explicit DescriptorQuery(const uint8_t* data) { std::memcpy(words, data, 32); }
  int distance(const uint8_t* data) const {
    uint64_t other[4];
    std::memcpy(other, data, 32);
    int result = 0;
    for (int i = 0; i < 4; ++i) result += __builtin_popcountll(words[i] ^ other[i]);
    return result;
  }
#endif
};


struct WordCount {
  int word_id;
  uint32_t count;
};

struct FeatureGroup {
  uint32_t node_id;
  uint32_t begin;
  uint32_t end;
};

// Construct once, then share as const. Quantized arrays use stable group order;
// raw ORB arrays retain input row order without a vocabulary directory.
struct FeatureBlock {
  static constexpr uint32_t format_version = 1;
  static constexpr uint32_t max_features = 8192;
  static constexpr uint32_t group_level = 4;
  static constexpr uint32_t invalid_node_id = UINT32_MAX;
  uint64_t vocabulary_hash = 0;
  std::vector<FeatureGroup> groups;
  std::vector<uint32_t> feature_ids;
  std::vector<int> word_ids;
  std::vector<cv::KeyPoint> keypoints;
  std::vector<cv::Point3f> points;
  cv::Mat descriptors;

  static std::shared_ptr<const FeatureBlock> create(uint64_t vocabulary_hash, const std::vector<uint32_t> &node_ids, const std::vector<int> &word_ids,
                                                    const std::vector<cv::KeyPoint> &keypoints, const std::vector<cv::Point3f> &points,
                                                    const cv::Mat &descriptors);
  // Raw ORB has no vocabulary, visual words or tree groups. Serialized as v2.
  bool is_raw() const { return vocabulary_hash == 0 && word_ids.empty(); }
  static std::shared_ptr<const FeatureBlock> create_raw(const std::vector<cv::KeyPoint> &keypoints, const std::vector<cv::Point3f> &points,
                                                        const cv::Mat &descriptors);
  uint32_t tree_node(size_t row) const;
  int word_id(size_t row) const { return word_ids.empty() ? -1 : word_ids.at(row); }
  void validate() const;
  std::vector<uint8_t> encode() const;
  static std::shared_ptr<const FeatureBlock> decode(const void *data, size_t size);
  size_t memory_size() const;
};

struct MatchPair {
  uint32_t source;
  uint32_t target;
  bool operator==(const MatchPair &other) const { return source == other.source && target == other.target; }
};

struct MatchOptions {
  float ratio = 0.75f;
  int max_distance = 50;
  uint64_t max_comparisons = 50000;
  uint32_t min_matches = 6;
  bool require_source_depth = true;
};

enum class MatchStatus { success, insufficient_matches, comparison_budget, vocabulary_mismatch };
struct MatchResult {
  MatchStatus status = MatchStatus::insufficient_matches;
  uint64_t comparison_bound = 0;
  uint64_t comparisons = 0;
  std::vector<MatchPair> pairs;
};

// No retry, scope expansion or partial NNDR result on budget exhaustion.
MatchResult match_features(const FeatureBlock &source, const FeatureBlock &target, const MatchOptions &options);

}  // namespace sapphire::features
