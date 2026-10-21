#include <backend/registration/descriptor/landmark_descriptors.hpp>
#include "backend/registration/matcher/nearest.hpp"
#include <algorithm>
#include <cmath>
#include <backend/tools/simd_sse_neon.hpp>
#include <backend/registration/matcher/descriptor_match.hpp>
#include <stdexcept>

namespace sapphire::backend::bow {
MatchResult bow_knn(const FeatureBlock& source, const FeatureBlock& target, const MatchOptions& options) {
  if (source.descriptor_model_id || target.descriptor_model_id) throw std::invalid_argument("Learned features cannot enter BoW matcher");
  hamming_match_gate(options.ratio, options.max_distance);
  MatchResult result;
  if (source.vocabulary_hash != target.vocabulary_hash) {
    result.status = MatchStatus::vocabulary_mismatch;
    return result;
  }
  std::vector<uint8_t> source_valid(source.keypoints.size()), target_valid(target.keypoints.size()), used(target.keypoints.size(), 0);
  for (size_t i = 0; i < source_valid.size(); ++i)
    source_valid[i] =
        std::isfinite(source.points[i].x) && std::isfinite(source.points[i].y) && std::isfinite(source.points[i].z);
  for (size_t i = 0; i < target_valid.size(); ++i) target_valid[i] = std::isfinite(target.keypoints[i].pt.x) && std::isfinite(target.keypoints[i].pt.y);
  uint64_t upper_bound = 0;
  size_t a = 0, b = 0;
  while (a < source.groups.size() && b < target.groups.size()) {
    const auto& left = source.groups[a];
    const auto& right = target.groups[b];
    if (left.node_id == FeatureBlock::invalid_node_id || right.node_id == FeatureBlock::invalid_node_id) break;
    if (left.node_id < right.node_id) {
      ++a;
      continue;
    }
    if (right.node_id < left.node_id) {
      ++b;
      continue;
    }
    const uint64_t n = std::count(source_valid.begin() + left.begin, source_valid.begin() + left.end, uint8_t(1));
    const uint64_t m = std::count(target_valid.begin() + right.begin, target_valid.begin() + right.end, uint8_t(1));
    upper_bound += std::min(n, m);
    result.comparison_bound += n * m;
    ++a;
    ++b;
  }
  if (upper_bound < options.min_matches) return result;
  if (result.comparison_bound > options.max_comparisons) {
    result.status = MatchStatus::comparison_budget;
    return result;
  }
  result.pairs.reserve(std::min(source_valid.size(), target_valid.size()));
  a = b = 0;
  while (a < source.groups.size() && b < target.groups.size()) {
    const auto& left = source.groups[a];
    const auto& right = target.groups[b];
    if (left.node_id == FeatureBlock::invalid_node_id || right.node_id == FeatureBlock::invalid_node_id) break;
    if (left.node_id < right.node_id) {
      ++a;
      continue;
    }
    if (right.node_id < left.node_id) {
      ++b;
      continue;
    }
    for (uint32_t i = left.begin; i < left.end; ++i) {
      if (!source_valid[i]) continue;
      const simd::Hamming256<simd::Native> query(source.descriptors.ptr(i));
      int best = 256, second = 256;
      uint32_t best_index = right.end;
      for (uint32_t j = right.begin; j < right.end; ++j) {
        if (used[j] || !target_valid[j]) continue;
        const int distance = query(target.descriptors.ptr(j));
        ++result.comparisons;
        if (distance < best) {
          second = best;
          best = distance;
          best_index = j;
        } else if (distance < second)
          second = distance;
      }
      if (best_index != right.end && best <= options.max_distance && best <= options.ratio * second) {
        result.pairs.push_back({i, best_index});
        used[best_index] = 1;
      }
    }
    ++a;
    ++b;
  }
  if (result.pairs.size() >= options.min_matches) result.status = MatchStatus::success;
  return result;
}
}  // namespace sapphire::backend::bow


namespace sapphire::backend::featmap {
using detail::budget_result;
using detail::finish;
using detail::invalid_index;
using detail::Nearest;
using detail::valid_pixel;
MatchResult bow_knn(const LandmarkMap& map, const bow::FeatureBlock& query, const MatchOptions& options) {
  using Format = Binary_U8;
  if (map.descriptor_model_id() || query.descriptor_model_id) throw std::invalid_argument("BoW KNN requires binary descriptors");
  hamming_match_gate(options.ratio, options.max_distance);
  if (options.min_matches == 0) throw std::invalid_argument("Zero submap match count");
  query.validate();
  MatchResult result;
  if (map.vocabulary_hash() != query.vocabulary_hash || map.descriptor_model_id() != query.descriptor_model_id ||
      map.descriptor_dimensions() != query.descriptors.cols) {
    result.status = MatchStatus::vocabulary_mismatch;
    return result;
  }
  if (map.points().size() < options.min_matches || query.keypoints.size() < options.min_matches) return result;
  std::vector<Nearest> queries(query.keypoints.size()), reverse(map.points().size());
  const feature::landmark_descriptors<Format> map_rows(map);
  const feature::descriptor_matrix<Format> descriptors(query.descriptors);
  size_t a = 0, b = 0;
  while (a < map.groups().size() && b < query.groups.size()) {
    const auto& left = map.groups()[a];
    const auto& right = query.groups[b];
    if (left.node_id == bow::FeatureBlock::invalid_node_id || right.node_id == bow::FeatureBlock::invalid_node_id) break;
    if (left.node_id < right.node_id) {
      ++a;
      continue;
    }
    if (right.node_id < left.node_id) {
      ++b;
      continue;
    }
    for (uint32_t q = right.begin; q < right.end; ++q) {
      if (!valid_pixel(query.keypoints[q])) continue;
      const uint64_t count = left.end - left.begin;
      if (count > options.max_comparisons - result.comparisons) return budget_result(std::move(result));
      const feature::descriptor_distance<Format> descriptor(descriptors.row(int(q)));
      for (uint32_t d = left.begin; d < left.end; ++d) {
        const auto ref = map.descriptors()[d];
        const int distance = descriptor.distance(map_rows.row(ref));
        ++result.comparisons;
        queries[q].add(ref, distance);
        reverse[ref].add(q, distance);
      }
    }
    ++a;
    ++b;
  }
  for (uint32_t q = 0; q < queries.size(); ++q) {
    const auto& nearest = queries[q];
    // Like vslam BowTree, a singleton group uses the absolute distance gate.
    // It is only a proposed association; PnP still establishes geometry.
    if (!nearest.accepts(options.ratio, options.max_distance, true)) continue;
    const auto& other = reverse[nearest.first.id];
    // Resolve competition after all distances have been evaluated. Never remove a
    // used candidate before NNDR, which could artificially increase the second distance.
    if (other.first.id != q || other.first.distance == other.second.distance) continue;
    result.pairs.push_back({nearest.first.id, q, nearest.first.distance});
  }
  finish(result, options.min_matches);
  return result;
}
}  // namespace sapphire::backend::featmap
