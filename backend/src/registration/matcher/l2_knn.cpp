#include <backend/registration/descriptor/landmark_descriptors.hpp>
#include <backend/registration/matcher/descriptor_match.hpp>

#include "backend/registration/matcher/nearest.hpp"
namespace sapphire::backend::featmap {
using detail::budget_result;
using detail::finish;
using detail::DescriptorNearest;
using detail::valid_pixel;
template <class Format>
MatchResult l2_knn_impl(const LandmarkMap& map, const bow::FeatureBlock& query, const MatchOptions& options) {
  const auto gate = UnitL2MatchPolicy::gate();
  if (options.min_matches == 0) throw std::invalid_argument("Zero submap match count");
  query.validate();
  MatchResult result;
  if (map.vocabulary_hash() != query.vocabulary_hash || map.descriptor_model_id() != query.descriptor_model_id ||
      map.descriptor_dimensions() != query.descriptors.cols) {
    result.status = MatchStatus::vocabulary_mismatch;
    return result;
  }
  if (map.points().size() < options.min_matches || query.keypoints.size() < options.min_matches) return result;
  const uint64_t bound = uint64_t(map.points().size()) * query.keypoints.size();
  if (bound > options.max_comparisons) return budget_result(std::move(result));
  std::vector<DescriptorNearest<Format>> queries(query.keypoints.size()), reverse(map.points().size());
  const feature::landmark_descriptors<Format> map_rows(map);
  const feature::descriptor_matrix<Format> descriptors(query.descriptors);
  std::vector<uint32_t> valid_queries;
  valid_queries.reserve(query.keypoints.size());
  for (uint32_t q = 0; q < query.keypoints.size(); ++q)
    if (valid_pixel(query.keypoints[q])) valid_queries.push_back(q);
  // Check pixels once while retaining sequential reads of the query matrix
  // in the inner loop; map appearances live in separate owned allocations.
  for (uint32_t p = 0; p < map.points().size(); ++p) {
    const feature::descriptor_distance<Format> matcher(map_rows.row(p));
    for (uint32_t q : valid_queries) {
      const float d = matcher.score(descriptors.row(int(q)));
      ++result.comparisons;
      queries[q].add(p, d);
      reverse[p].add(q, d);
    }
  }
  for (uint32_t q = 0; q < queries.size(); ++q) {
    const auto n = queries[q].distances();
    if (!n.accepts(gate)) continue;
    const auto r = reverse[n.first.id].distances();
    if (r.first.id == q && r.accepts(gate)) result.pairs.push_back({n.first.id, q, n.first.distance});
  }
  finish(result, options.min_matches);
  return result;
}
MatchResult l2_knn(const LandmarkMap& map, const bow::FeatureBlock& query, const MatchOptions& options) {
  if (!map.descriptor_model_id()) throw std::invalid_argument("L2 KNN requires floating-point descriptors");
  if (map.descriptor_dimensions() == 64) return l2_knn_impl<Float64_F32>(map, query, options);
  return l2_knn_impl<Float_F32>(map, query, options);
}
}  // namespace sapphire::backend::featmap
