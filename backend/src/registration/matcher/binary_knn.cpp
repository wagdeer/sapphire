#include "backend/registration/matcher/descriptor_match.hpp"

#include "backend/tools/simd.hpp"
#include "backend/registration/descriptor/landmark_descriptors.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace sapphire::backend {
size_t BinaryKnn::buffer_bytes() const {
  return match_source_.capacity() + match_target_.capacity() + match_rows_.capacity() * sizeof(uint32_t) + match_distances_.capacity() * sizeof(int32_t) +
         match_labels_.capacity() * sizeof(int64_t);
}
std::vector<featmap::Match> BinaryKnn::match(const featmap::LandmarkMap &map, const bow::FeatureBlock &query, int max_distance, float ratio) {
  query.validate();
  if (map.descriptor_model_id() || query.descriptor_model_id) throw std::invalid_argument("Learned descriptors cannot enter Hamming matcher");
  hamming_match_gate(ratio, max_distance);
  const auto &refs = map.descriptors();
  if (refs.empty() || query.keypoints.empty()) return {};

  const feature::landmark_descriptors<Binary_U8> map_rows(map);
  match_source_.clear();
  match_target_.clear();
  match_rows_.clear();
  match_source_.reserve(refs.size() * 32);
  // FeatureBlock validation already guarantees contiguous descriptor rows.
  const bool borrowed = std::all_of(query.keypoints.begin(), query.keypoints.end(), [](const auto& k) {
        return std::isfinite(k.pt.x) && std::isfinite(k.pt.y);
      });
  for (const auto &ref : refs) {
    const auto descriptor = map_rows.row(ref);
    match_source_.insert(match_source_.end(), descriptor.data(), descriptor.data()+32);
  }
  if (!borrowed) {
    match_target_.reserve(query.keypoints.size() * 32);
    match_rows_.reserve(query.keypoints.size());
    for (uint32_t row = 0; row < query.keypoints.size(); ++row) {
      const auto pixel = query.keypoints[row].pt;
      if (!std::isfinite(pixel.x) || !std::isfinite(pixel.y)) continue;
      match_rows_.push_back(row);
      match_target_.insert(match_target_.end(), query.descriptors.ptr(row), query.descriptors.ptr(row) + 32);
    }
  }
  const size_t count = borrowed ? query.keypoints.size() : match_rows_.size(), reverse_offset = count * 2;
  if (!count) return {};
  const size_t results = (count + refs.size()) * 2;
  match_distances_.resize(results);
  match_labels_.resize(results);
  std::fill_n(match_distances_.begin(), results, std::numeric_limits<int32_t>::max());
  std::fill(match_labels_.begin(), match_labels_.end(), -1);

  // Hamming distance is symmetric: evaluate each pair once and update both
  // directions' exact two nearest neighbours. Keep equal minima so ambiguity
  // rejection remains identical to the two independent BinaryFlat searches.
  // Fuse distance and selection: no quadratic distance scratch or second scan.
  const auto nearest = [&](size_t slot, int32_t distance, int64_t label) {
    auto* d = match_distances_.data() + slot;
    auto* ids = match_labels_.data() + slot;
    if (distance < d[0]) {
      d[1] = d[0]; ids[1] = ids[0];
      d[0] = distance; ids[0] = label;
    } else if (distance < d[1]) {
      d[1] = distance; ids[1] = label;
    }
  };
  const auto* query_bytes = borrowed ? query.descriptors.ptr<uint8_t>() : match_target_.data();
  // Above this exact integer boundary a pair cannot be accepted, tie an
  // accepted reverse match, or reject any accepted forward match by NNDR.
  // Use the original floating-point comparison, avoiding division rounding.
  int irrelevant_distance = max_distance + 1;
  while (irrelevant_distance <= 256 && max_distance > ratio * irrelevant_distance) ++irrelevant_distance;
  for (size_t q = 0; q < count; ++q) {
    const auto descriptor = simd::Kernel<>{}.hamming256(query_bytes + q * 32);
    for (size_t m = 0; m < refs.size(); ++m) {
      const auto* candidate = match_source_.data() + m * 32;
      const int distance = descriptor(candidate);
      if (distance >= irrelevant_distance) continue;
      nearest(q * 2, distance, int64_t(m));
      nearest(reverse_offset + m * 2, distance, int64_t(q));
    }
  }

  std::vector<featmap::Match> matches;
  matches.reserve(std::min(count, refs.size()));
  for (size_t q = 0; q < count; ++q) {
    const auto m = match_labels_[q * 2];
    const int distance = match_distances_[q * 2];
    if (m < 0 || distance > max_distance) continue;
    const int second = match_distances_[q * 2 + 1];
    if (match_labels_[q * 2 + 1] >= 0 && !(distance < second && distance <= ratio * second)) continue;
    const size_t reverse = reverse_offset + m * 2;
    if (match_labels_[reverse] != int64_t(q) || (match_labels_[reverse + 1] >= 0 && match_distances_[reverse] == match_distances_[reverse + 1])) continue;
    matches.push_back({refs[m], borrowed ? uint32_t(q) : match_rows_[q], float(distance)});
  }
  return matches;
}

}  // namespace sapphire::backend
