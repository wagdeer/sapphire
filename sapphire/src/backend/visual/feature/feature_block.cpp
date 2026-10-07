#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>
#include <stdexcept>
#include "backend/visual/feature/feature_block.hpp"

namespace sapphire::features {
namespace {
bool finite(const cv::Point3f &point) { return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z); }
void put_u32(std::vector<uint8_t> &data, uint32_t value) {
  for (int i = 0; i < 4; ++i) data.push_back(static_cast<uint8_t>(value >> (8 * i)));
}
void put_float(std::vector<uint8_t> &data, float value) {
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
  uint32_t bits;
  std::memcpy(&bits, &value, 4);
  put_u32(data, bits);
}
struct Reader {
  const uint8_t *data;
  size_t left;
  uint32_t u32() {
    if (left < 4) throw std::runtime_error("Truncated BoW feature block");
    uint32_t value = 0;
    for (int i = 0; i < 4; ++i) value |= uint32_t(data[i]) << (8 * i);
    data += 4;
    left -= 4;
    return value;
  }
  float real() {
    uint32_t bits = u32();
    float value;
    std::memcpy(&value, &bits, 4);
    return value;
  }
};
}  // namespace

std::shared_ptr<const FeatureBlock> FeatureBlock::create(uint64_t vocabulary_hash, const std::vector<uint32_t> &node_ids,
                                                         const std::vector<int> &word_ids, const std::vector<cv::KeyPoint> &keypoints,
                                                         const std::vector<cv::Point3f> &points, const cv::Mat &descriptors) {
  const size_t count = keypoints.size();
  if (count > max_features || node_ids.size() != count || word_ids.size() != count || (!points.empty() && points.size() != count) ||
      descriptors.rows != static_cast<int>(count) || (count && (descriptors.type() != CV_8UC1 || descriptors.cols != 32)))
    throw std::invalid_argument("Invalid BoW feature arrays");
  auto block = std::make_shared<FeatureBlock>();
  block->vocabulary_hash = vocabulary_hash;
  block->feature_ids.resize(count);
  std::iota(block->feature_ids.begin(), block->feature_ids.end(), 0);
  std::stable_sort(block->feature_ids.begin(), block->feature_ids.end(), [&](uint32_t a, uint32_t b) { return node_ids[a] < node_ids[b]; });
  block->descriptors.create(static_cast<int>(count), 32, CV_8UC1);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (uint32_t i = 0; i < count; ++i) {
    const uint32_t id = block->feature_ids[i];
    if (block->groups.empty() || block->groups.back().node_id != node_ids[id]) block->groups.push_back({node_ids[id], i, i});
    ++block->groups.back().end;
    block->word_ids.push_back(word_ids[id]);
    block->keypoints.push_back(keypoints[id]);
    block->points.push_back(points.empty() ? cv::Point3f(nan, nan, nan) : points[id]);
    std::memcpy(block->descriptors.ptr(i), descriptors.ptr(id), 32);
  }
  block->validate();
  return block;
}

std::shared_ptr<const FeatureBlock> FeatureBlock::create_raw(const std::vector<cv::KeyPoint> &keypoints, const std::vector<cv::Point3f> &points,
                                                             const cv::Mat &descriptors) {
  auto block = std::make_shared<FeatureBlock>();
  block->keypoints = keypoints;
  block->points = points.empty() ? std::vector<cv::Point3f>(keypoints.size(), {NAN, NAN, NAN}) : points;
  block->descriptors = descriptors.clone();
  block->feature_ids.resize(keypoints.size());
  std::iota(block->feature_ids.begin(), block->feature_ids.end(), 0);
  block->validate();
  return block;
}
uint32_t FeatureBlock::tree_node(size_t row) const {
  if (row >= keypoints.size()) throw std::out_of_range("Feature row");
  auto it = std::upper_bound(groups.begin(), groups.end(), row, [](size_t r, const FeatureGroup &g) { return r < g.end; });
  return it == groups.end() ? invalid_node_id : it->node_id;
}

void FeatureBlock::validate() const {
  const size_t count = keypoints.size();
  if (count > max_features || points.size() != count || (!is_raw() && word_ids.size() != count) || feature_ids.size() != count ||
      descriptors.rows != static_cast<int>(count) ||
      (count && (descriptors.type() != CV_8UC1 || descriptors.cols != 32 || !descriptors.isContinuous())))
    throw std::runtime_error("Invalid BoW feature block dimensions");
  if (is_raw()) {
    if (!groups.empty()) throw std::runtime_error("Raw ORB cannot have vocabulary groups");
    for (uint32_t i = 0; i < count; ++i)
      if (feature_ids[i] != i) throw std::runtime_error("Raw ORB feature order must be unchanged");
    return;
  }
  std::vector<uint8_t> seen(count, 0);
  uint32_t offset = 0, previous = 0;
  for (const auto &group : groups) {
    if (group.begin != offset || group.end <= group.begin || group.end > count || (offset && group.node_id <= previous))
      throw std::runtime_error("Invalid BoW group directory");
    for (uint32_t i = group.begin; i < group.end; ++i) {
      if (feature_ids[i] >= count || seen[feature_ids[i]]++ || (i > group.begin && feature_ids[i] <= feature_ids[i - 1]))
        throw std::runtime_error("Invalid BoW stable feature order");
    }
    offset = group.end;
    previous = group.node_id;
  }
  if (offset != count) throw std::runtime_error("Incomplete BoW group directory");
}

std::vector<uint8_t> FeatureBlock::encode() const {
  validate();
  std::vector<uint8_t> data;
  data.reserve(28 + groups.size() * 12 + keypoints.size() * 80);
  put_u32(data, 0x574f4247);  // GBOW, explicit little endian; never serialize C++ object storage.
  put_u32(data, is_raw() ? 2 : format_version);
  put_u32(data, vocabulary_hash);
  put_u32(data, vocabulary_hash >> 32);
  put_u32(data, is_raw() ? 0 : group_level);
  put_u32(data, keypoints.size());
  put_u32(data, groups.size());
  for (const auto &group : groups) {
    put_u32(data, group.node_id);
    put_u32(data, group.begin);
    put_u32(data, group.end);
  }
  for (size_t i = 0; i < keypoints.size(); ++i) {
    put_u32(data, feature_ids[i]);
    if (!is_raw()) put_u32(data, static_cast<uint32_t>(word_ids[i]));
    const auto &keypoint = keypoints[i];
    put_float(data, keypoint.pt.x);
    put_float(data, keypoint.pt.y);
    put_float(data, keypoint.size);
    put_float(data, keypoint.angle);
    put_float(data, keypoint.response);
    put_u32(data, static_cast<uint32_t>(keypoint.octave));
    put_u32(data, static_cast<uint32_t>(keypoint.class_id));
    put_float(data, points[i].x);
    put_float(data, points[i].y);
    put_float(data, points[i].z);
  }
  if (!descriptors.empty()) data.insert(data.end(), descriptors.data, descriptors.data + descriptors.total());
  return data;
}

std::shared_ptr<const FeatureBlock> FeatureBlock::decode(const void *data, size_t size) {
  if (!data || size < 28) throw std::runtime_error("Missing BoW feature block header");
  Reader reader{static_cast<const uint8_t *>(data), size};
  if (reader.u32() != 0x574f4247) throw std::runtime_error("Unsupported feature block magic");
  const auto version = reader.u32();
  if (version != 1 && version != 2) throw std::runtime_error("Unsupported feature block format");
  const bool raw = version == 2;
  auto block = std::make_shared<FeatureBlock>();
  block->vocabulary_hash = reader.u32();
  block->vocabulary_hash |= uint64_t(reader.u32()) << 32;
  if (reader.u32() != (raw ? 0 : group_level) || (raw && block->vocabulary_hash)) throw std::runtime_error("Unsupported BoW group level");
  const uint32_t count = reader.u32(), group_count = reader.u32();
  if (count > max_features || group_count > count || (raw && group_count) ||
      reader.left != size_t(group_count) * 12 + size_t(count) * (raw ? 76 : 80))
    throw std::runtime_error("Invalid BoW feature block lengths");
  for (uint32_t i = 0; i < group_count; ++i) {
    FeatureGroup group;
    group.node_id = reader.u32();
    group.begin = reader.u32();
    group.end = reader.u32();
    block->groups.push_back(group);
  }
  for (uint32_t i = 0; i < count; ++i) {
    block->feature_ids.push_back(reader.u32());
    if (!raw) block->word_ids.push_back(static_cast<int32_t>(reader.u32()));
    cv::KeyPoint keypoint;
    keypoint.pt.x = reader.real();
    keypoint.pt.y = reader.real();
    keypoint.size = reader.real();
    keypoint.angle = reader.real();
    keypoint.response = reader.real();
    keypoint.octave = static_cast<int32_t>(reader.u32());
    keypoint.class_id = static_cast<int32_t>(reader.u32());
    block->keypoints.push_back(keypoint);
    cv::Point3f point;
    point.x = reader.real();
    point.y = reader.real();
    point.z = reader.real();
    block->points.push_back(point);
  }
  block->descriptors.create(count, 32, CV_8UC1);
  if (count) std::memcpy(block->descriptors.data, reader.data, size_t(count) * 32);
  block->validate();
  return block;
}

size_t FeatureBlock::memory_size() const {
  return sizeof(*this) + groups.capacity() * sizeof(FeatureGroup) + feature_ids.capacity() * 4 + word_ids.capacity() * sizeof(int) +
         keypoints.capacity() * sizeof(cv::KeyPoint) + points.capacity() * sizeof(cv::Point3f) + descriptors.total();
}

MatchResult match_features(const FeatureBlock &source, const FeatureBlock &target, const MatchOptions &options) {
  if (!(options.ratio > 0 && options.ratio <= 1) || options.max_distance < 0 || options.max_distance > 256)
    throw std::invalid_argument("Invalid BoW match thresholds");
  MatchResult result;
  if (source.vocabulary_hash != target.vocabulary_hash) {
    result.status = MatchStatus::vocabulary_mismatch;
    return result;
  }
  std::vector<uint8_t> source_valid(source.keypoints.size()), target_valid(target.keypoints.size()), used(target.keypoints.size(), 0);
  for (size_t i = 0; i < source_valid.size(); ++i) source_valid[i] = !options.require_source_depth || finite(source.points[i]);
  for (size_t i = 0; i < target_valid.size(); ++i)
    target_valid[i] = std::isfinite(target.keypoints[i].pt.x) && std::isfinite(target.keypoints[i].pt.y);
  uint64_t upper_bound = 0;
  size_t a = 0, b = 0;
  while (a < source.groups.size() && b < target.groups.size()) {
    const auto &left = source.groups[a];
    const auto &right = target.groups[b];
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
    const auto &left = source.groups[a];
    const auto &right = target.groups[b];
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
      DescriptorQuery query(source.descriptors.ptr(i));
      int best = 256, second = 256;
      uint32_t best_index = right.end;
      for (uint32_t j = right.begin; j < right.end; ++j) {
        if (used[j] || !target_valid[j]) continue;
        const int distance = query.distance(target.descriptors.ptr(j));
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
}  // namespace sapphire::features
