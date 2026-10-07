#include <zlib.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <stdexcept>

#include "backend/visual/feature/scene_features.hpp"

namespace sapphire::mapping::scene {
namespace {
constexpr uint32_t magic = 0x4e435347;  // GSCN, little endian.
constexpr uint32_t version = 6;
constexpr size_t appearance_bytes = 72;
constexpr size_t max_bytes = 28 + point_capacity * (24 + appearance_capacity * appearance_bytes);
void put(std::vector<uint8_t>& data, uint32_t value) {
  for (int i = 0; i < 4; ++i) data.push_back(value >> (8 * i));
}
void put_real(std::vector<uint8_t>& data, float value) {
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
  uint32_t bits;
  std::memcpy(&bits, &value, 4);
  put(data, bits);
}
void put_ray(std::vector<uint8_t>& data, const ObservationRay& ray) {
  for (float value : {ray.origin.x, ray.origin.y, ray.origin.z, ray.direction.x, ray.direction.y, ray.direction.z, ray.pixel_angle})
    put_real(data, value);
}
struct Reader {
  const uint8_t* data;
  size_t left;
  uint32_t take() {
    if (left < 4) throw std::runtime_error("Truncated scene archive");
    uint32_t value = 0;
    for (int i = 0; i < 4; ++i) value |= uint32_t(data[i]) << (8 * i);
    data += 4;
    left -= 4;
    return value;
  }
  float real() {
    const uint32_t bits = take();
    float value;
    std::memcpy(&value, &bits, 4);
    return value;
  }
  ObservationRay ray() {
    ObservationRay result;
    result.origin = {real(), real(), real()};
    result.direction = {real(), real(), real()};
    result.pixel_angle = real();
    return result;
  }
};
}  // namespace

std::shared_ptr<const FeatureMap> FeatureMap::from_frame(const features::FeatureBlock& frame, uint32_t max_points, const UcmCameraModel* camera) {
  frame.validate();
  std::vector<Landmark> points;
  points.reserve(frame.points.size());
  for (uint32_t row = 0; row < frame.keypoints.size(); ++row) {
    const cv::Point3f xyz = frame.points.empty() ? cv::Point3f(0, 0, 0) : frame.points[row];
    if (!std::isfinite(xyz.x) || !std::isfinite(xyz.y) || !std::isfinite(xyz.z)) continue;
    Landmark point;
    point.id = uint64_t(frame.feature_ids[row]) + 1;  // Node-local stable identity, independent of sorted row.
    point.position = xyz;
    point.has_position = !frame.points.empty();
    point.appearance_count = 1;
    auto& sample = point.appearances[0];
    sample.tree_node = frame.tree_node(row);
    sample.word_id = frame.word_id(row) > 0 ? frame.word_id(row) : -1;
    // Neutral initialization, not a calibrated confidence or ORB-quality claim.
    sample.quality = 0.5f;
    if (camera) {
      const auto pixel = frame.keypoints[row].pt;
      if (!camera->contains(pixel.x, pixel.y)) continue;
      sample.ray = observation_ray(*camera, pixel);
    }
    std::memcpy(sample.descriptor.data(), frame.descriptors.ptr(row), sample.descriptor.size());
    points.push_back(point);
  }
  return create(frame.vocabulary_hash, std::move(points), max_points);
}

std::vector<features::WordCount> FeatureMap::retrieval_words() const {
  std::map<int, uint32_t> counts;
  for (const auto& point : points_) {
    const int word = point.appearances[0].word_id;
    if (word > 0) counts[word] += word_credits;
  }
  std::vector<features::WordCount> result;
  result.reserve(counts.size());
  for (const auto& item : counts) result.push_back({item.first, item.second});
  return result;
}

size_t FeatureMap::memory_size() const {
  return sizeof(*this) + points_.capacity() * sizeof(Landmark) + descriptors_.capacity() * sizeof(DescriptorRef) +
         groups_.capacity() * sizeof(features::FeatureGroup);
}

std::vector<uint8_t> FeatureMap::encode() const {
  std::vector<uint8_t> data;
  data.reserve(28 + points_.size() * 24 + descriptors_.size() * appearance_bytes);
  for (const uint32_t value : {magic, version, uint32_t(vocabulary_hash_), uint32_t(vocabulary_hash_ >> 32), max_points_, uint32_t(points_.size())})
    put(data, value);
  for (const auto& point : points_) {
    put(data, uint32_t(point.id));
    put(data, uint32_t(point.id >> 32));
    put_real(data, point.position.x);
    put_real(data, point.position.y);
    put_real(data, point.position.z);
    put(data, point.appearance_count | (point.has_position ? 0u : 0x80000000u));
    for (uint32_t i = 0; i < point.appearance_count; ++i) {
      const auto& sample = point.appearances[i];
      put(data, sample.tree_node);
      put(data, uint32_t(sample.word_id));
      put_real(data, sample.quality);
      put_ray(data, sample.ray);
      data.insert(data.end(), sample.descriptor.begin(), sample.descriptor.end());
    }
  }
  put(data, crc32(0, data.data(), data.size()));
  return data;
}

std::shared_ptr<const FeatureMap> FeatureMap::decode(const void* data, size_t size) {
  if (!data || size < 28 || size > max_bytes) throw std::runtime_error("Invalid scene archive size");
  const auto* bytes = static_cast<const uint8_t*>(data);
  Reader checksum{bytes + size - 4, 4};
  if (checksum.take() != crc32(0, bytes, size - 4)) throw std::runtime_error("Scene archive checksum mismatch");
  Reader reader{bytes, size - 4};
  if (reader.take() != magic) throw std::runtime_error("Unsupported scene archive format");
  const auto stored_version = reader.take();
  if (stored_version != version && stored_version != 5) throw std::runtime_error("Unsupported scene archive format");
  uint64_t hash = reader.take();
  hash |= uint64_t(reader.take()) << 32;
  const uint32_t capacity = reader.take(), count = reader.take();
  if (!capacity || capacity > point_capacity || count > capacity || reader.left < size_t(count) * (24 + appearance_bytes))
    throw std::runtime_error("Invalid scene archive capacity");
  std::vector<Landmark> points;
  points.reserve(count);
  for (uint32_t i = 0; i < count; ++i) {
    Landmark point;
    point.id = reader.take();
    point.id |= uint64_t(reader.take()) << 32;
    point.position.x = reader.real();
    point.position.y = reader.real();
    point.position.z = reader.real();
    const auto flags = reader.take();
    point.has_position = (flags & 0x80000000u) == 0;
    if ((flags & 0x7fffffffu) != 1 || reader.left < appearance_bytes) throw std::runtime_error("Invalid single-descriptor appearance count");
    auto& sample = point.appearances[0];
    sample.tree_node = reader.take();
    sample.word_id = static_cast<int32_t>(reader.take());
    sample.quality = reader.real();
    sample.ray = reader.ray();
    std::memcpy(sample.descriptor.data(), reader.data, 32);
    reader.data += 32;
    reader.left -= 32;
    point.appearance_count = 1;
    points.push_back(point);
  }
  if (reader.left) throw std::runtime_error("Trailing scene archive data");
  return create(hash, std::move(points), capacity);
}
}  // namespace sapphire::mapping::scene
