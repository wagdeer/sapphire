#include <zlib.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <stdexcept>

#include "backend/maplayer/featmap/landmark_map.hpp"
#include "backend/registration/descriptor/landmark_descriptors.hpp"
#include "backend/registration/descriptor/descriptor_codec.hpp"
#include "backend/database/stereo_frames.hpp"

namespace sapphire::backend::featmap {
namespace {
constexpr uint32_t magic = 0x4e435347;  // GSCN, little endian.
constexpr uint32_t version = 3;         // v2 binary banks remain readable.
constexpr size_t appearance_bytes = 72;
constexpr size_t stereo_bytes = 88;
constexpr size_t summary_bytes = 4 + 27 * sizeof(double);
constexpr size_t max_bytes = 36 + point_capacity * (32 + appearance_bytes + 1024 + stereo_bytes + summary_bytes);
void put(std::vector<uint8_t>& data, uint32_t value) {
  for (int i = 0; i < 4; ++i) data.push_back(value >> (8 * i));
}
void put_real(std::vector<uint8_t>& data, float value) {
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
  uint32_t bits;
  std::memcpy(&bits, &value, 4);
  put(data, bits);
}
void put_double(std::vector<uint8_t>& data, double value) {
  static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559);
  uint64_t bits;
  std::memcpy(&bits, &value, 8);
  put(data, uint32_t(bits));
  put(data, uint32_t(bits >> 32));
}
void put_ray(std::vector<uint8_t>& data, const ObservationRay& ray) {
  for (float value : {ray.origin.x, ray.origin.y, ray.origin.z, ray.direction.x, ray.direction.y, ray.direction.z, ray.pixel_angle}) put_real(data, value);
}
struct Reader {
  const uint8_t* data;
  size_t left;
  uint32_t take() {
    if (left < 4) throw std::runtime_error("Truncated submap archive");
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
  double real64() {
    uint64_t bits = take();
    bits |= uint64_t(take()) << 32;
    double value;
    std::memcpy(&value, &bits, 8);
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

template <class Format>
std::vector<uint8_t> encode_map(const LandmarkMap& map) {
  constexpr bool stores_model = std::is_same_v<typename descriptor_traits<Format>::scalar, float>;
  const feature::landmark_descriptors<Format> map_rows(map);
  std::vector<uint8_t> data;
  data.reserve((stores_model ? (descriptor_traits<Format>::dimensions == 64 ? 40 : 36) : 28) + map.points().size() * (32 + stereo_bytes + summary_bytes) +
               map.descriptors().size() * (appearance_bytes + (feature::descriptor_codec<Format>::bytes - 32)));
  for (const uint32_t value : {magic, stores_model ? (descriptor_traits<Format>::dimensions == 64 ? uint32_t(4) : version) : uint32_t(2), uint32_t(map.vocabulary_hash()), uint32_t(map.vocabulary_hash() >> 32), map.max_points(),
                               uint32_t(map.points().size())})
    put(data, value);
  if constexpr (stores_model) {
    put(data, uint32_t(map.descriptor_model_id()));
    put(data, uint32_t(map.descriptor_model_id() >> 32));
    if constexpr (descriptor_traits<Format>::dimensions == 64) put(data, 64);
  }
  for (size_t row = 0; row < map.points().size(); ++row) {
    const auto& point = map.points()[row];
    put(data, uint32_t(point.id));
    put(data, uint32_t(point.id >> 32));
    put_real(data, point.position.x);
    put_real(data, point.position.y);
    put_real(data, point.position.z);
    put(data, point.miss_count);
    put_real(data, point.depth_quality);
    const auto& sample = point.appearance;
    put(data, sample.tree_node);
    put(data, uint32_t(sample.word_id));
    put_real(data, sample.quality);
    put_ray(data, sample.ray);
    feature::descriptor_codec<Format>::append(data, map_rows.row(row));
    put(data, point.stereo.valid() ? 1 : 0);
    if (point.stereo.valid()) {
      for (float value : point.stereo.anchor_from_camera().coefficients()) put_real(data, value);
      for (float value : point.stereo.calibration()) put_real(data, value);
      for (float value : {point.stereo.pixel().x, point.stereo.pixel().y, point.stereo.pixel().z, point.stereo.sigma()}) put_real(data, value);
    }
    put(data, point.geometry ? 1 : 0);
    if (point.geometry) {
      const auto& geometry = *point.geometry;
      geometry.validate();
      for (double value : geometry.reference.coefficients()) put_double(data, value);
      for (int i = 0; i < 3; ++i) put_double(data, geometry.anchor[i]);
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) put_double(data, geometry.root(r, c));
      for (int i = 0; i < 3; ++i) put_double(data, geometry.residual[i]);
    }
  }
  put(data, crc32(0, data.data(), data.size()));
  return data;
}

std::vector<uint8_t> LandmarkMap::encode() const { return feature::dispatch_descriptor(descriptor_dimensions_, [&](auto format) { return encode_map<decltype(format)>(*this); }); }

std::shared_ptr<const LandmarkMap> LandmarkMap::decode(const void* data, size_t size) {
  if (!data || size < 28 || size > max_bytes) throw std::runtime_error("Invalid submap archive size");
  const auto* bytes = static_cast<const uint8_t*>(data);
  Reader checksum{bytes + size - 4, 4};
  if (checksum.take() != crc32(0, bytes, size - 4)) throw std::runtime_error("Node archive checksum mismatch");
  Reader reader{bytes, size - 4};
  if (reader.take() != magic) throw std::runtime_error("Unsupported submap archive format");
  const auto stored_version = reader.take();
  if (stored_version != 2 && stored_version != version && stored_version != 4) throw std::runtime_error("Unsupported submap archive format");
  uint64_t hash = reader.take();
  hash |= uint64_t(reader.take()) << 32;
  const uint32_t capacity = reader.take(), count = reader.take();
  uint64_t model_id = 0;
  if (stored_version >= 3) {
    model_id = reader.take();
    model_id |= uint64_t(reader.take()) << 32;
  }
  const int dimensions = stored_version == 4 ? int(reader.take()) : model_id ? 256 : 32;
  if ((stored_version >= 3 && !model_id) || (stored_version == 4 && dimensions != 64)) throw std::runtime_error("Invalid archive descriptor domain");
  if (!capacity || capacity > point_capacity || count > capacity || reader.left < size_t(count) * (32 + appearance_bytes + (model_id ? dimensions * 4 - 32 : 0)))
    throw std::runtime_error("Invalid submap archive capacity");
  const auto decode_body = [&](auto format) {
    using Format = decltype(format);
    camera::StereoArchiveFrames frames;
    std::vector<size_t> measured_rows;
    std::vector<Landmark> points;
    points.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
      Landmark point;
      point.id = reader.take();
      point.id |= uint64_t(reader.take()) << 32;
      point.position.x = reader.real();
      point.position.y = reader.real();
      point.position.z = reader.real();
      point.miss_count = reader.take();
      point.depth_quality = reader.real();
      if (reader.left < appearance_bytes) throw std::runtime_error("Truncated submap appearance");
      auto& sample = point.appearance;
      sample.tree_node = reader.take();
      sample.word_id = static_cast<int32_t>(reader.take());
      sample.quality = reader.real();
      sample.ray = reader.ray();
      const auto values = feature::descriptor_codec<Format>::read(reader.data, reader.left);
      if constexpr (std::is_same_v<typename descriptor_traits<Format>::scalar, float>)
        sample.descriptor = feature::LocalDescriptor::from_learned(values, model_id);
      else
        sample.descriptor = values;
      point.has_appearance = true;
      const auto present = reader.take();
      if (present > 1) throw std::runtime_error("Invalid stereo observation flag");
      if (present) {
        StateGroupf::Coefficients coefficients;
        for (auto& value : coefficients) value = reader.real();
        std::array<float, 5> calibration{};
        for (auto& value : calibration) value = reader.real();
        camera::StereoPixel pixel{{reader.real(), reader.real(), reader.real()}, reader.real()};
        if (!pixel.valid()) throw std::runtime_error("Invalid stereo observation noise");
        measured_rows.push_back(i);
        frames.add(0, StateGroupf(coefficients), calibration, pixel);
      }
      const auto has_summary = reader.take();
      if (has_summary > 1) throw std::runtime_error("Invalid point summary flag");
      if (has_summary) {
        auto& geometry = point.geometry.emplace();
        StateGroupd::Coefficients coefficients;
        for (auto& value : coefficients) value = reader.real64();
        geometry.reference = StateGroupd(coefficients);
        for (int j = 0; j < 3; ++j) geometry.anchor[j] = reader.real64();
        for (int r = 0; r < 3; ++r)
          for (int c = 0; c < 3; ++c) geometry.root(r, c) = reader.real64();
        for (int j = 0; j < 3; ++j) geometry.residual[j] = reader.real64();
        geometry.validate();
      }
      points.push_back(point);
    }
    if (reader.left) throw std::runtime_error("Trailing submap archive data");
    const auto observations = std::move(frames).finish();
    for (size_t i = 0; i < observations.size(); ++i) points[measured_rows[i]].stereo = observations[i];
    return create(hash, std::move(points), capacity, model_id, dimensions);
  };
  return feature::dispatch_descriptor(dimensions, decode_body);
}
}  // namespace sapphire::backend::featmap
