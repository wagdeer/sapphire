#include <backend/registration/descriptor/descriptor_codec.hpp>
#include <nlohmann/json.hpp>

#include "backend/database/stereo_frames.hpp"
#include "backend/maplayer/featmap/validation.hpp"

namespace sapphire::backend::featmap {
using J = nlohmann::json;
using detail::validate_bundle_structure;
using detail::validate_local_poses;

namespace {
// Stream the fixed-width observation schema directly. The surrounding archive
// still uses nlohmann CBOR; keys and scalar encodings remain byte-compatible.
struct ObservationWriter {
  std::vector<uint8_t>& data;
  void argument(uint8_t major, uint64_t value) {
    if (value < 24) {
      data.push_back((major << 5) | uint8_t(value));
      return;
    }
    const unsigned bytes = value <= UINT8_MAX ? 1 : value <= UINT16_MAX ? 2 : value <= UINT32_MAX ? 4 : 8;
    data.push_back((major << 5) | (bytes == 1 ? 24 : bytes == 2 ? 25 : bytes == 4 ? 26 : 27));
    for (unsigned i = bytes; i; --i) data.push_back(uint8_t(value >> (8 * (i - 1))));
  }
  void integer(int64_t value) { argument(value < 0 ? 1 : 0, value < 0 ? uint64_t(-(value + 1)) : uint64_t(value)); }
  void real(float value) {
    if (!std::isfinite(value)) {
      data.insert(data.end(), {0xf9, uint8_t(std::isnan(value) ? 0x7e : value > 0 ? 0x7c : 0xfc), 0});
      return;
    }
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    data.push_back(0xfa);
    for (int i = 3; i >= 0; --i) data.push_back(uint8_t(bits >> (8 * i)));
  }
  void row(const NodeObservations::Observation& o, size_t camera, bool typed) {
    argument(4, typed ? 10 : 9);
    integer(o.node);
    argument(0, o.landmark);
    argument(0, camera);
    argument(4, 3);
    for (float value : {o.stereo.pixel().x, o.stereo.pixel().y, o.stereo.pixel().z}) real(value);
    real(o.stereo.sigma());
    const auto& descriptor = o.appearance.descriptor;
    argument(2, typed ? descriptor.dimensions() * 4 : 32);
    if (typed)
      feature::dispatch_descriptor(descriptor.dimensions(), [&](auto format) {
        using Format = decltype(format);
        if constexpr (std::is_same_v<typename descriptor_traits<Format>::scalar, float>)
          feature::descriptor_codec<Format>::append(data, feature::descriptor_row<Format>(descriptor.floating().data()));
      });
    else
      feature::descriptor_codec<Binary_U8>::append(data, feature::descriptor_row<Binary_U8>(descriptor.binary().data()));
    argument(0, o.appearance.tree_node);
    integer(o.appearance.word_id);
    argument(0, o.feature_id);
    if (typed) argument(0, descriptor.model_id());
  }
};
}  // namespace

cv::Mat NodeObservations::encode() const {
  std::vector<size_t> row_cameras;
  row_cameras.reserve(observations.size());
  J cameras = J::array();
  std::map<std::array<float, 17>, size_t> camera_ids;
  const auto active_support = validate_bundle_structure(*this);
  uint64_t high_water = landmark_high_water;
  if (!active_support.empty()) high_water = std::max(high_water, active_support.rbegin()->first);
  for (const auto& o : observations) {
    const auto t = o.stereo.anchor_from_camera().coefficients();
    std::array<float, 17> camera_key;
    std::copy(t.begin(), t.end(), camera_key.begin());
    std::copy(o.stereo.calibration().begin(), o.stereo.calibration().end(), camera_key.begin() + 12);
    const auto [camera, inserted] = camera_ids.emplace(camera_key, cameras.size());
    if (inserted) cameras.push_back({std::vector<float>(t.begin(), t.end()), o.stereo.calibration()});
    row_cameras.push_back(camera->second);
  }
  J poses = J::array();
  for (const auto& [node, p] : observation_poses) {
    const auto t = p.coefficients();
    poses.push_back({node, std::vector<float>(t.begin(), t.end())});
  }
  J archive;
  // validate_bundle_structure already established a common descriptor domain.
  const bool typed = !observations.empty() && (observations.front().appearance.descriptor.model_id() != 0);
  const int dimensions = typed ? observations.front().appearance.descriptor.dimensions() : 32;
  archive["version"] = 10;
  archive["descriptor_dimensions"] = dimensions;
  archive["maintenance_captures"] = maintenance_captures;
  J reliability_rows = J::array();
  for (const auto& [id, state] : reliability) {
    reliability_rows.push_back({id, state.confirmed, state.trials});
  }
  archive["reliability"] = std::move(reliability_rows);
  archive["cameras"] = std::move(cameras);
  archive["observations"] = nullptr;  // Written directly in sorted key order below.
  archive["support"] = active_support;
  archive["reference_node"] = reference_node;
  archive["poses"] = std::move(poses);
  archive["landmark_high_water"] = high_water;
  constexpr size_t limit = 16 * 1024 * 1024;
  std::vector<uint8_t> data;
  data.reserve(std::min(limit, size_t(1024) + observations.size() * (typed ? 96 + dimensions * 4 : 96)));
  ObservationWriter writer{data};
  writer.argument(5, archive.size());
  for (const auto& [key, value] : archive.items()) {
    J::to_cbor(J(key), data);
    if (key == "observations") {
      writer.argument(4, observations.size());
      for (size_t row = 0; row < observations.size(); ++row) {
        writer.row(observations[row], row_cameras[row], typed);
        if (data.size() > limit) throw std::runtime_error("Node archive byte budget exceeded");
      }
    } else {
      J::to_cbor(value, data);
    }
    if (data.size() > limit) throw std::runtime_error("Node archive byte budget exceeded");
  }
  return cv::Mat(1, int(data.size()), CV_8UC1, data.data()).clone();
}
NodeObservations NodeObservations::decode(const cv::Mat& bytes) {
  NodeObservations out;
  if (bytes.empty()) return out;
  if (bytes.type() != CV_8UC1 || !bytes.isContinuous() || bytes.total() > 16 * 1024 * 1024) throw std::runtime_error("Invalid submap archive size");
  const auto j = J::from_cbor(bytes.ptr<uint8_t>(), bytes.ptr<uint8_t>() + bytes.total());
  if (j.at("version") != 10) throw std::runtime_error("Unsupported submap archive version; rebuild the map");
  const int dimensions = j.at("descriptor_dimensions").get<int>();
  if (dimensions != 32 && dimensions != 64 && dimensions != 256) throw std::runtime_error("Invalid observation descriptor dimensions");
  const bool typed = dimensions != 32;
  if (j.contains("support")) out.support = j.at("support").get<std::map<uint64_t, uint32_t>>();
  if (out.support.size() > point_capacity) throw std::runtime_error("Node support budget exceeded");
  for (const auto& [id, count] : out.support) out.landmark_high_water = std::max(out.landmark_high_water, id);
  camera::StereoArchiveFrames frames;
  std::set<std::pair<int, uint64_t>> seen;
  for (const auto& packed : j.at("observations")) {
    const uint64_t domain = typed && packed.size() == 10 ? packed.at(9).get<uint64_t>() : 0;
    if (packed.size() != (typed ? 10 : 9) || !packed.at(2).is_number_unsigned() || !packed.at(5).is_binary() ||
        packed.at(5).get_binary().size() != (domain ? dimensions * 4 : 32) || (typed && !domain))
      throw std::runtime_error("Invalid compact observation record");
    const auto& camera = j.at("cameras").at(packed.at(2).get<size_t>());
    if (camera.size() != 2 || camera.at(0).size() != 12 || camera.at(1).size() != 5) throw std::runtime_error("Invalid compact observation camera");
    const auto& r = packed;
    NodeObservations::Observation o;
    o.node = r.at(0);
    o.landmark = r.at(1);
    o.feature_id = r.at(8);
    StateGroupf::Coefficients t;
    for (size_t i = 0; i < t.size(); ++i) t[i] = camera.at(0).at(i);
    const auto calibration = camera.at(1).get<std::array<float, 5>>();
    const camera::StereoPixel pixel{{r.at(3).at(0), r.at(3).at(1), r.at(3).at(2)}, r.at(4)};
    const auto& binary = r.at(5).get_binary();
    const uint8_t* cursor = binary.data();
    size_t remaining = binary.size();
    if (domain)
      o.appearance.descriptor = feature::dispatch_descriptor(dimensions, [&](auto format) -> feature::LocalDescriptor {
        using Format = decltype(format);
        if constexpr (std::is_same_v<typename descriptor_traits<Format>::scalar, float>)
          return feature::LocalDescriptor::from_learned(feature::descriptor_codec<Format>::read(cursor, remaining), domain);
        else
          throw std::runtime_error("Invalid learned observation format");
      });
    else
      o.appearance.descriptor = feature::descriptor_codec<Binary_U8>::read(cursor, remaining);
    o.appearance.tree_node = r.at(6);
    o.appearance.word_id = r.at(7);
    if (o.node <= 0 || !o.landmark || !std::isfinite(pixel.sigma) || !pixel.valid() || !StateGroupf(t).isValid() ||
        !seen.emplace(o.node, o.landmark).second)
      throw std::runtime_error("Invalid or duplicate submap observation");
    for (float k : calibration)
      if (!std::isfinite(k)) throw std::runtime_error("Invalid submap calibration");
    if (calibration[0] <= 0 || calibration[1] <= 0 || calibration[4] <= 0 || !std::isfinite(pixel.pixel.x) || !std::isfinite(pixel.pixel.y) ||
        !std::isfinite(pixel.pixel.z))
      throw std::runtime_error("Invalid submap pixels");
    frames.add(o.node, StateGroupf(t), calibration, pixel);
    out.observations.push_back(o);
  }
  const auto measurements = std::move(frames).finish();
  for (size_t i = 0; i < measurements.size(); ++i) out.observations[i].stereo = measurements[i];
  {
    out.reference_node = j.at("reference_node");
    for (const auto& row : j.at("poses")) {
      StateGroupf::Coefficients t;
      if (row.size() != 2 || row.at(1).size() != t.size()) throw std::runtime_error("Invalid submap pose archive");
      for (size_t i = 0; i < t.size(); ++i) t[i] = row.at(1).at(i);
      if (!out.observation_poses.emplace(row.at(0).get<int>(), StateGroupf(t)).second) throw std::runtime_error("Duplicate submap pose");
    }
  }
  out.landmark_high_water = std::max(out.landmark_high_water, j.at("landmark_high_water").get<uint64_t>());

  {
    out.maintenance_captures = j.at("maintenance_captures").get<std::set<int>>();
    for (const auto& row : j.at("reliability")) {
      if (row.size() != 3 || !row.at(1).is_boolean()) throw std::runtime_error("Invalid reliability archive");
      Reliability state;
      state.confirmed = row.at(1);
      state.trials = row.at(2).get<std::vector<std::pair<int, bool>>>();
      if (!out.reliability.emplace(row.at(0).get<uint64_t>(), std::move(state)).second)
        throw std::runtime_error("Duplicate landmark reliability archive");
    }
  }
  // Old counters accumulated visits, including rejected/culled observations.
  // Rebuild from the actual retained graph when loading any archive version.
  out.support = validate_bundle_structure(out);
  if (!out.support.empty()) out.landmark_high_water = std::max(out.landmark_high_water, out.support.rbegin()->first);
  return out;
}
}  // namespace sapphire::backend::featmap
