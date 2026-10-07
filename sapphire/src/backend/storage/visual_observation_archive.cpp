#include "backend/storage/visual_observation_archive.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <type_traits>
#include <zlib.h>

namespace sapphire::database_detail {
namespace {
constexpr std::uint32_t magic = 0x314d4f56; // VOM1, little endian.
constexpr std::size_t header_v1 = 144, header_v2 = 184, row_bytes = 72, max_rows = 8192;
void require(bool valid) { if (!valid) throw std::runtime_error("Invalid visual observation archive"); }
void put(std::vector<std::uint8_t> &out, std::uint64_t value, int bytes) {
  for (int i = 0; i < bytes; ++i) out.push_back(value >> (8 * i));
}
template<class T> void real(std::vector<std::uint8_t> &out, T value) {
  static_assert(std::numeric_limits<T>::is_iec559);
  using Bits = std::conditional_t<sizeof(T) == 4, std::uint32_t, std::uint64_t>;
  Bits bits; std::memcpy(&bits, &value, sizeof(T)); put(out, bits, sizeof(T));
}
struct Reader {
  const std::uint8_t *data;
  std::size_t left;
  std::uint64_t take(int bytes) {
    require(left >= std::size_t(bytes));
    std::uint64_t value = 0;
    for (int i = 0; i < bytes; ++i) value |= std::uint64_t(data[i]) << (8 * i);
    data += bytes; left -= bytes; return value;
  }
  template<class T> T real() {
    using Bits = std::conditional_t<sizeof(T) == 4, std::uint32_t, std::uint64_t>;
    Bits bits = take(sizeof(T)); T value; std::memcpy(&value, &bits, sizeof(T)); return value;
  }
};
VisualPoint point(float x, float y, float level, float response) {
  require(std::isfinite(x) && std::isfinite(y) && std::isfinite(response) &&
          std::isfinite(level) && level >= 0 && level <= 255 && std::floor(level) == level);
  return VisualPoint({x, y}, int(level), response);
}
}  // namespace

std::vector<std::uint8_t> packVisualObservation(const VisualFrame &frame) {
  require(frame.descriptors().empty() ? frame.points().empty() : std::size_t(frame.descriptors().rows) == frame.points().size());
  require(!frame.has_image() || (frame.T_odom_camera() && frame.projection() && frame.points().empty() && frame.descriptors().empty() && frame.image_png().size() <= VisualFrame::max_image_bytes));
  bool extended = frame.T_odom_camera().has_value() || frame.projection().has_value();
  for (const auto &p : frame.points()) extended |= p.track_id() != 0 || p.geometry().has_value();
  std::vector<std::uint8_t> out;
  require(frame.points().size() <= std::size_t(std::numeric_limits<int>::max()) / row_bytes);
  if (!extended) {
    // Preserve the existing native four-float encoding byte-for-byte.
    out.reserve(frame.points().size() * 16);
    for (const auto &p : frame.points()) {
      (void)point(p.pixel().x(), p.pixel().y(), float(p.level()), p.response());
      const float values[]{p.pixel().x(), p.pixel().y(), float(p.level()), p.response()};
      const auto *bytes = reinterpret_cast<const std::uint8_t *>(values);
      out.insert(out.end(), bytes, bytes + sizeof(values));
    }
    return out;
  }
  require(frame.points().size() <= max_rows);
  const bool stereo = std::any_of(frame.points().begin(), frame.points().end(), [](const auto &p) {
    return p.geometry() && p.geometry()->source == VisualDepthSource::Stereo;
  });
  require(!stereo || frame.projection().has_value());
  const auto header_bytes = frame.projection() ? header_v2 : header_v1;
  out.reserve(header_bytes + row_bytes * frame.points().size() + frame.image_png().size() + 8);
  put(out, magic, 4); put(out, frame.has_image() ? 4 : stereo ? 3 : frame.projection() ? 2 : 1, 4); put(out, frame.points().size(), 4);
  put(out, unsigned(bool(frame.T_odom_camera())) | (frame.projection() ? 2u : 0u), 4);
  for (int row = 0; row < 4; ++row) for (int col = 0; col < 4; ++col)
    real(out, frame.T_odom_camera() ? frame.T_odom_camera()->matrix()(row, col) : 0.0);
  if (frame.projection()) {
    put(out, frame.projection()->width, 4); put(out, frame.projection()->height, 4);
    for (double value : frame.projection()->intrinsics) real(out, value);
  }
  for (const auto &p : frame.points()) {
    (void)point(p.pixel().x(), p.pixel().y(), float(p.level()), p.response());
    real(out, p.pixel().x()); real(out, p.pixel().y()); real(out, float(p.level())); real(out, p.response());
    put(out, p.track_id(), 8); put(out, p.geometry() ? unsigned(p.geometry()->source) : 0, 4);
    for (int axis = 0; axis < 3; ++axis) real(out, p.geometry() ? p.geometry()->position_camera[axis] : 0.0);
    real(out, p.geometry() ? p.geometry()->parallax_rad : 0.0);
    real(out, p.geometry() ? p.geometry()->max_reprojection_px : 0.0);
    put(out, p.geometry() ? p.geometry()->observations : 0, 4);
  }
  if (frame.has_image()) {
    put(out, frame.image_png().size(), 4);
    out.insert(out.end(), frame.image_png().begin(), frame.image_png().end());
  }
  put(out, crc32(0, out.data(), out.size()), 4);
  return out;
}

VisualFrame readVisualObservation(double timestamp, std::size_t camera, std::size_t count,
    const void *data, std::size_t bytes, cv::Mat descriptors) {
  require(std::isfinite(timestamp) && camera <= 1 && (data || bytes == 0));
  require(count <= std::size_t(std::numeric_limits<int>::max()) / row_bytes);
  VisualFrame frame(timestamp, camera);
  std::vector<VisualPoint> points;
  if (bytes == count * 16) {
    points.reserve(count);
    const auto *raw = static_cast<const std::uint8_t *>(data);
    for (std::size_t i = 0; i < count; ++i) {
      float v[4]; std::memcpy(v, raw + i * 16, sizeof(v));
      points.push_back(point(v[0], v[1], v[2], v[3]));
    }
  } else {
    require(count <= max_rows && (bytes == header_v1 + count * row_bytes + 4 || bytes == header_v2 + count * row_bytes + 4 ||
        (count == 0 && bytes >= header_v2 + 8 && bytes <= header_v2 + 8 + VisualFrame::max_image_bytes)));
    const auto *raw = static_cast<const std::uint8_t *>(data);
    Reader checksum{raw + bytes - 4, 4};
    require(checksum.take(4) == crc32(0, raw, bytes - 4));
    Reader reader{raw, bytes - 4};
    require(reader.take(4) == magic);
    const auto version = reader.take(4);
    require((version >= 1 && version <= 4) && reader.take(4) == count);
    require(version == 4 ? (count == 0 && descriptors.empty() && bytes >= header_v2 + 8) :
        bytes == (version == 1 ? header_v1 : header_v2) + count * row_bytes + 4);
    const auto flags = reader.take(4);
    require(version == 1 ? flags <= 1 : (flags == 2 || flags == 3));
    Eigen::Isometry3d pose;
    for (int row = 0; row < 4; ++row) for (int col = 0; col < 4; ++col) pose.matrix()(row, col) = reader.real<double>();
    if (flags & 1) frame.set_observation_pose(pose);
    else require(pose.matrix().isZero(0));
    if (version >= 2) {
      const auto width = reader.take(4), height = reader.take(4);
      require(width <= std::uint64_t(std::numeric_limits<int>::max()) && height <= std::uint64_t(std::numeric_limits<int>::max()));
      RectifiedProjection projection{int(width), int(height), {}};
      for (double &value : projection.intrinsics) value = reader.real<double>();
      frame.set_projection(projection);
    }
    bool has_stereo = false;
    points.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      const float x = reader.real<float>(), y = reader.real<float>();
      const float level = reader.real<float>(), response = reader.real<float>();
      auto p = point(x, y, level, response);
      const auto id = reader.take(8), metric = reader.take(4); require(metric <= (version == 3 ? 2u : 1u));
      has_stereo |= metric == 2;
      VisualGeometry geometry;
      for (int axis = 0; axis < 3; ++axis) geometry.position_camera[axis] = reader.real<double>();
      geometry.parallax_rad = reader.real<double>(); geometry.max_reprojection_px = reader.real<double>();
      geometry.observations = reader.take(4);
      geometry.source = metric == 2 ? VisualDepthSource::Stereo : VisualDepthSource::Temporal;
      if (!metric) require(geometry.position_camera.isZero(0) && geometry.parallax_rad == 0 &&
                           geometry.max_reprojection_px == 0 && geometry.observations == 0);
      require(id || !metric);
      if (id) p.set_tracking(id, metric ? std::optional<VisualGeometry>(geometry) : std::nullopt);
      points.push_back(std::move(p));
    }
    if (version == 4) {
      require(flags == 3);
      const auto length = reader.take(4);
      require(length == reader.left && length <= VisualFrame::max_image_bytes);
      frame.set_image_png({reader.data, reader.data + length});
      reader.data += length; reader.left -= length;
    }
    require(reader.left == 0 && (version != 3 || has_stereo));
  }
  require(descriptors.empty() ? count == 0 : std::size_t(descriptors.rows) == count);
  frame.update_features(std::move(points), std::move(descriptors));
  return frame;
}
}  // namespace sapphire::database_detail
