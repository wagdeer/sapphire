#include <backend/database/driver.hpp>
#include <backend/database/localmap_archive.hpp>
#include <algorithm>
#include <array>
#include <cstring>
#include <stdexcept>
#include <type_traits>
#include <lz4.h>
#include <zlib.h>

namespace sapphire::backend {

namespace {
constexpr std::size_t max_archive_bytes = 256U * 1024 * 1024;
constexpr std::size_t max_compressed_bytes = max_archive_bytes + max_archive_bytes / 255 + 32;
struct Writer {
  std::vector<std::uint8_t> bytes;
  template <typename T>
  void put(T value) {
    static_assert(std::is_trivially_copyable_v<T>);
    if (bytes.size() > max_archive_bytes - sizeof(T)) throw std::length_error("Localmap archive too large");
    // Explicit little-endian encoding, including IEEE float bit patterns.
    std::array<std::uint8_t, sizeof(T)> raw;
    std::memcpy(raw.data(), &value, sizeof(T));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    std::reverse(raw.begin(), raw.end());
#endif
    bytes.insert(bytes.end(), raw.begin(), raw.end());
  }
  void pose(const StateGroupf& p) {
    for (float v : p.coefficients()) put(v);
  }
};
struct Reader {
  const std::uint8_t* data;
  size_t remaining;
  template <typename T>
  T get() {
    if (remaining < sizeof(T)) throw std::runtime_error("Truncated localmap archive");
    std::array<std::uint8_t, sizeof(T)> raw;
    std::memcpy(raw.data(), data, sizeof(T));
    data += sizeof(T);
    remaining -= sizeof(T);
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    std::reverse(raw.begin(), raw.end());
#endif
    T value;
    std::memcpy(&value, raw.data(), sizeof(T));
    return value;
  }
  StateGroupf pose() {
    StateGroupf::Coefficients values;
    for (float& v : values) v = get<float>();
    StateGroupf result(values);
    if (result.isNull() || (result.rotationMatrix().transpose() * result.rotationMatrix() - Eigen::Matrix3f::Identity()).norm() > 0.01f ||
        std::abs(result.rotationMatrix().determinant() - 1.0f) > 0.01f)
      throw std::runtime_error("Invalid localmap pose");
    return result;
  }
};
}  // namespace

cv::Mat LocalMapArchive::encode(const LocalMap& map) {
  if (!map.sealed || !map.observation_count) throw std::logic_error("Only sealed localmaps can be archived");
  map.validate_summary();
  const auto loaded = map.navimap_resident() ? std::optional<NavimapGrid>{} : map.navimap_snapshot();
  const auto& nav = loaded ? *loaded : map.navimap;
  // Persist evidence, never the derived contour or eroded display raster.
  std::vector<NavimapCell> raw_cells;
  raw_cells.reserve(nav.cells.size());
  for (const auto& [key, cell] : nav.cells) raw_cells.push_back(cell);
  std::sort(raw_cells.begin(), raw_cells.end(), [](const auto& a, const auto& b) { return a.y != b.y ? a.y < b.y : a.x < b.x; });
  const size_t cell_count = raw_cells.size();
  Writer w;
  w.bytes.reserve(std::min(max_archive_bytes, 320 + cell_count * 37));
  w.put<std::uint32_t>(0x434d434c);  // LCMC: navimap evidence, grid frame and TSDF settings/state.
  w.put<std::int32_t>(map.anchor_id);
  w.pose(map.odom_from_anchor);
  w.put(map.observation_count);
  w.put(map.visit_count);
  for (const auto& o : {map.first_observation, map.last_observation}) {
    w.put<std::int32_t>(o.sequence);
    w.put(o.stamp);
    w.pose(o.anchor_from_body);
    w.pose(o.odom_from_body);
  }
  w.put(nav.cell_size);
  for (float v : {nav.settings.hit, nav.settings.miss, nav.settings.minimum, nav.settings.maximum, nav.settings.footprint})
    w.put(v);
  w.put(static_cast<uint32_t>(cell_count));
  for (const auto& c : raw_cells) {
    w.put(c.x);
    w.put(c.y);
    w.put(c.evidence.shift);
    w.put(c.evidence.lower);
    w.put(c.evidence.upper);
    w.put(c.projection_z);
    w.put(c.height);
    w.put<int32_t>(c.sequence);
    w.put<int32_t>(c.height_sequence);
    w.put(c.flags);
  }
  w.pose(nav.anchor_from_grid);
  w.put(nav.settings.truncation);
  w.put(nav.settings.surface_weight);
  w.put(nav.settings.maximum_weight);
  std::vector<uint64_t> tiles;
  uint32_t distance_count = 0;
  for (const auto& [key, tile] : nav.distance_tiles) {
    tiles.push_back(key);
    for (const auto& value : tile.values) distance_count += value.weight > 0;
  }
  std::sort(tiles.begin(), tiles.end());
  w.put(distance_count);
  for (const auto key : tiles) {
    const auto& tile = nav.distance_tiles.at(key);
    const int bx = int32_t(key >> 32) * 8, by = int32_t(key) * 8;
    for (int i = 0; i < 64; ++i) {
      const auto& value = tile.values[i];
      if (value.weight <= 0) continue;
      w.put<int32_t>(bx + i % 8); w.put<int32_t>(by + i / 8);
      w.put(value.distance); w.put(value.weight); w.put(value.height); w.put<int32_t>(value.sequence);
    }
  }
  const int capacity = LZ4_compressBound(static_cast<int>(w.bytes.size()));
  cv::Mat bytes(1, capacity + 12, CV_8UC1);
  const int compressed = LZ4_compress_default(reinterpret_cast<const char*>(w.bytes.data()), reinterpret_cast<char*>(bytes.ptr() + 12),
                                              static_cast<int>(w.bytes.size()), capacity);
  if (compressed <= 0) throw std::runtime_error("Cannot compress localmap archive");
  Writer header;
  header.put<std::uint32_t>(0x344d434c);  // LCM4: LZ4 block with size and CRC32
  header.put(static_cast<std::uint32_t>(w.bytes.size()));
  header.put(static_cast<std::uint32_t>(crc32(0, w.bytes.data(), w.bytes.size())));
  std::memcpy(bytes.ptr(), header.bytes.data(), 12);
  return bytes.colRange(0, compressed + 12);
}

std::shared_ptr<LocalMap> LocalMapArchive::decode(const cv::Mat& bytes) {

  if (bytes.type() != CV_8UC1 || bytes.rows != 1 || bytes.total() < 12 || bytes.total() > max_compressed_bytes)
    throw std::runtime_error("Invalid localmap payload");
  Reader header{bytes.ptr(), bytes.total()};
  if (header.get<std::uint32_t>() != 0x344d434c) throw std::runtime_error("Unsupported localmap compression");
  const auto size = header.get<std::uint32_t>(), checksum = header.get<std::uint32_t>();
  if (size == 0 || size > max_archive_bytes) throw std::runtime_error("Invalid localmap uncompressed size");
  cv::Mat raw(1, size, CV_8UC1);
  const int decoded = LZ4_decompress_safe(reinterpret_cast<const char*>(bytes.ptr() + 12), reinterpret_cast<char*>(raw.ptr()), bytes.cols - 12, raw.cols);
  if (decoded != raw.cols || crc32(0, raw.ptr(), raw.total()) != checksum) throw std::runtime_error("Corrupt localmap compressed payload");
  Reader r{raw.ptr(), raw.total()};
  const auto encoding = r.get<uint32_t>();
  if (encoding != 0x414d434c && encoding != 0x424d434c && encoding != 0x434d434c)
    throw std::runtime_error("Unsupported localmap encoding");
  auto result = std::make_shared<LocalMap>();
  result->anchor_id = r.get<int32_t>();
  result->odom_from_anchor = r.pose();
  const auto read_observation = [&r, &result]() {
    OdomObservation o;
    o.anchor_id = result->anchor_id;
    o.sequence = r.get<int32_t>();
    o.stamp = r.get<double>();
    o.anchor_from_body = r.pose();
    o.odom_from_body = r.pose();
    if (o.sequence <= 0 || !std::isfinite(o.stamp)) throw std::runtime_error("Invalid trajectory observation");
    return o;
  };
  result->observation_count = r.get<uint64_t>();
  result->visit_count = r.get<uint64_t>();
  result->first_observation = read_observation();
  result->last_observation = read_observation();
  auto& nav = result->navimap;
  nav.cell_size = r.get<float>();
  // Legacy occupancy archives have no distance settings. Derive the production
  // default from their resolution so a later TSDF visit can merge into them.
  if (encoding == 0x414d434c && nav.cell_size > 0) nav.settings.truncation = 3 * nav.cell_size;
  nav.settings.hit = r.get<float>();
  nav.settings.miss = r.get<float>();
  nav.settings.minimum = r.get<float>();
  nav.settings.maximum = r.get<float>();
  nav.settings.footprint = r.get<float>();
  if (!std::isfinite(nav.cell_size) || nav.cell_size < 0 || !std::isfinite(nav.settings.hit) || nav.settings.hit < 0 || !std::isfinite(nav.settings.miss) ||
      nav.settings.miss > 0 || !std::isfinite(nav.settings.minimum) || !std::isfinite(nav.settings.maximum) || nav.settings.minimum >= nav.settings.maximum ||
      !std::isfinite(nav.settings.footprint) || nav.settings.footprint < 0 || (nav.cell_size > 0 && nav.settings.footprint / nav.cell_size > 1000))
    throw std::runtime_error("Invalid navigation settings");
  const auto cells = r.get<uint32_t>();
  // Older navigation-disabled archives contain no raster and stored zero here.
  // Accept the absent raster, but never accept actual cells without a scale.
  if (nav.cell_size == 0) {
    if (cells != 0) throw std::runtime_error("Navigation cells without resolution");
    result->navimap_enabled = false;
  }
  if (cells > r.remaining / 37) throw std::runtime_error("Truncated navigation cells");
  nav.cells.reserve(cells);
  for (uint32_t i = 0; i < cells; ++i) {
    NavimapCell c;
    c.x = r.get<int32_t>();
    c.y = r.get<int32_t>();
    c.evidence.shift = r.get<float>();
    c.evidence.lower = r.get<float>();
    c.evidence.upper = r.get<float>();
    c.projection_z = r.get<float>();
    c.height = r.get<float>();
    c.sequence = r.get<int32_t>();
    c.height_sequence = r.get<int32_t>();
    c.flags = r.get<uint8_t>();
    if (!std::isfinite(c.projection_z) || std::abs(int64_t(c.x)) > 1000000 || std::abs(int64_t(c.y)) > 1000000 || !std::isfinite(c.evidence.shift) ||
        !std::isfinite(c.evidence.lower) || !std::isfinite(c.evidence.upper) || c.evidence.lower > c.evidence.upper ||
        c.evidence.lower < nav.settings.minimum - 1e-5f || c.evidence.upper > nav.settings.maximum + 1e-5f || c.sequence <= 0 ||
        c.sequence > result->last_observation.sequence || c.height_sequence < 0 || c.height_sequence > c.sequence ||
        (c.height_sequence ? !std::isfinite(c.height) : !std::isnan(c.height)) || c.flags > 2)
      throw std::runtime_error("Invalid navigation cell");
    if (!nav.cells.emplace(NavimapGrid::key(c.x, c.y), c).second) throw std::runtime_error("Duplicate navigation cell");
  }
  if (encoding != 0x414d434c) {
    nav.anchor_from_grid = r.pose();
    bool distances_allowed = true;
    if (encoding == 0x424d434c) { // Read the isolated trial format at the archive boundary only.
      const auto enabled = r.get<uint8_t>();
      if (enabled > 1) throw std::runtime_error("Invalid trial TSDF encoding");
      distances_allowed = enabled;
    } else {
      nav.settings.truncation = r.get<float>();
      nav.settings.surface_weight = r.get<float>();
      nav.settings.maximum_weight = r.get<float>();
      if (!std::isfinite(nav.settings.truncation) || !std::isfinite(nav.settings.surface_weight) || !std::isfinite(nav.settings.maximum_weight) ||
          nav.settings.truncation <= 0 || nav.settings.surface_weight <= 0 || nav.settings.maximum_weight < nav.settings.surface_weight)
        throw std::runtime_error("Invalid archived TSDF settings");
    }
    const auto count = r.get<uint32_t>();
    if (count > r.remaining / 24 || (count && (!distances_allowed || nav.cell_size <= 0)))
      throw std::runtime_error("Invalid TSDF payload size or resolution");
    for (uint32_t i = 0; i < count; ++i) {
      const int x = r.get<int32_t>(), y = r.get<int32_t>();
      NavimapGrid::Distance value;
      value.distance = r.get<float>(); value.weight = r.get<float>(); value.height = r.get<float>(); value.sequence = r.get<int32_t>();
      if (std::abs(int64_t(x)) > 1000000 || std::abs(int64_t(y)) > 1000000 || !std::isfinite(value.distance) ||
          std::abs(value.distance) > nav.settings.truncation + 1e-5f || !std::isfinite(value.weight) ||
          value.weight <= 0 || value.weight > nav.settings.maximum_weight + 1e-5f || !std::isfinite(value.height) ||
          value.sequence <= 0 || value.sequence > result->last_observation.sequence)
        throw std::runtime_error("Invalid archived TSDF value");
      const int bx = NavimapGrid::tile_index(x), by = NavimapGrid::tile_index(y);
      auto& target = nav.distance_tiles[NavimapGrid::key(bx, by)].values[(y - by * 8) * 8 + x - bx * 8];
      if (target.weight > 0) throw std::runtime_error("Duplicate archived TSDF value");
      target = value;
    }
  }
  if (r.remaining) throw std::runtime_error("Trailing localmap data");
  result->validate_summary();
  result->sealed = true;
  return result;
}

void LocalMapArchive::seal(LocalMap& map) {
  map.validate_summary();
  map.sealed = true;
}

void LocalMapArchive::unload_navimap(LocalMap& map, const cv::Mat& archive, const std::shared_ptr<DBDriver>& driver) {
  if (!driver || !driver->isConnected()) return;
  if (archive.total() < 12) throw std::runtime_error("Invalid committed localmap archive");
  std::array<uint8_t, 12> fingerprint;
  std::memcpy(fingerprint.data(), archive.ptr(), fingerprint.size());
  const int size = archive.cols, id = map.anchor_id;
  std::weak_ptr<DBDriver> weak = driver;
  map.unload_navimap([weak, id, size, fingerprint](const LocalMap& expected) {
    auto driver = weak.lock();
    if (!driver) throw std::runtime_error("Localmap database connection has expired");
    auto bytes = driver->load_localmap(id);
    // Size + archive format + uncompressed size + CRC guard stale revisions.
    // decode() subsequently validates the entire decompressed payload.
    if (bytes.cols != size || std::memcmp(bytes.ptr(), fingerprint.data(), fingerprint.size()))
      throw std::runtime_error("Localmap archive was replaced; refusing a stale read");
    auto restored = decode(bytes);
    if (restored->anchor_id != expected.anchor_id || restored->observation_count != expected.observation_count ||
        restored->last_observation.sequence != expected.last_observation.sequence)
      throw std::runtime_error("Localmap snapshot changed during cold read");
    return std::move(restored->navimap);
  });
}
void LocalMapArchive::prepare_append(LocalMap& map) {
  if (map.sealed) throw std::logic_error("Cannot hydrate a sealed map for append");
  if (!map.navimap_resident()) map.materialize_navimap();
}
}  // namespace sapphire::backend
