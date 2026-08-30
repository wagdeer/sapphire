#include <voxelmaps.hpp>
#include <algorithm>
#include <array>
#include <cstring>
#include <istream>
#include <limits>
#include <ostream>

namespace cpu {
namespace {

struct SparseBucket {
  std::uint32_t index;
  std::int32_t x;
  std::int32_t y;
  std::int32_t z;
};

constexpr std::array<char, 8> kVoxelMapsMagic{{'P', 'Y', 'R', 'V', 'O', 'X', '0', '4'}};
constexpr std::uint32_t kVoxelMapsVersion = 4;
const std::array<Eigen::Vector3i, 7> kLowerCornerNeighbors{{
    Eigen::Vector3i(-1, -1, 0), Eigen::Vector3i(-1, 0, 0),  Eigen::Vector3i(0, -1, 0),
    Eigen::Vector3i(-1, -1, -1), Eigen::Vector3i(-1, 0, -1), Eigen::Vector3i(0, -1, -1),
    Eigen::Vector3i(0, 0, -1),
}};

template <typename T>
void write(std::ostream& stream, const T& value) {
  stream.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

template <typename T>
void write(std::ostream& stream, const std::vector<T>& values) {
  const std::size_t size = values.size();
  write(stream, size);
  stream.write(reinterpret_cast<const char*>(values.data()), sizeof(T) * size);
}

template <typename T>
void read(std::istream& stream, T& value) {
  stream.read(reinterpret_cast<char*>(&value), sizeof(T));
}

template <typename T>
void read(std::istream& stream, std::vector<T>& values) {
  std::size_t size;
  read(stream, size);
  values.resize(size);
  stream.read(reinterpret_cast<char*>(values.data()), sizeof(T) * size);
}

std::uint32_t voxel_hash(const Eigen::Vector3i& coordinate) {
  return static_cast<std::uint32_t>(coordinate.x()) * 73856093U ^ static_cast<std::uint32_t>(coordinate.y()) * 19349669U ^
         static_cast<std::uint32_t>(coordinate.z()) * 83492791U;
}

void write_sparse_buckets(std::ostream& stream, const VoxelBuckets& buckets) {
  std::size_t occupied_count = 0;
  for (const Eigen::Vector4i& bucket : buckets) {
    occupied_count += bucket.w() != 0;
  }
  std::vector<SparseBucket> occupied;
  occupied.reserve(occupied_count);
  for (std::size_t index = 0; index < buckets.size(); ++index) {
    const Eigen::Vector4i& bucket = buckets[index];
    if (bucket.w() != 0) {
      occupied.push_back({static_cast<std::uint32_t>(index), bucket.x(), bucket.y(), bucket.z()});
    }
  }
  write(stream, buckets.size());
  write(stream, occupied);
}

VoxelBuckets read_sparse_buckets(std::istream& stream) {
  std::size_t bucket_count = 0;
  std::vector<SparseBucket> occupied;
  read(stream, bucket_count);
  read(stream, occupied);
  VoxelBuckets buckets(bucket_count, Eigen::Vector4i::Zero());
  for (const SparseBucket& bucket : occupied) {
    if (bucket.index >= bucket_count) {
      throw std::runtime_error("Invalid sparse voxel bucket index");
    }
    buckets[bucket.index] = Eigen::Vector4i(bucket.x, bucket.y, bucket.z, 1);
  }
  return buckets;
}

}  // namespace

bool contains_voxel(const VoxelBuckets& buckets, int max_bucket_scan_count, const Eigen::Vector3i& coordinate) noexcept {
  if (buckets.empty() || max_bucket_scan_count <= 0) {
    return false;
  }
  const std::uint32_t hash = voxel_hash(coordinate);
  for (int scan = 0; scan < max_bucket_scan_count; ++scan) {
    const Eigen::Vector4i& bucket = buckets[(hash + static_cast<std::uint32_t>(scan)) % buckets.size()];
    if (bucket.w() == 0) {
      return false;
    }
    if (bucket.head<3>() == coordinate) {
      return true;
    }
  }
  return false;
}

VoxelMaps::VoxelMaps() : min_level_res_(0.5F), max_level_(3), max_bucket_scan_count_(10) {}

void VoxelMaps::create_voxelmaps(const Eigen::Vector3f* points, std::size_t point_count) {
  if (!std::isfinite(min_level_res_) || min_level_res_ <= 0.0F || max_level_ < 0 || max_bucket_scan_count_ <= 0 ||
      (points == nullptr && point_count != 0)) {
    throw std::invalid_argument("Invalid pyramid voxel parameters");
  }

  level_buckets_.clear();
  level_buckets_.reserve(static_cast<std::size_t>(max_level_ + 1));
  for (int level = 0; level <= max_level_; ++level) {
    UnorderedVoxelSet occupied_voxels;
    occupied_voxels.reserve(point_count * 2U);
    const float inverse_resolution = 1.0F / std::ldexp(min_level_res_, level);
    for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
      if (!points[point_index].allFinite()) {
        continue;
      }
      const Eigen::Vector3i coordinate = (points[point_index].array() * inverse_resolution).floor().cast<int>();
      occupied_voxels.emplace(coordinate);
      for (const Eigen::Vector3i& offset : kLowerCornerNeighbors) {
        occupied_voxels.emplace(coordinate + offset);
      }
    }
    level_buckets_.push_back(create_hash_buckets(occupied_voxels));
  }
}

VoxelMapsData VoxelMaps::release_data() {
  VoxelMapsData data;
  data.min_level_resolution = min_level_res_;
  data.max_level = max_level_;
  data.max_bucket_scan_count = max_bucket_scan_count_;
  data.level_buckets = std::move(level_buckets_);
  return data;
}

void save_voxelmaps(std::ostream& stream, const VoxelMapsData& data) {
  if (!data.valid()) {
    throw std::invalid_argument("Cannot save invalid pyramid voxels");
  }
  stream.write(kVoxelMapsMagic.data(), static_cast<std::streamsize>(kVoxelMapsMagic.size()));
  write(stream, kVoxelMapsVersion);
  write(stream, data.min_level_resolution);
  write(stream, data.max_level);
  write(stream, data.max_bucket_scan_count);
  for (const VoxelBuckets& buckets : data.level_buckets) {
    write_sparse_buckets(stream, buckets);
  }
}

VoxelMapsData load_voxelmaps(std::istream& stream) {
  std::array<char, kVoxelMapsMagic.size()> magic{};
  stream.read(magic.data(), static_cast<std::streamsize>(magic.size()));
  if (!stream || magic != kVoxelMapsMagic) {
    throw std::runtime_error("Invalid pyramid voxel format");
  }

  VoxelMapsData data;
  std::uint32_t version = 0;
  read(stream, version);
  if (version != kVoxelMapsVersion) {
    throw std::runtime_error("Unsupported pyramid voxel version");
  }
  read(stream, data.min_level_resolution);
  read(stream, data.max_level);
  read(stream, data.max_bucket_scan_count);
  if (data.max_level < 0 || data.max_level > 32) {
    throw std::runtime_error("Invalid pyramid voxel level count");
  }
  data.level_buckets.resize(static_cast<std::size_t>(data.max_level + 1));
  for (int level = 0; level <= data.max_level; ++level) {
    data.level_buckets[static_cast<std::size_t>(level)] = read_sparse_buckets(stream);
  }
  if (!stream || !data.valid()) {
    throw std::runtime_error("Failed to load BBS voxel maps");
  }
  return data;
}

VoxelMaps::Buckets VoxelMaps::create_hash_buckets(const UnorderedVoxelSet& occupied_voxels) {
  if (occupied_voxels.empty()) {
    return Buckets(1, Eigen::Vector4i::Zero());
  }

  Buckets buckets;
  const std::size_t maximum_bucket_count =
      occupied_voxels.size() > std::numeric_limits<std::size_t>::max() / 16U
          ? std::numeric_limits<std::size_t>::max()
          : occupied_voxels.size() * 16U;
  for (std::size_t num_buckets = occupied_voxels.size(); num_buckets <= maximum_bucket_count;) {
    buckets.resize(num_buckets);
    std::fill(buckets.begin(), buckets.end(), Eigen::Vector4i::Zero());

    std::size_t success_count = 0;
    for (const Eigen::Vector3i& voxel : occupied_voxels) {
      const Eigen::Vector4i coord(voxel.x(), voxel.y(), voxel.z(), 1);
      const std::uint32_t hash = voxel_hash(voxel);

      for (int i = 0; i < max_bucket_scan_count_; ++i) {
        const std::size_t bucket_index = (static_cast<std::size_t>(hash) + static_cast<std::size_t>(i)) % num_buckets;
        if (buckets[bucket_index].w() == 0) {
          buckets[bucket_index] = coord;
          ++success_count;
          break;
        }
      }
    }

    if (static_cast<double>(success_count) / static_cast<double>(occupied_voxels.size()) > 0.999) {
      return buckets;
    }
    if (num_buckets > maximum_bucket_count / 2U) {
      break;
    }
    num_buckets *= 2U;
  }
  return buckets;
}
}  // namespace cpu
