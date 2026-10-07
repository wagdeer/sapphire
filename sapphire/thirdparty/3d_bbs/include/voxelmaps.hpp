#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iosfwd>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

namespace cpu {
using VoxelBuckets = std::vector<Eigen::Vector4i>;

struct VoxelMapsData {
  float min_level_resolution = 0.0F;
  int max_level = -1;
  int max_bucket_scan_count = 0;
  std::vector<VoxelBuckets> level_buckets;

  bool valid() const {
    if (!std::isfinite(min_level_resolution) || min_level_resolution <= 0.0F || max_level < 0 || max_bucket_scan_count <= 0 ||
        level_buckets.size() != static_cast<std::size_t>(max_level + 1)) {
      return false;
    }
    for (const VoxelBuckets& buckets : level_buckets) {
      if (buckets.empty()) {
        return false;
      }
    }
    return true;
  }

  float resolution(int level) const { return std::ldexp(min_level_resolution, level); }

  const VoxelBuckets& buckets_for_level(int level) const { return level_buckets.at(static_cast<std::size_t>(level)); }
};

void save_voxelmaps(std::ostream& stream, const VoxelMapsData& data);

VoxelMapsData load_voxelmaps(std::istream& stream);

bool contains_voxel(const VoxelBuckets& buckets, int max_bucket_scan_count, const Eigen::Vector3i& coordinate) noexcept;

class VoxelMaps {
 public:
  VoxelMaps();

  struct VectorHash {
    size_t operator()(const Eigen::Vector3i& x) const {
      size_t seed = 0;
      for (int dimension = 0; dimension < 3; ++dimension) {
        seed ^= std::hash<int>{}(x[dimension]) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
      }
      return seed;
    }
  };

  struct VctorEqual {
    bool operator()(const Eigen::Vector3i& v1, const Eigen::Vector3i& v2) const { return v1 == v2; }
  };

  using UnorderedVoxelSet = std::unordered_set<Eigen::Vector3i, VectorHash, VctorEqual>;
  using Buckets = VoxelBuckets;
  using PointAccessor = std::function<Eigen::Vector3f(std::size_t)>;

  void set_min_res(float min_level_res) { min_level_res_ = min_level_res; }

  void set_max_level(int max_level) { max_level_ = max_level; }

  void create_voxelmaps(const Eigen::Vector3f* points, std::size_t point_count);

  void create_voxelmaps(std::size_t point_count, const PointAccessor& point_at);

  VoxelMapsData release_data();

 private:
  Buckets create_hash_buckets(const UnorderedVoxelSet& occupied_voxels);

  std::vector<Buckets> level_buckets_;
  float min_level_res_;
  int max_level_, max_bucket_scan_count_;
};
}  // namespace cpu