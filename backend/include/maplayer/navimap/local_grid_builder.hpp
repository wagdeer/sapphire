#pragma once

#include <cstdint>
#include <unordered_map>

#include "backend/maplayer/navimap/local_grid.hpp"

namespace sapphire::backend::detail {

// Internal sink for height-classified points; optional voxel aggregation keeps
// obstacle evidence ahead of ground evidence without storing per-point labels.
class LocalGridBuilder {
 public:
  LocalGridBuilder(float cellSize, bool threeDimensional, bool voxelFiltering, std::size_t expectedPoints);
  void add(float x, float y, float z, bool obstacle);
  LocalGrid finish(std::array<float, 3> viewPoint = {}) &&;

 private:
  struct VoxelKey {
    std::int32_t x, y, z;
    bool operator==(const VoxelKey& other) const noexcept { return x == other.x && y == other.y && z == other.z; }
  };
  struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const noexcept;
  };
  struct VoxelAccumulator {
    float x = 0, y = 0, z = 0;
    std::uint32_t count = 0;
    bool obstacle = false;
  };

  void append(float x, float y, float z, bool obstacle);
  float cellSize_;
  bool threeDimensional_;
  bool voxelFiltering_;
  GridCells ground_, obstacles_;
  std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxels_;
};

}  // namespace sapphire::backend::detail
