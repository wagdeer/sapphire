#include "backend/maplayer/navimap/local_grid_builder.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace sapphire::backend::detail {

std::size_t LocalGridBuilder::VoxelKeyHash::operator()(const VoxelKey& key) const noexcept {
  std::size_t seed = static_cast<std::uint32_t>(key.x) * 73856093U;
  seed ^= static_cast<std::uint32_t>(key.y) * 19349663U;
  seed ^= static_cast<std::uint32_t>(key.z) * 83492791U;
  return seed;
}

LocalGridBuilder::LocalGridBuilder(float cellSize, bool threeDimensional, bool voxelFiltering, std::size_t expectedPoints)
    : cellSize_(cellSize), threeDimensional_(threeDimensional), voxelFiltering_(voxelFiltering) {
  if (!std::isfinite(cellSize) || cellSize <= 0) throw std::invalid_argument("Invalid local grid cell size");
  if (voxelFiltering) {
    voxels_.reserve(expectedPoints);
  } else {
    ground_.reserve(expectedPoints);
    obstacles_.reserve(expectedPoints);
  }
}

void LocalGridBuilder::append(float x, float y, float z, bool obstacle) {
  GridCells& cells = obstacle ? obstacles_ : ground_;
  if (threeDimensional_)
    cells.push3D(x, y, z);
  else
    cells.push2D(x, y);
}

void LocalGridBuilder::add(float x, float y, float z, bool obstacle) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return;
  if (!voxelFiltering_) {
    append(x, y, z, obstacle);
    return;
  }
  const float inverseCellSize = 1.0f / cellSize_;
  const VoxelKey key{static_cast<std::int32_t>(std::floor(x * inverseCellSize)), static_cast<std::int32_t>(std::floor(y * inverseCellSize)),
                     threeDimensional_ ? static_cast<std::int32_t>(std::floor(z * inverseCellSize)) : 0};
  VoxelAccumulator& accumulator = voxels_[key];
  if (accumulator.obstacle && !obstacle) return;
  if (obstacle && !accumulator.obstacle) {
    accumulator = {};
    accumulator.obstacle = true;
  }
  accumulator.x += x;
  accumulator.y += y;
  accumulator.z += z;
  ++accumulator.count;
}

LocalGrid LocalGridBuilder::finish(std::array<float, 3> viewPoint) && {
  if (voxelFiltering_) {
    ground_.reserve(voxels_.size());
    obstacles_.reserve(voxels_.size());
    for (const auto& entry : voxels_) {
      const auto& accumulator = entry.second;
      const float inverseCount = 1.0f / static_cast<float>(accumulator.count);
      append(accumulator.x * inverseCount, accumulator.y * inverseCount, accumulator.z * inverseCount, accumulator.obstacle);
    }
  }
  return LocalGrid(std::move(ground_), std::move(obstacles_), {}, cellSize_, viewPoint);
}

}  // namespace sapphire::backend::detail
