#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "common.hpp"
#include "parameters.h"

using GridPoint = Eigen::Vector2f;
using GridPoints = std::vector<GridPoint, Eigen::aligned_allocator<GridPoint>>;

struct LocalGrid {
  explicit LocalGrid(float resolution = 0.1f) : cellSize(resolution) {}

  GridPoints groundCells;
  GridPoints obstacleCells;
  GridPoints emptyCells;
  float cellSize = 0.1f;
  Eigen::Vector3f viewPoint = Eigen::Vector3f::Zero();
};

namespace local_grid_detail {

struct CellAccumulator {
  Eigen::Vector2f sum = Eigen::Vector2f::Zero();
  int count = 0;
};

inline std::uint64_t cellKey(int x, int y) {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) | static_cast<std::uint32_t>(y);
}

inline void addPoint(std::unordered_map<std::uint64_t, CellAccumulator> &cells, const Eigen::Vector2f &point, float resolution) {
  const int x = static_cast<int>(std::floor(point.x() / resolution));
  const int y = static_cast<int>(std::floor(point.y() / resolution));
  CellAccumulator &cell = cells[cellKey(x, y)];
  cell.sum += point;
  ++cell.count;
}

inline GridPoints centroids(const std::unordered_map<std::uint64_t, CellAccumulator> &cells) {
  GridPoints output;
  output.reserve(cells.size());
  for (const auto &entry : cells) {
    output.emplace_back(entry.second.sum / static_cast<float>(entry.second.count));
  }
  return output;
}

inline void traceRay(int endX, int endY, bool includeEnd, std::unordered_set<std::uint64_t> &freeCells) {
  int x = 0;
  int y = 0;
  const int dx = std::abs(endX);
  const int sx = endX >= 0 ? 1 : -1;
  const int dy = -std::abs(endY);
  const int sy = endY >= 0 ? 1 : -1;
  int error = dx + dy;
  while (true) {
    if (includeEnd || x != endX || y != endY) {
      freeCells.insert(cellKey(x, y));
    }
    if (x == endX && y == endY) {
      break;
    }
    const int twiceError = error * 2;
    if (twiceError >= dy) {
      error += dy;
      x += sx;
    }
    if (twiceError <= dx) {
      error += dx;
      y += sy;
    }
  }
}

}  // namespace local_grid_detail

class LocalGridMaker {
 public:
  explicit LocalGridMaker(const NaviMapParameters &parameters)
      : rangeMin_(static_cast<float>(parameters.min_range)),
        rangeMax_(static_cast<float>(parameters.usable_range)),
        cellSize_(static_cast<float>(parameters.resolution)),
        maxObstacleHeight_(static_cast<float>(parameters.h_clearance)),
        maxGroundHeight_(static_cast<float>(parameters.ground_margin)) {
    if (maxGroundHeight_ == 0.0f) {
      maxGroundHeight_ = cellSize_;
    }
  }

  void createLocalMap(const vvec<float, 3> &cloud, const Eigen::Isometry3f &pose, LocalGrid &grid) const {
    grid = LocalGrid(cellSize_);
    std::unordered_map<std::uint64_t, local_grid_detail::CellAccumulator> ground;
    std::unordered_map<std::uint64_t, local_grid_detail::CellAccumulator> obstacles;
    ground.reserve(cloud.size() / 2);
    obstacles.reserve(cloud.size() / 2);

    const float minRangeSquared = rangeMin_ * rangeMin_;
    const float maxRangeSquared = rangeMax_ > 0.0f ? rangeMax_ * rangeMax_ : 0.0f;
    for (const Eigen::Vector3f &point : cloud) {
      if (!point.allFinite()) {
        continue;
      }
      const Eigen::Vector3f &local = point;
      const float rangeSquared = local.squaredNorm();
      if (rangeSquared < minRangeSquared || (maxRangeSquared > 0.0f && rangeSquared > maxRangeSquared)) {
        continue;
      }

      const float leveledHeight = (pose.linear() * local).z();
      if (maxObstacleHeight_ > 0.0f && leveledHeight > maxObstacleHeight_) {
        continue;
      }
      const Eigen::Vector2f xy(point.x(), point.y());
      if (leveledHeight <= maxGroundHeight_) {
        local_grid_detail::addPoint(ground, xy, cellSize_);
      } else {
        local_grid_detail::addPoint(obstacles, xy, cellSize_);
      }
    }

    grid.groundCells = local_grid_detail::centroids(ground);
    grid.obstacleCells = local_grid_detail::centroids(obstacles);

    std::unordered_set<std::uint64_t> freeCells;
    freeCells.reserve((ground.size() + obstacles.size()) * 8);
    for (const GridPoint &point : grid.groundCells) {
      local_grid_detail::traceRay(static_cast<int>(std::floor(point.x() / cellSize_)), static_cast<int>(std::floor(point.y() / cellSize_)), true,
                                  freeCells);
    }
    for (const GridPoint &point : grid.obstacleCells) {
      const int x = static_cast<int>(std::floor(point.x() / cellSize_));
      const int y = static_cast<int>(std::floor(point.y() / cellSize_));
      local_grid_detail::traceRay(x, y, false, freeCells);
      freeCells.erase(local_grid_detail::cellKey(x, y));
    }

    grid.emptyCells.reserve(freeCells.size());
    for (const std::uint64_t key : freeCells) {
      const int x = static_cast<std::int32_t>(key >> 32);
      const int y = static_cast<std::int32_t>(key & 0xffffffffu);
      grid.emptyCells.emplace_back((static_cast<float>(x) + 0.5f) * cellSize_, (static_cast<float>(y) + 0.5f) * cellSize_);
    }
  }

 private:
  float rangeMin_;
  float rangeMax_;
  float cellSize_;
  float maxObstacleHeight_;
  float maxGroundHeight_;
};
