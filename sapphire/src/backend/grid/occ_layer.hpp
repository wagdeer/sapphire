#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "backend/grid/local_gridmap.hpp"

namespace sapphire {

struct OccupancyGridData {
  float resolution = 0.1f;
  float originX = 0.0f;
  float originY = 0.0f;
  int width = 0;
  int height = 0;
  std::vector<int8_t> cells;
};

struct GridFrame {
  GridFrame(int id, const Eigen::Isometry3f &framePose, const LocalGrid &localGrid) : nodeId(id), pose(framePose), grid(&localGrid) {}

  int nodeId = 0;
  Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
  const LocalGrid *grid = nullptr;
};

class GridMapUpdate;

class OccupancyGrid {
 public:
  using GridLoader = std::function<bool(int, LocalGrid &)>;

  explicit OccupancyGrid(const NaviMapParameters &parameters)
      : cellSize_(static_cast<float>(parameters.resolution)),
        occupancyThreshold_(static_cast<float>(parameters.occ_threshold)),
        hitLogOdds_(logOdds(static_cast<float>(parameters.hit_probability))),
        missLogOdds_(logOdds(static_cast<float>(parameters.miss_probability))),
        minLogOdds_(logOdds(static_cast<float>(parameters.clamp_min_probability))),
        maxLogOdds_(logOdds(static_cast<float>(parameters.clamp_max_probability))) {
    if (!std::isfinite(parameters.resolution) || !(cellSize_ > 0.0f) || !validProbability(parameters.occ_threshold) ||
        !validProbability(parameters.hit_probability) || !validProbability(parameters.miss_probability) ||
        !validProbability(parameters.clamp_min_probability) || !validProbability(parameters.clamp_max_probability) ||
        parameters.clamp_min_probability >= parameters.clamp_max_probability) {
      throw std::invalid_argument("invalid NaviMap occupancy-grid parameters");
    }
  }

  void clear() {
    cells_.clear();
    activeFrames_.clear();
    nodeTiles_.clear();
    tileNodes_.clear();
    lastAppendedNodeId_ = 0;
    ++revision_;
  }

  bool append(const GridFrame &frame) {
    if (!validFrame(frame) || frame.nodeId <= lastAppendedNodeId_ || activeFrames_.find(frame.nodeId) != activeFrames_.end()) {
      return false;
    }

    TileSet tiles = tilesForFrame(frame);
    addFrameIndex(frame.nodeId, frame.pose, tiles);
    fuseFrame(frame.nodeId, frame.pose, *frame.grid, nullptr);
    lastAppendedNodeId_ = frame.nodeId;
    ++revision_;
    return true;
  }

  bool updateSingle(int nodeId, const Eigen::Isometry3f &pose, const LocalGrid &grid) { return append(GridFrame(nodeId, pose, grid)); }

  GridMapUpdate update(GridLoader loader);

  OccupancyGridData getMap() const {
    OccupancyGridData output;
    output.resolution = cellSize_;
    if (cells_.empty()) {
      return output;
    }

    int minX = std::numeric_limits<int>::max();
    int minY = std::numeric_limits<int>::max();
    int maxX = std::numeric_limits<int>::lowest();
    int maxY = std::numeric_limits<int>::lowest();
    for (const auto &entry : cells_) {
      minX = std::min(minX, keyX(entry.first));
      minY = std::min(minY, keyY(entry.first));
      maxX = std::max(maxX, keyX(entry.first));
      maxY = std::max(maxY, keyY(entry.first));
    }
    output.originX = static_cast<float>(minX) * cellSize_;
    output.originY = static_cast<float>(minY) * cellSize_;
    // Widen BEFORE subtraction. All dense output allocation follows this preflight.
    const auto width = std::int64_t(maxX) - std::int64_t(minX) + 1;
    const auto height = std::int64_t(maxY) - std::int64_t(minY) + 1;
    if (width <= 0 || height <= 0 || width > 4000 || height > 4000 ||
        std::uint64_t(width) > 16000000ULL / std::uint64_t(height))
      throw std::length_error("Navigation extent exceeds B3 application envelope");
    const auto count = std::uint64_t(width) * std::uint64_t(height);
    if (count > (16ULL * 1024 * 1024 - 65536) || count > SIZE_MAX / sizeof(int8_t))
      throw std::length_error("Navigation dense allocation exceeds B3 application envelope");
    output.width = static_cast<int>(width);
    output.height = static_cast<int>(height);
    output.cells.assign(static_cast<size_t>(output.width) * output.height, static_cast<int8_t>(-1));
    for (const auto &entry : cells_) {
      if (!entry.second.observed) {
        continue;
      }
      const int x = keyX(entry.first) - minX;
      const int y = keyY(entry.first) - minY;
      output.cells[static_cast<size_t>(y) * output.width + x] = probability(entry.second.logOdds) >= occupancyThreshold_ ? 100 : 0;
    }
    return output;
  }

  float getCellSize() const { return cellSize_; }
  size_t revision() const { return revision_; }

  bool containsNode(int id) const noexcept { return activeFrames_.count(id) != 0; }

  std::vector<int> activeNodeIds() const {
    std::vector<int> ids;
    for (const auto &entry : activeFrames_) ids.push_back(entry.first);
    std::sort(ids.begin(), ids.end());
    return ids;
  }

  // Semantic evidence inspection, without exposing mutable cells or log-odds storage.
  void visitEvidence(const std::function<void(int, int, bool, bool, int)> &visitor) const {
    for (const auto &[coordinate, cell] : cells_) visitor(keyX(coordinate), keyY(coordinate), cell.observed, cell.occupied, cell.ownerNodeId);
  }

  size_t getMemoryUsed() const {
    size_t indexEntries = 0;
    for (const auto &entry : nodeTiles_) {
      indexEntries += entry.second.size();
    }
    for (const auto &entry : tileNodes_) {
      indexEntries += entry.second.size();
    }
    return cells_.size() * (sizeof(std::uint64_t) + sizeof(Cell)) + activeFrames_.size() * (sizeof(int) + sizeof(ActiveFrame)) +
           indexEntries * (sizeof(std::uint64_t) + sizeof(int));
  }

 private:
  friend class GridMapUpdate;

  struct Cell {
    float logOdds = 0.0f;
    bool observed = false;
    bool occupied = false;
    int ownerNodeId = 0;
  };

  struct Contribution {
    bool hasMiss = false;
    bool hasHit = false;
  };

  using CellKey = std::uint64_t;
  using TileKey = std::uint64_t;
  using TileSet = std::unordered_set<TileKey>;

  struct ActiveFrame {
    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
  };

  static constexpr int kTileSize = 32;

  static bool validProbability(double value) { return std::isfinite(value) && value > 0.0 && value < 1.0; }

  static bool validFrame(const GridFrame &frame) { return frame.nodeId > 0 && frame.grid && frame.pose.matrix().allFinite(); }

  static float logOdds(float value) { return std::log(value / (1.0f - value)); }

  static float probability(float value) { return 1.0f - 1.0f / (1.0f + std::exp(value)); }

  static std::uint64_t key(int x, int y) { return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) | static_cast<std::uint32_t>(y); }

  static int keyX(std::uint64_t value) { return static_cast<std::int32_t>(value >> 32); }

  static int keyY(std::uint64_t value) { return static_cast<std::int32_t>(value & 0xffffffffu); }

  static int floorDiv(int value, int divisor) {
    const int quotient = value / divisor;
    const int remainder = value % divisor;
    return remainder < 0 ? quotient - 1 : quotient;
  }

  static TileKey tileForCell(int x, int y) { return key(floorDiv(x, kTileSize), floorDiv(y, kTileSize)); }

  CellKey transformedCell(const GridPoint &point, const Eigen::Isometry3f &pose) const {
    const Eigen::Vector3f world = pose * Eigen::Vector3f(point.x(), point.y(), 0.0f);
    return key(static_cast<int>(std::floor(world.x() / cellSize_)), static_cast<int>(std::floor(world.y() / cellSize_)));
  }

  TileSet tilesForFrame(const GridFrame &frame) const {
    TileSet tiles;
    tiles.reserve(frame.grid->groundCells.size() + frame.grid->emptyCells.size() + frame.grid->obstacleCells.size());
    const auto addTiles = [&](const GridPoints &points) {
      for (const GridPoint &point : points) {
        const CellKey cell = transformedCell(point, frame.pose);
        tiles.insert(tileForCell(keyX(cell), keyY(cell)));
      }
    };
    addTiles(frame.grid->groundCells);
    addTiles(frame.grid->emptyCells);
    addTiles(frame.grid->obstacleCells);
    return tiles;
  }

  void addFrameIndex(int nodeId, const Eigen::Isometry3f &pose, const TileSet &tiles) {
    activeFrames_[nodeId].pose = pose;
    nodeTiles_[nodeId] = tiles;
    for (const TileKey tile : tiles) {
      tileNodes_[tile].insert(nodeId);
    }
  }

  void removeFrameIndex(int nodeId) {
    const auto tiles = nodeTiles_.find(nodeId);
    if (tiles != nodeTiles_.end()) {
      for (const TileKey tile : tiles->second) {
        const auto nodes = tileNodes_.find(tile);
        if (nodes == tileNodes_.end()) {
          continue;
        }
        nodes->second.erase(nodeId);
        if (nodes->second.empty()) {
          tileNodes_.erase(nodes);
        }
      }
      nodeTiles_.erase(tiles);
    }
    activeFrames_.erase(nodeId);
  }

  void fuseFrame(int nodeId, const Eigen::Isometry3f &pose, const LocalGrid &grid, const TileSet *includedTiles) {
    std::unordered_map<CellKey, Contribution> contributions;
    contributions.reserve(grid.groundCells.size() + grid.emptyCells.size() + grid.obstacleCells.size());
    const auto collect = [&](const GridPoints &points, bool hit) {
      for (const GridPoint &point : points) {
        const CellKey cell = transformedCell(point, pose);
        if (includedTiles && includedTiles->find(tileForCell(keyX(cell), keyY(cell))) == includedTiles->end()) {
          continue;
        }
        Contribution &contribution = contributions[cell];
        if (hit) {
          contribution.hasHit = true;
        } else {
          contribution.hasMiss = true;
        }
      }
    };

    collect(grid.groundCells, false);
    collect(grid.emptyCells, false);
    collect(grid.obstacleCells, true);
    for (const auto &entry : contributions) {
      Cell &cell = cells_[entry.first];
      if (cell.ownerNodeId >= nodeId) {
        continue;
      }
      if (entry.second.hasMiss) {
        cell.logOdds = std::clamp(cell.logOdds + missLogOdds_, minLogOdds_, maxLogOdds_);
      }
      if (entry.second.hasHit) {
        cell.logOdds = std::clamp(cell.logOdds + hitLogOdds_, minLogOdds_, maxLogOdds_);
      }
      cell.observed = true;
      cell.occupied = entry.second.hasHit;
      cell.ownerNodeId = nodeId;
    }
  }

  void clearTiles(const TileSet &tiles) {
    for (const TileKey tile : tiles) {
      const int firstX = keyX(tile) * kTileSize;
      const int firstY = keyY(tile) * kTileSize;
      for (int offsetX = 0; offsetX < kTileSize; ++offsetX) {
        for (int offsetY = 0; offsetY < kTileSize; ++offsetY) {
          cells_.erase(key(firstX + offsetX, firstY + offsetY));
        }
      }
    }
  }

  float cellSize_;
  float occupancyThreshold_;
  float hitLogOdds_;
  float missLogOdds_;
  float minLogOdds_;
  float maxLogOdds_;
  std::unordered_map<CellKey, Cell> cells_;
  std::unordered_map<int, ActiveFrame> activeFrames_;
  std::unordered_map<int, TileSet> nodeTiles_;
  std::unordered_map<TileKey, std::unordered_set<int>> tileNodes_;
  int lastAppendedNodeId_ = 0;
  size_t revision_ = 0;
};

class GridMapUpdate {
 public:
  GridMapUpdate(OccupancyGrid &grid, OccupancyGrid::GridLoader loader) : grid_(&grid), loader_(std::move(loader)) {}

  GridMapUpdate(const GridMapUpdate &) = delete;
  GridMapUpdate &operator=(const GridMapUpdate &) = delete;
  GridMapUpdate(GridMapUpdate &&) noexcept = default;

  GridMapUpdate &operator+=(const GridFrame &frame) {
    if (committed_ || !OccupancyGrid::validFrame(frame)) {
      return *this;
    }

    PendingFrame &pending = pendingFrame(frame.nodeId);
    if (pending.present) {
      dirtyTiles_.insert(pending.tiles.begin(), pending.tiles.end());
    }
    pending.pose = frame.pose;
    pending.tiles = grid_->tilesForFrame(frame);
    pending.grid = *frame.grid;
    pending.present = true;
    dirtyTiles_.insert(pending.tiles.begin(), pending.tiles.end());
    return *this;
  }

  GridMapUpdate &operator-=(int nodeId) {
    if (committed_ || nodeId <= 0) {
      return *this;
    }

    if (pendingFrames_.find(nodeId) == pendingFrames_.end() && grid_->activeFrames_.find(nodeId) == grid_->activeFrames_.end()) {
      return *this;
    }
    PendingFrame &pending = pendingFrame(nodeId);
    if (!pending.present) {
      return *this;
    }
    dirtyTiles_.insert(pending.tiles.begin(), pending.tiles.end());
    pending.present = false;
    return *this;
  }

  bool commit() {
    if (committed_) {
      return false;
    }
    if (pendingFrames_.empty()) {
      return false;
    }

    std::unordered_set<int> candidateSet;
    for (const OccupancyGrid::TileKey tile : dirtyTiles_) {
      const auto nodes = grid_->tileNodes_.find(tile);
      if (nodes != grid_->tileNodes_.end()) {
        candidateSet.insert(nodes->second.begin(), nodes->second.end());
      }
    }
    for (const auto &entry : pendingFrames_) {
      if (entry.second.present) {
        candidateSet.insert(entry.first);
      }
    }

    std::vector<int> candidates(candidateSet.begin(), candidateSet.end());
    std::sort(candidates.begin(), candidates.end());
    std::unordered_map<int, LocalGrid> loadedGrids;
    loadedGrids.reserve(candidates.size());
    for (const int nodeId : candidates) {
      const auto pending = pendingFrames_.find(nodeId);
      if (pending != pendingFrames_.end()) {
        if (pending->second.present) {
          loadedGrids.emplace(nodeId, pending->second.grid);
        }
        continue;
      }
      LocalGrid loaded(grid_->cellSize_);
      if (!loader_ || !loader_(nodeId, loaded)) {
        return false;
      }
      loadedGrids.emplace(nodeId, std::move(loaded));
    }

    for (const auto &entry : pendingFrames_) {
      grid_->removeFrameIndex(entry.first);
      if (entry.second.present) {
        grid_->addFrameIndex(entry.first, entry.second.pose, entry.second.tiles);
        grid_->lastAppendedNodeId_ = std::max(grid_->lastAppendedNodeId_, entry.first);
      }
    }

    grid_->clearTiles(dirtyTiles_);
    for (const int nodeId : candidates) {
      const auto frame = grid_->activeFrames_.find(nodeId);
      if (frame == grid_->activeFrames_.end()) {
        continue;
      }
      grid_->fuseFrame(nodeId, frame->second.pose, loadedGrids.at(nodeId), &dirtyTiles_);
    }
    ++grid_->revision_;
    committed_ = true;
    return true;
  }

 private:
  struct PendingFrame {
    bool present = false;
    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    OccupancyGrid::TileSet tiles;
    LocalGrid grid;
  };

  PendingFrame &pendingFrame(int nodeId) {
    auto [pending, inserted] = pendingFrames_.try_emplace(nodeId);
    if (inserted) {
      const auto active = grid_->activeFrames_.find(nodeId);
      if (active != grid_->activeFrames_.end()) {
        pending->second.present = true;
        pending->second.pose = active->second.pose;
        pending->second.tiles = grid_->nodeTiles_.at(nodeId);
      }
    }
    return pending->second;
  }

  OccupancyGrid *grid_;
  OccupancyGrid::GridLoader loader_;
  OccupancyGrid::TileSet dirtyTiles_;
  std::unordered_map<int, PendingFrame> pendingFrames_;
  bool committed_ = false;
};

inline GridMapUpdate OccupancyGrid::update(GridLoader loader) { return GridMapUpdate(*this, std::move(loader)); }

}  // namespace sapphire
