#include <backend/common/common_type.hpp>
#include <backend/params/navimap.hpp>
#include <backend/parameters.hpp>
#include <algorithm>
#include <cmath>
#include <backend/maplayer/navimap/elevation_layer.hpp>
#include <backend/tools/logging.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>

namespace sapphire::backend {

ElevationLayer::ElevationLayer(const LocalGridCache* cache, const ParametersMap& parameters)
    : ElevationLayer(cache, NavimapParams{}.withOverrides(parameters)) {}
ElevationLayer::ElevationLayer(const LocalGridCache* cache, const NavimapParams& parameters)
    : IncrementalLayer<ElevationLayer>(cache, parameters), gridMap_(new grid_map::GridMap) {}

ElevationLayer::~ElevationLayer() { delete gridMap_; }

ElevationLayer::Key ElevationLayer::pack(std::int32_t x, std::int32_t y) noexcept {
  return (static_cast<Key>(static_cast<std::uint32_t>(x)) << 32U) | static_cast<std::uint32_t>(y);
}

std::int32_t ElevationLayer::unpackX(Key key) noexcept { return static_cast<std::int32_t>(key >> 32U); }

std::int32_t ElevationLayer::unpackY(Key key) noexcept { return static_cast<std::int32_t>(key); }

std::int32_t ElevationLayer::floorDiv(std::int32_t value, std::int32_t divisor) noexcept {
  const std::int32_t quotient = value / divisor;
  return value % divisor < 0 ? quotient - 1 : quotient;
}

ElevationLayer::Key ElevationLayer::tileOf(Key cell) noexcept { return pack(floorDiv(unpackX(cell), kTileSize), floorDiv(unpackY(cell), kTileSize)); }

void ElevationLayer::clear() {
  cells_.clear();
  nodeRasters_.clear();
  tileNodes_.clear();
  dirtyTiles_.clear();
  delete gridMap_;
  gridMap_ = new grid_map::GridMap;
  ++revision_;
  snapshotRevision_ = std::numeric_limits<std::uint64_t>::max();
  IncrementalLayer<ElevationLayer>::clear();
}

ElevationLayer::NodeRaster ElevationLayer::rasterize(const StateGroupf& pose, std::shared_ptr<const LocalGrid> grid) const {
  if (grid->skip_integration) {
    NodeRaster raster;
    raster.pose = pose;
    raster.grid = std::move(grid);
    return raster;
  }
  if (grid->fused_map) {
    const auto grid_pose = pose.compose(grid->anchor_from_grid);
    std::unordered_map<Key, CellStamp> projected;
    for (const auto& cell : grid->fused_cells) {
      if (!std::isfinite(cell.height)) continue;
      const auto p = grid_pose.transformPoint({(cell.x + .5f) * grid->cellSize, (cell.y + .5f) * grid->cellSize, cell.height});
      const Key key = pack(std::floor(p.x() / cellSize_), std::floor(p.y() / cellSize_));
      const auto old = projected.find(key);
      if (old == projected.end() || cell.height_sequence > old->second.sequence ||
          (cell.height_sequence == old->second.sequence && p.z() > old->second.elevation))
        projected.insert_or_assign(key, CellStamp{key, p.z(), cell.height_sequence});
    }
    NodeRaster raster;
    raster.pose = pose;
    raster.grid = std::move(grid);
    for (const auto& [key, stamp] : projected) raster.tiles[tileOf(key)].push_back(stamp);
    return raster;
  }
  std::unordered_map<Key, float> elevations;
  elevations.reserve(grid->ground.size() + grid->obstacles.size());
  const auto collect = [&](const GridCells& points) {
    if (!points.is3D()) {
      return;
    }
    for (std::size_t i = 0; i < points.size(); ++i) {
      const Eigen::Vector3f transformed = pose.transformPoint(Eigen::Vector3f(points.xy.x[i], points.xy.y[i], points.z[i]));
      const float x = transformed.x();
      const float y = transformed.y();
      const float z = transformed.z();
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      const Key cell = pack(static_cast<std::int32_t>(std::floor(x / cellSize_)), static_cast<std::int32_t>(std::floor(y / cellSize_)));
      const auto found = elevations.find(cell);
      if (found == elevations.end()) {
        elevations.emplace(cell, z);
      } else {
        found->second = std::max(found->second, z);
      }
    }
  };
  collect(grid->ground);
  collect(grid->obstacles);

  NodeRaster raster;
  raster.pose = pose;
  raster.grid = std::move(grid);
  for (const auto& [cell, elevation] : elevations) {
    raster.tiles[tileOf(cell)].push_back({cell, elevation});
  }
  return raster;
}

void ElevationLayer::indexNode(int nodeId, const NodeRaster& raster) {
  for (const auto& [tile, cells] : raster.tiles) {
    static_cast<void>(cells);
    tileNodes_[tile].insert(nodeId);
  }
}

void ElevationLayer::unindexNode(int nodeId, const NodeRaster& raster) {
  for (const auto& [tile, cells] : raster.tiles) {
    static_cast<void>(cells);
    const auto found = tileNodes_.find(tile);
    if (found == tileNodes_.end()) {
      continue;
    }
    found->second.erase(nodeId);
    if (found->second.empty()) {
      tileNodes_.erase(found);
    }
  }
}

bool ElevationLayer::apply(const std::vector<int>& removed, const std::vector<std::pair<int, StateGroupf>>& changed) {
  bool touched = false;
  const auto erase = [&](int nodeId) {
    const auto found = nodeRasters_.find(nodeId);
    if (found == nodeRasters_.end()) {
      return false;
    }
    for (const auto& [tile, cells] : found->second.tiles) {
      static_cast<void>(cells);
      dirtyTiles_.insert(tile);
    }
    unindexNode(nodeId, found->second);
    nodeRasters_.erase(found);
    return true;
  };

  for (const int nodeId : removed) {
    touched = erase(nodeId) || touched;
  }
  for (const auto& [nodeId, pose] : changed) {
    std::shared_ptr<const LocalGrid> source;
    const auto current = nodeRasters_.find(nodeId);
    if (current != nodeRasters_.end()) {
      source = current->second.grid;
    }
    touched = erase(nodeId) || touched;
    const auto grid = cache().find(nodeId);
    if (grid != cache().end()) {
      source = grid->second;
    }
    if (!source || (!source->fused_map && !source->is3D())) {
      continue;
    }
    NodeRaster raster = rasterize(pose, std::move(source));
    for (const auto& [tile, cells] : raster.tiles) {
      static_cast<void>(cells);
      dirtyTiles_.insert(tile);
    }
    indexNode(nodeId, raster);
    nodeRasters_.insert_or_assign(nodeId, std::move(raster));
    touched = true;
  }

  if (!touched) {
    return false;
  }
  rebuild();
  invalidate();
  return true;
}

void ElevationLayer::reset(Key tile) {
  const std::int32_t firstX = unpackX(tile) * kTileSize;
  const std::int32_t firstY = unpackY(tile) * kTileSize;
  for (std::int32_t y = firstY; y < firstY + kTileSize; ++y) {
    for (std::int32_t x = firstX; x < firstX + kTileSize; ++x) {
      cells_.erase(pack(x, y));
    }
  }
}

void ElevationLayer::stamp(int nodeId, const std::vector<CellStamp>& stamps) {
  for (const CellStamp& stamp : stamps) {
    const int sequence = stamp.sequence ? stamp.sequence : nodeId;
    const auto found = cells_.find(stamp.key);
    if (found == cells_.end()) {
      cells_.emplace(stamp.key, Cell{stamp.elevation, nodeId, sequence});
    } else if ((nodeId > 0 && sequence >= found->second.sequence) || (nodeId <= 0 && stamp.elevation > found->second.elevation)) {
      found->second = {stamp.elevation, nodeId, sequence};
    }
  }
}

void ElevationLayer::rebuild() {
  for (const Key tile : dirtyTiles_) {
    reset(tile);
    const auto nodes = tileNodes_.find(tile);
    if (nodes == tileNodes_.end()) {
      continue;
    }
    for (const int nodeId : nodes->second) {
      if (nodeId <= 0) {
        continue;
      }
      stamp(nodeId, nodeRasters_.at(nodeId).tiles.at(tile));
    }
    const auto temporary = nodes->second.find(-1);
    if (temporary != nodes->second.end()) {
      stamp(*temporary, nodeRasters_.at(*temporary).tiles.at(tile));
    }
  }
  dirtyTiles_.clear();
}

void ElevationLayer::invalidate() noexcept {
  ++revision_;
  snapshotRevision_ = std::numeric_limits<std::uint64_t>::max();
}

void ElevationLayer::rebuildSnapshot() const {
  if (snapshotRevision_ == revision_) {
    return;
  }
  delete gridMap_;
  gridMap_ = new grid_map::GridMap;
  if (cells_.empty()) {
    snapshotRevision_ = revision_;
    return;
  }

  std::int32_t minX = std::numeric_limits<std::int32_t>::max();
  std::int32_t minY = std::numeric_limits<std::int32_t>::max();
  std::int32_t maxX = std::numeric_limits<std::int32_t>::lowest();
  std::int32_t maxY = std::numeric_limits<std::int32_t>::lowest();
  for (const auto& [key, cell] : cells_) {
    static_cast<void>(cell);
    minX = std::min(minX, unpackX(key));
    minY = std::min(minY, unpackY(key));
    maxX = std::max(maxX, unpackX(key));
    maxY = std::max(maxY, unpackY(key));
  }

  const int width = maxX - minX + 1;
  const int height = maxY - minY + 1;
  const grid_map::Length length(static_cast<double>(width) * cellSize_, static_cast<double>(height) * cellSize_);
  const grid_map::Position center((static_cast<double>(minX) + width * 0.5) * cellSize_, (static_cast<double>(minY) + height * 0.5) * cellSize_);
  gridMap_->setGeometry(length, cellSize_, center);
  gridMap_->add("elevation");
  gridMap_->add("node_ids");
  gridMap_->setBasicLayers({"elevation"});

  for (const auto& [key, cell] : cells_) {
    const grid_map::Position position((static_cast<double>(unpackX(key)) + 0.5) * cellSize_, (static_cast<double>(unpackY(key)) + 0.5) * cellSize_);
    grid_map::Index index;
    if (gridMap_->getIndex(position, index)) {
      gridMap_->at("elevation", index) = cell.elevation;
      gridMap_->at("node_ids", index) = static_cast<float>(cell.owner);
    }
  }
  snapshotRevision_ = revision_;
}

const grid_map::GridMap* ElevationLayer::gridMap() const {
  rebuildSnapshot();
  return gridMap_;
}

cv::Mat ElevationLayer::createHeightMap(float& xMin, float& yMin, float& cellSize) const { return toImage(xMin, yMin, cellSize); }

cv::Mat ElevationLayer::toImage(float& xMin, float& yMin, float& cellSize) const {
  rebuildSnapshot();
  if (!gridMap_->hasBasicLayers()) {
    return {};
  }
  const grid_map::Matrix& elevation = (*gridMap_)["elevation"];
  cv::Mat image = cv::Mat::zeros(gridMap_->getSize()(1), gridMap_->getSize()(0), CV_32FC1);
  for (grid_map::GridMapIterator iterator(*gridMap_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    const grid_map::Index output(iterator.getUnwrappedIndex());
    if (std::isfinite(elevation(index(0), index(1)))) {
      image.at<float>(image.rows - 1 - output(1), image.cols - 1 - output(0)) = elevation(index(0), index(1));
    }
  }
  xMin = gridMap_->getPosition().x() - gridMap_->getLength().x() * 0.5f;
  yMin = gridMap_->getPosition().y() - gridMap_->getLength().y() * 0.5f;
  cellSize = gridMap_->getResolution();
  return image;
}

unsigned long ElevationLayer::getMemoryUsed() const {
  std::size_t rasterCells = 0;
  for (const auto& [nodeId, raster] : nodeRasters_) {
    static_cast<void>(nodeId);
    for (const auto& [tile, cells] : raster.tiles) {
      static_cast<void>(tile);
      rasterCells += cells.size();
    }
  }
  return IncrementalLayer<ElevationLayer>::getMemoryUsed() + cells_.size() * (sizeof(Key) + sizeof(Cell)) + rasterCells * sizeof(CellStamp);
}

}  // namespace sapphire::backend
