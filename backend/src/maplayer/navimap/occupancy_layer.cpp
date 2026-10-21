#include <backend/common/common_type.hpp>
#include <backend/params/navimap.hpp>
#include <backend/parameters.hpp>
#include <algorithm>
#include <climits>
#include <cmath>
#include <backend/maplayer/navimap/occupancy_layer.hpp>
#include <backend/tools/logging.hpp>

namespace sapphire::backend {

namespace {

cv::Mat erodeMap(const cv::Mat& map) {
  EXT_ASSERT(map.type() == CV_8SC1);
  cv::Mat result = map.clone();
  for (int row = 1; row < map.rows - 1; ++row) {
    for (int column = 1; column < map.cols - 1; ++column) {
      const auto value = map.at<std::int8_t>(row, column);
      if (value != 100 && value != -1) {
        continue;
      }
      const int freeNeighbors = (map.at<std::int8_t>(row + 1, column) == 0) + (map.at<std::int8_t>(row - 1, column) == 0) +
                                (map.at<std::int8_t>(row, column + 1) == 0) + (map.at<std::int8_t>(row, column - 1) == 0);
      // One pass over the original map: occupied and unknown cells share the
      // same free-space boundary rule, without cascading into larger regions.
      if (freeNeighbors >= 3) {
        result.at<std::int8_t>(row, column) = 0;
      }
    }
  }
  return result;
}

}  // namespace

cv::Mat OccupancyGrid::toImage(const cv::Mat& map, bool pgmFormat) {
  EXT_ASSERT(map.type() == CV_8SC1);
  cv::Mat image(map.rows, map.cols, CV_8UC1);
  for (int row = 0; row < map.rows; ++row) {
    const int sourceRow = pgmFormat ? map.rows - row - 1 : row;
    for (int column = 0; column < map.cols; ++column) {
      const std::int8_t value = map.at<std::int8_t>(sourceRow, column);
      std::uint8_t gray;
      if (value == 0) {
        gray = pgmFormat ? 254 : 178;
      } else if (value == 100) {
        gray = 0;
      } else if (value == -2) {
        gray = pgmFormat ? 254 : 200;
      } else if (pgmFormat || value == -1) {
        gray = pgmFormat ? 205 : 89;
      } else if (value > 50) {
        gray = static_cast<std::uint8_t>(static_cast<double>(100 - value) * 2.0 / 100.0 * 89.0);
      } else {
        gray = static_cast<std::uint8_t>(static_cast<double>(50 - value) * 2.0 / 100.0 * 89.0 + 89.0);
      }
      image.at<std::uint8_t>(row, column) = gray;
    }
  }
  return image;
}

OccupancyGrid::OccupancyGrid(const LocalGridCache* cache, const ParametersMap& parameters)
    : OccupancyGrid(cache, NavimapParams{}.withOverrides(parameters)) {}
OccupancyGrid::OccupancyGrid(const LocalGridCache* cache, const NavimapParams& parameters)
    : IncrementalLayer<OccupancyGrid>(cache, parameters),
      erode_(parameters.raster_erode_obstacles), footprintRadius_(parameters.raster_footprint_radius_m) {}

OccupancyGrid::Key OccupancyGrid::pack(std::int32_t x, std::int32_t y) noexcept {
  return (static_cast<Key>(static_cast<std::uint32_t>(x)) << 32U) | static_cast<std::uint32_t>(y);
}

std::int32_t OccupancyGrid::unpackX(Key key) noexcept { return static_cast<std::int32_t>(key >> 32U); }

std::int32_t OccupancyGrid::unpackY(Key key) noexcept { return static_cast<std::int32_t>(key); }

std::int32_t OccupancyGrid::floorDiv(std::int32_t value, std::int32_t divisor) noexcept {
  const std::int32_t quotient = value / divisor;
  return value % divisor < 0 ? quotient - 1 : quotient;
}

OccupancyGrid::Key OccupancyGrid::tileOf(Key cell) noexcept { return pack(floorDiv(unpackX(cell), kTileSize), floorDiv(unpackY(cell), kTileSize)); }

void OccupancyGrid::clear() {
  tiles_.clear();
  imported_.clear();
  nodeRasters_.clear();
  tileNodes_.clear();
  dirtyTiles_.clear();
  stateSnapshot_.release();
  probabilitySnapshot_.release();
  ++revision_;
  snapshotRevision_ = std::numeric_limits<std::uint64_t>::max();
  IncrementalLayer<OccupancyGrid>::clear();
}

void OccupancyGrid::setMap(const cv::Mat& map, float xMin, float yMin, float cellSize, const std::map<int, StateGroupf>& poses) {
  clear();
  if (map.empty()) {
    return;
  }
  EXT_ASSERT(map.type() == CV_8SC1 && cellSize > 0.0f);
  static_cast<void>(poses);
  cellSize_ = cellSize;
  const std::int32_t firstX = static_cast<std::int32_t>(std::floor(xMin / cellSize_));
  const std::int32_t firstY = static_cast<std::int32_t>(std::floor(yMin / cellSize_));
  imported_.reserve(map.total());
  for (int row = 0; row < map.rows; ++row) {
    const auto* source = map.ptr<std::int8_t>(row);
    for (int column = 0; column < map.cols; ++column) {
      if (source[column] < 0) {
        continue;
      }
      Cell cell;
      cell.logOdds = source[column] == 0 ? logOddsClampingMin_ : logOddsClampingMax_;
      cell.occupied = source[column] != 0;
      cell.observed = true;
      imported_.emplace(pack(firstX + column, firstY + row), cell);
    }
  }
  for (const auto& [key, cell] : imported_) {
    const Key tile = tileOf(key);
    const size_t slot = (unpackY(key) - unpackY(tile) * kTileSize) * kTileSize + unpackX(key) - unpackX(tile) * kTileSize;
    render_cell(tiles_[tile], slot, cell);
  }
  invalidate();
}

OccupancyGrid::NodeRaster OccupancyGrid::rasterize(const StateGroupf& pose, std::shared_ptr<const LocalGrid> grid) const {
  if (grid->skip_integration) {
    NodeRaster raster;
    raster.pose = pose;
    raster.grid = std::move(grid);
    return raster;
  }
  const auto grid_pose = pose.compose(grid->anchor_from_grid);
  const auto rotation = grid_pose.rotationMatrix();
  const auto translation = grid_pose.translationVector();
  if (grid->fused_map) {
    const float threshold = occupancyThr_ == 0.0f ? 0.0f : logOdds(occupancyThr_);
    std::unordered_map<Key, CellStamp> projected;
    projected.reserve(grid->fused_cells.size());
    for (const auto& cell : grid->fused_cells) {
      const Eigen::Vector3f p = rotation * Eigen::Vector3f((cell.x + .5f) * grid->cellSize, (cell.y + .5f) * grid->cellSize, cell.projection_z) + translation;
      const Key key = pack(std::floor(p.x() / cellSize_), std::floor(p.y() / cellSize_));
      // LocalMap still fuses its own observations. World assembly consumes its
      // current classification only, including snapshots of the active map.
      const bool occupied = occupancyThr_ == 0.0f ? (cell.flags & 1) : cell.evidence.apply(0) >= threshold;
      CellStamp stamp{key, static_cast<uint8_t>(Classified | (occupied ? ClassifiedHit : 0) |
          (cell.flags & 1 ? Hit : 0) | (cell.flags & 2 ? Footprint : 0)), cell.sequence};
      const auto old = projected.find(key);
      // Keep the existing source-cell collision rule; classification must not
      // change which source cell wins a rotation/resampling collision.
      if (old == projected.end() || stamp.sequence > old->second.sequence ||
          (stamp.sequence == old->second.sequence && (stamp.flags & (Hit | Footprint)) > (old->second.flags & (Hit | Footprint))))
        projected.insert_or_assign(key, stamp);
    }
    NodeRaster raster;
    raster.pose = pose;
    raster.grid = std::move(grid);
    for (const auto& [key, stamp] : projected) raster.tiles[tileOf(key)].push_back(stamp);
    return raster;
  }
  std::unordered_map<Key, std::uint8_t> stamps;
  stamps.reserve(grid->ground.size() + grid->empty.size() + grid->obstacles.size());

  const auto collect = [&](const GridCells& points, std::uint8_t flag) {
    for (std::size_t i = 0; i < points.size(); ++i) {
      const float z = points.is3D() ? points.z[i] : 0.0f;
      const Eigen::Vector3f transformed = rotation * Eigen::Vector3f(points.xy.x[i], points.xy.y[i], z) + translation;
      const float x = transformed.x();
      const float y = transformed.y();
      if (!std::isfinite(x) || !std::isfinite(y)) {
        continue;
      }
      stamps[pack(static_cast<std::int32_t>(std::floor(x / cellSize_)), static_cast<std::int32_t>(std::floor(y / cellSize_)))] |= flag;
    }
  };

  collect(grid->ground, Miss);
  collect(grid->empty, Miss);
  collect(grid->obstacles, Hit);

  if (footprintRadius_ >= cellSize_ * 1.5f) {
    const std::int32_t centerX = static_cast<std::int32_t>(std::floor(pose.x() / cellSize_));
    const std::int32_t centerY = static_cast<std::int32_t>(std::floor(pose.y() / cellSize_));
    const std::int32_t radius = static_cast<std::int32_t>(std::ceil(footprintRadius_ / cellSize_));
    const float radiusSquared = footprintRadius_ * footprintRadius_;
    for (std::int32_t y = centerY - radius; y <= centerY + radius; ++y) {
      for (std::int32_t x = centerX - radius; x <= centerX + radius; ++x) {
        const float dx = (static_cast<float>(x) + 0.5f) * cellSize_ - pose.x();
        const float dy = (static_cast<float>(y) + 0.5f) * cellSize_ - pose.y();
        if (dx * dx + dy * dy <= radiusSquared) {
          stamps[pack(x, y)] |= Footprint;
        }
      }
    }
  }

  NodeRaster raster;
  raster.pose = pose;
  raster.grid = std::move(grid);
  for (const auto& [cell, flags] : stamps) {
    raster.tiles[tileOf(cell)].push_back({cell, flags});
  }
  return raster;
}

void OccupancyGrid::indexNode(int nodeId, const NodeRaster& raster) {
  for (const auto& [tile, cells] : raster.tiles) {
    static_cast<void>(cells);
    tileNodes_[tile].insert(nodeId);
  }
}

void OccupancyGrid::unindexNode(int nodeId, const NodeRaster& raster) {
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

bool OccupancyGrid::apply(const std::vector<int>& removed, const std::vector<std::pair<int, StateGroupf>>& changed) {
  bool touched = false;
  const bool replacingImport = !imported_.empty() && std::any_of(changed.begin(), changed.end(), [](const auto& entry) { return entry.first > 0; });
  if (replacingImport) {
    imported_.clear();
    tiles_.clear();
    touched = true;
  }

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
    if (!source || source->cellSize <= 0.0f) {
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

void OccupancyGrid::reset(Key tile) {
  RasterTile pixels;
  const int32_t firstX = unpackX(tile) * kTileSize, firstY = unpackY(tile) * kTileSize;
  for (int y = 0; y < kTileSize; ++y) for (int x = 0; x < kTileSize; ++x) {
    const auto base = imported_.find(pack(firstX + x, firstY + y));
    if (base != imported_.end()) render_cell(pixels, y * kTileSize + x, base->second);
  }
  if (pixels.empty()) tiles_.erase(tile);
  else tiles_.insert_or_assign(tile, std::move(pixels));
}

void OccupancyGrid::render_cell(RasterTile& tile, size_t slot, const Cell& cell) const {
  if (!cell.observed && !cell.classified) return;
  const float threshold = occupancyThr_ == 0.0f ? 0.0f : logOdds(occupancyThr_);
  const bool occupied = occupancyThr_ == 0.0f ? cell.occupied : cell.logOdds >= threshold;
  if (cell.classified) {
    // Legacy frame grids, if mixed in, form one independently classified
    // contribution, with the same unit weight as a compact node.
    const auto votes = cell.votes + (cell.observed ? (occupied ? 1 : -1) : 0);
    tile.state[slot] = votes == 0 ? -1 : (votes > 0 ? 100 : 0);
    // There is no fused posterior across classified maps. Keep the diagnostic
    // layer categorical too; tied votes remain unknown.
    tile.probability[slot] = tile.state[slot];
  } else {
    tile.state[slot] = occupied ? 100 : 0;
    tile.probability[slot] = static_cast<int8_t>(probability(cell.logOdds) * 100.0f);
  }
  const int x = slot % kTileSize, y = slot / kTileSize;
  tile.minX = std::min(tile.minX, x); tile.maxX = std::max(tile.maxX, x);
  tile.minY = std::min(tile.minY, y); tile.maxY = std::max(tile.maxY, y);
}

void OccupancyGrid::stamp_cell(Cell& cell, int nodeId, const CellStamp& stamp) {
  if (stamp.flags & Classified) {
    cell.classified = true;
    cell.votes += stamp.flags & ClassifiedHit ? 1 : -1;
    return;
  }
  if ((stamp.flags & Footprint) != 0U) {
    cell.logOdds = logOddsClampingMin_;
    cell.owner = nodeId;
    cell.occupied = false;
    cell.observed = true;
    cell.footprint = true;
    return;
  }
  if (cell.footprint) {
    if (nodeId > 0 && cell.owner > 0 && nodeId <= cell.owner) {
      return;
    }
    cell.footprint = false;
  }
  if ((stamp.flags & Miss) != 0U && (stamp.flags & Hit) == 0U) {
    cell.logOdds = std::clamp(cell.logOdds + logOddsMiss_, logOddsClampingMin_, logOddsClampingMax_);
    cell.occupied = false;
  }
  if ((stamp.flags & Hit) != 0U) {
    cell.logOdds = std::clamp(cell.logOdds + logOddsHit_, logOddsClampingMin_, logOddsClampingMax_);
    cell.occupied = true;
  }
  cell.owner = nodeId;
  cell.observed = true;
}

void OccupancyGrid::rebuild() {
  // Cells are independent. Bucket by cell before ordering evidence, instead of
  // sorting all contributions across the whole tile and hashing every stamp.
  std::vector<std::pair<int, const CellStamp*>> ordered;
  for (const Key tile : dirtyTiles_) {
    const auto nodes = tileNodes_.find(tile);
    if (nodes == tileNodes_.end()) { reset(tile); continue; }
    constexpr size_t count = kTileSize * kTileSize;
    std::array<size_t, count + 1> offsets{};
    const int32_t firstX = unpackX(tile) * kTileSize, firstY = unpackY(tile) * kTileSize;
    const auto slot = [&](Key key) { return size_t(unpackY(key) - firstY) * kTileSize + size_t(unpackX(key) - firstX); };
    for (int id : nodes->second)
      for (const auto& stamp : nodeRasters_.at(id).tiles.at(tile)) ++offsets[slot(stamp.key) + 1];
    for (size_t i = 1; i <= count; ++i) offsets[i] += offsets[i - 1];
    ordered.resize(offsets.back());
    auto cursor = offsets;
    for (int id : nodes->second)
      for (const auto& stamp : nodeRasters_.at(id).tiles.at(tile)) ordered[cursor[slot(stamp.key)]++] = {id, &stamp};
    RasterTile pixels;
    for (size_t i = 0; i < count; ++i) {
      const Key key = pack(firstX + static_cast<int32_t>(i % kTileSize), firstY + static_cast<int32_t>(i / kTileSize));
      auto begin = ordered.begin() + offsets[i], end = ordered.begin() + offsets[i + 1];
      const auto base = imported_.find(key);
      if (begin == end && base == imported_.end()) continue;
      Cell cell = base == imported_.end() ? Cell{} : base->second;
      // The normal compact-submap path is commutative and needs no ordering.
      // Only legacy per-frame grids retain temporal hit/miss semantics.
      if (!std::all_of(begin, end, [](const auto& entry) { return entry.second->flags & Classified; }))
        std::sort(begin, end, [](const auto& a, const auto& b) {
          const int sa = a.first < 0 ? INT_MAX : (a.second->sequence ? a.second->sequence : a.first);
          const int sb = b.first < 0 ? INT_MAX : (b.second->sequence ? b.second->sequence : b.first);
          return sa != sb ? sa < sb : a.first < b.first;
        });
      for (auto it = begin; it != end; ++it) stamp_cell(cell, it->first, *it->second);
      render_cell(pixels, i, cell);
    }
    if (pixels.empty()) tiles_.erase(tile);
    else tiles_.insert_or_assign(tile, std::move(pixels));
  }
  dirtyTiles_.clear();
}

void OccupancyGrid::invalidate() noexcept {
  ++revision_;
  snapshotRevision_ = std::numeric_limits<std::uint64_t>::max();
}

void OccupancyGrid::rebuildSnapshot() const {
  if (snapshotRevision_ == revision_) {
    return;
  }
  stateSnapshot_.release();
  probabilitySnapshot_.release();
  snapshotXMin_ = 0.0f;
  snapshotYMin_ = 0.0f;
  if (tiles_.empty()) {
    snapshotRevision_ = revision_;
    return;
  }

  std::int32_t minX = std::numeric_limits<std::int32_t>::max();
  std::int32_t minY = std::numeric_limits<std::int32_t>::max();
  std::int32_t maxX = std::numeric_limits<std::int32_t>::lowest();
  std::int32_t maxY = std::numeric_limits<std::int32_t>::lowest();
  for (const auto& [key, tile] : tiles_) {
    const int32_t x = unpackX(key) * kTileSize, y = unpackY(key) * kTileSize;
    minX = std::min(minX, x + tile.minX); maxX = std::max(maxX, x + tile.maxX);
    minY = std::min(minY, y + tile.minY); maxY = std::max(maxY, y + tile.maxY);
  }
  if (minX > maxX || minY > maxY) {
    snapshotRevision_ = revision_;
    return;
  }

  const int width = maxX - minX + 1;
  const int height = maxY - minY + 1;
  stateSnapshot_ = cv::Mat(height, width, CV_8SC1, cv::Scalar(-1));
  probabilitySnapshot_ = cv::Mat(height, width, CV_8SC1, cv::Scalar(-1));
  for (const auto& [key, tile] : tiles_) {
    const int32_t x = unpackX(key) * kTileSize - minX, y = unpackY(key) * kTileSize - minY;
    for (int row = tile.minY; row <= tile.maxY; ++row) {
      const size_t offset = row * kTileSize + tile.minX, count = tile.maxX - tile.minX + 1;
      std::copy_n(tile.state.data() + offset, count, stateSnapshot_.ptr<int8_t>(y + row) + x + tile.minX);
      std::copy_n(tile.probability.data() + offset, count, probabilitySnapshot_.ptr<int8_t>(y + row) + x + tile.minX);
    }
  }
  if (erode_) {
    stateSnapshot_ = erodeMap(stateSnapshot_);
  }
  snapshotXMin_ = static_cast<float>(minX) * cellSize_;
  snapshotYMin_ = static_cast<float>(minY) * cellSize_;
  snapshotRevision_ = revision_;
}

cv::Mat OccupancyGrid::getMap(float& xMin, float& yMin) const {
  rebuildSnapshot();
  xMin = snapshotXMin_;
  yMin = snapshotYMin_;
  return stateSnapshot_;
}

cv::Mat OccupancyGrid::getProbMap(float& xMin, float& yMin) const {
  rebuildSnapshot();
  xMin = snapshotXMin_;
  yMin = snapshotYMin_;
  return probabilitySnapshot_;
}

void OccupancyGrid::getMaps(cv::Mat& stateMap, cv::Mat& probabilityMap, float& xMin, float& yMin) const {
  rebuildSnapshot();
  stateMap = stateSnapshot_;
  probabilityMap = probabilitySnapshot_;
  xMin = snapshotXMin_;
  yMin = snapshotYMin_;
}

unsigned long OccupancyGrid::getMemoryUsed() const {
  std::size_t rasterCells = 0;
  for (const auto& [nodeId, raster] : nodeRasters_) {
    static_cast<void>(nodeId);
    for (const auto& [tile, cells] : raster.tiles) {
      static_cast<void>(tile);
      rasterCells += cells.size();
    }
  }
  return IncrementalLayer<OccupancyGrid>::getMemoryUsed() + tiles_.size() * (sizeof(Key) + sizeof(RasterTile)) + imported_.size() * (sizeof(Key) + sizeof(Cell)) +
         rasterCells * sizeof(CellStamp) + stateSnapshot_.total() * stateSnapshot_.elemSize() + probabilitySnapshot_.total() * probabilitySnapshot_.elemSize();
}

}  // namespace sapphire::backend
