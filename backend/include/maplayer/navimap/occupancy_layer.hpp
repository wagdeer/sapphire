#pragma once

#include <backend/common/common_type.hpp>
#include <backend/params/navimap.hpp>

#include <cstdint>
#include <backend/maplayer/navimap/incremental_layer.hpp>
#include <limits>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace sapphire::backend {

class OccupancyGrid : public IncrementalLayer<OccupancyGrid> {
 public:
  OccupancyGrid(const LocalGridCache* cache, const ParametersMap& parameters = ParametersMap());
  OccupancyGrid(const LocalGridCache* cache, const NavimapParams& parameters);

  void setMap(const cv::Mat& map, float xMin, float yMin, float cellSize, const std::map<int, StateGroupf>& poses);

  void clear();

  cv::Mat getMap(float& xMin, float& yMin) const;
  cv::Mat getProbMap(float& xMin, float& yMin) const;
  void getMaps(cv::Mat& stateMap, cv::Mat& probabilityMap, float& xMin, float& yMin) const;

  static cv::Mat toImage(const cv::Mat& map, bool pgmFormat = false);
  // Decode the fixed PGM palette (occupied=0, unknown=205, free=254).
  // These are file-format thresholds, independent of node.navimap.raster.occupied_probability.
  static constexpr double kPgmOccupiedThreshold = 0.65;
  static constexpr double kPgmFreeThreshold = 0.196;

  unsigned long getMemoryUsed() const;
  // The caller will provide source evidence again on the next pose/content update.
  // Keep projected contributions for exact removal, but do not pin source payloads.
  void release_source(int id) { auto it = nodeRasters_.find(id); if (it != nodeRasters_.end()) it->second.grid.reset(); }
  std::uint64_t revision() const noexcept { return revision_; }

 private:
  friend class IncrementalLayer<OccupancyGrid>;
  bool apply(const std::vector<int>& removed, const std::vector<std::pair<int, StateGroupf>>& changed);
  using Key = std::uint64_t;

  enum Stamp : std::uint8_t {
    Miss = 1U << 0U,
    Hit = 1U << 1U,
    Footprint = 1U << 2U,
    Classified = 1U << 3U,
    ClassifiedHit = 1U << 4U,
  };

  struct Cell {
    float logOdds = 0.0f;
    int owner = 0;
    bool occupied = false;
    bool observed = false;
    bool footprint = false;
    // One signed vote per independently classified submap; ties are unknown.
    // Track presence separately so a tie retains its observed extent.
    bool classified = false;
    std::int64_t votes = 0;
  };

  struct CellStamp {
    Key key = 0;
    std::uint8_t flags = 0;
    int sequence = 0;
  };

  struct NodeRaster {
    StateGroupf pose;
    std::shared_ptr<const LocalGrid> grid;
    std::unordered_map<Key, std::vector<CellStamp>> tiles;
  };

  static constexpr std::int32_t kTileSize = 32;
  // Rebuilt evidence is consumed only as state/probability pixels. Store those
  // contiguously per tile instead of a second hash node for every world cell.
  struct RasterTile {
    std::array<int8_t, kTileSize * kTileSize> state, probability;
    int minX = kTileSize, minY = kTileSize, maxX = -1, maxY = -1;
    RasterTile() { state.fill(-1); probability.fill(-1); }
    bool empty() const { return maxX < minX; }
  };

  static Key pack(std::int32_t x, std::int32_t y) noexcept;
  static std::int32_t unpackX(Key key) noexcept;
  static std::int32_t unpackY(Key key) noexcept;
  static std::int32_t floorDiv(std::int32_t value, std::int32_t divisor) noexcept;
  static Key tileOf(Key cell) noexcept;

  NodeRaster rasterize(const StateGroupf& pose, std::shared_ptr<const LocalGrid> grid) const;
  void indexNode(int nodeId, const NodeRaster& raster);
  void unindexNode(int nodeId, const NodeRaster& raster);
  void rebuild();
  void reset(Key tile);
  void stamp_cell(Cell& cell, int nodeId, const CellStamp& stamp);
  void render_cell(RasterTile& tile, size_t slot, const Cell& cell) const;
  void rebuildSnapshot() const;
  void invalidate() noexcept;

  std::unordered_map<Key, RasterTile> tiles_;
  std::unordered_map<Key, Cell> imported_;
  std::unordered_map<int, NodeRaster> nodeRasters_;
  std::unordered_map<Key, std::set<int>> tileNodes_;
  std::unordered_set<Key> dirtyTiles_;

  mutable cv::Mat stateSnapshot_;
  mutable cv::Mat probabilitySnapshot_;
  mutable float snapshotXMin_ = 0.0f;
  mutable float snapshotYMin_ = 0.0f;
  std::uint64_t revision_ = 0;
  mutable std::uint64_t snapshotRevision_ = std::numeric_limits<std::uint64_t>::max();

  bool erode_;
  float footprintRadius_;
};

}  // namespace sapphire::backend
