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

namespace grid_map {
class GridMap;
}

namespace sapphire::backend {

class ElevationLayer : public IncrementalLayer<ElevationLayer> {
 public:
  ElevationLayer(const LocalGridCache* cache, const ParametersMap& parameters = {});
  ElevationLayer(const LocalGridCache* cache, const NavimapParams& parameters);

  ~ElevationLayer();
  void clear();

  const grid_map::GridMap* gridMap() const;

  cv::Mat createHeightMap(float& xMin, float& yMin, float& cellSize) const;
  unsigned long getMemoryUsed() const;
  void release_source(int id) { auto it = nodeRasters_.find(id); if (it != nodeRasters_.end()) it->second.grid.reset(); }

 private:
  friend class IncrementalLayer<ElevationLayer>;
  bool apply(const std::vector<int>& removed, const std::vector<std::pair<int, StateGroupf>>& changed);

  using Key = std::uint64_t;

  struct Cell {
    float elevation = 0.0f;
    int owner = 0;
    int sequence = 0;
  };

  struct CellStamp {
    Key key = 0;
    float elevation = 0.0f;
    int sequence = 0;
  };

  struct NodeRaster {
    StateGroupf pose;
    std::shared_ptr<const LocalGrid> grid;
    std::unordered_map<Key, std::vector<CellStamp>> tiles;
  };

  static constexpr std::int32_t kTileSize = 32;

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
  void stamp(int nodeId, const std::vector<CellStamp>& cells);
  void rebuildSnapshot() const;
  void invalidate() noexcept;
  cv::Mat toImage(float& xMin, float& yMin, float& cellSize) const;

  mutable grid_map::GridMap* gridMap_;
  std::unordered_map<Key, Cell> cells_;
  std::unordered_map<int, NodeRaster> nodeRasters_;
  std::unordered_map<Key, std::set<int>> tileNodes_;
  std::unordered_set<Key> dirtyTiles_;
  std::uint64_t revision_ = 0;
  mutable std::uint64_t snapshotRevision_ = std::numeric_limits<std::uint64_t>::max();
};

}  // namespace sapphire::backend
