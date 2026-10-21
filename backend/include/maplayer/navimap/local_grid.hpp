#pragma once
#include <array>
#include <cstddef>
#include <map>
#include <memory>
#include <vector>
#include <opencv2/core/mat.hpp>
#include <backend/tools/simd.hpp>
#include <backend/maplayer/navimap/navimap_grid.hpp>
#include <backend/sensor/camera/perspective.hpp>

namespace sapphire::backend {

struct GridCells {
  simd::SoA2<float> xy;
  simd::aligned_vector<float> z;
  simd::SoA3<float> normals;

  std::size_t size() const noexcept;
  bool empty() const noexcept;
  bool valid() const noexcept;
  bool is3D() const noexcept;
  bool hasNormals() const noexcept;
  std::size_t memoryUsed() const noexcept;

  void clear() noexcept;
  void reserve(std::size_t count);
  void push2D(float x, float y);
  void push3D(float x, float y, float zValue);

  static GridCells fromCv(const cv::Mat& cells);
  cv::Mat toCv() const;
};

struct LocalGrid {
  struct DepthImage { cv::Mat meters; camera::Perspective camera; };
  // Transient, normalized observation; never retained by a fused map or archive.
  std::vector<DepthImage> depth_images;
  Eigen::Vector4f ground_plane = Eigen::Vector4f::Zero(); // height = n.dot(body_point) + offset
  float min_obstacle_height = 0, max_obstacle_height = 0;
  float min_depth = 0, max_depth = std::numeric_limits<float>::infinity();
  StateGroupf anchor_from_grid = StateGroupf::identity();
  LocalGrid() = default;
  LocalGrid(GridCells ground, GridCells obstacles, GridCells empty, float cellSize, std::array<float, 3> viewPoint = {});
  LocalGrid(const cv::Mat& ground, const cv::Mat& obstacles, const cv::Mat& empty, float cellSize, const cv::Point3f& viewPoint = cv::Point3f());

  bool is3D() const noexcept;
  void rayTraceFreeSpace();
  std::size_t memoryUsed() const noexcept;

  // Populated only for a fused LocalMap snapshot; consumed without another raycast.
  bool fused_map = false;
  // Rejected geometric observation: retain its trajectory record, but do not
  // apply hit/miss, elevation, or footprint clearing from this frame.
  bool skip_integration = false;
  std::vector<NavimapCell> fused_cells;
  GridCells ground;
  GridCells obstacles;
  GridCells empty;
  float cellSize = 0.0f;
  std::array<float, 3> viewPoint = {};
};

class LocalGridCache {
 public:
  void add(int nodeId, const cv::Mat& ground, const cv::Mat& obstacles, const cv::Mat& empty, float cellSize, const cv::Point3f& viewPoint = cv::Point3f());
  void add(int nodeId, LocalGrid localGrid);
  void add(int nodeId, std::shared_ptr<const LocalGrid> localGrid);
  bool shareTo(int nodeId, LocalGridCache& anotherCache) const;
  unsigned long getMemoryUsed() const;
  void clear(bool temporaryOnly = false);

  void erase(int id) { localGrids_.erase(id); }
  std::size_t size() const noexcept { return localGrids_.size(); }
  bool empty() const noexcept { return localGrids_.empty(); }
  const std::map<int, std::shared_ptr<const LocalGrid>>& localGrids() const noexcept { return localGrids_; }
  std::map<int, std::shared_ptr<const LocalGrid>>::const_iterator find(int nodeId) const { return localGrids_.find(nodeId); }
  std::map<int, std::shared_ptr<const LocalGrid>>::const_iterator begin() const { return localGrids_.begin(); }
  std::map<int, std::shared_ptr<const LocalGrid>>::const_iterator end() const { return localGrids_.end(); }

 private:
  std::map<int, std::shared_ptr<const LocalGrid>> localGrids_;
};


}  // namespace sapphire::backend
