#include <cmath>
#include <cstdint>
#include <backend/maplayer/navimap/local_grid.hpp>
#include <backend/tools/logging.hpp>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace sapphire::backend {

std::size_t GridCells::size() const noexcept { return xy.size(); }

bool GridCells::empty() const noexcept { return xy.empty(); }

bool GridCells::valid() const noexcept {
  return (z.empty() || z.size() == size()) && (normals.empty() || normals.size() == size()) && !(z.empty() && !normals.empty());
}

bool GridCells::is3D() const noexcept { return empty() || z.size() == size(); }

bool GridCells::hasNormals() const noexcept { return !normals.empty() && normals.size() == size(); }

std::size_t GridCells::memoryUsed() const noexcept {
  return (xy.x.capacity() + xy.y.capacity() + z.capacity() + normals.x.capacity() + normals.y.capacity() + normals.z.capacity()) * sizeof(float);
}

void GridCells::clear() noexcept {
  xy.clear();
  z.clear();
  normals.clear();
}

void GridCells::reserve(std::size_t count) {
  xy.reserve(count);
  z.reserve(count);
}

void GridCells::push2D(float x, float y) {
  if (!z.empty() || !normals.empty()) {
    throw std::logic_error("Cannot append 2D cells to a 3D grid");
  }
  xy.push_back(x, y);
}

void GridCells::push3D(float x, float y, float zValue) {
  if (!xy.empty() && z.empty()) {
    throw std::logic_error("Cannot append 3D cells to a 2D grid");
  }
  xy.push_back(x, y);
  z.push_back(zValue);
}

GridCells GridCells::fromCv(const cv::Mat& cells) {
  GridCells output;
  if (cells.empty()) {
    return output;
  }
  if (cells.rows != 1 || cells.depth() != CV_32F || (cells.channels() != 2 && cells.channels() != 3 && cells.channels() != 6)) {
    throw std::invalid_argument("Local grid cells must be 1xN CV_32FC2, CV_32FC3 or CV_32FC6");
  }
  output.xy.reserve(cells.cols);
  if (cells.channels() != 2) {
    output.z.reserve(cells.cols);
  }
  if (cells.channels() == 6) {
    output.normals.reserve(cells.cols);
  }
  for (int i = 0; i < cells.cols; ++i) {
    const float* value = cells.ptr<float>(0, i);
    output.xy.push_back(value[0], value[1]);
    if (cells.channels() != 2) {
      output.z.push_back(value[2]);
    }
    if (cells.channels() == 6) {
      output.normals.push_back(value[3], value[4], value[5]);
    }
  }
  return output;
}

cv::Mat GridCells::toCv() const {
  if (!valid()) {
    throw std::logic_error("Cannot export invalid local grid cells");
  }
  if (empty()) {
    return {};
  }
  const int channels = hasNormals() ? 6 : (is3D() ? 3 : 2);
  cv::Mat output(1, static_cast<int>(size()), CV_MAKETYPE(CV_32F, channels));
  for (std::size_t i = 0; i < size(); ++i) {
    float* value = output.ptr<float>(0, static_cast<int>(i));
    value[0] = xy.x[i];
    value[1] = xy.y[i];
    if (channels >= 3) {
      value[2] = z[i];
    }
    if (channels == 6) {
      value[3] = normals.x[i];
      value[4] = normals.y[i];
      value[5] = normals.z[i];
    }
  }
  return output;
}

LocalGrid::LocalGrid(GridCells groundIn, GridCells obstaclesIn, GridCells emptyIn, float cellSizeIn, std::array<float, 3> viewPointIn)
    : ground(std::move(groundIn)), obstacles(std::move(obstaclesIn)), empty(std::move(emptyIn)), cellSize(cellSizeIn), viewPoint(viewPointIn) {
  EXT_ASSERT(cellSize > 0.0f);
}

LocalGrid::LocalGrid(const cv::Mat& groundIn, const cv::Mat& obstaclesIn, const cv::Mat& emptyIn, float cellSizeIn, const cv::Point3f& viewPointIn)
    : LocalGrid(GridCells::fromCv(groundIn), GridCells::fromCv(obstaclesIn), GridCells::fromCv(emptyIn), cellSizeIn,
                {viewPointIn.x, viewPointIn.y, viewPointIn.z}) {}

bool LocalGrid::is3D() const noexcept { return ground.is3D() && obstacles.is3D() && empty.is3D(); }

void LocalGrid::rayTraceFreeSpace() {
  if (cellSize <= 0.0f || (!ground.empty() && ground.is3D()) || (!obstacles.empty() && obstacles.is3D())) {
    throw std::logic_error("Ray tracing requires a valid 2D local grid");
  }
  const auto pack = [](std::int32_t x, std::int32_t y) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U) | static_cast<std::uint32_t>(y);
  };
  const float inverseCellSize = 1.0f / cellSize;
  std::unordered_set<std::uint64_t> occupied;
  occupied.reserve(ground.size() + obstacles.size());
  const auto addOccupied = [&](const GridCells& cells) {
    for (std::size_t i = 0; i < cells.size(); ++i) {
      occupied.insert(
          pack(static_cast<std::int32_t>(std::floor(cells.xy.x[i] * inverseCellSize)), static_cast<std::int32_t>(std::floor(cells.xy.y[i] * inverseCellSize))));
    }
  };
  addOccupied(ground);
  addOccupied(obstacles);

  std::unordered_set<std::uint64_t> freeCells;
  freeCells.reserve(occupied.size() * 2U);
  const std::int32_t originX = static_cast<std::int32_t>(std::floor(viewPoint[0] * inverseCellSize));
  const std::int32_t originY = static_cast<std::int32_t>(std::floor(viewPoint[1] * inverseCellSize));
  const auto trace = [&](std::int32_t targetX, std::int32_t targetY) {
    std::int32_t x = originX;
    std::int32_t y = originY;
    const std::int32_t deltaX = std::abs(targetX - originX);
    const std::int32_t deltaY = std::abs(targetY - originY);
    const std::int32_t stepX = originX < targetX ? 1 : -1;
    const std::int32_t stepY = originY < targetY ? 1 : -1;
    std::int32_t error = deltaX - deltaY;
    while (x != targetX || y != targetY) {
      const std::uint64_t key = pack(x, y);
      if (occupied.find(key) == occupied.end()) {
        freeCells.insert(key);
      }
      const std::int32_t doubledError = error * 2;
      if (doubledError > -deltaY) {
        error -= deltaY;
        x += stepX;
      }
      if (doubledError < deltaX) {
        error += deltaX;
        y += stepY;
      }
    }
  };
  const auto traceLayer = [&](const GridCells& cells) {
    for (std::size_t i = 0; i < cells.size(); ++i) {
      trace(static_cast<std::int32_t>(std::floor(cells.xy.x[i] * inverseCellSize)), static_cast<std::int32_t>(std::floor(cells.xy.y[i] * inverseCellSize)));
    }
  };
  traceLayer(ground);
  traceLayer(obstacles);

  empty.clear();
  empty.xy.reserve(freeCells.size());
  for (const std::uint64_t key : freeCells) {
    const std::int32_t x = static_cast<std::int32_t>(key >> 32U);
    const std::int32_t y = static_cast<std::int32_t>(key & 0xffffffffU);
    empty.push2D((static_cast<float>(x) + 0.5f) * cellSize, (static_cast<float>(y) + 0.5f) * cellSize);
  }
}

void LocalGridCache::add(int nodeId, const cv::Mat& groundIn, const cv::Mat& obstaclesIn, const cv::Mat& emptyIn, float cellSize,
                         const cv::Point3f& viewPoint) {
  add(nodeId, LocalGrid(groundIn, obstaclesIn, emptyIn, cellSize, viewPoint));
}

void LocalGridCache::add(int nodeId, LocalGrid localGrid) { add(nodeId, std::make_shared<const LocalGrid>(std::move(localGrid))); }

void LocalGridCache::add(int nodeId, std::shared_ptr<const LocalGrid> localGrid) {
  if (nodeId < 0) {
    EXT_LOG_WARN("Cannot add nodes with negative id (nodeId={})", nodeId);
    return;
  }
  if (!localGrid) {
    throw std::invalid_argument("Cannot cache a null LocalGrid");
  }
  localGrids_.insert_or_assign(nodeId == 0 ? -1 : nodeId, std::move(localGrid));
}

bool LocalGridCache::shareTo(int nodeId, LocalGridCache& anotherCache) const {
  const auto source = localGrids_.find(nodeId);
  if (source == localGrids_.end() || anotherCache.find(nodeId) != anotherCache.end()) {
    return false;
  }
  anotherCache.add(nodeId, source->second);
  return true;
}

std::size_t LocalGrid::memoryUsed() const noexcept {
  size_t depth_bytes = depth_images.capacity() * sizeof(DepthImage);
  for (const auto& image : depth_images) depth_bytes += image.meters.total() * image.meters.elemSize();
  return sizeof(LocalGrid) + depth_bytes + ground.memoryUsed() + obstacles.memoryUsed() + empty.memoryUsed() + fused_cells.capacity() * sizeof(NavimapCell);
}

unsigned long LocalGridCache::getMemoryUsed() const {
  std::size_t memoryUsage =
      localGrids_.size() * (sizeof(int) + sizeof(std::shared_ptr<const LocalGrid>) + sizeof(std::map<int, std::shared_ptr<const LocalGrid>>::iterator)) +
      sizeof(std::map<int, std::shared_ptr<const LocalGrid>>);
  for (const auto& entry : localGrids_) {
    memoryUsage += entry.second->memoryUsed();
  }
  return static_cast<unsigned long>(memoryUsage);
}

void LocalGridCache::clear(bool temporaryOnly) {
  if (temporaryOnly) {
    for (auto iter = localGrids_.begin(); iter != localGrids_.end() && iter->first < 0;) {
      iter = localGrids_.erase(iter);
    }
  } else {
    localGrids_.clear();
  }
}

}  // namespace sapphire::backend
