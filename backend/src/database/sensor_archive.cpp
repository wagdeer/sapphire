#include "backend/database/sensor_archive.hpp"

#include <array>
#include <functional>
#include <stdexcept>

#include "backend/database/compression.hpp"
#include "backend/tools/parallel_executor.hpp"

namespace sapphire::backend {
namespace {
size_t bytes(const cv::Mat& m) { return m.total() * m.elemSize(); }

// Keep small batches on the caller; use the same policy for encoding/decoding.
void invoke(tools::ParallelExecutor<>* executor, const std::vector<std::function<void()>>& tasks, size_t work) {
  if (executor && tasks.size() > 1 && work >= 64 * 1024) {
    executor->parallel_invoke(tasks.size(), [&](size_t i) { tasks[i](); });
  } else {
    for (const auto& task : tasks) task();
  }
}
void checkBytes(const cv::Mat& m) {
  if (!m.empty() && (m.type() != CV_8UC1 || m.rows != 1)) throw std::invalid_argument("Archive payload must be a byte row");
}
}  // namespace

void SensorArchive::clear(bool images, bool userData, bool occupancyGrid) {
  if (images) {
    image.release();
    depth.release();
  }
  if (userData) user.release();
  if (occupancyGrid) grid = {};
}
std::size_t SensorArchive::memoryUsed() const {
  // Restored/caller-provided payloads may still be views of larger allocations.
  const auto allocated = [](const cv::Mat& m) { return m.u ? m.u->size : bytes(m); };
  return sizeof(SensorArchive) + allocated(image) + allocated(depth) + allocated(user) + allocated(grid.ground) + allocated(grid.obstacles) + allocated(grid.empty);
}
SensorRecord archivedSensorData(const cv::Mat& image, const cv::Mat& depth, const std::vector<camera::Perspective>& cameras, int id, double stamp,
                                const cv::Mat& user) {
  checkBytes(image);
  checkBytes(depth);
  checkBytes(user);
  SensorRecord record;
  record.observation = SensorData({}, {}, cameras, id, stamp);
  record.archive.image = image;
  record.archive.depth = depth;
  record.archive.user = user;
  return record;
}
void selectSensorData(SensorRecord& record, bool images, bool userData, bool occupancyGrid) {
  record.archive.clear(!images, !userData, !occupancyGrid);
  record.observation.clearRawData(!images, !userData, !occupancyGrid);
  if (!images) record.observation.setImageData({}, {}, std::vector<camera::Perspective>{});
}

void encodeSensorData(SensorRecord& record, bool images, bool userData, bool occupancyGrid, tools::ParallelExecutor<>* executor) {
  // Build a private archive so failed compression cannot publish a partial batch.
  auto archive = record.archive;
  const auto& data = record.observation;
  std::vector<std::function<void()>> tasks;
  size_t work = 0;
  if (images && !data.imageRaw().empty()) {
    work += bytes(data.imageRaw());
    tasks.emplace_back([&] { archive.image = compressImage(data.imageRaw()); });
  }
  if (images && !data.depthRaw().empty()) {
    work += bytes(data.depthRaw());
    tasks.emplace_back([&] { archive.depth = compressDepth(data.depthRaw()); });
  }
  if (userData && !data.userDataRaw().empty()) {
    work += bytes(data.userDataRaw());
    tasks.emplace_back([&] { archive.user = compressData(data.userDataRaw()); });
  }
  if (occupancyGrid && data.localGrid()) {
    const auto& grid = *data.localGrid();
    archive.grid = {};
    archive.grid.cell_size = grid.cellSize;
    archive.grid.view_point = {grid.viewPoint[0], grid.viewPoint[1], grid.viewPoint[2]};
    const std::array<const GridCells*, 3> cells{&grid.ground, &grid.obstacles, &grid.empty};
    const std::array<cv::Mat*, 3> outputs{&archive.grid.ground, &archive.grid.obstacles, &archive.grid.empty};
    for (size_t i = 0; i < cells.size(); ++i) {
      if (cells[i]->empty()) continue;
      work += cells[i]->size() * (cells[i]->hasNormals() ? 6 : cells[i]->is3D() ? 3 : 2) * sizeof(float);
      tasks.emplace_back([cell = cells[i], output = outputs[i]] { *output = compressGridCells(*cell); });
    }
  }
  invoke(executor, tasks, work);
  record.archive = std::move(archive);
}

void decodeSensorDataConst(const SensorRecord& record, cv::Mat* image, cv::Mat* depth, cv::Mat* user, cv::Mat* ground, cv::Mat* obstacles, cv::Mat* empty,
                           tools::ParallelExecutor<>* executor) {
  const auto& data = record.observation;
  const auto& archive = record.archive;
  const std::array<cv::Mat*, 6> outputs{image, depth, user, ground, obstacles, empty};
  const std::array<const cv::Mat*, 6> encoded{&archive.image,       &archive.depth,          &archive.user,
                                              &archive.grid.ground, &archive.grid.obstacles, &archive.grid.empty};
  std::array<cv::Mat, 6> decoded{data.imageRaw(), data.depthRaw(), data.userDataRaw(), {}, {}, {}};
  if (data.localGrid()) {
    if (ground) decoded[3] = data.localGrid()->ground.toCv();
    if (obstacles) decoded[4] = data.localGrid()->obstacles.toCv();
    if (empty) decoded[5] = data.localGrid()->empty.toCv();
  }
  std::vector<std::function<void()>> tasks;
  size_t work = 0;
  for (size_t i = 0; i < outputs.size(); ++i) {
    if (!outputs[i] || !decoded[i].empty() || encoded[i]->empty() || (i >= 3 && data.localGrid())) continue;
    work += bytes(*encoded[i]);
    tasks.emplace_back([&, i] {
      decoded[i] = i == 0 ? uncompressImage(*encoded[i]) : i == 1 ? uncompressDepth(*encoded[i]) : uncompressData(*encoded[i]);
      if (decoded[i].empty()) throw std::runtime_error("Failed to decode sensor archive");
      if (i == 0 && decoded[i].type() != CV_8UC1) throw std::runtime_error("Archived image is not grayscale");
      if (i == 1 && decoded[i].type() != CV_16UC1) throw std::runtime_error("Archived depth is not uint16 millimeters");
    });
  }
  invoke(executor, tasks, work);
  for (size_t i = 0; i < outputs.size(); ++i)
    if (outputs[i]) *outputs[i] = decoded[i];
}

std::shared_ptr<const LocalGrid> decodeLocalGrid(SensorRecord& record, tools::ParallelExecutor<>* executor) {
  if (!record.observation.localGrid() && record.archive.grid.cell_size > 0) {
    cv::Mat ground, obstacles, empty;
    decodeSensorDataConst(record, nullptr, nullptr, nullptr, &ground, &obstacles, &empty, executor);
    const auto flatten = [](const cv::Mat& cells) { return cells.empty() ? cells : cells.reshape(cells.channels(), 1); };
    record.observation.setLocalGrid(
        std::make_shared<const LocalGrid>(flatten(ground), flatten(obstacles), flatten(empty), record.archive.grid.cell_size, record.archive.grid.view_point));
  }
  return record.observation.localGrid();
}
void decodeSensorData(SensorRecord& record, cv::Mat* image, cv::Mat* depth, cv::Mat* user, cv::Mat* ground, cv::Mat* obstacles, cv::Mat* empty,
                      tools::ParallelExecutor<>* executor) {
  if (ground || obstacles || empty) decodeLocalGrid(record, executor);
  decodeSensorDataConst(record, image, depth, user, ground, obstacles, empty, executor);
  auto& data = record.observation;
  if (image || depth) {
    // The raw setter is the single calibration-size and mosaic validation path.
    data.setImageData(image ? *image : data.imageRaw(), depth ? *depth : data.depthRaw(), data.perspectives(), false);
  }
  if (user) data.setUserData(*user);
}
void decodeSensorData(SensorRecord& record, tools::ParallelExecutor<>* executor) {
  cv::Mat image, depth, user;
  decodeSensorData(record, record.archive.image.empty() ? nullptr : &image, record.archive.depth.empty() ? nullptr : &depth,
                   record.archive.user.empty() ? nullptr : &user, nullptr, nullptr, nullptr, executor);
  decodeLocalGrid(record, executor);
}

void restoreGrid(SensorRecord& record, GridArchive grid) {
  checkBytes(grid.ground);
  checkBytes(grid.obstacles);
  checkBytes(grid.empty);
  record.archive.grid = std::move(grid);
  record.observation.setLocalGrid({});
}
void storeGrid(SensorRecord& record, LocalGrid grid, tools::ParallelExecutor<>* executor) {
  if (grid.cellSize <= 0) {
    restoreGrid(record, {});
    return;
  }
  if (record.gridCellSize() > 0) throw std::logic_error("Clear the local grid before replacing it");
  SensorRecord draft = record;
  draft.observation.setLocalGrid(std::make_shared<const LocalGrid>(std::move(grid)));
  encodeSensorData(draft, false, false, true, executor);
  record = std::move(draft);
}
void storeGrid(SensorRecord& record, const cv::Mat& ground, const cv::Mat& obstacles, const cv::Mat& empty, float cellSize, const cv::Point3f& viewPoint,
               tools::ParallelExecutor<>* executor) {
  if (cellSize <= 0) {
    restoreGrid(record, {});
    return;
  }
  if (record.gridCellSize() > 0) throw std::logic_error("Clear the local grid before replacing it");
  // Preserve legacy matrix shapes on disk; materialize a canonical LocalGrid only on demand.
  GridArchive grid;
  grid.cell_size = cellSize;
  grid.view_point = viewPoint;
  const std::array<const cv::Mat*, 3> inputs{&ground, &obstacles, &empty};
  const std::array<cv::Mat*, 3> outputs{&grid.ground, &grid.obstacles, &grid.empty};
  std::vector<std::function<void()>> tasks;
  size_t work = 0;
  for (size_t i = 0; i < inputs.size(); ++i) {
    const auto* input = inputs[i];
    if (input->empty()) continue;
    if (input->depth() != CV_32F || (input->channels() != 2 && input->channels() != 3 && input->channels() != 6))
      throw std::invalid_argument("Raw grid layers must have 2, 3 or 6 float channels");
    work += bytes(*input);
    tasks.emplace_back([input, output = outputs[i]] { *output = compressData(*input); });
  }
  invoke(executor, tasks, work);
  restoreGrid(record, std::move(grid));
}
}  // namespace sapphire::backend
