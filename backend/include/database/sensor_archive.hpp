#pragma once
#include "backend/tools/parallel_executor.hpp"
#include <backend/common/sensor_data.hpp>
#include <backend/maplayer/navimap/local_grid.hpp>

namespace sapphire::backend {
namespace tools {
template <class ISA_TYPE> class ParallelExecutor;
}

// On-disk representations only; never accepted by SensorData's raw setters.
struct GridArchive {
  cv::Mat ground, obstacles, empty;
  float cell_size = 0;
  cv::Point3f view_point;
};
struct SensorArchive {
  cv::Mat image, depth, user;
  GridArchive grid;
  void clear(bool images = true, bool userData = true, bool occupancyGrid = true);
  std::size_t memoryUsed() const;
};

// Database/cache boundary. Runtime algorithms receive observation explicitly.
// Copies share immutable grid results and OpenCV buffers, but not cache state.
struct SensorRecord {
  SensorData observation;
  SensorArchive archive;
  float gridCellSize() const { return observation.localGrid() ? observation.localGrid()->cellSize : archive.grid.cell_size; }
  void clearCompressedData(bool images = true, bool userData = true, bool occupancyGrid = true) { archive.clear(images, userData, occupancyGrid); }
  void clearRawData(bool images = true, bool userData = true, bool occupancyGrid = true) { observation.clearRawData(images, userData, occupancyGrid); }
  std::size_t memoryUsed() const {
    return sizeof(SensorRecord) + observation.getMemoryUsed() - sizeof(SensorData) + archive.memoryUsed() - sizeof(SensorArchive);
  }
};

// Explicit archive restore: no guessing based on matrix dimensions.
SensorRecord archivedSensorData(const cv::Mat& image, const cv::Mat& depth, const std::vector<camera::Perspective>& cameras, int id = 0, double stamp = 0,
                                const cv::Mat& user = {});
void selectSensorData(SensorRecord& record, bool images, bool userData, bool occupancyGrid);
void encodeSensorData(SensorRecord& record, bool images = true, bool userData = true, bool occupancyGrid = true, tools::ParallelExecutor<>* executor = nullptr);

// Null outputs are skipped. The mutable form caches decoded observations.
// Grid matrices are temporary exports; the sole resident grid is LocalGrid.
void decodeSensorData(SensorRecord& record, tools::ParallelExecutor<>* executor = nullptr);
void decodeSensorData(SensorRecord& record, cv::Mat* image, cv::Mat* depth, cv::Mat* user = nullptr, cv::Mat* ground = nullptr, cv::Mat* obstacles = nullptr,
                      cv::Mat* empty = nullptr, tools::ParallelExecutor<>* executor = nullptr);
void decodeSensorDataConst(const SensorRecord& record, cv::Mat* image, cv::Mat* depth, cv::Mat* user = nullptr, cv::Mat* ground = nullptr,
                           cv::Mat* obstacles = nullptr, cv::Mat* empty = nullptr, tools::ParallelExecutor<>* executor = nullptr);
std::shared_ptr<const LocalGrid> decodeLocalGrid(SensorRecord& record, tools::ParallelExecutor<>* executor = nullptr);

// Raw grids and encoded grids have separate, explicit entry points.
void storeGrid(SensorRecord& record, LocalGrid grid, tools::ParallelExecutor<>* executor = nullptr);
void storeGrid(SensorRecord& record, const cv::Mat& ground, const cv::Mat& obstacles, const cv::Mat& empty, float cellSize, const cv::Point3f& viewPoint,
               tools::ParallelExecutor<>* executor = nullptr);
void restoreGrid(SensorRecord& record, GridArchive grid);
}  // namespace sapphire::backend
