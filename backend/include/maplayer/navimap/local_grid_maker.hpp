#pragma once
#include <backend/common/common_type.hpp>
#include <backend/params/navimap.hpp>
#include <limits>
#include <backend/maplayer/navimap/flat_ground.hpp>
#include <backend/maplayer/navimap/local_grid.hpp>

namespace sapphire::backend {
class SensorData;

struct LocalGridTiming {
  double projectionMs = 0.0;
  double filteringMs = 0.0;
  double surfaceClassificationMs = 0.0;
  double groundRefinementMs = 0.0;
  double transformMergeMs = 0.0;
  // Height classification and output generation, including optional voxel aggregation.
  double voxelizationMs = 0.0;
  double rayTracingMs = 0.0;
  std::size_t projectedPoints = 0;
  std::size_t validPoints = 0;
  std::size_t outputCells = 0;
  bool groundReferenceReady = false;
  bool groundObserved = false;
  float groundReferenceZ = std::numeric_limits<float>::quiet_NaN();
  float groundCorrectionZ = 0;
  std::size_t groundSupportPatches = 0;
};

class LocalGridMaker {
 public:
  explicit LocalGridMaker(const ParametersMap& parameters = {});

  explicit LocalGridMaker(const NavimapParams& configuration);
  float getCellSize() const noexcept { return config_.cell_size_m; }
  bool isMapFrameProjection() const noexcept { return config_.project_in_map_frame; }
  // Call on odometry-session reset. Sequential frame processing owns this state.
  void resetGroundReference() const { flatGroundReference_.reset(); }

  LocalGrid createLocalGrid(const SensorData& data, const StateGroupf& pose, LocalGridTiming* timing = nullptr) const;
  LocalGrid create_local_grid(const SensorData& data, const StateGroupf& pose, LocalGridTiming* timing = nullptr) const;

 private:
  NavimapParams config_;
  std::vector<float> roiRatios_;
  mutable FlatGroundReference flatGroundReference_;
};

}  // namespace sapphire::backend
