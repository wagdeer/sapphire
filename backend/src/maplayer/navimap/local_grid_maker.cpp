#include "backend/sensor/lidar/ground.hpp"
#include <backend/common/common_type.hpp>
#include <backend/params/navimap.hpp>
#include "backend/sensor/camera/utils.hpp"
#include "backend/maplayer/navimap/local_grid_builder.hpp"
#include <chrono>
#include <backend/common/depth_frame.hpp>
#include <backend/common/sensor_data.hpp>
#include <backend/maplayer/navimap/local_grid_maker.hpp>
#include <backend/tools/logging.hpp>
#include <backend/tools/strings.hpp>
#include <limits>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace sapphire::backend {
namespace {

// Keep range-limited rays separate from surface samples. In particular, their
// artificial endpoints must never enter ground classification or voxelization.
struct ClippedRays {
  Eigen::Vector3f origin;
  std::vector<Eigen::Vector3f> endpoints;
};

std::int32_t rayIndex(float value, float cellSize) {
  const double cell = std::floor(double(value) / cellSize);
  if (!std::isfinite(cell) || std::abs(cell) > 1000000) throw std::invalid_argument("Range ray coordinate out of bounds");
  return static_cast<std::int32_t>(cell);
}

std::uint64_t rayKey(const Eigen::Vector3f& point, float cellSize) { return NavimapGrid::key(rayIndex(point.x(), cellSize), rayIndex(point.y(), cellSize)); }

void appendClippedFreeSpace(LocalGrid& grid, const std::vector<ClippedRays>& cameras, const StateGroupf& planeFromBase, bool grid3D,
                            const StateGroupf& worldFromBase, const FlatGroundReference* floor, float lower, float upper) {
  const auto planeRotation = planeFromBase.rotationMatrix();
  const auto planeTranslation = planeFromBase.translationVector();
  // One lookup per traversed cell covers both obstacle stopping and miss deduplication.
  enum CellState : uint8_t { Observed, Obstacle, Free };
  std::unordered_map<std::uint64_t, CellState> cellStates;
  cellStates.reserve(grid.ground.size() + grid.empty.size() + grid.obstacles.size());
  const auto collect = [&](const GridCells& cells, bool hit) {
    for (std::size_t i = 0; i < cells.size(); ++i) {
      const Eigen::Vector3f point = planeRotation * Eigen::Vector3f(cells.xy.x[i], cells.xy.y[i], cells.is3D() ? cells.z[i] : 0.0f) + planeTranslation;
      const auto key = rayKey(point, grid.cellSize);
      cellStates.insert_or_assign(key, hit ? Obstacle : Observed);
    }
  };
  collect(grid.ground, false);
  collect(grid.empty, false);
  collect(grid.obstacles, true);
  // A single miss per cell/frame, even if many pixels or cameras see beyond it.
  // z is only a projection reference for the 3D representation, not ground height.
  std::vector<std::pair<std::uint64_t, float>> free;
  const auto worldFromPlane = worldFromBase.compose(planeFromBase.inverse());
  const auto worldRotation = worldFromPlane.rotationMatrix();
  const auto worldTranslation = worldFromPlane.translationVector();
  struct RayHash {
    size_t operator()(const std::array<int, 4>& ray) const noexcept {
      size_t h = 0;
      for (int coordinate : ray) h ^= std::hash<int>{}(coordinate) + size_t(0x9e3779b9) + (h << 6) + (h >> 2);
      return h;
    }
  };
  for (const auto& camera : cameras) {
    std::unordered_set<std::array<int, 4>, RayHash> visited;
    visited.reserve(camera.endpoints.size());
    for (const auto& endpoint : camera.endpoints) {
      Eigen::Vector3f begin = camera.origin, end = endpoint;
      if (floor) {
        const float h0 = floor->height((worldRotation * begin + worldTranslation).z());
        const float h1 = floor->height((worldRotation * end + worldTranslation).z());
        float t0 = 0, t1 = 1;
        if (std::abs(h1 - h0) < 1e-6f) {
          if (h0 < lower || h0 > upper) continue;
        } else {
          float a = (lower - h0) / (h1 - h0), b = (upper - h0) / (h1 - h0);
          if (a > b) std::swap(a, b);
          t0 = std::max(t0, a);
          t1 = std::min(t1, b);
          if (t0 > t1) continue;
        }
        const Eigen::Vector3f delta = end - begin;
        end = begin + t1 * delta;
        begin += t0 * delta;
      }
      const int ox = rayIndex(begin.x(), grid.cellSize), oy = rayIndex(begin.y(), grid.cellSize);
      const int tx = rayIndex(end.x(), grid.cellSize), ty = rayIndex(end.y(), grid.cellSize);
      if (!visited.insert({ox, oy, tx, ty}).second) continue;
      const int dx = std::abs(tx - ox), dy = std::abs(ty - oy), sx = ox < tx ? 1 : -1, sy = oy < ty ? 1 : -1;
      int x = ox, y = oy, error = dx - dy;
      for (;;) {
        const auto key = NavimapGrid::key(x, y);
        // Stop at this frame's actual obstacle, but let new misses clear old map
        // evidence. Existing ground/empty observations already contribute a miss.
        const auto [entry, inserted] = cellStates.try_emplace(key, Free);
        if (entry->second == Obstacle) break;
        if (inserted) free.emplace_back(key, camera.origin.z());
        if (x == tx && y == ty) break;  // Include the limit cell, never go beyond it.
        const int twice = error * 2;
        if (twice > -dy) {
          error -= dy;
          x += sx;
        }
        if (twice < dx) {
          error += dx;
          y += sy;
        }
      }
    }
  }
  const auto baseFromPlane = planeFromBase.inverse();
  const auto baseRotation = baseFromPlane.rotationMatrix();
  const auto baseTranslation = baseFromPlane.translationVector();
  for (const auto& [key, z] : free) {
    const Eigen::Vector3f point = baseRotation * Eigen::Vector3f((std::int32_t(key >> 32) + .5f) * grid.cellSize, (std::int32_t(key) + .5f) * grid.cellSize, z) + baseTranslation;
    if (grid3D)
      grid.empty.push3D(point.x(), point.y(), point.z());
    else
      grid.empty.push2D(point.x(), point.y());
  }
}

}  // namespace

LocalGridMaker::LocalGridMaker(const ParametersMap& parameters)
    : LocalGridMaker(NavimapParams{}.withOverrides(parameters)) {}

LocalGridMaker::LocalGridMaker(const NavimapParams& configuration) : config_(configuration) {
  config_.validate();
  const auto ratios = config_.roiRatios();
  roiRatios_.assign(ratios.begin(), ratios.end());
}

LocalGrid LocalGridMaker::createLocalGrid(const SensorData& data, const StateGroupf& pose, LocalGridTiming* timing) const {
  // Historical nodes may be requested out of timestamp order or in map rather
  // than odometry coordinates. Never feed them into the live floor tracker.
  return LocalGridMaker(config_).create_local_grid(data, pose, timing);
}

LocalGrid LocalGridMaker::create_local_grid(const SensorData& sensorData, const StateGroupf& pose, LocalGridTiming* timing) const {
  if (timing != nullptr) {
    *timing = {};
  }
  using Clock = std::chrono::steady_clock;
  const auto elapsedMs = [](Clock::time_point start) { return std::chrono::duration<double, std::milli>(Clock::now() - start).count(); };

  if (!sensorData.lidar_cloud && (sensorData.depthRaw().empty() || sensorData.perspectives().empty())) {
    return {};
  }
  if (!sensorData.depthRaw().empty()) camera::validateCameraMosaic(sensorData.depthRaw(), sensorData.perspectives(), "Depth");

  const bool grid3D = config_.method == 1;
  const std::vector<cv::Rect> cameraRois = camera::cameraImageRois(sensorData.perspectives());
  simd::SoA3<float> merged;
  std::vector<LocalGrid::DepthImage> depth_images;
  thread_local std::vector<GroundImagePatch> groundPatches;
  groundPatches.clear();
  if (sensorData.lidar_cloud) {
    lidar::ground_patches(*sensorData.lidar_cloud,pose,config_.ground_plane_tolerance_m,groundPatches);
    for(const auto& p:*sensorData.lidar_cloud) merged.push_back(p.mean.x(),p.mean.y(),p.mean.z());
    if(timing) timing->projectedPoints=timing->validPoints=sensorData.lidar_cloud->size();
  }
  // Method 1 traces on the gravity-aligned map plane and transforms free cells
  // back to base coordinates; method 0 retains its existing base-XY projection.
  const auto rayPlaneFromBase = grid3D ? pose : StateGroupf::identity();
  std::vector<ClippedRays> clippedRays;
  const bool clipRays = config_.ray_tracing_enabled && config_.max_depth_m > 0.0f;
  std::array<float, 3> viewPoint = {};
  float cameraCount = 0.0f;
  const Eigen::Matrix3f poseRotation = pose.rotationMatrix();
  const float heading = std::atan2(poseRotation(1, 0), poseRotation(0, 0));
  const float cosine = std::cos(heading);
  const float sine = std::sin(heading);
  Eigen::Matrix3f removeHeading;
  removeHeading << cosine, sine, 0.0f, -sine, cosine, 0.0f, 0.0f, 0.0f, 1.0f;
  const Eigen::Matrix3f levelingRotation = removeHeading * poseRotation;
  const StateGroupf leveling = StateGroupf::fromRotationTranslation(levelingRotation, Eigen::Vector3f(0.0f, 0.0f, config_.project_in_map_frame ? pose.z() : 0.0f));

  for (std::size_t cameraIndex = 0; !sensorData.depthRaw().empty() && cameraIndex < sensorData.perspectives().size(); ++cameraIndex) {
    camera::Perspective model = sensorData.perspectives()[cameraIndex];
    if (!model.isValidForProjection() || model.cameraToBase().isNull()) {
      continue;
    }
    cv::Mat depth = sensorData.depthRaw()(cameraRois[cameraIndex]);
    if (roiRatios_.size() == 4 && (roiRatios_[0] > 0.0f || roiRatios_[1] > 0.0f || roiRatios_[2] > 0.0f || roiRatios_[3] > 0.0f)) {
      const cv::Rect depthRoi = camera::imageRoi(depth, roiRatios_);
      depth = depth(depthRoi);
      model = model.cropped(camera::imageRoi(model.imageSize(), roiRatios_));
    }
    if (depth.rows % static_cast<int>(config_.depth_decimation) != 0 || depth.cols % static_cast<int>(config_.depth_decimation) != 0) {
      EXT_LOG_ERROR("Depth dimensions {}x{} cannot be divided by decimation {}", depth.cols, depth.rows, config_.depth_decimation);
      continue;
    }

    model = model.resized(depth.size());
    // Normalize once at the input boundary. Both organized projection and TSDF
    // consume the same metric image and cropped camera model.
    cv::Mat meters;
    if (depth.type() == CV_16UC1) {
      meters.create(depth.size(), CV_32FC1);
      for (int row = 0; row < depth.rows; ++row) {
        const auto* input = depth.ptr<uint16_t>(row);
        auto* output = meters.ptr<float>(row);
        for (int column = 0; column < depth.cols; ++column)
          output[column] = input[column] == 65535 ? std::numeric_limits<float>::quiet_NaN() : input[column] * .001f;
      }
    } else if (depth.type() == CV_32FC1) meters = depth.clone();
    else throw std::invalid_argument("Navimap requires millimeter uint16 or metric float depth");
    depth = meters;
    if (grid3D) depth_images.push_back({meters, model});
    const DepthProjectionOptions projection{static_cast<int>(config_.depth_decimation), config_.min_depth_m, clipRays ? 0.0f : config_.max_depth_m};
    auto phaseStart = Clock::now();
    DepthFrame frame = DepthFrame::fromDepth(depth, model, projection);
    if (timing != nullptr) {
      timing->projectionMs += elapsedMs(phaseStart);
      timing->projectedPoints += frame.size();
    }
    const StateGroupf sensorToGround = leveling.compose(model.cameraToBase());
    ClippedRays cameraRays;
    const auto rayPlaneFromCamera = rayPlaneFromBase.compose(model.cameraToBase());
    cameraRays.origin = rayPlaneFromCamera.translationVector();

    phaseStart = Clock::now();
    std::size_t validPoints = 0;
    for (std::size_t i = 0; i < frame.size(); ++i) {
      if (!std::isfinite(frame.points.x[i]) || !std::isfinite(frame.points.y[i]) || !std::isfinite(frame.points.z[i])) {
        continue;
      }
      if (clipRays && frame.points.z[i] >= config_.max_depth_m) {
        const float scale = config_.max_depth_m / frame.points.z[i];
        const auto endpoint = rayPlaneFromCamera.transformPoint({frame.points.x[i] * scale, frame.points.y[i] * scale, config_.max_depth_m});
        cameraRays.endpoints.push_back(endpoint);
        // Keep clipped ray endpoints out of organized surface fitting
        // as well as grid classification.
        const float invalid = std::numeric_limits<float>::quiet_NaN();
        frame.points.set(i, invalid, invalid, invalid);
        continue;
      }
      bool inFootprint = false;
      if (config_.footprint_length_m > 0.0f) {
        const auto point = sensorToGround.transformPoint({frame.points.x[i], frame.points.y[i], frame.points.z[i]});
        inFootprint = point.x() >= -config_.footprint_length_m * .5f && point.x() <= config_.footprint_length_m * .5f &&
                      (config_.footprint_width_m <= 0.0f || (point.y() >= -config_.footprint_width_m * .5f && point.y() <= config_.footprint_width_m * .5f)) &&
                      point.z() >= 0.0f && (config_.footprint_height_m <= 0.0f || point.z() <= config_.footprint_height_m);
      }
      if (inFootprint) {
        const float invalid = std::numeric_limits<float>::quiet_NaN();
        frame.points.set(i, invalid, invalid, invalid);
      } else {
        ++validPoints;
      }
    }
    if (!cameraRays.endpoints.empty()) clippedRays.push_back(std::move(cameraRays));
    if (timing != nullptr) {
      timing->filteringMs += elapsedMs(phaseStart);
      timing->validPoints += validPoints;
    }

    phaseStart = Clock::now();
    if (config_.ground_method == 0)
      collectGroundImagePatches(frame, pose.compose(model.cameraToBase()), pose.z(), config_.ground_plane_tolerance_m, groundPatches);
    else
      collectGroundCapePatches(frame, pose.compose(model.cameraToBase()), pose.z(), config_.ground_plane_tolerance_m, groundPatches);
    if (timing) timing->groundRefinementMs += elapsedMs(phaseStart);
    phaseStart = Clock::now();
    model.cameraToBase().transformPointsInPlace(frame.points.x.data(), frame.points.y.data(), frame.points.z.data(), frame.size());
    // The first camera can hand off its storage after organized ground fitting.
    if (merged.empty()) merged = std::move(frame.points);
    else merged.append(frame.points);
    if (timing != nullptr) {
      timing->transformMergeMs += elapsedMs(phaseStart);
    }
    const Eigen::Vector3f cameraTranslation = model.cameraToBase().translationVector();
    viewPoint[0] += cameraTranslation.x();
    viewPoint[1] += cameraTranslation.y();
    viewPoint[2] += cameraTranslation.z();
    cameraCount += 1.0f;
  }

  if (cameraCount > 0.0f) {
    viewPoint[0] /= cameraCount;
    viewPoint[1] /= cameraCount;
    viewPoint[2] /= cameraCount;
  }
  {
    const auto start = Clock::now();
    const bool observed = flatGroundReference_.updateImage(groundPatches, config_.ground_plane_tolerance_m, config_.ground_max_height_correction_m);
    if (timing) {
      timing->groundObserved = observed;
      timing->groundReferenceReady = flatGroundReference_.ready();
      timing->groundReferenceZ = flatGroundReference_.referenceZ();
      timing->groundCorrectionZ = flatGroundReference_.correctionZ();
      timing->groundSupportPatches = flatGroundReference_.supportPatches();
    }
    if (!observed) {
      // Keep the reference for reacquisition, but never fuse a frame using a
      // stale floor correction. The rejection also disables footprint updates.
      LocalGrid rejected({}, {}, {}, config_.cell_size_m, viewPoint);
      rejected.skip_integration = true;
      if (timing) timing->groundRefinementMs += elapsedMs(start);
      return rejected;
    }
    if (timing) timing->groundRefinementMs += elapsedMs(start);
  }
  if (merged.empty()) {
    return {};
  }
  const auto voxelizationStart = Clock::now();
  detail::LocalGridBuilder builder(config_.cell_size_m, grid3D, config_.voxel_filter_enabled, merged.size());
  for (std::size_t i = 0; i < merged.size(); ++i) {
    const Eigen::Vector3f point(merged.x[i], merged.y[i], merged.z[i]);
    if (!point.allFinite()) continue;
    const float height = flatGroundReference_.height(pose.transformPoint(point).z());
    constexpr float boundaryRoundoff = 1e-6f;
    if (height > config_.max_obstacle_height_m + boundaryRoundoff) continue;
    // Classify using the aligned floor datum, keeping physical coordinates unchanged.
    const bool obstacle = height >= config_.min_obstacle_height_m - boundaryRoundoff;
    builder.add(point.x(), point.y(), point.z(), obstacle);
  }
  LocalGrid grid = std::move(builder).finish(viewPoint);
  grid.depth_images = std::move(depth_images);
  grid.ground_plane.head<3>() = pose.rotationMatrix().row(2).transpose();
  grid.ground_plane.w() = pose.z() + flatGroundReference_.correctionZ() - flatGroundReference_.referenceZ();
  grid.min_obstacle_height = config_.min_obstacle_height_m;
  grid.max_obstacle_height = config_.max_obstacle_height_m;
  grid.min_depth = config_.min_depth_m;
  grid.max_depth = config_.max_depth_m > 0 ? config_.max_depth_m : std::numeric_limits<float>::infinity();
  if (timing != nullptr) {
    timing->voxelizationMs = elapsedMs(voxelizationStart);
  }
  if (!sensorData.lidar_cloud && !grid3D && config_.ray_tracing_enabled) {
    const auto rayTracingStart = Clock::now();
    grid.rayTraceFreeSpace();
    if (timing != nullptr) {
      timing->rayTracingMs = elapsedMs(rayTracingStart);
    }
  }
  if (!clippedRays.empty()) {
    const auto rayTracingStart = Clock::now();
    appendClippedFreeSpace(grid, clippedRays, rayPlaneFromBase, grid3D, pose, &flatGroundReference_, config_.min_obstacle_height_m, config_.max_obstacle_height_m);
    if (timing != nullptr) timing->rayTracingMs += elapsedMs(rayTracingStart);
  }
  if (timing != nullptr) {
    timing->outputCells = grid.ground.size() + grid.obstacles.size() + grid.empty.size();
  }
  return grid;
}

}  // namespace sapphire::backend
