#include "backend/system/action/preprocess.hpp"

#include <algorithm>
#include <backend/database/statistics.hpp>
#include <backend/maplayer/navimap/local_grid_maker.hpp>
#include <backend/sensor/camera/utils.hpp>
#include <backend/tools/timer.hpp>
#include <cmath>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

#include "backend/tools/logging.hpp"

namespace sapphire::backend {

void StereoRectifier::configure(bool right, const camera::KB4& model, const Eigen::Matrix3d& rotation, const Eigen::Matrix3d& intrinsics,
                                cv::Size output_size, cv::Size expected_input_size) {
  const int i = right ? 1 : 0;
  if (output_size.width <= 0 || output_size.height <= 0) throw std::invalid_argument("Invalid rectification image dimensions");
  output_size_[i] = output_size;
  if (expected_input_size != cv::Size() && (expected_input_size.width <= 0 || expected_input_size.height <= 0))
    throw std::invalid_argument("Invalid expected raw image dimensions");
  expected_input_size_[i] = expected_input_size;
  camera::makeRectificationMaps(model, rotation, intrinsics, output_size_[i], map1_[i], map2_[i]);
  input_size_[i] = {};
  rectified_mask_[i].release();
  offsets_[i].release();
  inside_[i] = false;
}
void StereoRectifier::setRawMask(bool right, const cv::Mat& mask) {
  camera::validateImageMask(mask);
  const int i = right ? 1 : 0;
  if (!input_size_[i].empty()) throw std::logic_error("Raw feature masks must be configured before image input");
  raw_mask_[i] = mask.clone();
  rectified_mask_[i].release();
}
cv::Mat StereoRectifier::rectify(const cv::Mat& mono, bool right) {
  if (mono.empty()) return {};
  const int i = right ? 1 : 0;
  if (mono.type() != CV_8UC1) throw std::invalid_argument("Rectification requires mono8 pixels");
  if (map1_[i].empty()) throw std::logic_error("Rectification camera is not configured");
  if (!expected_input_size_[i].empty() && mono.size() != expected_input_size_[i])
    throw std::invalid_argument("Raw image dimensions disagree with the calibrated capture profile");
  if (input_size_[i].empty()) {
    if (!raw_mask_[i].empty()) {
      const auto& mask = raw_mask_[i];
      if (int64_t(mask.cols) * mono.rows != int64_t(mask.rows) * mono.cols)
        throw std::invalid_argument("Raw mask and camera must have the same aspect ratio; cropping/stretching is not supported");
      cv::Mat floating, remapped;
      camera::resizeImageMask(mask, mono.size()).convertTo(floating, CV_32F);
      cv::remap(floating, remapped, map1_[i], map2_[i], cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255));
      cv::compare(remapped, 0, rectified_mask_[i], cv::CMP_GT);
    }
    // A fixed, tightly packed mono camera allows source addresses to be
    // computed once from the first actual image. Keep the generic maps for
    // borders and padded images; calibration geometry is already configured.
    offsets_[i].create(output_size_[i], CV_32S);
    // Invalid map support is excluded even when no user mask is configured;
    // black remap borders must never become artificial detector features.
    if (rectified_mask_[i].empty()) rectified_mask_[i] = cv::Mat::zeros(output_size_[i], CV_8U);
    inside_[i] = true;
    for (int y = 0; y < output_size_[i].height; ++y)
      for (int x = 0; x < output_size_[i].width; ++x) {
        const auto q = map1_[i].at<cv::Vec2s>(y, x);
        const int64_t offset = int64_t(q[1]) * mono.cols + q[0];
        const bool valid =
            q[0] >= 0 && q[1] >= 0 && q[0] + 1 < mono.cols && q[1] + 1 < mono.rows && offset + mono.cols + 1 <= std::numeric_limits<int>::max();
        inside_[i] &= valid;
        if (!valid) rectified_mask_[i].at<uint8_t>(y, x) = 255;
        offsets_[i].at<int>(y, x) = valid ? static_cast<int>(offset) : 0;
      }
    input_size_[i] = mono.size();
    if (cv::countNonZero(rectified_mask_[i]) == 0) rectified_mask_[i].release();
  } else if (mono.size() != input_size_[i]) {
    throw std::invalid_argument(right ? "Right rectification input size changed after first image"
                                      : "Left rectification input size changed after first image");
  }
  cv::Mat result;
  if (!inside_[i] || mono.step != static_cast<size_t>(input_size_[i].width)) {
    cv::remap(mono, result, map1_[i], map2_[i], cv::INTER_LINEAR, cv::BORDER_CONSTANT);
  } else {
    result.create(output_size_[i], CV_8U);
    const int stride = input_size_[i].width;
    const auto* src = mono.ptr<uint8_t>();
    // Same fixed-point fractions and round-to-nearest rule as INTER_LINEAR.
    // This is the separable bilinear formula, not a changed camera model.
    constexpr int bits = cv::INTER_BITS, scale = cv::INTER_TAB_SIZE;
    cv::parallel_for_(
        cv::Range(0, output_size_[i].height),
        [&](const cv::Range& rows) {
          for (int y = rows.start; y < rows.end; ++y) {
            const int* offsets = offsets_[i].ptr<int>(y);
            const uint16_t* alpha = map2_[i].ptr<uint16_t>(y);
            uint8_t* out = result.ptr<uint8_t>(y);
            for (int x = 0; x < output_size_[i].width; ++x) {
              const int pos = offsets[x], fx = alpha[x] & (scale - 1), fy = alpha[x] >> bits;
              const int a = src[pos], b = src[pos + 1], c = src[pos + stride], d = src[pos + stride + 1];
              const int top = a * scale + (b - a) * fx;
              const int bottom = c * scale + (d - c) * fx;
              out[x] = static_cast<uint8_t>((top * scale + (bottom - top) * fy + scale * scale / 2) >> (2 * bits));
            }
          }
        },
        result.total() / 65536.0);
  }
  return result;
}

bool prepare_navimap(SensorData& observation, const SensorData& input, const StateGroupf& pose, const LocalGridMaker& maker, bool enabled,
                     Statistics* stats) {
  const auto provided = input.localGrid();
  if (!enabled || (input.id() < 0 && (!provided || provided->cellSize <= 0))) return false;
  if (provided && provided->cellSize > 0) {
    observation.setLocalGrid(provided);
    if (stats) stats->addStatistic(Statistics::kTimingMemOccupancy_grid(), 0.0f);
    return true;
  }
  if (input.depthRaw().empty() && !input.lidar_cloud) return false;
  observation.lidar_cloud = input.lidar_cloud;
  tools::Timer timer;
  LocalGridTiming gridTiming;
  auto grid = maker.createLocalGrid(observation, pose, &gridTiming);
  observation.setLocalGrid(std::make_shared<LocalGrid>(std::move(grid)));
  if (stats) {
    stats->addStatistic(Statistics::kTimingMemOccupancy_grid(), timer.lap_seconds() * 1000.0f);
    stats->addStatistic(Statistics::kTimingGridProjection(), gridTiming.projectionMs);
    stats->addStatistic(Statistics::kTimingGridFiltering(), gridTiming.filteringMs);
    stats->addStatistic(Statistics::kTimingGridSurface_classification(), gridTiming.surfaceClassificationMs);
    stats->addStatistic(Statistics::kTimingGridGround_refinement(), gridTiming.groundRefinementMs);
    stats->addStatistic(Statistics::kTimingGridTransform_merge(), gridTiming.transformMergeMs);
    stats->addStatistic(Statistics::kTimingGridVoxelization(), gridTiming.voxelizationMs);
    stats->addStatistic(Statistics::kTimingGridRay_tracing(), gridTiming.rayTracingMs);
    stats->addStatistic(Statistics::kGridProjected_points(), static_cast<float>(gridTiming.projectedPoints));
    stats->addStatistic(Statistics::kGridValid_points(), static_cast<float>(gridTiming.validPoints));
    stats->addStatistic(Statistics::kGridOutput_cells(), static_cast<float>(gridTiming.outputCells));
  }
  return true;
}

std::shared_ptr<const LocalGrid> preprocess_depth(const SensorData& data, const StateGroupf& odom, const LocalGridMaker& maker, bool enabled,
                                                  LocalGridTiming* timing) {
  LocalGrid grid;
  if (enabled && (!data.depthRaw().empty() || data.lidar_cloud)) {
    grid = maker.create_local_grid(data, odom, timing);
  } else {
    grid.skip_integration = true;
    grid.cellSize = maker.getCellSize();
  }
  return std::make_shared<LocalGrid>(std::move(grid));
}

SensorData prepareSensorData(const cv::Mat& image, const cv::Mat& depth, const std::vector<camera::Perspective>& cameras, int id, double stamp,
                             const cv::Mat& userData) {
  if (depth.type() == CV_32FC2) {
    cv::Mat meters;
    cv::extractChannel(depth, meters, 0);
    return SensorData(image, camera::depthToMillimeters(meters), cameras, id, stamp, userData);
  }
  return SensorData(image, depth.type() == CV_32FC1 ? camera::depthToMillimeters(depth) : depth, cameras, id, stamp, userData);
}

SensorData prepareSensorData(const cv::Mat& image, const cv::Mat& depth, const camera::Perspective& camera, int id, double stamp,
                             const cv::Mat& userData) {
  return prepareSensorData(image, depth, std::vector<camera::Perspective>{camera}, id, stamp, userData);
}

void decimateImages(cv::Mat& image, cv::Mat& depth, std::vector<camera::Perspective>& cameras, int factor) {
  if (factor < 1) throw std::invalid_argument("Image decimation must be positive");
  if (factor == 1 || cameras.empty()) return;
  if (cameras.size() == 1) {
    image = camera::decimateImage(image, factor);
    depth = camera::decimateImage(depth, factor);
    cameras.front() = cameras.front().scaled(1.0 / factor);
    return;
  }
  auto images = camera::splitCameraMosaic(image, cameras);
  auto depths = camera::splitCameraMosaic(depth, cameras);
  for (std::size_t i = 0; i < cameras.size(); ++i) {
    if (!images.empty()) images[i] = camera::decimateImage(images[i], factor);
    if (!depths.empty()) depths[i] = camera::decimateImage(depths[i], factor);
    cameras[i] = cameras[i].scaled(1.0 / factor);
  }
  image = camera::makeCameraMosaic(images);
  depth = camera::makeCameraMosaic(depths);
}

}  // namespace sapphire::backend
