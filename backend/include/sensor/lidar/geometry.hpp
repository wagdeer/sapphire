#pragma once
#include <opencv2/core.hpp>

#include "frontend/common/point.hpp"

namespace sapphire::backend::lidar {
// Immutable marginalized Gaussian geometry, expressed in the graph node's base
// frame, in metres. Original sample counts and voxel identity survive storage.
// The frontend never shares its mutable VoxelMap with the backend.
cv::Mat encode(const GaussianCloud& cloud);
GaussianCloudPtr decode(const cv::Mat& archive);
void validate(const GaussianCloud& cloud);
inline constexpr std::size_t max_points = 24000;
}  // namespace sapphire::backend::lidar
