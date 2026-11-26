#pragma once

#include <sapphire/types.hpp>

namespace sapphire {

struct LidarData {
  double start_time = 0.0;
  double end_time = 0.0;
  PointCloudConstPtr pc;
};

/// Retain finite points outside the LiDAR-frame crop box and collect their time bounds.
LidarData preprocessLidar(const PointCloud& points, const Config::Odometry::CropBox& box, double max_range_m = 0.0);

/// Deterministic centroid voxel-grid downsampling.
///
/// Voxel-key generation and sorting are parallelized with OpenMP. The final
/// reduction is serial and uses the original point index as a tie-breaker, so
/// repeated runs produce the same point order and floating-point sums.
PointCloudPtr deterministicVoxelDownsample(const PointCloud& points, double leaf_size, int max_threads = 4);

}  // namespace sapphire
