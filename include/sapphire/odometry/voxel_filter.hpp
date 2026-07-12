#pragma once

#include <sapphire/types.hpp>

namespace sapphire {

/// Deterministic centroid voxel-grid downsampling.
///
/// Voxel-key generation and sorting are parallelized with OpenMP. The final
/// reduction is serial and uses the original point index as a tie-breaker, so
/// repeated runs produce the same point order and floating-point sums.
PointCloudPtr deterministicVoxelDownsample(
    const PointCloud& points,
    double leaf_size,
    int max_threads = 4);

}  // namespace sapphire
