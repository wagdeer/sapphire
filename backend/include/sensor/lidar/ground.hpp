#pragma once
#include "backend/maplayer/navimap/flat_ground.hpp"
#include "backend/sensor/lidar/geometry.hpp"
namespace sapphire::backend::lidar {
// Same gravity-aligned floor-support contract as depth patches, consuming
// frozen Gaussian moments without inventing organized camera pixels.
void ground_patches(const GaussianCloud&, const StateGroupf& world_from_node, float tolerance, std::vector<GroundImagePatch>&);
}  // namespace sapphire::backend::lidar
