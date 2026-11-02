#pragma once
#include <array>

#include "common/common_type.hpp"
#include "common/se3_state.hpp"
#include "parameters.hpp"
namespace sapphire::lidar {
// Retrieval-only, rigid-invariant radial occupancy signature; never a metric factor.
std::array<float, 64> geometry_signature(const GaussianCloud&, float resolution);
// Frozen-normal FPFH associations and G3Reg PAGOR/GNC produce full SE3 proposals.
// Only subsequent full-geometry verification may accept a constraint.
std::vector<SE3f> initial_poses(const GaussianCloud&, const GaussianCloud&, const RelocalizerParams&, std::string& failure);
}  // namespace sapphire::lidar
