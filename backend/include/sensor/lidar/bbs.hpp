#pragma once

#include <Eigen/Geometry>
#include <array>
#include <bbs3d.hpp>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "frontend/common/point.hpp"

namespace sapphire {

struct BbsResult {
  bool accepted = false;
  std::uint64_t target_id = std::numeric_limits<std::uint64_t>::max();
  double overlap = 0.0;
  double initial_overlap = 0.0;
  double elapsed_ms = 0.0;
  Eigen::Isometry3d T_target_query = Eigen::Isometry3d::Identity();
  gpu::SearchTermination termination = gpu::SearchTermination::kNoCandidates;
  std::size_t root_nodes = 0;
  std::array<std::size_t, 4> expanded_nodes{};
  std::array<std::size_t, 4> pruned_nodes{};
};

// One fixed-pose LiDAR overlap evaluation, using the existing scorer. No search.
bool acceptsLoopSeedScore(const gpu::PoseScore &score); // Invalid numeric evidence throws.
gpu::PoseScore scoreLoopSeed(const GaussianCloud &query, const cpu::VoxelMapsData &target,
                            const Eigen::Isometry3f &T_target_query);

BbsResult alignLoopBbs(const vvec<float, 3> &query, const std::vector<gpu::LocalSearchTarget> &targets,
                       const std::optional<gpu::LocalSearchOptions> &options = std::nullopt);

BbsResult alignLoopBbs(const GaussianCloud &query, const std::vector<gpu::LocalSearchTarget> &targets,
                       const std::optional<gpu::LocalSearchOptions> &options = std::nullopt);

}  // namespace sapphire
