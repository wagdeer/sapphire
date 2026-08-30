#pragma once

#include <Eigen/Geometry>
#include <bbs3d.hpp>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include "../common/common.hpp"

namespace sapphire {

class Memory;

struct LoopCandidateStats {
  std::size_t spatial_matches = 0;
  std::size_t rejected_non_historical = 0;
  std::size_t rejected_adjacent = 0;
  std::size_t cold_loads = 0;

  std::size_t rejected() const noexcept { return rejected_non_historical + rejected_adjacent; }
};

struct LoopCandidate {
  std::uint64_t target_id = 0;
  Eigen::Isometry3f target_T_query_initial = Eigen::Isometry3f::Identity();
  cpu::VoxelMapsData target_voxelmaps;
};

struct LoopCandidateBatch {
  std::vector<LoopCandidate> candidates;
  std::shared_ptr<const vvec<float, 3>> query_cloud;
  LoopCandidateStats stats;
};

bool isEligibleLoopTarget(std::uint64_t target_id, std::size_t query_id, LoopCandidateStats &stats) noexcept;

LoopCandidateBatch prepareLoopCandidates(Memory &memory, std::size_t query_id, const Eigen::Isometry3d &map_T_query);

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

struct GicpResult {
  bool attempted = false;
  bool converged = false;
  bool accepted = false;
  std::size_t inliers = 0;
  std::size_t query_points = 0;
  std::size_t target_points = 0;
  double fitness = std::numeric_limits<double>::infinity();
  double preparation_elapsed_ms = 0.0;
  double alignment_elapsed_ms = 0.0;
  double total_elapsed_ms = 0.0;
  Eigen::Isometry3d T_target_query = Eigen::Isometry3d::Identity();
};

BbsResult alignLoopBbs(const vvec<float, 3> &query, const std::vector<gpu::LocalSearchTarget> &targets);

GicpResult refineLoopGicp(const vvec<float, 3> &query, const vvec<float, 3> &target, const Eigen::Isometry3d &initial);

}  // namespace sapphire
