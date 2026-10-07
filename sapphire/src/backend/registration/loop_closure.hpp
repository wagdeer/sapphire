#pragma once

#include <Eigen/Geometry>
#include <array>
#include <bbs3d.hpp>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include "common/common.hpp"
#include "backend/visual/visual_loop.hpp"

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
  bool visual = false;
  bool metric_visual = false;  // initial pose is validated PnP; never run BBS for this target
};

struct LoopCandidateBatch {
  std::vector<LoopCandidate> candidates;
  GaussianCloudPtr query_cloud;
  LoopCandidateStats stats;
};

bool isEligibleLoopTarget(std::uint64_t target_id, std::size_t query_id, LoopCandidateStats &stats) noexcept;

LoopCandidateBatch prepareLoopCandidates(Memory &memory, std::size_t query_id, const Eigen::Isometry3d &map_T_query,
                                         const std::vector<VisualSubmapMatch> &visual = {});

LoopCandidateBatch prepareLoopCandidates(Memory &memory, const SubmapFrame &query, const Eigen::Isometry3d &map_T_query,
                                         const std::vector<VisualSubmapMatch> &visual = {});

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

// One fixed-pose LiDAR overlap evaluation, using the existing scorer. No search.
bool acceptsLoopSeedScore(const gpu::PoseScore &score); // Invalid numeric evidence throws.
gpu::PoseScore scoreLoopSeed(const GaussianCloud &query, const cpu::VoxelMapsData &target,
                            const Eigen::Isometry3f &T_target_query);

BbsResult alignLoopBbs(const vvec<float, 3> &query, const std::vector<gpu::LocalSearchTarget> &targets,
                       const std::optional<gpu::LocalSearchOptions> &options = std::nullopt);

BbsResult alignLoopBbs(const GaussianCloud &query, const std::vector<gpu::LocalSearchTarget> &targets,
                       const std::optional<gpu::LocalSearchOptions> &options = std::nullopt);

/// Refines a loop transform from precomputed Gaussian means and covariances.
/// Covariances must be regularized when the Gaussian submap is frozen; this path performs no neighborhood re-estimation.
GicpResult refineLoopGicp(const GaussianCloudPtr &query, const GaussianCloudPtr &target, const Eigen::Isometry3d &initial);

/// Synchronous non-owning wrapper for callers that already guarantee both cloud lifetimes.
GicpResult refineLoopGicp(const GaussianCloud &query, const GaussianCloud &target, const Eigen::Isometry3d &initial);

}  // namespace sapphire
