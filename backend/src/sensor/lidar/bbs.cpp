#include "backend/sensor/lidar/bbs.hpp"

#include <algorithm>
#include <bbs3d.hpp>
#include <cmath>
#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

#include "frontend/common/point.hpp"

namespace sapphire {
namespace {

vvec<float, 3> sampleVoxelRepresentatives(const vvec<float, 3> &points, float voxel_size) {
  vvec<float, 3> sampled;
  if (!(std::isfinite(voxel_size) && voxel_size > 0.0F)) {
    return sampled;
  }
  std::unordered_set<Eigen::Vector3i, cpu::VoxelMaps::VectorHash, cpu::VoxelMaps::VctorEqual> occupied;
  occupied.reserve(points.size());
  sampled.reserve(points.size());
  const float inverse_voxel_size = 1.0F / voxel_size;
  for (const Eigen::Vector3f &point : points) {
    if (!point.allFinite()) {
      continue;
    }
    const Eigen::Vector3i coordinate = (point.array() * inverse_voxel_size).floor().cast<int>();
    if (occupied.insert(coordinate).second) {
      sampled.push_back(point);
    }
  }
  return sampled;
}

}  // namespace

bool acceptsLoopSeedScore(const gpu::PoseScore &score) {
  if (score.available && (!std::isfinite(score.overlap) || score.overlap < 0 || score.overlap > 1 ||
      !score.evaluated_points || score.matched_points > score.evaluated_points))
    throw std::runtime_error("Invalid numerical LiDAR seed score");
  return score.available && score.overlap >= .5;
}

gpu::PoseScore scoreLoopSeed(const GaussianCloud &query, const cpu::VoxelMapsData &target,
                            const Eigen::Isometry3f &T_target_query) {
  vvec<float, 3> means;
  means.reserve(query.size());
  for (const auto &point : query) if (point.mean.allFinite()) means.push_back(point.mean);
  const auto representatives = sampleVoxelRepresentatives(means, 1.0F);
  return gpu::BBS3D().score_pose(representatives, target, T_target_query);
}

BbsResult alignLoopBbs(const vvec<float, 3> &query, const std::vector<gpu::LocalSearchTarget> &targets,
                       const std::optional<gpu::LocalSearchOptions> &override_options) {
  BbsResult result;
  if (query.empty() || targets.empty()) {
    return result;
  }

  vvec<float, 3> bbs_query = sampleVoxelRepresentatives(query, 1.0F);
  if (bbs_query.empty()) {
    return result;
  }

  gpu::LocalSearchOptions options;
  options.translation_half_window = Eigen::Vector3f(6.0F, 6.0F, 2.0F);
  options.min_yaw_offset = -toRAD(90.0F);
  options.max_yaw_offset = toRAD(90.0F);
  options.minimum_overlap = 0.5;
  if (override_options) options = *override_options;
  gpu::BBS3D bbs;
  for (const gpu::LocalSearchTarget &target : targets) {
    const gpu::LocalSearchResult search = bbs.search_local(bbs_query, target, options);
    result.termination = search.termination;
    result.elapsed_ms += search.elapsed_ms;
    result.root_nodes += search.root_nodes;
    for (std::size_t level = 0; level < result.expanded_nodes.size(); ++level) {
      result.expanded_nodes[level] += search.expanded_nodes[level];
      result.pruned_nodes[level] += search.pruned_nodes[level];
    }
    if (!search.accepted || (result.accepted && (search.score.overlap < result.overlap ||
                                                 (search.score.overlap == result.overlap && search.target_id > result.target_id)))) {
      result.initial_overlap = std::max(result.initial_overlap, search.initial_overlap);
      continue;
    }
    result.accepted = true;
    result.target_id = search.target_id;
    result.overlap = search.score.overlap;
    result.initial_overlap = search.initial_overlap;
    result.T_target_query = search.target_T_source.cast<double>();
    result.termination = search.termination;
  }
  result.accepted = result.accepted && result.T_target_query.matrix().allFinite();
  return result;
}

BbsResult alignLoopBbs(const GaussianCloud &query, const std::vector<gpu::LocalSearchTarget> &targets,
                       const std::optional<gpu::LocalSearchOptions> &override_options) {
  vvec<float, 3> means;
  means.reserve(query.size());
  for (const GaussianPoint &point : query) {
    if (point.mean.allFinite()) {
      means.emplace_back(point.mean);
    }
  }
  return alignLoopBbs(means, targets, override_options);
}

}  // namespace sapphire
