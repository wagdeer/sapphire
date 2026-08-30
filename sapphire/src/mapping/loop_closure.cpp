#include "loop_closure.hpp"

#include <bbs3d.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

#include "../common/common.hpp"
#include "memory.hpp"

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

bool isEligibleLoopTarget(std::uint64_t target_id, std::size_t query_id, LoopCandidateStats &stats) noexcept {
  ++stats.spatial_matches;
  if (target_id >= query_id) {
    ++stats.rejected_non_historical;
    return false;
  }
  if (query_id - static_cast<std::size_t>(target_id) <= 3) {
    ++stats.rejected_adjacent;
    return false;
  }
  return true;
}

LoopCandidateBatch prepareLoopCandidates(Memory &memory, std::size_t query_id, const Eigen::Isometry3d &map_T_query) {
  LoopCandidateBatch batch;
  const Eigen::Isometry3f map_T_query_f = map_T_query.cast<float>();
  for (const SpatialMatch &match : memory.recallSpatial(query_id, map_T_query_f)) {
    if (!isEligibleLoopTarget(match.submap_id, query_id, batch.stats)) {
      continue;
    }
    if (!batch.query_cloud) {
      batch.query_cloud = memory.loadCloud(query_id);
      ++batch.stats.cold_loads;
    }
    if (!batch.query_cloud || batch.query_cloud->empty()) {
      break;
    }

    const Eigen::Isometry3f target_T_query_initial = match.map_T_submap.inverse() * map_T_query_f;
    cpu::VoxelMapsData target_voxelmaps = memory.loadPyramidVoxel(match.submap_id);
    ++batch.stats.cold_loads;
    batch.candidates.push_back({match.submap_id, target_T_query_initial, std::move(target_voxelmaps)});
  }
  return batch;
}

BbsResult alignLoopBbs(const vvec<float, 3> &query, const std::vector<gpu::LocalSearchTarget> &targets) {
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
    if (!search.accepted ||
        (result.accepted && (search.score.overlap < result.overlap ||
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

GicpResult refineLoopGicp(const vvec<float, 3> &query, const vvec<float, 3> &target, const Eigen::Isometry3d &initial) {
  const auto total_start = std::chrono::steady_clock::now();
  GicpResult result;
  result.attempted = true;
  constexpr std::size_t min_inliers = 64;
  result.query_points = query.size();
  result.target_points = target.size();
  if (query.size() < min_inliers || target.size() < min_inliers || !initial.matrix().allFinite()) {
    result.total_elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - total_start).count();
    return result;
  }

  constexpr int num_threads = 4;
  constexpr int num_neighbors = 16;
  auto target_points = std::make_shared<small_gicp::PointCloud>(target);
  auto query_points = std::make_shared<small_gicp::PointCloud>(query);
  small_gicp::KdTree<small_gicp::PointCloud> target_tree(target_points, small_gicp::KdTreeBuilderOMP(num_threads));
  small_gicp::KdTree<small_gicp::PointCloud> query_tree(query_points, small_gicp::KdTreeBuilderOMP(num_threads));
  small_gicp::estimate_covariances_omp(*target_points, target_tree, num_neighbors, num_threads);
  small_gicp::estimate_covariances_omp(*query_points, query_tree, num_neighbors, num_threads);
  result.preparation_elapsed_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - total_start).count();

  small_gicp::RegistrationSetting setting;
  setting.type = small_gicp::RegistrationSetting::GICP;
  setting.num_threads = num_threads;
  setting.max_iterations = 32;
  setting.max_correspondence_distance = 2.0;
  setting.translation_eps = 0.01;
  setting.rotation_eps = 0.01;
  const auto alignment_start = std::chrono::steady_clock::now();
  const small_gicp::RegistrationResult registration = small_gicp::align(*target_points, *query_points, target_tree, initial, setting);
  result.alignment_elapsed_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - alignment_start).count();

  result.converged = registration.converged;
  result.inliers = registration.num_inliers;
  result.T_target_query = registration.T_target_source;

  constexpr double maximum_squared_distance = 4.0;
  double squared_error = 0.0;
  std::size_t fitness_inliers = 0;
  for (const Eigen::Vector4d &point : query_points->points) {
    std::size_t index = 0;
    double squared_distance = 0.0;
    const Eigen::Vector4d transformed = result.T_target_query.matrix() * point;
    if (small_gicp::traits::nearest_neighbor_search(target_tree, transformed, &index, &squared_distance) &&
        squared_distance <= maximum_squared_distance) {
      squared_error += squared_distance;
      ++fitness_inliers;
    }
  }
  result.fitness =
      fitness_inliers > 0 ? squared_error / static_cast<double>(fitness_inliers) : std::numeric_limits<double>::infinity();
  result.accepted = result.converged && result.inliers >= min_inliers && result.T_target_query.matrix().allFinite() &&
                    std::isfinite(result.fitness) && result.fitness < 0.5;
  result.total_elapsed_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - total_start).count();
  return result;
}

}  // namespace sapphire
