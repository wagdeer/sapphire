#include "backend/registration/loop_closure.hpp"

#include <omp.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <bbs3d.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <unordered_set>
#include <utility>
#include <vector>

#include "common/common.hpp"
#include "backend/storage/memory.hpp"
#include "backend/registration/revisit_coverage.hpp"
#include "tools/timer.hpp"

namespace sapphire {
namespace {

using GicpClock = std::chrono::steady_clock;
constexpr std::size_t kMinimumGicpInliers = 64;

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

GicpResult rejectedGicp(std::size_t query_size, std::size_t target_size, const GicpClock::time_point &total_start) {
  GicpResult result;
  result.attempted = true;
  result.query_points = query_size;
  result.target_points = target_size;
  result.total_elapsed_ms = std::chrono::duration<double, std::milli>(GicpClock::now() - total_start).count();
  return result;
}

GicpResult alignPreparedGicp(const GaussianCloudPtr &query_points, const GaussianCloudPtr &target_points,
                             const small_gicp::KdTree<GaussianCloud> &target_tree, const Eigen::Isometry3d &initial,
                             const GicpClock::time_point &total_start) {
  GicpResult result;
  result.attempted = true;
  result.query_points = query_points->size();
  result.target_points = target_points->size();
  result.preparation_elapsed_ms = std::chrono::duration<double, std::milli>(GicpClock::now() - total_start).count();

  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
  registration.reduction.num_threads = 4;
  registration.optimizer.max_iterations = 32;
  registration.rejector.max_dist_sq = 2.0 * 2.0;
  registration.criteria.translation_eps = 0.01;
  registration.criteria.rotation_eps = 0.01;
  const auto alignment_start = GicpClock::now();
  const small_gicp::RegistrationResult registration_result = registration.align(*target_points, *query_points, target_tree, initial);
  result.alignment_elapsed_ms = std::chrono::duration<double, std::milli>(GicpClock::now() - alignment_start).count();

  result.converged = registration_result.converged;
  result.inliers = registration_result.num_inliers;
  result.T_target_query = registration_result.T_target_source;

  constexpr double maximum_squared_distance = 4.0;
  double squared_error = 0.0;
  std::size_t fitness_inliers = 0;
  for (std::size_t query_index = 0; query_index < query_points->size(); ++query_index) {
    std::size_t index = 0;
    double squared_distance = 0.0;
    const Eigen::Vector4d transformed = result.T_target_query.matrix() * small_gicp::traits::point(*query_points, query_index);
    if (small_gicp::traits::nearest_neighbor_search(target_tree, transformed, &index, &squared_distance) &&
        squared_distance <= maximum_squared_distance) {
      squared_error += squared_distance;
      ++fitness_inliers;
    }
  }
  result.fitness = fitness_inliers > 0 ? squared_error / static_cast<double>(fitness_inliers) : std::numeric_limits<double>::infinity();
  result.accepted = result.converged && result.inliers >= kMinimumGicpInliers && result.T_target_query.matrix().allFinite() &&
                    std::isfinite(result.fitness) && result.fitness < 0.5;
  result.total_elapsed_ms = std::chrono::duration<double, std::milli>(GicpClock::now() - total_start).count();
  return result;
}

}  // namespace

bool isEligibleLoopTarget(std::uint64_t target_id, std::size_t query_id, LoopCandidateStats &stats) noexcept {
  ++stats.spatial_matches;
  if (target_id >= query_id) {
    ++stats.rejected_non_historical;
    return false;
  }
  if (!loop_policy::eligible(query_id, target_id, 0, 0, query_id)) {
    ++stats.rejected_adjacent;
    return false;
  }
  return true;
}

static LoopCandidateBatch prepareCandidates(Memory &memory, std::size_t query_id, std::uint64_t query_root,
                                            const Eigen::Isometry3d &map_T_query, const std::vector<SpatialMatch> &spatial,
                                            GaussianCloudPtr transient, const std::vector<VisualSubmapMatch> &visual) {
  LoopCandidateBatch batch;
  const Eigen::Isometry3f map_T_query_f = map_T_query.cast<float>();
  for (const SpatialMatch &match : spatial) {
    ++batch.stats.spatial_matches;
    if (!memory.eligibleLoop(query_id, match.submap_id, query_root)) {
      if (match.submap_id >= query_id) ++batch.stats.rejected_non_historical;
      else ++batch.stats.rejected_adjacent;
      continue;
    }
    if (!batch.query_cloud) {
      batch.query_cloud = transient ? transient : memory.loadCloud(query_id);
      if (!transient) ++batch.stats.cold_loads;
    }
    if (!batch.query_cloud || batch.query_cloud->empty()) {
      break;
    }

    const Eigen::Isometry3f target_T_query_initial = match.map_T_submap.inverse() * map_T_query_f;
    cpu::VoxelMapsData target_voxelmaps = memory.loadPyramidVoxel(match.submap_id);
    ++batch.stats.cold_loads;
    batch.candidates.push_back({match.submap_id, target_T_query_initial, std::move(target_voxelmaps)});
  }
  for (const auto &seed : visual) {
    if (!memory.eligibleLoop(query_id, seed.target_id, query_root)) continue;
    auto existing = std::find_if(batch.candidates.begin(), batch.candidates.end(), [&](const auto &c) { return c.target_id == seed.target_id; });
    if (seed.T_target_query) {
      const auto &pose = *seed.T_target_query;
      if (!pose.matrix().allFinite() || !pose.matrix().cast<float>().allFinite() ||
          (pose.linear().transpose() * pose.linear() - Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
          std::abs(pose.linear().determinant() - 1) > 1e-6 ||
          (pose.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() > 1e-9)
        throw std::invalid_argument("Invalid metric visual loop seed");
      if (existing != batch.candidates.end()) {
        existing->target_T_query_initial = pose.cast<float>();
        existing->visual = existing->metric_visual = true;
        continue;
      }
    }
    // Appearance-only retrieval preserves a spatial initial guess. One target,
    // one initializer; metric visual evidence deliberately supersedes that guess.
    if (existing != batch.candidates.end()) continue;
    const auto target_pose = memory.submapPose(seed.target_id);
    if (!target_pose) continue;
    Eigen::Isometry3f initial = target_pose->inverse() * map_T_query_f;
    // Without a metric pose, search around the historical origin to allow
    // translation drift independently of spatial overlap retrieval.
    if (seed.T_target_query) initial = seed.T_target_query->cast<float>();
    else initial.translation().setZero();
    if (!batch.query_cloud) {
      batch.query_cloud = transient ? transient : memory.loadCloud(query_id);
      if (!transient) ++batch.stats.cold_loads;
    }
    if (!batch.query_cloud || batch.query_cloud->empty()) break;
    auto voxels = memory.loadPyramidVoxel(seed.target_id);
    ++batch.stats.cold_loads;
    batch.candidates.push_back({seed.target_id, initial, std::move(voxels), true, bool(seed.T_target_query)});
  }
  std::stable_sort(batch.candidates.begin(), batch.candidates.end(), [](const auto &a, const auto &b) { return a.metric_visual != b.metric_visual ? a.metric_visual > b.metric_visual : a.visual > b.visual; });
  return batch;
}

LoopCandidateBatch prepareLoopCandidates(Memory &memory, std::size_t query_id, const Eigen::Isometry3d &map_T_query,
                                        const std::vector<VisualSubmapMatch> &visual) {
  return prepareCandidates(memory, query_id, memory.chainRoot(query_id), map_T_query,
                           memory.recallSpatial(query_id, map_T_query.cast<float>()), {}, visual);
}
LoopCandidateBatch prepareLoopCandidates(Memory &memory, const SubmapFrame &query, const Eigen::Isometry3d &map_T_query,
                                        const std::vector<VisualSubmapMatch> &visual) {
  if (!query.id() || memory.submapPose(query.id()) || !memory.submapPose(query.id() - 1))
    throw std::logic_error("Transient loop query requires the committed tail predecessor");
  return prepareCandidates(memory, query.id(), memory.chainRoot(query.id() - 1), map_T_query,
                           memory.recallSpatial(query.bounds(), map_T_query.cast<float>()), query.lio().pcd, visual);
}

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

GicpResult refineLoopGicp(const GaussianCloudPtr &query, const GaussianCloudPtr &target, const Eigen::Isometry3d &initial) {
  const auto total_start = GicpClock::now();
  const std::size_t query_size = query ? query->size() : 0;
  const std::size_t target_size = target ? target->size() : 0;
  if (!query || !target || query_size < kMinimumGicpInliers || target_size < kMinimumGicpInliers || !initial.matrix().allFinite()) {
    return rejectedGicp(query_size, target_size, total_start);
  }

  const auto invalid_gaussian = [](const GaussianPoint &point) { return !point.valid(); };
  if (std::any_of(query->begin(), query->end(), invalid_gaussian) || std::any_of(target->begin(), target->end(), invalid_gaussian)) {
    return rejectedGicp(query_size, target_size, total_start);
  }

  constexpr int num_threads = 4;
  small_gicp::KdTree<GaussianCloud> target_tree(target, small_gicp::KdTreeBuilderOMP(num_threads));
  auto result = alignPreparedGicp(query, target, target_tree, initial, total_start);
  if (result.accepted && timer::enabled()) {
    const auto start = GicpClock::now();
    const auto coverage = measureRevisitCoverage(*query, *target, result.T_target_query);
    if (coverage.support) {
      const auto &s = *coverage.support;
      spdlog::info("[revisit-coverage] status=complete query_points={} target_points={} pairs={} query_fraction={} target_fraction={} query_cells={}/{} target_cells={}/{} comparisons={} ms={}",
                   coverage.query_points, coverage.target_points, s.pairs, s.query_fraction, s.target_fraction,
                   s.supported_query_cells, s.query_cells, s.supported_target_cells, s.target_cells,
                   coverage.comparisons, timer::ms(start));
    } else {
      spdlog::info("[revisit-coverage] status={} query_points={} target_points={} comparisons={} ms={}",
                   int(coverage.status), coverage.query_points, coverage.target_points, coverage.comparisons, timer::ms(start));
    }
  }
  return result;
}

GicpResult refineLoopGicp(const GaussianCloud &query, const GaussianCloud &target, const Eigen::Isometry3d &initial) {
  const GaussianCloudPtr query_view(&query, [](const GaussianCloud *) {});
  const GaussianCloudPtr target_view(&target, [](const GaussianCloud *) {});
  return refineLoopGicp(query_view, target_view, initial);
}

}  // namespace sapphire
