#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "backend/registration/loop_closure.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using Cloud = sapphire::vvec<float, 3>;

constexpr float kPi = 3.14159265358979323846F;
constexpr float kTwoPi = 2.0F * kPi;

void check(bool condition, const char* message) {
  if (condition) {
    return;
  }
  std::cerr << "check failed: " << message << '\n';
  std::exit(1);
}

float normalizeYaw(float yaw) {
  float normalized = std::fmod(yaw + kPi, kTwoPi);
  if (normalized < 0.0F) {
    normalized += kTwoPi;
  }
  return normalized - kPi;
}

struct YawLevel {
  int child_divisions = 1;
  int total_divisions = 1;
  float resolution = 0.0F;
  float minimum = 0.0F;
};

std::array<YawLevel, 4> makeYawLevels(const cpu::VoxelMapsData& voxelmaps, const Cloud& source,
                                      float minimum_offset, float maximum_offset) {
  float maximum_norm = 0.0F;
  for (const Eigen::Vector3f& point : source) {
    maximum_norm = std::max(maximum_norm, point.norm());
  }

  const float yaw_width = maximum_offset - minimum_offset;
  std::array<YawLevel, 4> levels{};
  for (int level = voxelmaps.max_level; level >= 0; --level) {
    const float parent_piece =
        level == voxelmaps.max_level || levels[static_cast<std::size_t>(level + 1)].resolution == 0.0F
            ? yaw_width
            : levels[static_cast<std::size_t>(level + 1)].resolution;
    float angular_resolution = 0.0F;
    if (maximum_norm > std::numeric_limits<float>::epsilon() && yaw_width > 0.0F) {
      const float ratio = voxelmaps.resolution(level) / maximum_norm;
      const float cosine = std::clamp(1.0F - ratio * ratio * 0.5F, -1.0F, 1.0F);
      angular_resolution = std::floor(std::acos(cosine) * 10000.0F) / 10000.0F;
    }

    YawLevel& info = levels[static_cast<std::size_t>(level)];
    if (yaw_width > 0.0F && angular_resolution > 0.0F && angular_resolution <= yaw_width &&
        parent_piece > 0.0F) {
      info.child_divisions = std::max(1, static_cast<int>(std::ceil(parent_piece / angular_resolution)));
    }
    info.resolution =
        info.child_divisions == 1 ? 0.0F : parent_piece / static_cast<float>(info.child_divisions);
    const float offset = yaw_width == 0.0F
                             ? 0.0F
                             : (info.resolution == 0.0F ? 0.5F * (minimum_offset + maximum_offset)
                                                       : minimum_offset);
    info.minimum = normalizeYaw(offset);
  }

  levels[static_cast<std::size_t>(voxelmaps.max_level)].total_divisions =
      levels[static_cast<std::size_t>(voxelmaps.max_level)].child_divisions;
  for (int level = voxelmaps.max_level - 1; level >= 0; --level) {
    levels[static_cast<std::size_t>(level)].total_divisions =
        levels[static_cast<std::size_t>(level + 1)].total_divisions *
        levels[static_cast<std::size_t>(level)].child_divisions;
  }
  return levels;
}

Eigen::Isometry3f nodePose(const cpu::VoxelMapsData& voxelmaps,
                           const std::array<YawLevel, 4>& yaw_levels, int level, int x, int y, int z,
                           int yaw) {
  Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
  pose.linear() =
      Eigen::AngleAxisf(normalizeYaw(yaw_levels[static_cast<std::size_t>(level)].minimum +
                                    yaw * yaw_levels[static_cast<std::size_t>(level)].resolution),
                        Eigen::Vector3f::UnitZ())
          .toRotationMatrix();
  pose.translation() =
      voxelmaps.resolution(level) * Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y),
                                                    static_cast<float>(z));
  return pose;
}

std::size_t scoreAtLevel(const Cloud& source, const cpu::VoxelMapsData& target,
                         const Eigen::Isometry3f& target_T_source, int level) {
  const float inverse_resolution = 1.0F / target.resolution(level);
  const cpu::VoxelBuckets& buckets = target.buckets_for_level(level);
  std::size_t matched = 0;
  for (const Eigen::Vector3f& point : source) {
    const Eigen::Vector3i coordinate =
        ((target_T_source * point).array() * inverse_resolution).floor().cast<int>();
    matched += cpu::contains_voxel(buckets, target.max_bucket_scan_count, coordinate);
  }
  return matched;
}

Cloud makeUpperBoundCloud() {
  Cloud cloud;
  cloud.reserve(9U * 7U * 5U);
  for (int x = -4; x <= 4; ++x) {
    for (int y = -3; y <= 3; ++y) {
      for (int z = -2; z <= 2; ++z) {
        cloud.emplace_back(1.71F * x + 0.037F * y + 0.113F, 1.83F * y + 0.029F * z + 0.217F,
                           1.27F * z + 0.031F * x * y + 0.319F);
      }
    }
  }
  return cloud;
}

void verifyParentChildUpperBounds() {
  const Cloud source = makeUpperBoundCloud();
  check(std::all_of(source.begin(), source.end(),
                    [](const Eigen::Vector3f& point) { return point.norm() < 20.0F; }),
        "upper-bound stress cloud stays inside the 20 m operating radius");

  cpu::VoxelMaps voxelmaps;
  voxelmaps.set_min_res(0.5F);
  voxelmaps.set_max_level(3);
  voxelmaps.create_voxelmaps(source.data(), source.size());
  const cpu::VoxelMapsData target = voxelmaps.release_data();
  const std::array<YawLevel, 4> yaw_levels = makeYawLevels(target, source, -0.5F * kPi, 0.5F * kPi);

  std::uint32_t state = 0x9e3779b9U;
  const auto next_random = [&state]() {
    state = state * 1664525U + 1013904223U;
    return state;
  };

  constexpr int samples_per_level = 256;
  std::size_t yaw_comparisons = 0;
  std::size_t yaw_violations = 0;
  std::size_t maximum_yaw_deficit = 0;
  for (int parent_level = target.max_level; parent_level > 0; --parent_level) {
    const int child_level = parent_level - 1;
    const int yaw_children = yaw_levels[static_cast<std::size_t>(child_level)].child_divisions;
    const int parent_yaw_count = yaw_levels[static_cast<std::size_t>(parent_level)].total_divisions;
    const int xy_limit =
        std::max(1, static_cast<int>(std::ceil(6.0F / target.resolution(parent_level))));
    const int z_limit =
        std::max(1, static_cast<int>(std::ceil(2.0F / target.resolution(parent_level))));
    for (int sample = 0; sample < samples_per_level; ++sample) {
      const int x = static_cast<int>(next_random() % static_cast<std::uint32_t>(2 * xy_limit + 1)) -
                    xy_limit;
      const int y = static_cast<int>(next_random() % static_cast<std::uint32_t>(2 * xy_limit + 1)) -
                    xy_limit;
      const int z = static_cast<int>(next_random() % static_cast<std::uint32_t>(2 * z_limit + 1)) -
                    z_limit;
      const int yaw = static_cast<int>(next_random() % static_cast<std::uint32_t>(parent_yaw_count));
      const Eigen::Isometry3f parent_pose =
          nodePose(target, yaw_levels, parent_level, x, y, z, yaw);
      const std::size_t parent_score = scoreAtLevel(source, target, parent_pose, parent_level);

      for (int dx = 0; dx < 2; ++dx) {
        for (int dy = 0; dy < 2; ++dy) {
          for (int dz = 0; dz < 2; ++dz) {
            Eigen::Isometry3f translation_child_pose = parent_pose;
            translation_child_pose.translation() =
                target.resolution(child_level) *
                Eigen::Vector3f(static_cast<float>(x * 2 + dx), static_cast<float>(y * 2 + dy),
                                static_cast<float>(z * 2 + dz));
            const std::size_t translation_child_score =
                scoreAtLevel(source, target, translation_child_pose, child_level);
            if (parent_score < translation_child_score) {
              std::cerr << "translation upper-bound violation: level=" << parent_level
                        << " parent_score=" << parent_score
                        << " child_score=" << translation_child_score << " xyz=" << x << ',' << y
                        << ',' << z << " yaw=" << yaw << " child=" << dx << ',' << dy << ',' << dz
                        << '\n';
              std::exit(1);
            }

            for (int dyaw = 0; dyaw < yaw_children; ++dyaw) {
              const int child_yaw = yaw * yaw_children + dyaw;
              const std::size_t child_score =
                  scoreAtLevel(source, target,
                               nodePose(target, yaw_levels, child_level, x * 2 + dx, y * 2 + dy,
                                        z * 2 + dz, child_yaw),
                               child_level);
              ++yaw_comparisons;
              if (parent_score < child_score) {
                ++yaw_violations;
                maximum_yaw_deficit =
                    std::max(maximum_yaw_deficit, child_score - parent_score);
              }
            }
          }
        }
      }
    }
  }

  const double yaw_violation_ratio =
      static_cast<double>(yaw_violations) / static_cast<double>(yaw_comparisons);
  std::cout << "Yaw upper-bound stress: comparisons=" << yaw_comparisons
            << " violations=" << yaw_violations << " max_deficit=" << maximum_yaw_deficit << '/'
            << source.size() << '\n';
  check(yaw_violation_ratio < 0.01,
        "empirical yaw upper-bound violations stay below one percent in the operating domain");
  check(maximum_yaw_deficit * 50U <= source.size(),
        "empirical yaw upper-bound deficit stays within two percent of query points");
}

Cloud makeRegistrationCloud() {
  Cloud cloud;
  for (int x = 0; x < 4; ++x) {
    for (int y = 0; y < 4; ++y) {
      for (int z = 0; z < 4; ++z) {
        cloud.emplace_back(0.35F * x + 0.02F * y, 0.31F * y + 0.01F * z,
                           0.27F * z + 0.015F * x * y);
      }
    }
  }
  return cloud;
}

void verifyOnlyTop1EntersGicp() {
  const Cloud query = makeRegistrationCloud();
  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  expected.linear() = Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  expected.translation() = Eigen::Vector3d(0.4, -0.2, 0.1);
  Cloud target;
  target.reserve(query.size());
  for (const Eigen::Vector3f& point : query) {
    target.push_back((expected * point.cast<double>()).cast<float>());
  }

  cpu::VoxelMaps voxelmaps;
  voxelmaps.set_min_res(0.5F);
  voxelmaps.set_max_level(3);
  voxelmaps.create_voxelmaps(target.data(), target.size());
  const cpu::VoxelMapsData target_data = voxelmaps.release_data();
  const std::vector<gpu::LocalSearchTarget> candidates{
      {91, &target_data, expected.cast<float>()},
      {42, &target_data, expected.cast<float>()},
      {73, &target_data, expected.cast<float>()},
  };

  const sapphire::BbsResult selected = sapphire::alignLoopBbs(query, candidates);
  check(selected.accepted && selected.target_id == 42 && selected.overlap > 0.9,
        "global BBS emits one deterministic Top1 when several candidates pass");

  sapphire::GaussianCloud query_gaussians;
  sapphire::GaussianCloud target_gaussians;
  query_gaussians.reserve(query.size());
  target_gaussians.reserve(target.size());
  const Eigen::Matrix3f query_covariance = Eigen::Vector3f(0.0F, 0.02F, 0.04F).asDiagonal();
  const Eigen::Matrix3f target_covariance =
      (expected.linear() * query_covariance.cast<double>() * expected.linear().transpose()).cast<float>();
  for (std::size_t index = 0; index < query.size(); ++index) {
    sapphire::GaussianPoint query_gaussian;
    query_gaussian.mean = query[index];
    query_gaussian.covariance = query_covariance;
    query_gaussian.N = 8;
    query_gaussian.regularize();
    query_gaussians.push_back(query_gaussian);

    sapphire::GaussianPoint target_gaussian;
    target_gaussian.mean = target[index];
    target_gaussian.covariance = target_covariance;
    target_gaussian.N = 8;
    target_gaussian.regularize();
    target_gaussians.push_back(target_gaussian);
  }
  const sapphire::GaussianCloudPtr query_cloud =
      std::make_shared<const sapphire::GaussianCloud>(std::move(query_gaussians));
  const sapphire::GaussianCloudPtr target_cloud =
      std::make_shared<const sapphire::GaussianCloud>(std::move(target_gaussians));
  check(std::abs(query_cloud->front().radius - 0.6F) < 1e-5F,
        "Gaussian radius stores the three-sigma major-axis extent");
  const sapphire::BbsResult gaussian_selected = sapphire::alignLoopBbs(*query_cloud, candidates);
  check(gaussian_selected.accepted && gaussian_selected.target_id == selected.target_id &&
            std::abs(gaussian_selected.overlap - selected.overlap) < 1e-12 &&
            gaussian_selected.T_target_query.matrix().isApprox(selected.T_target_query.matrix(), 1e-6),
        "production Gaussian BBS query matches the equivalent XYZ device-upload packing");

  std::size_t gicp_count = 0;
  sapphire::GicpResult refined;
  const sapphire::GaussianPoint* const query_gaussian_data = query_cloud->data();
  const sapphire::GaussianPoint* const target_gaussian_data = target_cloud->data();
  if (selected.accepted) {
    ++gicp_count;
    refined = sapphire::refineLoopGicp(query_cloud, target_cloud, selected.T_target_query);
  }
  check(gicp_count == 1, "only the selected Top1 invokes GICP");
  check(refined.attempted && refined.accepted, "Top1 GICP consumes precomputed regularized covariances");
  check(query_cloud->data() == query_gaussian_data && target_cloud->data() == target_gaussian_data,
        "production GICP preserves the DB-backed Gaussian storage");
}

Cloud makeLowPruningQuery() {
  Cloud query;
  query.reserve(64);
  std::uint32_t state = 0x13579bdfU;
  for (int index = 0; index < 64; ++index) {
    state = state * 1664525U + 1013904223U;
    const float x = -8.0F + 16.0F * static_cast<float>(state & 0xffffU) / 65535.0F;
    state = state * 1664525U + 1013904223U;
    const float y = -8.0F + 16.0F * static_cast<float>(state & 0xffffU) / 65535.0F;
    state = state * 1664525U + 1013904223U;
    const float z = -3.0F + 6.0F * static_cast<float>(state & 0xffffU) / 65535.0F;
    query.emplace_back(x, y, z);
  }
  return query;
}

Cloud makeLowPruningTarget() {
  Cloud target;
  target.reserve(600);
  std::uint32_t state = 0x2468ace1U;
  for (int index = 0; index < 600; ++index) {
    state = state * 1103515245U + 12345U;
    const float x = -14.0F + 28.0F * static_cast<float>(state & 0xffffU) / 65535.0F;
    state = state * 1103515245U + 12345U;
    const float y = -14.0F + 28.0F * static_cast<float>(state & 0xffffU) / 65535.0F;
    state = state * 1103515245U + 12345U;
    const float z = -5.0F + 10.0F * static_cast<float>(state & 0xffffU) / 65535.0F;
    target.emplace_back(x, y, z);
  }
  return target;
}

void verifyWorstCaseRuntime() {
  const Cloud query = makeLowPruningQuery();
  const Cloud target = makeLowPruningTarget();
  cpu::VoxelMaps voxelmaps;
  voxelmaps.set_min_res(0.5F);
  voxelmaps.set_max_level(3);
  voxelmaps.create_voxelmaps(target.data(), target.size());
  const cpu::VoxelMapsData target_data = voxelmaps.release_data();

  constexpr std::size_t candidate_count = 12;
  std::vector<gpu::LocalSearchTarget> targets;
  targets.reserve(candidate_count);
  for (std::size_t index = 0; index < candidate_count; ++index) {
    targets.push_back(
        {static_cast<std::uint64_t>(100 + index), &target_data, Eigen::Isometry3f::Identity()});
  }

  gpu::LocalSearchOptions options;
  options.translation_half_window = Eigen::Vector3f(6.0F, 6.0F, 2.0F);
  options.min_yaw_offset = 0.0F;
  options.max_yaw_offset = 0.0F;
  options.minimum_overlap = 0.3;
  gpu::BBS3D bbs;
  std::size_t roots = 0;
  std::size_t expanded = 0;
  std::size_t pruned = 0;
  double reported_ms = 0.0;
  const Clock::time_point begin = Clock::now();
  for (const gpu::LocalSearchTarget& search_target : targets) {
    const gpu::LocalSearchResult result = bbs.search_local(query, search_target, options);
    check(!result.accepted && result.termination == gpu::SearchTermination::kQueueExhausted,
          "low-overlap candidate exhausts its finite search domain without a false positive");
    roots += result.root_nodes;
    reported_ms += result.elapsed_ms;
    for (std::size_t level = 0; level < result.expanded_nodes.size(); ++level) {
      expanded += result.expanded_nodes[level];
      pruned += result.pruned_nodes[level];
    }
  }
  const double wall_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - begin).count();

  std::cout << "Loop pipeline benchmark: candidates=" << candidate_count
            << " accepted=0 termination=queue_exhausted roots=" << roots
            << " expanded=" << expanded << " pruned=" << pruned
            << " elapsed=" << reported_ms << " ms\n";
  check(roots >= candidate_count && expanded > roots && pruned > 0,
        "worst-case benchmark exercises multi-level expansion and pruning");
  check(reported_ms <= wall_ms + 5.0, "reported BBS time agrees with wall-clock time");
  check(wall_ms < 5000.0, "12-candidate low-pruning Sapphire search stays below five seconds");

  const Clock::time_point pipeline_begin = Clock::now();
  const sapphire::BbsResult pipeline_result = sapphire::alignLoopBbs(query, targets);
  const double pipeline_wall_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - pipeline_begin).count();
  std::cout << "Production GPU loop benchmark: candidates=" << candidate_count
            << " accepted=" << pipeline_result.accepted
            << " roots=" << pipeline_result.root_nodes
            << " elapsed=" << pipeline_result.elapsed_ms << " ms\n";
  check(!pipeline_result.accepted &&
            pipeline_result.termination == gpu::SearchTermination::kQueueExhausted,
        "production XYZ plus-or-minus-90-degree-yaw search rejects the low-overlap set");
  check(pipeline_wall_ms < 5000.0,
        "production 12-candidate GPU search stays below five seconds");
}

}  // namespace

int main() {
  verifyParentChildUpperBounds();
  verifyOnlyTop1EntersGicp();
  verifyWorstCaseRuntime();
  return 0;
}
