#include <array>
#include <cmath>
#include <cstdlib>
#include <bbs3d.hpp>
#include <iostream>

#include "loop_closure.hpp"
#include "retrieval_index.hpp"

namespace {

void check(bool condition, const char *message) {
  if (condition) {
    return;
  }
  std::cerr << "check failed: " << message << '\n';
  std::exit(1);
}

Eigen::Isometry3f pose(float x, float y, float z) {
  Eigen::Isometry3f value = Eigen::Isometry3f::Identity();
  value.translation() = Eigen::Vector3f(x, y, z);
  return value;
}

Eigen::Isometry3f yawPose(float yaw) {
  Eigen::Isometry3f value = Eigen::Isometry3f::Identity();
  value.linear() = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  return value;
}

sapphire::vvec<float, 3> makeCloud() {
  sapphire::vvec<float, 3> cloud;
  for (int x = 0; x < 4; ++x) {
    for (int y = 0; y < 4; ++y) {
      for (int z = 0; z < 4; ++z) {
        cloud.emplace_back(0.35F * x + 0.02F * y, 0.31F * y + 0.01F * z, 0.27F * z + 0.015F * x * y);
      }
    }
  }
  return cloud;
}

}  // namespace

int main() {
  sapphire::PoseRecallIndex pose_index;
  pose_index.upsert(10, pose(0.0F, 0.0F, 0.0F));
  pose_index.upsert(15, pose(1.0F, 0.0F, 0.0F));
  pose_index.upsert(20, pose(2.0F, 0.0F, 0.0F));
  pose_index.upsert(30, pose(5.0F, 0.0F, 0.0F));

  std::array<sapphire::RecallMatch, 2> knn_matches;
  const auto accept_all = [](const sapphire::RecallMatch &) { return true; };
  std::size_t found = pose_index.knnSearch(pose(1.8F, 0.0F, 0.0F), knn_matches.size(), 100.0F, accept_all, knn_matches.data());
  check(found == 2 && knn_matches[0].submap_id == 20 && knn_matches[1].submap_id == 15, "generic pose KNN returns ordered neighbors");

  sapphire::RecallMatch boundary;
  found = pose_index.knnSearch(pose(0.0F, 0.0F, 0.0F), 1, 4.0F,
                               [](const sapphire::RecallMatch &candidate) { return candidate.submap_id == 20; }, &boundary);
  check(found == 1 && boundary.submap_id == 20, "filtered pose KNN includes squared-radius boundary");

  std::vector<sapphire::RecallMatch> matches = pose_index.query(99, pose(1.8F, 0.0F, 0.0F), 2, 100.0F);
  check(matches.size() == 2 && matches.front().submap_id == 20, "pose KD-tree nearest neighbors");

  pose_index.upsert(20, pose(8.0F, 0.0F, 0.0F));
  matches = pose_index.query(99, pose(2.0F, 0.0F, 0.0F), 1, 0.25F);
  check(matches.empty(), "pose KD-tree removes stale updated position");
  matches = pose_index.query(99, pose(8.0F, 0.0F, 0.0F), 1, 0.25F);
  check(matches.size() == 1 && matches.front().submap_id == 20, "pose KD-tree inserts updated position");

  pose_index.upsert(
      {{10, pose(10.0F, 0.0F, 0.0F)}, {15, pose(11.0F, 0.0F, 0.0F)}, {20, pose(12.0F, 0.0F, 0.0F)}, {40, pose(13.0F, 0.0F, 0.0F)}});
  matches = pose_index.query(99, pose(0.0F, 0.0F, 0.0F), 1, 0.25F);
  check(matches.empty(), "pose KD-tree batch rebuild removes stale positions");
  matches = pose_index.query(99, pose(12.1F, 0.0F, 0.0F), 2, 100.0F);
  check(matches.size() == 2 && matches[0].submap_id == 20 && matches[1].submap_id == 40 && pose_index.size() == 5,
        "pose KD-tree batch rebuild updates and inserts entries");

  pose_index.erase(20);
  check(!pose_index.contains(20), "pose KD-tree erase");

  sapphire::SubmapSpatialIndex spatial_index;
  const sapphire::AABB local_bounds{Eigen::Vector3f(-2.0F, -1.0F, -0.5F), Eigen::Vector3f(2.0F, 1.0F, 0.5F)};
  const sapphire::AABB overlap_bounds{Eigen::Vector3f(0.0F, -2.0F, 0.0F), Eigen::Vector3f(3.0F, 0.0F, 1.0F)};
  const sapphire::AABB intersection = local_bounds.intersection(overlap_bounds);
  check(intersection.valid() && intersection.minimum.isApprox(Eigen::Vector3f(0.0F, -1.0F, 0.0F)) &&
            intersection.maximum.isApprox(Eigen::Vector3f(2.0F, 0.0F, 0.5F)),
        "AABB computes its geometric intersection without Boost");
  check(std::abs(local_bounds.xyOverlapRatio(overlap_bounds) - 0.25F) < 1e-6F,
        "AABB computes query-normalized XY overlap");
  spatial_index.upsert(0, local_bounds, pose(6.1F, 0.0F, 0.0F));
  spatial_index.upsert(1, local_bounds, pose(8.0F, 0.0F, 0.0F));
  spatial_index.upsert(2, local_bounds, pose(10.0F, 0.0F, 0.0F));
  spatial_index.upsert(3, local_bounds, pose(4.5F, 0.0F, 0.0F));
  std::vector<sapphire::SpatialMatch> spatial_matches = spatial_index.query(1, pose(8.0F, 0.0F, 0.0F));
  check(spatial_matches.size() == 2, "R-tree requires more than 50 percent query XY overlap");
  spatial_index.updatePoses({{0, pose(-20.0F, 0.0F, 0.0F)}});
  spatial_matches = spatial_index.query(1, pose(8.0F, 0.0F, 0.0F));
  check(spatial_matches.size() == 1 && spatial_matches.front().submap_id == 1,
        "R-tree removes stale bounds after optimized-pose update");

  sapphire::LoopCandidateStats adjacent_stats;
  check(!sapphire::isEligibleLoopTarget(2, 5, adjacent_stats) && adjacent_stats.rejected_adjacent == 1,
        "loop candidate gate rejects the previous three submaps");

  sapphire::LoopCandidateStats historical_stats;
  check(!sapphire::isEligibleLoopTarget(5, 5, historical_stats) && historical_stats.rejected_non_historical == 1,
        "loop candidate gate rejects current and future submaps");
  sapphire::LoopCandidateStats valid_stats;
  check(sapphire::isEligibleLoopTarget(0, 5, valid_stats), "loop candidate gate accepts valid non-adjacent history");

  const sapphire::vvec<float, 3> query = makeCloud();
  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  expected.linear() = Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  expected.translation() = Eigen::Vector3d(0.4, -0.2, 0.1);
  sapphire::vvec<float, 3> target;
  target.reserve(query.size());
  for (const Eigen::Vector3f &point : query) {
    target.push_back((expected * point.cast<double>()).cast<float>());
  }

  cpu::VoxelMaps target_voxelmaps;
  target_voxelmaps.set_min_res(0.1F);
  target_voxelmaps.set_max_level(1);
  target_voxelmaps.create_voxelmaps(target.data(), target.size());
  const cpu::VoxelMapsData target_voxel_data = target_voxelmaps.release_data();
  gpu::BBS3D bbs;
  const gpu::PoseScore tolerant_score = bbs.score_pose(query, target_voxel_data, expected.cast<float>());
  check(tolerant_score.available && tolerant_score.overlap > 0.9,
        "score_pose reports GPU tolerant-voxel overlap");
  gpu::LocalSearchOptions local_options;
  local_options.translation_half_window.x() = 0.25F;
  local_options.min_yaw_offset = 0.0F;
  local_options.max_yaw_offset = 0.0F;
  local_options.minimum_overlap = 0.9;
  Eigen::Isometry3f local_initial = expected.cast<float>();
  local_initial.translation().x() += 0.2F;
  const gpu::LocalSearchTarget local_target{7, &target_voxel_data, local_initial};
  const gpu::LocalSearchResult local_search = bbs.search_local(query, local_target, local_options);
  check(local_search.accepted && local_search.target_id == 7 && local_search.score.overlap > 0.9,
        "search_local performs bounded BBS search and exact-score validation");

  sapphire::vvec<float, 3> adversarial_query;
  sapphire::vvec<float, 3> coarse_decoy_target;
  sapphire::vvec<float, 3> exact_winner_target;
  for (int index = 0; index < 10; ++index) {
    const Eigen::Vector3f point(2.0F * index + 0.01F, 0.01F, 0.01F);
    adversarial_query.push_back(point);
    if (index < 5) {
      coarse_decoy_target.push_back(point + Eigen::Vector3f(2.11F + 0.1F * (index % 3), 0.0F, 0.0F));
    }
    if (index < 7) {
      exact_winner_target.push_back(point + Eigen::Vector3f(-2.0F, 0.0F, 0.0F));
    }
  }
  cpu::VoxelMaps coarse_decoy_voxelmaps;
  coarse_decoy_voxelmaps.set_min_res(0.1F);
  coarse_decoy_voxelmaps.set_max_level(2);
  coarse_decoy_voxelmaps.create_voxelmaps(coarse_decoy_target.data(), coarse_decoy_target.size());
  const cpu::VoxelMapsData coarse_decoy_data = coarse_decoy_voxelmaps.release_data();
  cpu::VoxelMaps exact_winner_voxelmaps;
  exact_winner_voxelmaps.set_min_res(0.1F);
  exact_winner_voxelmaps.set_max_level(2);
  exact_winner_voxelmaps.create_voxelmaps(exact_winner_target.data(), exact_winner_target.size());
  const cpu::VoxelMapsData exact_winner_data = exact_winner_voxelmaps.release_data();
  gpu::LocalSearchOptions adversarial_options;
  adversarial_options.translation_half_window = Eigen::Vector3f(3.0F, 0.0F, 0.0F);
  adversarial_options.min_yaw_offset = 0.0F;
  adversarial_options.max_yaw_offset = 0.0F;
  adversarial_options.minimum_overlap = 0.5;
  const std::vector<gpu::LocalSearchTarget> adversarial_targets{
      {10, &coarse_decoy_data, Eigen::Isometry3f::Identity()},
      {20, &exact_winner_data, Eigen::Isometry3f::Identity()}};
  const sapphire::BbsResult adversarial_search =
      sapphire::alignLoopBbs(adversarial_query, adversarial_targets);
  check(adversarial_search.accepted && adversarial_search.target_id == 20 &&
            adversarial_search.overlap >= 0.7 && adversarial_search.expanded_nodes[2] > 0,
        "independent per-candidate GPU searches retain the best terminal score");

  sapphire::vvec<float, 3> window_query;
  sapphire::vvec<float, 3> window_target;
  for (int y = 0; y < 2; ++y) {
    for (int z = 0; z < 2; ++z) {
      const Eigen::Vector3f point(0.1F, 0.6F * y + 0.1F, 0.6F * z + 0.1F);
      window_query.push_back(point);
      window_target.push_back(point + Eigen::Vector3f(8.0F, 0.0F, 0.0F));
      if (y == 0) {
        window_target.push_back(point + Eigen::Vector3f(6.0F, 0.0F, 0.0F));
      }
    }
  }
  cpu::VoxelMaps window_voxelmaps;
  window_voxelmaps.set_min_res(0.5F);
  window_voxelmaps.set_max_level(3);
  window_voxelmaps.create_voxelmaps(window_target.data(), window_target.size());
  const cpu::VoxelMapsData window_data = window_voxelmaps.release_data();
  gpu::LocalSearchOptions window_options;
  window_options.translation_half_window = Eigen::Vector3f(6.0F, 0.0F, 0.0F);
  window_options.min_yaw_offset = 0.0F;
  window_options.max_yaw_offset = 0.0F;
  window_options.minimum_overlap = 0.49;
  const gpu::LocalSearchTarget window_target_data{30, &window_data, Eigen::Isometry3f::Identity()};
  const gpu::LocalSearchResult window_search = bbs.search_local(window_query, window_target_data, window_options);
  check(window_search.accepted && window_search.score.matched_points >= 2 &&
            window_search.target_T_source.translation().x() <= 6.0F,
        "leaf search stays inside the requested XYZ window when an outside pose scores higher");

  Eigen::Isometry3f negative_initial = Eigen::Isometry3f::Identity();
  negative_initial.translation().x() = -20.0F;
  sapphire::vvec<float, 3> negative_window_cloud;
  for (const Eigen::Vector3f& point : window_query) {
    negative_window_cloud.push_back(point + Eigen::Vector3f(-12.0F, 0.0F, 0.0F));
    if (point.y() < 0.5F) {
      negative_window_cloud.push_back(point + Eigen::Vector3f(-14.0F, 0.0F, 0.0F));
    }
  }
  cpu::VoxelMaps negative_window_voxelmaps;
  negative_window_voxelmaps.set_min_res(0.5F);
  negative_window_voxelmaps.set_max_level(3);
  negative_window_voxelmaps.create_voxelmaps(negative_window_cloud.data(), negative_window_cloud.size());
  const cpu::VoxelMapsData negative_window_data = negative_window_voxelmaps.release_data();
  const gpu::LocalSearchTarget negative_window_target{31, &negative_window_data, negative_initial};
  const gpu::LocalSearchResult negative_window_search = bbs.search_local(window_query, negative_window_target, window_options);
  check(negative_window_search.accepted && negative_window_search.score.matched_points >= 2 &&
            negative_window_search.target_T_source.translation().x() >= -26.0F &&
            negative_window_search.target_T_source.translation().x() <= -14.0F,
        "leaf search applies the same strict window in negative coordinates");

  const sapphire::vvec<float, 3> yaw_query = makeCloud();
  sapphire::vvec<float, 3> yaw_target;
  const Eigen::Isometry3f expected_yaw = yawPose(0.5F);
  yaw_target.reserve(yaw_query.size());
  for (const Eigen::Vector3f& point : yaw_query) {
    yaw_target.push_back(expected_yaw * point);
  }
  cpu::VoxelMaps yaw_voxelmaps;
  yaw_voxelmaps.set_min_res(0.05F);
  yaw_voxelmaps.set_max_level(1);
  yaw_voxelmaps.create_voxelmaps(yaw_target.data(), yaw_target.size());
  const cpu::VoxelMapsData yaw_data = yaw_voxelmaps.release_data();
  gpu::LocalSearchOptions yaw_options;
  yaw_options.translation_half_window = Eigen::Vector3f::Zero();
  yaw_options.min_yaw_offset = -0.75F;
  yaw_options.max_yaw_offset = 0.75F;
  yaw_options.minimum_overlap = 0.8;
  const gpu::LocalSearchTarget yaw_target_data{40, &yaw_data, Eigen::Isometry3f::Identity()};
  const gpu::LocalSearchResult yaw_search = bbs.search_local(yaw_query, yaw_target_data, yaw_options);
  const float recovered_yaw =
      std::atan2(yaw_search.target_T_source.linear()(1, 0), yaw_search.target_T_source.linear()(0, 0));
  check(yaw_search.accepted && recovered_yaw >= yaw_options.min_yaw_offset && recovered_yaw < yaw_options.max_yaw_offset &&
            std::abs(recovered_yaw - 0.5F) < 0.15F,
        "half-open yaw search recovers a nonzero in-range rotation");
  yaw_options.min_yaw_offset = 0.5F;
  yaw_options.max_yaw_offset = 0.5F;
  const gpu::LocalSearchResult zero_width_yaw_search = bbs.search_local(yaw_query, yaw_target_data, yaw_options);
  check(!zero_width_yaw_search.accepted, "zero-width yaw range evaluates only the initial yaw");

  cpu::VoxelMaps unsupported_voxelmaps;
  unsupported_voxelmaps.set_min_res(0.5F);
  unsupported_voxelmaps.set_max_level(4);
  unsupported_voxelmaps.create_voxelmaps(window_target.data(), window_target.size());
  const cpu::VoxelMapsData unsupported_data = unsupported_voxelmaps.release_data();
  bool rejected_unsupported_level = false;
  try {
    static_cast<void>(
        bbs.search_local(window_query, {50, &unsupported_data, Eigen::Isometry3f::Identity()}, window_options));
  } catch (const std::invalid_argument&) {
    rejected_unsupported_level = true;
  }
  check(rejected_unsupported_level, "BBS rejects voxel levels above the implemented Level 3 limit");

  const sapphire::vvec<float, 3> exact_target{Eigen::Vector3f(0.1F, 0.1F, 0.1F)};
  const sapphire::vvec<float, 3> neighbor_only_source{Eigen::Vector3f(-0.1F, 0.1F, 0.1F)};
  cpu::VoxelMaps exact_only_voxelmaps;
  exact_only_voxelmaps.set_min_res(0.5F);
  exact_only_voxelmaps.set_max_level(0);
  exact_only_voxelmaps.create_voxelmaps(exact_target.data(), exact_target.size());
  const cpu::VoxelMapsData exact_only_data = exact_only_voxelmaps.release_data();
  check(bbs.score_pose(neighbor_only_source, exact_only_data, Eigen::Isometry3f::Identity()).overlap == 1.0,
        "level-zero scoring keeps the original neighborhood tolerance");

  sapphire::vvec<float, 3> threshold_query;
  for (int index = 0; index < 10; ++index) {
    threshold_query.emplace_back(static_cast<float>(index), 0.0F, 0.0F);
  }
  const sapphire::vvec<float, 3> threshold_target(threshold_query.begin(), threshold_query.begin() + 3);
  cpu::VoxelMaps threshold_voxelmaps;
  threshold_voxelmaps.set_min_res(0.25F);
  threshold_voxelmaps.set_max_level(0);
  threshold_voxelmaps.create_voxelmaps(threshold_target.data(), threshold_target.size());
  const cpu::VoxelMapsData threshold_data = threshold_voxelmaps.release_data();
  gpu::LocalSearchOptions threshold_options;
  threshold_options.min_yaw_offset = 0.0F;
  threshold_options.max_yaw_offset = 0.0F;
  threshold_options.minimum_overlap = 0.3;
  const gpu::LocalSearchTarget threshold_target_data{9, &threshold_data, Eigen::Isometry3f::Identity()};
  const gpu::LocalSearchResult threshold_search =
      bbs.search_local(threshold_query, threshold_target_data, threshold_options);
  check(!threshold_search.accepted && threshold_search.score.matched_points == 3,
        "original strict threshold rejects exactly 30 percent overlap");
  const sapphire::BbsResult production_threshold_search =
      sapphire::alignLoopBbs(threshold_query, {threshold_target_data});
  check(!production_threshold_search.accepted,
        "production tolerant-overlap policy rejects candidates below 50 percent");

  sapphire::vvec<float, 3> tied_query;
  for (int y = 0; y < 2; ++y) {
    for (int z = 0; z < 2; ++z) {
      tied_query.emplace_back(0.1F, 0.6F * static_cast<float>(y) + 0.1F, 0.6F * static_cast<float>(z) + 0.1F);
    }
  }
  sapphire::vvec<float, 3> tied_target;
  tied_target.reserve(tied_query.size());
  for (const Eigen::Vector3f& point : tied_query) {
    tied_target.push_back(point + Eigen::Vector3f(1.0F, 0.0F, 0.0F));
  }
  cpu::VoxelMaps tied_voxelmaps;
  tied_voxelmaps.set_min_res(0.5F);
  tied_voxelmaps.set_max_level(2);
  tied_voxelmaps.create_voxelmaps(tied_target.data(), tied_target.size());
  const cpu::VoxelMapsData tied_data = tied_voxelmaps.release_data();
  gpu::LocalSearchOptions tied_options;
  tied_options.translation_half_window = Eigen::Vector3f(1.0F, 0.0F, 0.0F);
  tied_options.min_yaw_offset = 0.0F;
  tied_options.max_yaw_offset = 0.0F;
  tied_options.minimum_overlap = 0.99;
  const std::vector<gpu::LocalSearchTarget> tied_targets{{91, &tied_data, Eigen::Isometry3f::Identity()},
                                                        {42, &tied_data, Eigen::Isometry3f::Identity()}};
  for (int repetition = 0; repetition < 20; ++repetition) {
    const sapphire::BbsResult tied_search = sapphire::alignLoopBbs(tied_query, tied_targets);
    check(tied_search.accepted && tied_search.target_id == 42 && tied_search.overlap == 1.0,
          "equal-score candidate searches deterministically select the smaller target ID");
  }
  return 0;
}
