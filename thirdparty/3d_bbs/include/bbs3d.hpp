#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <voxelmaps.hpp>

namespace gpu {

void initialize_device();

struct PoseScore {
  bool available = false;
  std::size_t matched_points = 0;
  std::size_t evaluated_points = 0;
  double overlap = 0.0;
};

struct LocalSearchTarget {
  std::uint64_t target_id = 0;
  const cpu::VoxelMapsData* voxelmaps = nullptr;
  Eigen::Isometry3f initial_target_T_source = Eigen::Isometry3f::Identity();
};

struct LocalSearchOptions {
  Eigen::Vector3f translation_half_window = Eigen::Vector3f::Zero();
  float min_yaw_offset = -3.14159265358979323846F;
  float max_yaw_offset = 3.14159265358979323846F;
  double minimum_overlap = 0.0;
};

enum class SearchTermination : std::uint8_t {
  kNoCandidates = 0,
  kQueueExhausted,
};

const char* search_termination_name(SearchTermination termination) noexcept;

struct LocalSearchResult {
  bool accepted = false;
  std::uint64_t target_id = std::numeric_limits<std::uint64_t>::max();
  double elapsed_ms = 0.0;
  double initial_overlap = 0.0;
  Eigen::Isometry3f target_T_source = Eigen::Isometry3f::Identity();
  PoseScore score;
  SearchTermination termination = SearchTermination::kNoCandidates;
  std::size_t root_nodes = 0;
  std::array<std::size_t, 4> expanded_nodes{};
  std::array<std::size_t, 4> pruned_nodes{};
};

class BBS3D {
 public:
  PoseScore score_pose(const Eigen::Vector3f* source_points, std::size_t point_count,
                       const cpu::VoxelMapsData& target,
                       const Eigen::Isometry3f& target_T_source) const;

  template <typename Allocator>
  PoseScore score_pose(const std::vector<Eigen::Vector3f, Allocator>& source_points,
                       const cpu::VoxelMapsData& target,
                       const Eigen::Isometry3f& target_T_source) const {
    return score_pose(source_points.data(), source_points.size(), target, target_T_source);
  }

  LocalSearchResult search_local(const Eigen::Vector3f* source_points, std::size_t point_count,
                                 const LocalSearchTarget& target,
                                 const LocalSearchOptions& options) const;

  template <typename Allocator>
  LocalSearchResult search_local(const std::vector<Eigen::Vector3f, Allocator>& source_points,
                                 const LocalSearchTarget& target,
                                 const LocalSearchOptions& options) const {
    return search_local(source_points.data(), source_points.size(), target, options);
  }
};

}  // namespace gpu
