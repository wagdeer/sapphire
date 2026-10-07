#pragma once

#include <cstddef>
#include <optional>
#include "common/common.hpp"

namespace sapphire {

struct RevisitCoverageOptions {
  double mean_distance = .20; // metres, in target's original anchor
  double covariance_difference = .35; // Frobenius difference / larger covariance norm
  double support_cell_size = 1.; // metres; diagnostic spatial distribution only
  std::size_t max_points = 32768; // per cloud
  std::size_t max_comparisons = 200000;
};

enum class RevisitCoverageStatus { Complete, Empty, InputBudget, ComparisonBudget };
struct RevisitSupport {
  std::size_t pairs = 0;
  std::size_t query_cells = 0, target_cells = 0;
  std::size_t supported_query_cells = 0, supported_target_cells = 0;
  double query_fraction = 0, target_fraction = 0;
};
struct RevisitCoverage {
  RevisitCoverageStatus status = RevisitCoverageStatus::Empty;
  std::size_t query_points = 0, target_points = 0, comparisons = 0;
  // Absent on incomplete work. Never use a partial scan as coverage evidence.
  std::optional<RevisitSupport> support;
};

// Read-only evidence after independent registration, not a loop/deletion decision.
// T_target_query maps original query coordinates to the original target anchor.
// Means and covariances are transformed together; voxel keys are not comparable
// across anchors. Unique distance-ordered pairing is conservative under resampling.
// Invalid numerical/model inputs throw; bounded exhaustion is an explicit result.
RevisitCoverage measureRevisitCoverage(const GaussianCloud &query, const GaussianCloud &target,
                                      const Eigen::Isometry3d &T_target_query,
                                      const RevisitCoverageOptions &options = {});

} // namespace sapphire
