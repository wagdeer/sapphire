#include "backend/registration/revisit_coverage.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace sapphire {
namespace {
using Cell = std::array<std::int64_t, 3>;
struct CellHash {
  std::size_t operator()(const Cell &c) const noexcept {
    std::uint64_t h = 0;
    for (auto v : c) h ^= std::uint64_t(v) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    return std::size_t(h);
  }
};
Cell cellAt(const Eigen::Vector3d &p, double size) {
  Cell cell;
  for (int i = 0; i < 3; ++i) {
    const double coordinate = std::floor(p[i] / size);
    // Leave ample room for neighbor offsets and exact integer representation.
    if (!std::isfinite(coordinate) || std::abs(coordinate) > 0x1p50)
      throw std::invalid_argument("Revisit coverage coordinate exceeds safe cell domain");
    cell[i] = static_cast<std::int64_t>(coordinate);
  }
  return cell;
}
struct Sample {
  Eigen::Vector3d mean;
  Eigen::Matrix3d covariance;
};
std::vector<Sample> prepare(const GaussianCloud &cloud, const Eigen::Isometry3d &transform) {
  std::vector<Sample> samples;
  samples.reserve(cloud.size());
  for (const auto &point : cloud) {
    const Eigen::Matrix3d covariance = point.covariance.cast<double>();
    if (!point.valid() || !covariance.allFinite() ||
        (covariance - covariance.transpose()).norm() > 1e-6 * covariance.norm() ||
        Eigen::LLT<Eigen::Matrix3d>(covariance).info() != Eigen::Success)
      throw std::invalid_argument("Invalid Gaussian mean/covariance for revisit coverage");
    Sample sample{transform * point.mean.cast<double>(), transform.linear() * covariance * transform.linear().transpose()};
    if (!sample.mean.allFinite() || !sample.covariance.allFinite())
      throw std::invalid_argument("Nonfinite transformed revisit Gaussian");
    samples.push_back(std::move(sample));
  }
  return samples;
}
} // namespace

RevisitCoverage measureRevisitCoverage(const GaussianCloud &query, const GaussianCloud &target,
                                      const Eigen::Isometry3d &T, const RevisitCoverageOptions &options) {
  if (!std::isfinite(options.mean_distance) || options.mean_distance < 1e-6 || options.mean_distance > 100 ||
      !std::isfinite(options.covariance_difference) || options.covariance_difference < 0 || options.covariance_difference > 2 ||
      !std::isfinite(options.support_cell_size) || options.support_cell_size < 1e-6 || options.support_cell_size > 1000 ||
      !options.max_points || options.max_points > 65536 || !options.max_comparisons || options.max_comparisons > 2000000)
    throw std::invalid_argument("Invalid revisit coverage bounds/tolerances");
  if (!T.matrix().allFinite() || (T.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() > 1e-9 ||
      (T.linear().transpose() * T.linear() - Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
      std::abs(T.linear().determinant() - 1.) > 1e-6)
    throw std::invalid_argument("Revisit coverage requires a finite rigid transform");
  RevisitCoverage result;
  result.query_points = query.size(); result.target_points = target.size();
  if (query.size() > options.max_points || target.size() > options.max_points) {
    result.status = RevisitCoverageStatus::InputBudget; return result;
  }
  if (query.empty() || target.empty()) return result;
  const auto q = prepare(query, T), t = prepare(target, Eigen::Isometry3d::Identity());
  std::unordered_map<Cell, std::vector<std::size_t>, CellHash> cells;
  cells.reserve(t.size());
  std::unordered_set<Cell, CellHash> query_cells, target_cells, supported_query, supported_target;
  for (std::size_t i = 0; i < t.size(); ++i) {
    cells[cellAt(t[i].mean, options.mean_distance)].push_back(i);
    target_cells.insert(cellAt(t[i].mean, options.support_cell_size));
  }
  struct Pair { double distance; std::size_t query, target; };
  std::vector<Pair> pairs;
  pairs.reserve(std::min(options.max_comparisons, q.size() * 4));
  const double max_distance_squared = options.mean_distance * options.mean_distance;
  for (std::size_t i = 0; i < q.size(); ++i) {
    query_cells.insert(cellAt(q[i].mean, options.support_cell_size));
    const auto center = cellAt(q[i].mean, options.mean_distance);
    for (int x = -1; x <= 1; ++x) for (int y = -1; y <= 1; ++y) for (int z = -1; z <= 1; ++z) {
      const auto found = cells.find({center[0] + x, center[1] + y, center[2] + z});
      if (found == cells.end()) continue;
      for (auto j : found->second) {
        if (result.comparisons == options.max_comparisons) {
          result.status = RevisitCoverageStatus::ComparisonBudget; return result;
        }
        ++result.comparisons;
        const double distance = (q[i].mean - t[j].mean).squaredNorm();
        if (distance > max_distance_squared) continue;
        const double scale = std::max(q[i].covariance.norm(), t[j].covariance.norm());
        if ((q[i].covariance - t[j].covariance).norm() > options.covariance_difference * scale) continue;
        pairs.push_back({distance, i, j});
      }
    }
  }
  std::sort(pairs.begin(), pairs.end(), [](const Pair &a, const Pair &b) {
    return std::tie(a.distance, a.query, a.target) < std::tie(b.distance, b.query, b.target);
  });
  std::vector<bool> used_query(q.size(), false), used_target(t.size(), false);
  RevisitSupport support;
  for (const auto &pair : pairs) {
    if (used_query[pair.query] || used_target[pair.target]) continue;
    used_query[pair.query] = used_target[pair.target] = true;
    ++support.pairs;
    supported_query.insert(cellAt(q[pair.query].mean, options.support_cell_size));
    supported_target.insert(cellAt(t[pair.target].mean, options.support_cell_size));
  }
  support.query_fraction = double(support.pairs) / q.size();
  support.target_fraction = double(support.pairs) / t.size();
  support.query_cells = query_cells.size(); support.target_cells = target_cells.size();
  support.supported_query_cells = supported_query.size(); support.supported_target_cells = supported_target.size();
  result.status = RevisitCoverageStatus::Complete; result.support = support;
  return result;
}
} // namespace sapphire
