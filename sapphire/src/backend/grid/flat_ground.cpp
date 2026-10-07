#include <Eigen/Eigenvalues>
#include <algorithm>
#include "backend/grid/flat_ground.hpp"
#include <stdexcept>
#include <vector>

namespace sapphire::mapping {
bool FlatGroundReference::update(std::vector<GroundPatch>& patches, float tolerance, float maxCorrection) {
  if (!std::isfinite(tolerance) || tolerance <= 0 || !std::isfinite(maxCorrection) || maxCorrection <= 0)
    throw std::invalid_argument("Invalid image-ground reference input");
  support_patches_ = 0;
  patches.erase(
      std::remove_if(patches.begin(), patches.end(),
                     [&](const auto& p) { return !p.center.allFinite() || (ready() && std::abs(p.center.z() - observed_z_) > maxCorrection); }),
      patches.end());
  constexpr std::size_t minimumSupport = 20;
  if (patches.size() < minimumSupport) return false;
  std::sort(patches.begin(), patches.end(), [](const auto& a, const auto& b) { return a.center.z() < b.center.z(); });
  std::size_t first = 0, bestFirst = 0, bestEnd = 0;
  for (std::size_t end = 0; end < patches.size(); ++end) {
    while (patches[end].center.z() - patches[first].center.z() > 2 * tolerance) ++first;
    if (end + 1 - first > bestEnd - bestFirst) {
      bestFirst = first;
      bestEnd = end + 1;
    }
  }
  const float floor = patches[(bestFirst + bestEnd) / 2].center.z();
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  std::size_t count = 0;
  for (const auto& p : patches)
    if (std::abs(p.center.z() - floor) <= tolerance) {
      mean += p.center.head<2>().cast<double>();
      ++count;
    }
  if (count < minimumSupport) return false;
  mean /= count;
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const auto& p : patches)
    if (std::abs(p.center.z() - floor) <= tolerance) {
      const Eigen::Vector2d delta = p.center.head<2>().cast<double>() - mean;
      covariance += p.covariance.cast<double>() + delta * delta.transpose();
    }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance / count);
  if (solver.info() != Eigen::Success || solver.eigenvalues()[0] < .01) return false;
  if (!ready()) reference_z_ = floor;
  observed_z_ = floor;
  support_patches_ = count;
  return true;
}

}  // namespace sapphire::mapping
