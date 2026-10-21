#include <Eigen/Eigenvalues>
#include <algorithm>
#include <backend/common/depth_frame.hpp>
#include <backend/maplayer/navimap/flat_ground.hpp>
#include <stdexcept>
#include <vector>

namespace sapphire::backend {
void collectGroundImagePatches(const DepthFrame& frame, const StateGroupf& worldFromCamera, float bodyZ, float tolerance,
                               std::vector<GroundImagePatch>& patches) {
  if (!frame.organized() || !worldFromCamera.isValid() || !std::isfinite(bodyZ) || !std::isfinite(tolerance) || tolerance <= 0)
    throw std::invalid_argument("Ground fitting requires organized depth, a valid pose and positive tolerance");
  constexpr std::size_t block = 8;
  const Eigen::Matrix3d rotation = worldFromCamera.rotationMatrix().cast<double>();
  const Eigen::Vector3d translation = worldFromCamera.translationVector().cast<double>();
  for (std::size_t v = 0; v < frame.height; v += block)
    for (std::size_t u = 0; u < frame.width; u += block) {
      const auto endV = std::min(v + block, frame.height), endU = std::min(u + block, frame.width);
      Eigen::Vector3d sum = Eigen::Vector3d::Zero();
      Eigen::Matrix3d square = Eigen::Matrix3d::Zero();
      std::size_t count = 0;
      for (auto y = v; y < endV; ++y)
        for (auto x = u; x < endU; ++x) {
          const auto i = y * frame.width + x;
          const Eigen::Vector3d point(frame.points.x[i], frame.points.y[i], frame.points.z[i]);
          if (!point.allFinite()) continue;
          sum += point;
          square.noalias() += point * point.transpose();
          ++count;
        }
      if (count < 4 || count * 2 < (endV - v) * (endU - u)) continue;
      const Eigen::Vector3d mean = sum / count;
      const Eigen::Vector3d world = rotation * mean + translation;
      if (world.z() >= bodyZ) continue;
      const Eigen::Matrix3d covariance = rotation * (square / count - mean * mean.transpose()) * rotation.transpose();
      // Test the entire block BEFORE applying the tracked-height gate. A wall
      // clipped to a thin height interval must not manufacture a floor patch.
      if (covariance(2, 2) > tolerance * tolerance) continue;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
      if (solver.info() != Eigen::Success || solver.eigenvalues()[1] < 1e-8 || std::abs(solver.eigenvectors()(2, 0)) < .965925826) continue;
      patches.push_back({world.cast<float>(), covariance.topLeftCorner<2, 2>().cast<float>()});
    }
}

bool FlatGroundReference::updateImage(std::vector<GroundImagePatch>& patches, float tolerance, float maxCorrection) {
  if (!std::isfinite(tolerance) || tolerance <= 0 || !std::isfinite(maxCorrection) || maxCorrection <= 0)
    throw std::invalid_argument("Invalid image-ground reference input");
  support_patches_ = 0;
  patches.erase(std::remove_if(patches.begin(), patches.end(),
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

}  // namespace sapphire::backend
