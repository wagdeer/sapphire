#include "backend/sensor/lidar/ground.hpp"

#include <Eigen/Eigenvalues>
namespace sapphire::backend::lidar {
void ground_patches(const GaussianCloud& cloud, const StateGroupf& pose, float tolerance, std::vector<GroundImagePatch>& patches) {
  if (!pose.isValid() || !std::isfinite(tolerance) || tolerance <= 0) throw std::invalid_argument("Invalid LiDAR floor input");
  const Eigen::Matrix3d rotation = pose.rotationMatrix().cast<double>();
  const Eigen::Vector3d translation = pose.translationVector().cast<double>();
  for (const auto& p : cloud) {
    if (!p.is_plane || p.N < 4 || !p.valid()) continue;
    const Eigen::Vector3d world = rotation * p.mean.cast<double>() + translation;
    const Eigen::Matrix3d covariance = rotation * p.covariance.cast<double>() * rotation.transpose();
    if (world.z() >= pose.z() || covariance(2, 2) > tolerance * tolerance) continue;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance);
    if (eig.info() != Eigen::Success || eig.eigenvalues()[1] < 1e-8 || std::abs(eig.eigenvectors()(2, 0)) < .965925826) continue;
    patches.push_back({world.cast<float>(), covariance.topLeftCorner<2, 2>().cast<float>()});
  }
}
}  // namespace sapphire::backend::lidar
