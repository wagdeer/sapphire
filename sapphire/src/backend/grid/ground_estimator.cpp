#include "backend/grid/ground_estimator.hpp"

#include <spdlog/spdlog.h>

#include <Eigen/Eigenvalues>
#include "backend/grid/flat_ground.hpp"

namespace sapphire {
class GroundEstimator::Impl {
 public:
  explicit Impl(const NaviMapParameters &g) : grid(g) {}
  std::optional<float> update(const SubmapFrame &submap) {
    if (!grid.adaptive_ground) return {};
    std::vector<mapping::GroundPatch> patches;
    mapping::collectGroundCapePatches(*submap.lio().pcd, submap.lio().T_odom_base, grid.ground_plane_tolerance, patches);
    const auto cape_patches = patches.size();
    // Unorganized laser Gaussians already carry plane statistics. Preserve
    // those statistics instead of manufacturing image neighbors for CAPE.
    if (patches.size() < 20) {
      patches.clear();
      const auto &pose = submap.lio().T_odom_base;
      for (const auto &point : *submap.lio().pcd) {
        if (!point.is_plane || !point.valid()) continue;
        const Eigen::Vector3d world = pose * point.mean.cast<double>();
        if (world.z() >= pose.translation().z()) continue;
        const Eigen::Matrix3d covariance = pose.linear() * point.covariance.cast<double>() * pose.linear().transpose();
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        if (solver.info() != Eigen::Success || std::abs(solver.eigenvectors()(2, 0)) < 0.965925826 ||
            covariance(2, 2) > grid.ground_plane_tolerance * grid.ground_plane_tolerance)
          continue;
        patches.push_back({world.cast<float>(), covariance.topLeftCorner<2, 2>().cast<float>()});
      }
    }
    const bool observed = reference.update(patches, grid.ground_plane_tolerance, grid.ground_max_correction);
    spdlog::info("[ground] submap={} cape_patches={} observed={} ready={} patches={} floor={:.3f}", submap.id(), cape_patches, observed,
                 reference.ready(), reference.supportPatches(), reference.observedZ());
    if (!observed) return {};
    return reference.observedZ();
  }
  NaviMapParameters grid;
  mapping::FlatGroundReference reference;
};
GroundEstimator::GroundEstimator(const NaviMapParameters &g) : impl_(std::make_unique<Impl>(g)) {}
GroundEstimator::~GroundEstimator() = default;
std::optional<float> GroundEstimator::update(const SubmapFrame &s) { return impl_->update(s); }
}  // namespace sapphire
