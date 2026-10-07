#pragma once

#include <Eigen/Geometry>
#include <limits>
#include <vector>

#include "common/common.hpp"

namespace sapphire::mapping {
struct GroundPatch {
  Eigen::Vector3f center;
  Eigen::Matrix2f covariance;
};
void collectGroundCapePatches(const GaussianCloud &points, const Eigen::Isometry3d &world_from_body, float tolerance,
                              std::vector<GroundPatch> &patches);

// One horizontal floor per odometry session. Only scalar state is retained;
// current-frame observations are never appended to a historical point cloud.
class FlatGroundReference {
 public:
  bool update(std::vector<GroundPatch> &patches, float tolerance, float maxCorrection);
  void reset() { *this = {}; }
  bool ready() const { return std::isfinite(reference_z_); }
  float referenceZ() const { return reference_z_; }
  float observedZ() const { return observed_z_; }
  float correctionZ() const { return ready() ? reference_z_ - observed_z_ : 0.f; }
  float height(float worldZ) const { return worldZ + correctionZ() - reference_z_; }
  std::size_t supportPatches() const { return support_patches_; }

 private:
  float reference_z_ = std::numeric_limits<float>::quiet_NaN();
  float observed_z_ = std::numeric_limits<float>::quiet_NaN();
  std::size_t support_patches_ = 0;
};
}  // namespace sapphire::mapping
