#pragma once

#include <backend/common/state_group.hpp>
#include <limits>
#include <vector>

namespace sapphire::backend {
struct DepthFrame;

struct GroundImagePatch {
  Eigen::Vector3f center;
  Eigen::Matrix2f covariance;
};
// Appends current-frame image-block statistics, without retaining depth images.
void collectGroundImagePatches(const DepthFrame& frame, const StateGroupf& worldFromCamera, float bodyZ, float tolerance,
                               std::vector<GroundImagePatch>& patches);

// CAPE plane extraction followed by the same horizontal support statistics.
void collectGroundCapePatches(const DepthFrame& frame, const StateGroupf& worldFromCamera, float bodyZ, float tolerance,
                             std::vector<GroundImagePatch>& patches);

// One horizontal floor per odometry session. Only scalar state is retained;
// current-frame observations are never appended to a historical point cloud.
class FlatGroundReference {
 public:
  bool updateImage(std::vector<GroundImagePatch>& patches, float tolerance, float maxCorrection);
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
}  // namespace sapphire::backend
