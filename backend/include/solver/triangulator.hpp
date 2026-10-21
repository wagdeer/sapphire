#pragma once
#include <backend/common/common_type.hpp>
#include <Eigen/Core>

#include "backend/common/state_group.hpp"

namespace sapphire::backend::camera {
struct StereoObservation;
struct StereoPixel;
}
namespace sapphire::backend::solver {
class Triangulator {
 public:
  static bool triangulateRays(const Vec3_t<double>& bearing_1, const Vec3_t<double>& bearing_2, const Mat33_t<double>& rotation_21,
                              const Vec3_t<double>& translation_21, Vec3_t<double>& point_1);

  // Seed admission permits weak disparity; depth support requires resolved parallax.
  static bool stereo_seed_valid(const camera::StereoObservation& observation);

  static bool stereo_depth_support(const camera::StereoObservation& observation);
  static bool stereo_seed_valid(const std::array<float, 5>& calibration, const camera::StereoPixel& pixel);
  static bool stereo_depth_support(const std::array<float, 5>& calibration, const camera::StereoPixel& pixel);

  static bool temporal_depth_support(const camera::StereoObservation& a, const StateGroupf& frame_from_a, const camera::StereoObservation& b,
                                     const StateGroupf& frame_from_b);


};
}  // namespace sapphire::backend::solver
