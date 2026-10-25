#pragma once
#include <optional>
#include <string>

#include "common/se3_state.hpp"
#include <parameters.hpp>
#include "common/common_type.hpp"

namespace sapphire::lidar {
struct Alignment {
  SE3f T_target_query;
  std::optional<Mat66_t<double>> covariance;  // query-local right tangent [translation, rotation]
  std::size_t inliers = 0;
  double fitness = 0;
  std::string failure;
};
Alignment align(const GaussianCloudPtr& target, const GaussianCloudPtr& query, const SE3f& T_target_query, const RegistrationParams& config);
}  // namespace sapphire::lidar
