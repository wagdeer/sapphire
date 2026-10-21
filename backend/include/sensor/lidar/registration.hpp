#pragma once
#include <optional>
#include <string>

#include "backend/common/state_group.hpp"
#include "backend/params/registration.hpp"
#include "backend/sensor/lidar/geometry.hpp"

namespace sapphire::backend::lidar {
struct Alignment {
  StateGroupf T_target_query;
  std::optional<Mat66_t<double>> covariance;  // query-local right tangent [translation, rotation]
  std::size_t inliers = 0;
  double fitness = 0;
  std::string failure;
};
Alignment align(const GaussianCloudPtr& target, const GaussianCloudPtr& query, const StateGroupf& T_target_query, const RegistrationParams& config);
}  // namespace sapphire::backend::lidar
