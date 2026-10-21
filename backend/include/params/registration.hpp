#pragma once

#include <backend/common/common_type.hpp>

namespace sapphire::backend {

struct RegistrationParams {
  float lidar_odometry_translation_sigma_m = 0.01;
  float lidar_odometry_rotation_sigma_rad = 0.001;
  bool lidar_bbs_enabled = true;
  int lidar_min_inliers = 64;
  int lidar_max_iterations = 32;
  float lidar_max_correspondence_m = 2;
  float lidar_min_overlap = 0.5;
  float lidar_max_fitness_m2 = 0.5;
  float lidar_translation_sigma_m = 0.2;
  float lidar_rotation_sigma_rad = 0.1;
  float matching_distance_ratio = 0.75;
  int matching_max_comparisons = 50000;
  int matching_max_hamming_distance = 50;
  float max_reprojection_error_px = 2;
  float min_inlier_distribution = 0.0;
  int min_inliers = 20;
  int ransac_iterations = 300;
  int refine_iterations = 3;
  bool stereo_refinement_enabled = false;

  RegistrationParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
};

}  // namespace sapphire::backend
