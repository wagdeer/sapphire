#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace sapphire {

struct GeneralParameters {
  std::string save_path;
  int save_map = 0;
};

struct SensorParameters {
  int lidar_type = 0;
  int point_filter_num = 3;
  double blind = 0.1;
  double blind_squared = 0.01;
  std::vector<double> extrinsic_tran{0.0, 0.0, 0.0};
  std::vector<double> extrinsic_rota{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
};

struct InitializerParameters {
  int imu_init_samples = 30;
  double down_size = 0.5;
  double down_size_inv = 2.0;
  int max_iterations = 10;
  int min_plane_factors = 10;
  int ba_iterations = 3;
  double convergence_threshold = 0.05;
  double refined_convergence_threshold = 0.01;
  double degeneracy_threshold = 15.0;
  double min_eigen_value = 0.02;
  double plane_eigen_value_thre = 4.0;
  double plane_eigen_value_thre_inv = 0.25;
};

struct OdometryParameters {
  double cov_gyr = 0.1;
  double cov_acc = 0.1;
  double rdw_gyr = 1e-4;
  double rdw_acc = 1e-4;
  double down_size = 0.1;
  double down_size_inv = 10.0;
  double dept_err = 0.02;
  double beam_err = 0.05;
  double voxel_size = 1.0;
  double voxel_size_inv = 1.0;
  double min_eigen_value = 0.0025;
  int degrade_bound = 10;
};

struct LocalSubmapParameters {
  int win_size = 10;
  int max_layer = 2;
  double cov_gyr = 0.1;
  double cov_acc = 0.1;
  double rdw_gyr = 1e-4;
  double rdw_acc = 1e-4;
  std::vector<double> plane_eigen_value_thre{1.0, 1.0, 1.0, 1.0};
  std::vector<double> plane_eigen_value_thre_inv{1.0, 1.0, 1.0, 1.0};
  double imu_coef = 1e-4;
  int thread_num = 5;
};

struct NaviMapParameters {
  bool enabled = true;
  double resolution = 0.1;
  double h_clearance = 2.0;
  double ground_margin = 0.5;
  double clear_height_eps = 0.1;
  double d_max = 6.0;
  double occ_threshold = 0.3;
  double usable_range = 40.0;
  double min_range = 0.5;
  double margin = 5.0;
  double hit_probability = 0.7;
  double miss_probability = 0.4;
  double clamp_min_probability = 0.1192;
  double clamp_max_probability = 0.971;
};

struct PoseGraphParameters {
  bool enabled = true;
  double loop_min_time_separation = 30.0;
  double loop_min_travel_distance = 30.0;
  double loop_max_rotation = 3.14;
  double loop_search_radius = 15.0;
  int loop_search_stride = 1;
  int target_frame_count = 50;
  double keyframe_voxel_size = 0.25;
  double keyframe_voxel_size_inv = 4.0;
  double fitness_threshold = 0.5;
  double update_period_sec = 1.0;
  int map_frame_stride = 3;
  double map_voxel_size = 0.8;
  int memory_stm_size = 30;
  int memory_wm_size = 200;
  int gicp_max_iterations = 32;
  int gicp_k_correspondences = 16;
  int gicp_min_inliers = 64;
  int gicp_num_threads = 2;
  double gicp_max_correspondence_distance = 2.0;
  double gicp_transformation_epsilon = 0.01;
  double gicp_rotation_epsilon = 0.01;
};

struct SapphireParameters {
  GeneralParameters general;
  SensorParameters sensor;
  InitializerParameters initializer;
  OdometryParameters odometry;
  LocalSubmapParameters local_submap;
  PoseGraphParameters pose_graph;
  NaviMapParameters navi_map;
};

SapphireParameters load_parameters(const std::filesystem::path &path);
void validate_parameters(SapphireParameters &parameters);

}  // namespace sapphire

// Internal algorithm headers are still global in this first-stage refactor.
using GeneralParameters = sapphire::GeneralParameters;
using SensorParameters = sapphire::SensorParameters;
using InitializerParameters = sapphire::InitializerParameters;
using OdometryParameters = sapphire::OdometryParameters;
using LocalSubmapParameters = sapphire::LocalSubmapParameters;
using PoseGraphParameters = sapphire::PoseGraphParameters;
using NaviMapParameters = sapphire::NaviMapParameters;