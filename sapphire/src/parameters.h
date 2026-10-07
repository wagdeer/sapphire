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
  double max_scan_duration = 0.11;
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

// Experimental shared-map observation, constructed only after geometric rejection.
struct GicpFallbackParameters {
  bool enabled = false;
  double source_resolution = 0.25;
  double max_distance = 0.5;
  double variance_floor = 0.0005;
  int max_iterations = 20;  // small_gicp LM default budget
};

struct OdometryParameters {
  std::string frontend = "eskf";
  GicpFallbackParameters gicp_fallback;
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
  bool adaptive_ground = true;
  double ground_plane_tolerance = 0.05;
  double ground_max_correction = 0.3;
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

// Optical camera coordinates: +x right, +y down, +z forward.
struct CameraParameters {
  int width = 0, height = 0;
  int input_width = 0, input_height = 0;  // zero = same as calibrated processing dimensions
  std::vector<double> intrinsics;         // fx, fy, cx, cy
  std::string distortion_model = "radtan";  // radtan or equidistant
  std::vector<double> distortion;         // radtan: empty or k1,k2,p1,p2,k3; equidistant: k1..k4
  std::vector<double> camera_to_imu_rotation;
  std::vector<double> camera_to_imu_translation;
  double time_offset = 0.0;
};

bool valid_camera_distortion(const CameraParameters &camera);

struct VisualLoopParameters {
  bool enabled = false;
  bool attributes_only = false; // selected images only; no feature archive or visual retrieval
  bool loopEnabled() const noexcept { return enabled && !attributes_only; }
  std::string mode = "mono";  // one or two image streams; never selects a depth source
  CameraParameters left, right;
  double global_xy_window = 20.0;
  double global_z_window = 4.0;
  double image_interval = 0.5;
  bool stereo_depth = false; // paired-camera sparse depth, never temporal triangulation
  bool keyframe_selection = false; // flow + read-only LIO gate before stereo/descriptors
  double keyframe_min_interval = 1.0;
  double keyframe_translation_m = .5;
  double keyframe_rotation_deg = 60;
  double keyframe_renewal = .4;
  double keyframe_min_coverage = .5;
  bool tracking_enabled = false;  // single-pool flow and visual submap triggering
  double tracking_interval = 0.05;  // distinct from legacy descriptor sampling
  int max_features = 1200;
  int max_frames = 8;  // per camera, per submap
  int top_k = 5;
  int min_matches = 20;
  const CameraParameters &camera(std::size_t id) const { return id == 0 ? left : right; }
};

struct StorageParameters {
  int cloud_cache_mb = 128;
  int grid_cache_mb = 32;
  int scene_cache_mb = 32;
  int scene_cache_entries = 16;
};

struct PoseGraphParameters {
  std::string map_mode = "new";
  bool automatic_attachment = false; // bounded visual-assisted resume; initial root must overlap history
  bool scene_refresh = false; // explicit c4/schema3 retained-scene format; no implicit migration
  int association_pending_submaps = 8;
  int association_pending_mb = 128;
  std::string database_path;     // Empty in new mode selects the normal timestamp destination.
  StorageParameters storage;
  VisualLoopParameters visual;
  bool enabled = true;
  double submap_voxel_size = 0.25;
  double submap_voxel_size_inv = 4.0;
  double submap_travel_distance = 15.0;
  double submap_max_point_range = 20.0;
  double update_period_sec = 1.0;
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

// Stable compatibility identity, not a serialization of frontend/runtime state.
std::string map_config_identity(const PoseGraphParameters &graph, const NaviMapParameters &grid);

}  // namespace sapphire
