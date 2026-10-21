#pragma once
#include <map>

#include <array>
#include <filesystem>
#include <string>
#include <stdexcept>
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
  // Retire voxels after this traveled distance since their last active window.
  double map_retention_distance = 700.0;
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
  int thread_num = 1;
};

// Optical camera coordinates: +x right, +y down, +z forward.
struct CameraParameters {
  int width = 0, height = 0; // calibration grid; normalized to image_algo_size at setup
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
  std::string mode = "mono";  // primary mono or stereo group; independent of depth source
  CameraParameters left, right;
  std::vector<CameraParameters> auxiliary; // stereo-only independent mono cameras, IDs 2/3
  double global_xy_window = 20.0;
  double global_z_window = 4.0;
  std::array<int, 2> image_algo_size{0, 0}; // width,height; omitted config resolves to left calibration size
  double image_interval = 0.5;
  int max_features = 1200;
  int max_frames = 8;  // per camera, per submap
  int top_k = 5;
  int min_matches = 20;
  std::size_t cameraCount() const { return (mode == "stereo" ? 2U : 1U) + auxiliary.size(); }
  const CameraParameters &camera(std::size_t id) const {
    if (id >= cameraCount()) throw std::out_of_range("Unknown visual camera");
    if (id == 0) return left;
    if (id == 1) return right;
    return auxiliary.at(id - 2);
  }
};

struct StorageParameters {
  int cloud_cache_mb = 128;
  int scene_cache_mb = 32;
  int scene_cache_entries = 16;
};

struct PoseGraphParameters {
  std::map<std::string,std::string> backend_parameters; // Native Sapphire keys under [backend.*].
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
};

SapphireParameters load_parameters(const std::filesystem::path &path);
void validate_parameters(SapphireParameters &parameters);

// Stable compatibility identity, not a serialization of frontend/runtime state.
std::string map_config_identity(const PoseGraphParameters &graph);

}  // namespace sapphire
