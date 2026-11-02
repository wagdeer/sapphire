#pragma once

#include <common/common_type.hpp>
#include <Eigen/Geometry>
#include <cstdlib>
#include <array>
#include <filesystem>
#include <map>
#include <stdexcept>
#include <vector>
#include <optional>
#include <string>
#include <string_view>

namespace sapphire {

struct ArchiveParams {
  bool keep_unlinked_nodes = true;

  ArchiveParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
};

struct DatabaseParams {
  unsigned int sqlite_cache_size_pages = 10000;
  bool sqlite_in_memory = false;
  int sqlite_journal_mode = 2;
  int sqlite_synchronous_mode = 2;
  int sqlite_temp_store = 2;
  unsigned int writer_batch_max_bytes = 33554432;
  unsigned int writer_batch_max_items = 128;
  unsigned int writer_batch_max_latency_ms = 50;
  unsigned int writer_commit_timeout_ms = 2000;
  unsigned int writer_max_bytes = 268435456;
  unsigned int writer_max_items = 1024;
  unsigned int writer_producer_wait_ms = 0;

  DatabaseParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
};

struct MemoryParams {
  bool eviction_order_by_id = false;
  bool load_all_nodes_on_start = false;
  float protected_local_ratio = 0.25;
  float protected_recent_ratio = 0.2;
  unsigned int recent_nodes = 10;
  unsigned int resident_budget_bytes = 0;
  int resident_budget_nodes = 0;
  float update_budget_ms = 0;

  MemoryParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
};

struct NodeParams {
  bool generate_node_ids = true;
  bool ignore_bad_frames = false;
  NodeParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
};

struct PGOParams {
  bool anchor_at_last_node = false;
  bool gnc_enabled = false;
  float gravity_sigma = 0.3;
  bool ignore_constraint_variance = false;
  bool ignore_priors = true;
  bool incremental_enabled = true;
  bool isam2_adaptive_reorder_enabled = true;
  double isam2_relinearize_rotation_rad = 0.01;
  int isam2_relinearize_skip = 1;
  double isam2_relinearize_translation_m = 0.05;
  double isam2_reorder_growth_ratio = 2.0;
  double isam2_wildfire_threshold = 0.001;
  float max_constraint_error_sigma = 3.0;
  int max_iterations = 20;
  double min_error_improvement = 0.0;
  int optimizer = 1;
  float repair_radius_m = 0.0;

  PGOParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
};

struct RegistrationParams {
  float lidar_odometry_translation_sigma_m = 0.01;
  float lidar_odometry_rotation_sigma_rad = 0.001;
  bool lidar_bbs_enabled = false;
  int lidar_min_inliers = 64;
  int lidar_max_iterations = 32;
  float lidar_max_correspondence_m = 2;
  float lidar_min_overlap = 0.5;
  float lidar_max_fitness_m2 = 0.5;
  float lidar_translation_sigma_m = 0.2;
  float lidar_rotation_sigma_rad = 0.1;
  RegistrationParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
};

struct RelocalizerParams {
  bool global_enabled = false;
  float global_resolution_m = .5f;
  float global_feature_radius_m = 2.5f;
  float global_clique_time_sec = .5f;
  unsigned int global_max_associations = 3000;
  unsigned int global_index_max_nodes = 16384;
  bool limit_covariance_by_odometry = false;
  float max_loop_distance_m = 0.0;
  bool spatial_enabled = true;
  unsigned int spatial_max_candidates = 5;
  float spatial_radius_m = 1;
  unsigned int spatial_max_graph_hops = 49;
  unsigned int path_max_nodes = 3;
  unsigned int path_max_bytes = 64U * 1024U * 1024U;

  RelocalizerParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
};

struct StatisticsParams {
  bool buffer_logs = true;
  bool log_enabled = false;
  bool log_headers = true;
  bool save_resident_nodes = false;

  StatisticsParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
};

struct TopographParams {
  float new_session_odometry_jump_m = 0;
  bool start_session_on_good_frame = false;
  bool use_previous_map = true;

  bool automatic_map_labels = true;
  bool ignore_odometry_cross_covariance = true;
  // Establish component datums when building a map; loading never invents missing datums.
  bool persistent_root_frames = false;
  bool reduce_graph = false;
  bool use_odometry_gravity = false;

  TopographParams withOverrides(const ParametersMap& input) const;
  ParametersMap parameters() const;
  void validate() const;
};

// Typed mapping configuration. String maps are only configuration/record formats.
struct MappingParameters {
  ArchiveParams archive;
  DatabaseParams database;
  MemoryParams memory;
  NodeParams node;
  PGOParams pose_graph;
  RegistrationParams registration;
  RelocalizerParams relocalizer;
  StatisticsParams statistics;
  TopographParams topograph;
  static MappingParameters from_record(const ParametersMap&);
  MappingParameters withOverrides(const ParametersMap&) const;
  ParametersMap parameters() const;
  void validate() const;
};

class Parameters {
 public:
  enum class scalar_type_t { Bool, Int, UnsignedInt, Float, Double, String };

  static const ParametersMap& getDefaultParameters();

  // Unknown names have no type; startup configuration rejects them.
  static std::optional<scalar_type_t> getType(const std::string& paramKey);

  static std::string getDescription(const std::string& paramKey);

  // Validate current catalog names and scalar encoding; module constraints belong to *Params::validate().
  static void validate(const ParametersMap& parameters);

  // Supported scalar types: bool, int, unsigned int, float, double and string.
  template <typename T>
  static bool parse(const ParametersMap& parameters, std::string_view key, T& value);
  static void parse(const ParametersMap& parameters, ParametersMap& parametersOut);

  // Historical records are provenance, not startup configuration. Validate only
  // the name:value;... storage grammar, preserving names and values verbatim.
  static void validateRecord(const ParametersMap& parameters);
  static std::string serialize(const ParametersMap& parameters);
  static ParametersMap deserialize(const std::string& parameters);

 private:
  Parameters() = delete;
};


struct GeneralParameters {
  std::string save_path;
};

struct SensorParameters {
  std::string topic;
  double time_offset = 0.0;  // corrected time = device time + offset, seconds
  int lidar_type = 0;
  double max_scan_duration = 0.11;
  int point_filter_num = 3;
  double blind = 0.1;
  double blind_squared = 0.01;
  std::vector<double> extrinsic_tran{0.0, 0.0, 0.0};
  std::vector<double> extrinsic_rota{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
};

// One installation, independently loaded from algorithm tuning. All extrinsics
// map LiDAR coordinates into the selected IMU; lidar0 is the reference cloud.
struct SensorConfig {
  std::vector<SensorParameters> lidars;
  std::string imu_topic;
  double imu_rate_hz = 200.0;
  double lidar_merge_hz = 10.0;
  double max_time_gap = 0.02;
  double max_wait = 0.3;
  int min_points = 1;
  int queue_size = 8;
};

struct LidarSyncParameters {
  Eigen::Isometry3d extrinsic = Eigen::Isometry3d::Identity();  // rear LiDAR -> front LiDAR
  double front_time_offset = 0.0;  // corrected time = device timestamp + offset, seconds
  double rear_time_offset = 0.0;
  double max_scan_duration = 0.25;
  double max_time_gap = 0.02;
  double max_wait = 0.3;
  double lidar_merge_hz = 10.0;  // 0: legacy front-frame windows; >0: symmetric fixed windows
  double imu_rate_hz = 200.0;    // Expected INPUT rate; does not configure the IMU hardware.
  size_t min_points = 1;  // Valid points per LiDAR per window.
  size_t queue_size = 8;
  int point_filter_num = 1;
  double blind_squared = 0.01;
};

SensorConfig load_sensor_parameters(const std::filesystem::path &path);

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
  // Disposable fixed-history cache. Active BA observations are protected
  // separately and are not covered by this byte budget.
  std::size_t fixed_map_budget_bytes = 64U * 1024 * 1024;
  double min_eigen_value = 0.0025;
  int degrade_bound = 10;
};

struct LocalSubmapParameters {
  int win_size = 10;  // Joint LiDAR/IMU window during initialization and tracking.
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

struct PoseGraphParameters {
  MappingParameters mapping = [] {
    MappingParameters p;
    p.memory.recent_nodes = 3;
    p.memory.resident_budget_bytes = 128 * 1024 * 1024;
    p.relocalizer.spatial_radius_m = 30;
    p.relocalizer.spatial_max_graph_hops = 0;
    p.topograph.persistent_root_frames = true;
    return p;
  }();
  std::string map_mode = "new";
  std::string database_path;  // Empty in new mode selects the normal timestamp destination.
  bool enabled = true;
  double submap_travel_distance = 15.0;
  double update_period_sec = 1.0;
};

struct RecoveryParameters {
  // Wall-clock silence budget. Zero is explicit offline/externally-clocked mode.
  double input_timeout_sec = 2.0;
  // Sensor-clock continuity budgets, independent of playback speed.
  double max_imu_gap_sec = 0.05;
  double max_lidar_gap_sec = 0.5;
};

struct SapphireParameters {
  GeneralParameters general;
  SensorParameters sensor;
  InitializerParameters initializer;
  OdometryParameters odometry;
  LocalSubmapParameters local_submap;
  PoseGraphParameters pose_graph;
  RecoveryParameters recovery;
};

SapphireParameters load_parameters(const std::filesystem::path &path, const SensorParameters &sensor);
void validate_parameters(SapphireParameters &parameters);

// Stable compatibility identity, not a serialization of frontend/runtime state.
std::string map_config_identity(const PoseGraphParameters &graph);

}  // namespace sapphire
