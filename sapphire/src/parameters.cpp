#include "parameters.h"

#include <Eigen/Core>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <toml++/toml.hpp>
#include <vector>

namespace sapphire {
namespace {

void require_positive(double value, const char *name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and positive");
  }
}

void require_nonnegative(double value, const char *name) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and nonnegative");
  }
}

void require_positive(int value, const char *name) {
  if (value <= 0) {
    throw std::invalid_argument(std::string(name) + " must be positive");
  }
}

void require_probability(double value, const char *name) {
  if (!std::isfinite(value) || value <= 0.0 || value >= 1.0) {
    throw std::invalid_argument(std::string(name) + " must be in (0, 1)");
  }
}

template <typename T>
void read_value(const toml::table &config, const char *path, T &value) {
  const auto node = config.at_path(path);
  if (!node) {
    return;
  }
  const std::optional<T> parsed = node.value<T>();
  if (!parsed) {
    throw std::invalid_argument(std::string(path) + " has an invalid type");
  }
  value = *parsed;
}

void read_array(const toml::table &config, const char *path, std::vector<double> &values) {
  const auto node = config.at_path(path);
  if (!node) {
    return;
  }
  const toml::array *array = node.as_array();
  if (array == nullptr) {
    throw std::invalid_argument(std::string(path) + " must be an array");
  }

  std::vector<double> parsed;
  parsed.reserve(array->size());
  for (const toml::node &node : *array) {
    const std::optional<double> value = node.value<double>();
    if (!value) {
      throw std::invalid_argument(std::string(path) + " must contain only numbers");
    }
    parsed.push_back(*value);
  }
  values = std::move(parsed);
}

}  // namespace

SapphireParameters load_parameters(const std::filesystem::path &path) {
  const toml::table config = toml::parse_file(path.string());
  SapphireParameters parameters;
  // Application default is scene geometry only. Explicit enabled=true remains
  // the legacy archive profile for opening existing schema-1 maps.
  parameters.navi_map.enabled = false;

  read_value(config, "general.save_path", parameters.general.save_path);
  read_value(config, "general.save_map", parameters.general.save_map);
  read_value(config, "map.mode", parameters.pose_graph.map_mode);
  read_value(config, "map.database_path", parameters.pose_graph.database_path);
  read_value(config, "map.automatic_attachment", parameters.pose_graph.automatic_attachment);
  read_value(config, "map.scene_refresh", parameters.pose_graph.scene_refresh);
  read_value(config, "map.association_pending_submaps", parameters.pose_graph.association_pending_submaps);
  read_value(config, "map.association_pending_mb", parameters.pose_graph.association_pending_mb);

  if (const auto name = config.at_path("sensor.lidar_type").value<std::string>()) {
    if (*name == "livox")
      parameters.sensor.lidar_type = 0;
    else if (*name == "airy")
      parameters.sensor.lidar_type = 1;
    else if (*name == "hesai")
      parameters.sensor.lidar_type = 2;
    else
      throw std::invalid_argument("sensor.lidar_type must be livox, airy or hesai");
  } else {
    // Keep loading existing numeric configurations: 0 = Livox, 1 = Airy, 2 = Hesai.
    read_value(config, "sensor.lidar_type", parameters.sensor.lidar_type);
  }
  read_value(config, "sensor.max_scan_duration", parameters.sensor.max_scan_duration);
  read_value(config, "sensor.point_filter_num", parameters.sensor.point_filter_num);
  read_value(config, "sensor.blind", parameters.sensor.blind);
  read_array(config, "sensor.extrinsic_tran", parameters.sensor.extrinsic_tran);
  read_array(config, "sensor.extrinsic_rota", parameters.sensor.extrinsic_rota);

  read_value(config, "initializer.imu_init_samples", parameters.initializer.imu_init_samples);
  read_value(config, "initializer.down_size", parameters.initializer.down_size);
  read_value(config, "initializer.max_iterations", parameters.initializer.max_iterations);
  read_value(config, "initializer.min_plane_factors", parameters.initializer.min_plane_factors);
  read_value(config, "initializer.ba_iterations", parameters.initializer.ba_iterations);
  read_value(config, "initializer.convergence_threshold", parameters.initializer.convergence_threshold);
  read_value(config, "initializer.refined_convergence_threshold", parameters.initializer.refined_convergence_threshold);
  read_value(config, "initializer.degeneracy_threshold", parameters.initializer.degeneracy_threshold);
  read_value(config, "initializer.min_eigen_value", parameters.initializer.min_eigen_value);
  read_value(config, "initializer.plane_eigen_value_thre", parameters.initializer.plane_eigen_value_thre);

  read_value(config, "odometry.frontend", parameters.odometry.frontend);
  if (config.at_path("odometry.gaussian_fallback"))
    throw std::invalid_argument("odometry.gaussian_fallback was renamed to odometry.gicp_fallback; update the configuration");
  read_value(config, "odometry.gicp_fallback.enabled", parameters.odometry.gicp_fallback.enabled);
  read_value(config, "odometry.gicp_fallback.source_resolution", parameters.odometry.gicp_fallback.source_resolution);
  read_value(config, "odometry.gicp_fallback.max_distance", parameters.odometry.gicp_fallback.max_distance);
  read_value(config, "odometry.gicp_fallback.variance_floor", parameters.odometry.gicp_fallback.variance_floor);
  read_value(config, "odometry.gicp_fallback.max_iterations", parameters.odometry.gicp_fallback.max_iterations);
  if (config.at_path("odometry.ricp"))
    throw std::invalid_argument("The standalone RICP configuration is retired; use frontend=eskf and odometry.gicp_fallback.enabled=true");
  read_value(config, "odometry.cov_gyr", parameters.odometry.cov_gyr);
  read_value(config, "odometry.cov_acc", parameters.odometry.cov_acc);
  read_value(config, "odometry.rdw_gyr", parameters.odometry.rdw_gyr);
  read_value(config, "odometry.rdw_acc", parameters.odometry.rdw_acc);
  read_value(config, "odometry.down_size", parameters.odometry.down_size);
  read_value(config, "odometry.dept_err", parameters.odometry.dept_err);
  read_value(config, "odometry.beam_err", parameters.odometry.beam_err);
  read_value(config, "odometry.voxel_size", parameters.odometry.voxel_size);
  read_value(config, "odometry.min_eigen_value", parameters.odometry.min_eigen_value);
  read_value(config, "odometry.degrade_bound", parameters.odometry.degrade_bound);

  read_value(config, "local_submap.win_size", parameters.local_submap.win_size);
  read_value(config, "local_submap.max_layer", parameters.local_submap.max_layer);
  read_value(config, "local_submap.cov_gyr", parameters.local_submap.cov_gyr);
  read_value(config, "local_submap.cov_acc", parameters.local_submap.cov_acc);
  read_value(config, "local_submap.rdw_gyr", parameters.local_submap.rdw_gyr);
  read_value(config, "local_submap.rdw_acc", parameters.local_submap.rdw_acc);
  read_array(config, "local_submap.plane_eigen_value_thre", parameters.local_submap.plane_eigen_value_thre);
  read_value(config, "local_submap.imu_coef", parameters.local_submap.imu_coef);
  read_value(config, "local_submap.thread_num", parameters.local_submap.thread_num);

  read_value(config, "pose_graph.enabled", parameters.pose_graph.enabled);
  read_value(config, "pose_graph.submap_voxel_size", parameters.pose_graph.submap_voxel_size);
  read_value(config, "pose_graph.submap_travel_distance", parameters.pose_graph.submap_travel_distance);
  read_value(config, "pose_graph.submap_max_point_range", parameters.pose_graph.submap_max_point_range);
  read_value(config, "pose_graph.update_period_sec", parameters.pose_graph.update_period_sec);

  read_value(config, "storage.cloud_cache_mb", parameters.pose_graph.storage.cloud_cache_mb);
  read_value(config, "storage.grid_cache_mb", parameters.pose_graph.storage.grid_cache_mb);
  read_value(config, "storage.scene_cache_mb", parameters.pose_graph.storage.scene_cache_mb);
  read_value(config, "storage.scene_cache_entries", parameters.pose_graph.storage.scene_cache_entries);
  read_value(config, "visual_loop.enabled", parameters.pose_graph.visual.enabled);
  read_value(config, "visual_loop.attributes_only", parameters.pose_graph.visual.attributes_only);
  read_value(config, "visual_loop.mode", parameters.pose_graph.visual.mode);
  read_value(config, "visual_loop.global_xy_window", parameters.pose_graph.visual.global_xy_window);
  read_value(config, "visual_loop.global_z_window", parameters.pose_graph.visual.global_z_window);
  read_value(config, "visual_loop.image_interval", parameters.pose_graph.visual.image_interval);
  read_value(config, "visual_loop.tracking_enabled", parameters.pose_graph.visual.tracking_enabled);
  read_value(config, "visual_loop.tracking_interval", parameters.pose_graph.visual.tracking_interval);
  read_value(config, "visual_loop.stereo_depth", parameters.pose_graph.visual.stereo_depth);
  read_value(config, "visual_loop.keyframe_selection", parameters.pose_graph.visual.keyframe_selection);
  read_value(config, "visual_loop.keyframe_min_interval", parameters.pose_graph.visual.keyframe_min_interval);
  read_value(config, "visual_loop.keyframe_translation_m", parameters.pose_graph.visual.keyframe_translation_m);
  read_value(config, "visual_loop.keyframe_rotation_deg", parameters.pose_graph.visual.keyframe_rotation_deg);
  read_value(config, "visual_loop.keyframe_renewal", parameters.pose_graph.visual.keyframe_renewal);
  read_value(config, "visual_loop.keyframe_min_coverage", parameters.pose_graph.visual.keyframe_min_coverage);
  read_value(config, "visual_loop.max_features", parameters.pose_graph.visual.max_features);
  read_value(config, "visual_loop.max_frames", parameters.pose_graph.visual.max_frames);
  read_value(config, "visual_loop.top_k", parameters.pose_graph.visual.top_k);
  read_value(config, "visual_loop.min_matches", parameters.pose_graph.visual.min_matches);
  read_value(config, "visual_loop.left.width", parameters.pose_graph.visual.left.width);
  read_value(config, "visual_loop.left.input_width", parameters.pose_graph.visual.left.input_width);
  read_value(config, "visual_loop.left.input_height", parameters.pose_graph.visual.left.input_height);
  read_value(config, "visual_loop.left.height", parameters.pose_graph.visual.left.height);
  read_value(config, "visual_loop.left.time_offset", parameters.pose_graph.visual.left.time_offset);
  read_array(config, "visual_loop.left.intrinsics", parameters.pose_graph.visual.left.intrinsics);
  read_value(config, "visual_loop.left.distortion_model", parameters.pose_graph.visual.left.distortion_model);
  read_array(config, "visual_loop.left.distortion", parameters.pose_graph.visual.left.distortion);
  read_array(config, "visual_loop.left.camera_to_imu_rotation", parameters.pose_graph.visual.left.camera_to_imu_rotation);
  read_array(config, "visual_loop.left.camera_to_imu_translation", parameters.pose_graph.visual.left.camera_to_imu_translation);
  read_value(config, "visual_loop.right.width", parameters.pose_graph.visual.right.width);
  read_value(config, "visual_loop.right.input_width", parameters.pose_graph.visual.right.input_width);
  read_value(config, "visual_loop.right.input_height", parameters.pose_graph.visual.right.input_height);
  read_value(config, "visual_loop.right.height", parameters.pose_graph.visual.right.height);
  read_value(config, "visual_loop.right.time_offset", parameters.pose_graph.visual.right.time_offset);
  read_array(config, "visual_loop.right.intrinsics", parameters.pose_graph.visual.right.intrinsics);
  read_value(config, "visual_loop.right.distortion_model", parameters.pose_graph.visual.right.distortion_model);
  read_array(config, "visual_loop.right.distortion", parameters.pose_graph.visual.right.distortion);
  read_array(config, "visual_loop.right.camera_to_imu_rotation", parameters.pose_graph.visual.right.camera_to_imu_rotation);
  read_array(config, "visual_loop.right.camera_to_imu_translation", parameters.pose_graph.visual.right.camera_to_imu_translation);

  read_value(config, "navi_map.adaptive_ground", parameters.navi_map.adaptive_ground);
  read_value(config, "navi_map.ground_plane_tolerance", parameters.navi_map.ground_plane_tolerance);
  read_value(config, "navi_map.ground_max_correction", parameters.navi_map.ground_max_correction);
  read_value(config, "navi_map.enabled", parameters.navi_map.enabled);
  read_value(config, "navi_map.resolution", parameters.navi_map.resolution);
  read_value(config, "navi_map.h_clearance", parameters.navi_map.h_clearance);
  read_value(config, "navi_map.ground_margin", parameters.navi_map.ground_margin);
  read_value(config, "navi_map.clear_height_eps", parameters.navi_map.clear_height_eps);
  read_value(config, "navi_map.d_max", parameters.navi_map.d_max);
  read_value(config, "navi_map.occ_threshold", parameters.navi_map.occ_threshold);
  read_value(config, "navi_map.usable_range", parameters.navi_map.usable_range);
  read_value(config, "navi_map.min_range", parameters.navi_map.min_range);
  read_value(config, "navi_map.margin", parameters.navi_map.margin);
  read_value(config, "navi_map.hit_probability", parameters.navi_map.hit_probability);
  read_value(config, "navi_map.miss_probability", parameters.navi_map.miss_probability);
  read_value(config, "navi_map.clamp_min_probability", parameters.navi_map.clamp_min_probability);
  read_value(config, "navi_map.clamp_max_probability", parameters.navi_map.clamp_max_probability);

  validate_parameters(parameters);
  return parameters;
}

void validate_parameters(SapphireParameters &parameters) {
  if (parameters.pose_graph.scene_refresh && (!parameters.pose_graph.enabled || parameters.navi_map.enabled))
    throw std::invalid_argument("Scene refresh requires enabled geometry-only mapping");
  const auto &mode = parameters.pose_graph.map_mode;
  if (mode != "new" && mode != "resume" && mode != "localize") throw std::invalid_argument("map.mode must be new, resume, or localize");
  if (mode == "resume" && parameters.pose_graph.database_path.empty())
    throw std::invalid_argument("map.database_path is required for resume eligibility validation");
  if (mode == "new" && !parameters.pose_graph.enabled && !parameters.pose_graph.database_path.empty())
    throw std::invalid_argument("an explicit map.database_path requires pose_graph.enabled");
  if (parameters.pose_graph.association_pending_submaps < 1 || parameters.pose_graph.association_pending_submaps > 64 ||
      parameters.pose_graph.association_pending_mb < 1 || parameters.pose_graph.association_pending_mb > 1024)
    throw std::invalid_argument("Automatic association bounds require 1..64 submaps and 1..1024 MiB");
  if (parameters.pose_graph.automatic_attachment && (mode != "resume" || !parameters.pose_graph.enabled ||
      !parameters.pose_graph.visual.loopEnabled() || !parameters.pose_graph.visual.tracking_enabled))
    throw std::invalid_argument("Automatic attachment requires resume, pose graph and visual tracking");
  if (parameters.general.save_map != 0 && parameters.general.save_map != 1) {
    throw std::invalid_argument("General.save_map must be 0 or 1");
  }
  if (parameters.sensor.lidar_type < 0 || parameters.sensor.lidar_type > 2) {
    throw std::invalid_argument("Sensor.lidar_type must be 0, 1 or 2");
  }
  require_positive(parameters.sensor.point_filter_num, "Sensor.point_filter_num");
  require_positive(parameters.sensor.max_scan_duration, "Sensor.max_scan_duration");
  require_nonnegative(parameters.sensor.blind, "Sensor.blind");
  if (parameters.sensor.extrinsic_tran.size() != 3) {
    throw std::invalid_argument("Sensor.extrinsic_tran must contain 3 values");
  }
  if (parameters.sensor.extrinsic_rota.size() != 9) {
    throw std::invalid_argument("Sensor.extrinsic_rota must contain 9 values, got " + std::to_string(parameters.sensor.extrinsic_rota.size()));
  }
  for (double value : parameters.sensor.extrinsic_tran) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("Sensor.extrinsic_tran must be finite");
    }
  }
  for (double value : parameters.sensor.extrinsic_rota) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("Sensor.extrinsic_rota must be finite");
    }
  }
  const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rotation(parameters.sensor.extrinsic_rota.data());
  if (!rotation.isUnitary(1e-6) || std::abs(rotation.determinant() - 1.0) > 1e-6) {
    throw std::invalid_argument("Sensor.extrinsic_rota must be a rotation matrix");
  }

  require_positive(parameters.initializer.imu_init_samples, "Initializer.imu_init_samples");
  require_positive(parameters.initializer.down_size, "Initializer.down_size");
  require_positive(parameters.initializer.max_iterations, "Initializer.max_iterations");
  require_positive(parameters.initializer.min_plane_factors, "Initializer.min_plane_factors");
  require_positive(parameters.initializer.ba_iterations, "Initializer.ba_iterations");
  require_positive(parameters.initializer.convergence_threshold, "Initializer.convergence_threshold");
  require_positive(parameters.initializer.refined_convergence_threshold, "Initializer.refined_convergence_threshold");
  require_positive(parameters.initializer.degeneracy_threshold, "Initializer.degeneracy_threshold");
  require_positive(parameters.initializer.min_eigen_value, "Initializer.min_eigen_value");
  require_positive(parameters.initializer.plane_eigen_value_thre, "Initializer.plane_eigen_value_thre");

  if (parameters.odometry.frontend != "eskf")
    throw std::invalid_argument("odometry.frontend must be eskf; RICP is now an optional component enabled by odometry.gicp_fallback.enabled=true");
  require_positive(parameters.odometry.gicp_fallback.source_resolution, "Odometry.gicp_fallback.source_resolution");
  require_positive(parameters.odometry.gicp_fallback.max_distance, "Odometry.gicp_fallback.max_distance");
  require_positive(parameters.odometry.gicp_fallback.variance_floor, "Odometry.gicp_fallback.variance_floor");
  if (parameters.odometry.gicp_fallback.max_iterations < 1 || parameters.odometry.gicp_fallback.max_iterations > 20 ||
      (parameters.odometry.gicp_fallback.enabled && parameters.odometry.gicp_fallback.max_distance > parameters.odometry.voxel_size))
    throw std::invalid_argument("GICP fallback requires 1..20 iterations and distance <= root voxel size");
  require_positive(parameters.odometry.cov_gyr, "Odometry.cov_gyr");
  require_positive(parameters.odometry.cov_acc, "Odometry.cov_acc");
  require_positive(parameters.odometry.rdw_gyr, "Odometry.rdw_gyr");
  require_positive(parameters.odometry.rdw_acc, "Odometry.rdw_acc");
  require_positive(parameters.odometry.down_size, "Odometry.down_size");
  require_positive(parameters.odometry.dept_err, "Odometry.dept_err");
  require_positive(parameters.odometry.beam_err, "Odometry.beam_err");
  require_positive(parameters.odometry.voxel_size, "Odometry.voxel_size");
  require_positive(parameters.odometry.min_eigen_value, "Odometry.min_eigen_value");
  require_positive(parameters.odometry.degrade_bound, "Odometry.degrade_bound");

  require_positive(parameters.local_submap.win_size, "LocalSubmap.win_size");
  require_positive(parameters.local_submap.max_layer, "LocalSubmap.max_layer");
  if (parameters.local_submap.max_layer > 30) {
    throw std::invalid_argument("LocalSubmap.max_layer must not exceed 30");
  }
  require_positive(parameters.local_submap.cov_gyr, "LocalSubmap.cov_gyr");
  require_positive(parameters.local_submap.cov_acc, "LocalSubmap.cov_acc");
  require_positive(parameters.local_submap.rdw_gyr, "LocalSubmap.rdw_gyr");
  require_positive(parameters.local_submap.rdw_acc, "LocalSubmap.rdw_acc");
  require_positive(parameters.local_submap.imu_coef, "LocalSubmap.imu_coef");
  require_positive(parameters.local_submap.thread_num, "LocalSubmap.thread_num");
  if (parameters.local_submap.plane_eigen_value_thre.size() <= static_cast<size_t>(parameters.local_submap.max_layer)) {
    throw std::invalid_argument("LocalSubmap.plane_eigen_value_thre must cover max_layer");
  }
  parameters.local_submap.plane_eigen_value_thre_inv.clear();
  for (double threshold : parameters.local_submap.plane_eigen_value_thre) {
    require_positive(threshold, "LocalSubmap.plane_eigen_value_thre");
    parameters.local_submap.plane_eigen_value_thre_inv.push_back(1.0 / threshold);
  }

  require_positive(parameters.pose_graph.submap_voxel_size, "PoseGraph.submap_voxel_size");
  require_positive(parameters.pose_graph.submap_travel_distance, "PoseGraph.submap_travel_distance");
  require_positive(parameters.pose_graph.submap_max_point_range, "PoseGraph.submap_max_point_range");
  require_positive(parameters.pose_graph.update_period_sec, "PoseGraph.update_period_sec");

  const auto &storage = parameters.pose_graph.storage;
  for (int budget : {storage.cloud_cache_mb, storage.grid_cache_mb, storage.scene_cache_mb})
    if (budget < 1 || budget > 4096) throw std::invalid_argument("Storage cache budget must be 1..4096 MiB");
  if (storage.scene_cache_entries < 1 || storage.scene_cache_entries > 256) throw std::invalid_argument("Storage scene cache entries must be 1..256");
  const auto &visual = parameters.pose_graph.visual;
  if (visual.enabled) {
    if (visual.mode != "mono" && visual.mode != "stereo") throw std::invalid_argument("visual_loop.mode must be mono or stereo");
    if (!parameters.pose_graph.enabled) throw std::invalid_argument("visual_loop requires pose_graph.enabled");
    require_positive(visual.global_xy_window, "visual_loop.global_xy_window");
    require_positive(visual.global_z_window, "visual_loop.global_z_window");
    if (visual.global_xy_window > 50 || visual.global_z_window > 10) throw std::invalid_argument("visual_loop BBS search window exceeds budget");
    for (int i = 0; i < (visual.mode == "stereo" ? 2 : 1); ++i) {
      const auto &camera = visual.camera(i);
      if ((camera.input_width == 0) != (camera.input_height == 0) || camera.input_width < 0 || camera.input_height < 0 || camera.input_width > 8192 ||
          camera.input_height > 8192)
        throw std::invalid_argument("Invalid camera input dimensions");
      if (camera.width <= 0 || camera.height <= 0 || camera.width > 8192 || camera.height > 8192 || camera.intrinsics.size() != 4 ||
          camera.camera_to_imu_rotation.size() != 9 || camera.camera_to_imu_translation.size() != 3 ||
          !valid_camera_distortion(camera))
        throw std::invalid_argument(
            "Each enabled visual camera requires dimensions, fx/fy/cx/cy, camera-to-IMU calibration and valid radtan/equidistant distortion");
      for (const auto *values : {&camera.intrinsics, &camera.distortion, &camera.camera_to_imu_rotation, &camera.camera_to_imu_translation})
        for (double value : *values)
          if (!std::isfinite(value)) throw std::invalid_argument("Camera calibration must be finite");
      require_positive(camera.intrinsics[0], "camera.fx");
      require_positive(camera.intrinsics[1], "camera.fy");
      const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rotation(camera.camera_to_imu_rotation.data());
      if (!rotation.isUnitary(1e-6) || std::abs(rotation.determinant() - 1) > 1e-6) throw std::invalid_argument("Camera-to-IMU rotation is invalid");
      if (!std::isfinite(camera.time_offset)) throw std::invalid_argument("Camera time offset must be finite");
      if ((visual.tracking_enabled || visual.keyframe_selection) && (camera.width < 64 || camera.height < 64 || camera.width > 4096 || camera.height > 4096))
        throw std::invalid_argument("Visual tracking requires processing dimensions between 64 and 4096");
    }
    if (visual.attributes_only && (!visual.keyframe_selection || visual.stereo_depth || visual.tracking_enabled || visual.max_features > 500))
      throw std::invalid_argument("Image attributes require keyframe selection, <=500 features and no depth tracking");
    if (visual.attributes_only && (visual.left.width * std::int64_t(visual.left.height) > 1024*1024 ||
        (visual.mode == "stereo" && visual.right.width * std::int64_t(visual.right.height) > 1024*1024)))
      throw std::invalid_argument("Image attributes exceed pixel budget");
    if (visual.stereo_depth) {
      if (visual.mode != "stereo" || visual.tracking_enabled || visual.max_features > 500)
        throw std::invalid_argument("Direct stereo requires two cameras, tracking_enabled=false and <=500 features");
      const Eigen::Map<const Eigen::Vector3d> a(visual.left.camera_to_imu_translation.data());
      const Eigen::Map<const Eigen::Vector3d> b(visual.right.camera_to_imu_translation.data());
      if ((a-b).norm() < 1e-4) throw std::invalid_argument("Stereo requires a nonzero calibrated baseline");
    }
    require_positive(visual.image_interval, "visual_loop.image_interval");
    require_positive(visual.tracking_interval, "visual_loop.tracking_interval");
    if (visual.keyframe_selection) {
      if (!visual.stereo_depth && !visual.attributes_only) throw std::invalid_argument("Flow/LIO keyframe selection requires direct stereo");
      require_positive(visual.keyframe_min_interval, "visual_loop.keyframe_min_interval");
      require_positive(visual.keyframe_translation_m, "visual_loop.keyframe_translation_m");
      require_positive(visual.keyframe_rotation_deg, "visual_loop.keyframe_rotation_deg");
      if (visual.keyframe_rotation_deg > 180) throw std::invalid_argument("Keyframe rotation exceeds 180 degrees");
      require_positive(visual.keyframe_renewal, "visual_loop.keyframe_renewal");
      require_probability(visual.keyframe_renewal, "visual_loop.keyframe_renewal");
      require_positive(visual.keyframe_min_coverage, "visual_loop.keyframe_min_coverage");
      require_probability(visual.keyframe_min_coverage, "visual_loop.keyframe_min_coverage");
    }
    if ((visual.tracking_enabled || visual.keyframe_selection) && (visual.tracking_interval > .2 || visual.max_features < 40))
      throw std::invalid_argument("Visual tracking requires interval <= 0.2 s and at least 40 features");
    if (visual.max_features < visual.min_matches || visual.max_features > 8192 || visual.max_frames < 1 || visual.max_frames > 32 ||
        visual.top_k < 1 || visual.top_k > 20 || visual.min_matches < 6)
      throw std::invalid_argument("visual_loop feature/frame/candidate budgets or inlier threshold are invalid");
  }

  require_positive(parameters.navi_map.ground_plane_tolerance, "navi_map.ground_plane_tolerance");
  require_positive(parameters.navi_map.ground_max_correction, "navi_map.ground_max_correction");
  require_positive(parameters.navi_map.resolution, "NaviMap.resolution");
  require_nonnegative(parameters.navi_map.h_clearance, "NaviMap.h_clearance");
  require_nonnegative(parameters.navi_map.ground_margin, "NaviMap.ground_margin");
  require_nonnegative(parameters.navi_map.clear_height_eps, "NaviMap.clear_height_eps");
  require_positive(parameters.navi_map.d_max, "NaviMap.d_max");
  require_probability(parameters.navi_map.occ_threshold, "NaviMap.occ_threshold");
  require_positive(parameters.navi_map.usable_range, "NaviMap.usable_range");
  require_nonnegative(parameters.navi_map.min_range, "NaviMap.min_range");
  if (parameters.navi_map.min_range >= parameters.navi_map.usable_range) {
    throw std::invalid_argument("NaviMap.min_range must be less than usable_range");
  }
  require_nonnegative(parameters.navi_map.margin, "NaviMap.margin");
  require_probability(parameters.navi_map.hit_probability, "NaviMap.hit_probability");
  require_probability(parameters.navi_map.miss_probability, "NaviMap.miss_probability");
  require_probability(parameters.navi_map.clamp_min_probability, "NaviMap.clamp_min_probability");
  require_probability(parameters.navi_map.clamp_max_probability, "NaviMap.clamp_max_probability");
  if (parameters.navi_map.clamp_min_probability >= parameters.navi_map.clamp_max_probability) {
    throw std::invalid_argument("NaviMap clamp probabilities must be ordered");
  }

  parameters.sensor.blind_squared = parameters.sensor.blind * parameters.sensor.blind;
  parameters.initializer.down_size_inv = 1.0 / parameters.initializer.down_size;
  parameters.initializer.plane_eigen_value_thre_inv = 1.0 / parameters.initializer.plane_eigen_value_thre;
  parameters.odometry.down_size_inv = 1.0 / parameters.odometry.down_size;
  parameters.odometry.voxel_size_inv = 1.0 / parameters.odometry.voxel_size;
  parameters.pose_graph.submap_voxel_size_inv = 1.0 / parameters.pose_graph.submap_voxel_size;
}

bool valid_camera_distortion(const CameraParameters &camera) {
  const bool layout = camera.distortion_model == "radtan"
      ? (camera.distortion.empty() || camera.distortion.size() == 5)
      : (camera.distortion_model == "equidistant" && camera.distortion.size() == 4);
  return layout && std::all_of(camera.distortion.begin(), camera.distortion.end(),
      [](double value) { return std::isfinite(value); });
}

std::string map_config_identity(const PoseGraphParameters &graph, const NaviMapParameters &grid) {
  if (graph.scene_refresh && grid.enabled) throw std::invalid_argument("Scene refresh cannot archive navigation");
  std::ostringstream encoded;
  encoded.imbue(std::locale::classic());
  encoded << std::setprecision(std::numeric_limits<double>::max_digits10);
  const auto add = [&](const auto &value) { encoded << value << ';'; };
  add(grid.enabled ? "sapphire-map-config-2" : (graph.scene_refresh ? "sapphire-retained-scene-config-4" : "sapphire-geometry-map-config-3"));
  // Legacy archived LocalGrid evidence is interpreted using these six values.
  // Geometry-only maps have no such evidence or navigation configuration identity.
  // Construction, calibration and search policies do not decode stored metric
  // geometry or appearance-only scenes. See docs/PHASE1_DESIGN.md for the audit.
  if (grid.enabled) {
    add(grid.resolution);
    add(grid.occ_threshold);
    add(grid.hit_probability);
    add(grid.miss_probability);
    add(grid.clamp_min_probability);
    add(grid.clamp_max_probability);
  }
  // FNV-1a: stable across processes/platforms; a compatibility check, not authentication.
  std::uint64_t hash = 14695981039346656037ULL;
  for (unsigned char byte : encoded.str()) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  std::ostringstream result;
  result.imbue(std::locale::classic());
  result << (grid.enabled ? "c2:" : (graph.scene_refresh ? "c4:" : "c3:")) << std::hex << std::setfill('0') << std::setw(16) << hash;
  return result.str();
}

}  // namespace sapphire
