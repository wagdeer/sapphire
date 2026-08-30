#include "parameters.h"

#include <Eigen/Core>
#include <Eigen/LU>
#include <cmath>
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

  read_value(config, "general.save_path", parameters.general.save_path);
  read_value(config, "general.save_map", parameters.general.save_map);

  read_value(config, "sensor.lidar_type", parameters.sensor.lidar_type);
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
  if (parameters.general.save_map != 0 && parameters.general.save_map != 1) {
    throw std::invalid_argument("General.save_map must be 0 or 1");
  }
  if (parameters.sensor.lidar_type != 0 && parameters.sensor.lidar_type != 1) {
    throw std::invalid_argument("Sensor.lidar_type must be 0 or 1");
  }
  require_positive(parameters.sensor.point_filter_num, "Sensor.point_filter_num");
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

}  // namespace sapphire
