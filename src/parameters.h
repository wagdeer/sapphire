#pragma once

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

template <typename T>
inline T declare_get_param(const rclcpp::Node::SharedPtr &node, const std::string &name, const T &default_value)
{
  if(!node->has_parameter(name))
    node->declare_parameter<T>(name, default_value);
  return node->get_parameter(name).get_value<T>();
}

struct GeneralParameters
{
  std::string lidar_topic;
  std::string imu_topic;
  std::string save_path;
  int save_map;

  explicit GeneralParameters(const rclcpp::Node::SharedPtr &node)
    : lidar_topic(declare_get_param<std::string>(node, "General.lidar_topic", "/livox/lidar")),
      imu_topic(declare_get_param<std::string>(node, "General.imu_topic", "/livox/imu")),
      save_path(declare_get_param<std::string>(node, "General.save_path", "")),
      save_map(declare_get_param<int>(node, "General.save_map", 0))
  {}
};

struct SensorParameters
{
  int lidar_type;
  int point_filter_num;
  double blind;
  double blind_squared;
  std::vector<double> extrinsic_tran;
  std::vector<double> extrinsic_rota;

  explicit SensorParameters(const rclcpp::Node::SharedPtr &node)
    : lidar_type(declare_get_param<int>(node, "Sensor.lidar_type", 0)),
      point_filter_num(declare_get_param<int>(node, "Sensor.point_filter_num", 3)),
      blind(declare_get_param<double>(node, "Sensor.blind", 0.1)),
      blind_squared(blind * blind),
      extrinsic_tran(declare_get_param<std::vector<double>>(node, "Sensor.extrinsic_tran", {0.0, 0.0, 0.0})),
      extrinsic_rota(declare_get_param<std::vector<double>>(node, "Sensor.extrinsic_rota", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}))
  {}
};

struct InitializerParameters
{
  int imu_init_samples;
  double down_size;
  double down_size_inv;
  int max_iterations;
  int min_plane_factors;
  int ba_iterations;
  double convergence_threshold;
  double refined_convergence_threshold;
  double degeneracy_threshold;
  double min_eigen_value;
  double plane_eigen_value_thre_inv;

  explicit InitializerParameters(const rclcpp::Node::SharedPtr &node)
    : imu_init_samples(declare_get_param<int>(node, "Initializer.imu_init_samples", 30)),
      down_size(declare_get_param<double>(node, "Initializer.down_size", 0.5)),
      down_size_inv(1.0 / down_size),
      max_iterations(declare_get_param<int>(node, "Initializer.max_iterations", 10)),
      min_plane_factors(declare_get_param<int>(node, "Initializer.min_plane_factors", 10)),
      ba_iterations(declare_get_param<int>(node, "Initializer.ba_iterations", 3)),
      convergence_threshold(declare_get_param<double>(node, "Initializer.convergence_threshold", 0.05)),
      refined_convergence_threshold(declare_get_param<double>(node, "Initializer.refined_convergence_threshold", 0.01)),
      degeneracy_threshold(declare_get_param<double>(node, "Initializer.degeneracy_threshold", 15.0)),
      min_eigen_value(declare_get_param<double>(node, "Initializer.min_eigen_value", 0.02)),
      plane_eigen_value_thre_inv(1.0 / declare_get_param<double>(node, "Initializer.plane_eigen_value_thre", 4.0))
  {}
};

struct OdometryParameters
{
  double cov_gyr;
  double cov_acc;
  double rdw_gyr;
  double rdw_acc;
  double down_size;
  double down_size_inv;
  double dept_err;
  double beam_err;
  double voxel_size;
  double voxel_size_inv;
  double min_eigen_value;
  int degrade_bound;

  explicit OdometryParameters(const rclcpp::Node::SharedPtr &node)
    : cov_gyr(declare_get_param<double>(node, "Odometry.cov_gyr", 0.1)),
      cov_acc(declare_get_param<double>(node, "Odometry.cov_acc", 0.1)),
      rdw_gyr(declare_get_param<double>(node, "Odometry.rdw_gyr", 1e-4)),
      rdw_acc(declare_get_param<double>(node, "Odometry.rdw_acc", 1e-4)),
      down_size(declare_get_param<double>(node, "Odometry.down_size", 0.1)),
      down_size_inv(1.0 / down_size),
      dept_err(declare_get_param<double>(node, "Odometry.dept_err", 0.02)),
      beam_err(declare_get_param<double>(node, "Odometry.beam_err", 0.05)),
      voxel_size(declare_get_param<double>(node, "Odometry.voxel_size", 1.0)),
      voxel_size_inv(1.0 / voxel_size),
      min_eigen_value(declare_get_param<double>(node, "Odometry.min_eigen_value", 0.0025)),
      degrade_bound(declare_get_param<int>(node, "Odometry.degrade_bound", 10))
  {}
};

struct LocalSubmapParameters
{
  int win_size;
  int max_layer;
  double cov_gyr;
  double cov_acc;
  double rdw_gyr;
  double rdw_acc;
  std::vector<double> plane_eigen_value_thre;
  double imu_coef;
  int thread_num;

  explicit LocalSubmapParameters(const rclcpp::Node::SharedPtr &node)
    : win_size(declare_get_param<int>(node, "LocalSubmap.win_size", 10)),
      max_layer(declare_get_param<int>(node, "LocalSubmap.max_layer", 2)),
      cov_gyr(declare_get_param<double>(node, "LocalSubmap.cov_gyr", 0.1)),
      cov_acc(declare_get_param<double>(node, "LocalSubmap.cov_acc", 0.1)),
      rdw_gyr(declare_get_param<double>(node, "LocalSubmap.rdw_gyr", 1e-4)),
      rdw_acc(declare_get_param<double>(node, "LocalSubmap.rdw_acc", 1e-4)),
      plane_eigen_value_thre(declare_get_param<std::vector<double>>(node, "LocalSubmap.plane_eigen_value_thre", {1.0, 1.0, 1.0, 1.0})),
      imu_coef(declare_get_param<double>(node, "LocalSubmap.imu_coef", 1e-4)),
      thread_num(declare_get_param<int>(node, "LocalSubmap.thread_num", 5))
  {
    for(double &threshold: plane_eigen_value_thre)
      threshold = 1.0 / threshold;
  }
};

struct NaviMapParameters
{
  bool enabled;
  double resolution;
  double h_clearance;
  double ground_margin;
  double clear_height_eps;
  double d_max;
  double occ_threshold;
  double usable_range;
  double min_range;
  double margin;
  double hit_probability;
  double miss_probability;
  double clamp_min_probability;
  double clamp_max_probability;

  explicit NaviMapParameters(const rclcpp::Node::SharedPtr &node)
    : enabled(declare_get_param<bool>(node, "NaviMap.enabled", true)),
      resolution(declare_get_param<double>(node, "NaviMap.resolution", 0.1)),
      h_clearance(declare_get_param<double>(node, "NaviMap.h_clearance", 2.0)),
      ground_margin(declare_get_param<double>(node, "NaviMap.ground_margin", 0.5)),
      clear_height_eps(declare_get_param<double>(node, "NaviMap.clear_height_eps", 0.1)),
      d_max(declare_get_param<double>(node, "NaviMap.d_max", 6.0)),
      occ_threshold(declare_get_param<double>(node, "NaviMap.occ_threshold", 0.3)),
      usable_range(declare_get_param<double>(node, "NaviMap.usable_range", 40.0)),
      min_range(declare_get_param<double>(node, "NaviMap.min_range", 0.5)),
      margin(declare_get_param<double>(node, "NaviMap.margin", 5.0)),
      hit_probability(declare_get_param<double>(node, "NaviMap.hit_probability", 0.7)),
      miss_probability(declare_get_param<double>(node, "NaviMap.miss_probability", 0.4)),
      clamp_min_probability(declare_get_param<double>(node, "NaviMap.clamp_min_probability", 0.1192)),
      clamp_max_probability(declare_get_param<double>(node, "NaviMap.clamp_max_probability", 0.971))
  {}
};

struct PoseGraphParameters
{
  bool enabled;
  double loop_min_time_separation;
  double loop_min_travel_distance;
  double loop_max_rotation;
  double loop_search_radius;
  int loop_search_stride;
  int target_frame_count;
  double keyframe_voxel_size;
  double keyframe_voxel_size_inv;
  double fitness_threshold;
  double update_period_sec;
  int map_frame_stride;
  double map_voxel_size;
  int memory_stm_size;
  int memory_wm_size;
  int gicp_max_iterations;
  int gicp_k_correspondences;
  int gicp_min_inliers;
  int gicp_num_threads;
  double gicp_max_correspondence_distance;
  double gicp_transformation_epsilon;
  double gicp_rotation_epsilon;

  explicit PoseGraphParameters(const rclcpp::Node::SharedPtr &node)
    : enabled(declare_get_param<bool>(node, "PoseGraph.enabled", true)),
      loop_min_time_separation(declare_get_param<double>(node, "PoseGraph.loop_min_time_separation", 30.0)),
      loop_min_travel_distance(declare_get_param<double>(node, "PoseGraph.loop_min_travel_distance", 30.0)),
      loop_max_rotation(declare_get_param<double>(node, "PoseGraph.loop_max_rotation", 3.14)),
      loop_search_radius(declare_get_param<double>(node, "PoseGraph.loop_search_radius", 15.0)),
      loop_search_stride(declare_get_param<int>(node, "PoseGraph.loop_search_stride", 1)),
      target_frame_count(declare_get_param<int>(node, "PoseGraph.target_frame_count", 50)),
      keyframe_voxel_size(declare_get_param<double>(node, "PoseGraph.keyframe_voxel_size", 0.25)),
      keyframe_voxel_size_inv(1.0 / keyframe_voxel_size),
      fitness_threshold(declare_get_param<double>(node, "PoseGraph.fitness_threshold", 0.5)),
      update_period_sec(declare_get_param<double>(node, "PoseGraph.update_period_sec", 1.0)),
      map_frame_stride(declare_get_param<int>(node, "PoseGraph.map_frame_stride", 3)),
      map_voxel_size(declare_get_param<double>(node, "PoseGraph.map_voxel_size", 0.8)),
      memory_stm_size(declare_get_param<int>(node, "PoseGraph.memory_stm_size", 30)),
      memory_wm_size(declare_get_param<int>(node, "PoseGraph.memory_wm_size", 200)),
      gicp_max_iterations(declare_get_param<int>(node, "PoseGraph.gicp_max_iterations", 32)),
      gicp_k_correspondences(declare_get_param<int>(node, "PoseGraph.gicp_k_correspondences", 16)),
      gicp_min_inliers(declare_get_param<int>(node, "PoseGraph.gicp_min_inliers", 64)),
      gicp_num_threads(declare_get_param<int>(node, "PoseGraph.gicp_num_threads", 2)),
      gicp_max_correspondence_distance(declare_get_param<double>(node, "PoseGraph.gicp_max_correspondence_distance", 2.0)),
      gicp_transformation_epsilon(declare_get_param<double>(node, "PoseGraph.gicp_transformation_epsilon", 0.01)),
      gicp_rotation_epsilon(declare_get_param<double>(node, "PoseGraph.gicp_rotation_epsilon", 0.01))
  {}
};

struct VoxelSlamParameters
{
  GeneralParameters general;
  SensorParameters sensor;
  InitializerParameters initializer;
  OdometryParameters odometry;
  LocalSubmapParameters local_submap;
  PoseGraphParameters pose_graph;
  NaviMapParameters navi_map;

  explicit VoxelSlamParameters(const rclcpp::Node::SharedPtr &node)
    : general(node),
      sensor(node),
      initializer(node),
      odometry(node),
      local_submap(node),
      pose_graph(node),
      navi_map(node)
  {}
};