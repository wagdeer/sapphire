#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace sapphire {

// ═══════════════════════════════════════════════════════════════════
//  Point Types
// ═══════════════════════════════════════════════════════════════════

/// Unified LiDAR point type for all sensors.
///
/// ROS wrapper maps sensor-specific fields (Livox 'offset_time', Velodyne 'time',
/// Ouster 't', etc.) into this canonical type. Downstream algorithms only see this.
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;  // x, y, z as floats, data[3]=1
  float intensity;
  double timestamp;  // absolute seconds, same clock as ImuData.stamp

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using PointCloud = pcl::PointCloud<Point>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;

// ═══════════════════════════════════════════════════════════════════
//  Math Types
// ═══════════════════════════════════════════════════════════════════

using Vector3d = Eigen::Vector3d;
using Isometry3f = Eigen::Isometry3f;
using Isometry3d = Eigen::Isometry3d;

// ═══════════════════════════════════════════════════════════════════
//  Sensor Data
// ═══════════════════════════════════════════════════════════════════

/// Raw IMU measurement (single sample)
struct ImuData {
  double stamp = 0.0;     // seconds
  Eigen::Vector3d accel;  // m/s^2, in IMU frame
  Eigen::Vector3d gyro;   // rad/s, in IMU frame

  bool isFinite() const { return std::isfinite(stamp) && accel.allFinite() && gyro.allFinite(); }
};

/// Latest published LiDAR result in the map frame.
struct OdometryResult {
  Isometry3d T_world_lidar;  // T_map_odom * T_odom_lidar
  Eigen::Vector3d v_world = Eigen::Vector3d::Zero();
  double stamp = 0.0;
  bool converged = false;
};

/// Accepted lidar sample in the odometry frame. Map correction stays in the
/// publishers above; recording this into the pose graph must not apply it.
struct OdomFrameSample {
  Isometry3d T_odom_lidar = Isometry3d::Identity();
  Eigen::Vector3d v_odom = Eigen::Vector3d::Zero();
  double stamp = 0.0;
  bool converged = false;
};

// ═══════════════════════════════════════════════════════════════════
//  Configuration
// ═══════════════════════════════════════════════════════════════════

struct ImuInitResult {
  Eigen::Vector3d gyro_bias;     // rad/s
  Eigen::Vector3d accel_bias;    // m/s²
  Eigen::Quaterniond q_gravity;  // IMU to world
  double accel_std;
  double accel_scale = 1.0;      // raw specific force to m/s²
};

struct ImuConfig {
  struct Init {
    double gravity_mag = 9.80665;
  } init;
  struct Noise {
    // Diagonal coefficients of the odometry iESKF process noise.
    double gyro_var = 0.01;
    double accel_var = 0.01;
    double gyro_bias_var = 0.0001;
    double accel_bias_var = 0.0001;
  } noise;
};

/// Top-level configuration — will be loaded from TOML
struct Config {
  struct Extrinsics {
    /// Maps LiDAR-frame points into the IMU frame.
    Isometry3d T_imu_lidar = Isometry3d::Identity();
  } extrinsics;

  struct Io {
    std::string map_directory;
    double global_map_voxel_size = 0.8;
  } io;

  struct Odometry {
    /// Leaf size used to sample each deskewed scan for registration.
    double voxel_size = 0.25;
    int voxel_capacity = 350000;
    bool enable_map_feedback = false;
    bool map_feedback_ba = false;
    /// Axis-aligned box in LiDAR frame; points inside are removed (robot body).
    struct CropBox {
      double min_x = -1.0;
      double min_y = -1.0;
      double min_z = -1.0;
      double max_x = 1.0;
      double max_y = 1.0;
      double max_z = 1.0;

      bool contains(const Point& point) const {
        return point.x >= min_x && point.x <= max_x && point.y >= min_y && point.y <= max_y && point.z >= min_z && point.z <= max_z;
      }
    } crop_box;
  } odometry;

  struct Submap {
    int min_voxel_num = 72;  // Minimum fitted voxels to seal an online block.
    int min_frames = 20;
    double max_range_m = 30.0;
    double max_deformation_m = 0.10;

    struct Ba {
      int min_frames = 8;
      int min_planes = 16;
      int max_iterations = 3;
      double relative_cost_tolerance = 1e-6;
    } ba;
  } submap;

  struct Pgo {
    bool enabled = true;
    double relinearize_threshold = 0.1;
    int relinearize_skip = 10;

    struct LoopClosure {
      bool enabled = true;
      double local_search_radius_m = 18.0;
      double local_departure_radius_m = 22.0;
      int local_max_candidates = 3;
      double local_min_overlap = 0.45;
      double global_search_radius_min_m = 20.0;
      double global_search_radius_max_m = 90.0;
      double global_search_radius_growth = 0.014;
      double global_departure_radius_m = 25.0;
      int global_max_candidates = 15;
      double fft_min_score = 0.10;
      double fft_min_ambiguity = 1.18;
      double global_min_overlap = 0.53;
      double global_max_rms_m = 0.085;
      int global_refine_updates = 8;
    } loop_closure;

    /// 2.5D occupancy grid built from PGO keyframes.
    ///
    /// Height band uses signed distance d (world Z relative to sensor, or
    /// equivalently body-z when attitude is flat). Hits are kept only for
    /// d in [-h_clearance + ground_margin, d_max].
    /// For a roof-mounted Mid-360, set h_clearance ≈ sensor height above
    /// ground (e.g. 1.5–2.5 m); 0.15 m only keeps a thin slice at the
    /// sensor and will miss tree trunks / curbs below the lidar.
    /// Use ground_margin > 0 to keep the near-ground slice out of HIT
    /// (avoids flooring the map and locking d_min for clearing).
    struct Occupancy {
      bool enabled = false;
      double resolution = 0.1;
      /// Lower band edge before ground_margin (≈ sensor mount height).
      double h_clearance = 2.0;
      /// Raise the HIT lower bound by this many meters:
      /// hit iff d >= -h_clearance + ground_margin. 0 keeps legacy band.
      double ground_margin = 0.0;
      /// Drop hits with d > d_max (ceilings / high canopy). <=0 disables.
      double d_max = 3.0;
      /// Free rays may clear a hit cell when d_ray <= d_min + clear_height_eps.
      /// Strict d_ray < d_min blocks same-height pass-through, so a wall that
      /// jitters by one cell along the ray leaves a permanent occupied layer.
      double clear_height_eps = 0.05;
      double occ_threshold = 0.3;
      double usable_range = 40.0;
      double min_range = 0.5;
      /// Voxel size used before inserting a keyframe cloud.
      double cloud_voxel_size = 0.2;
      /// Extra meters reserved when expanding the map AABB.
      double margin = 5.0;
    } occupancy;
  } pgo;

  /// Optional LiDAR time classification. Omitted TOML keys stay automatic.
  /// time_field, time_unit, time_ref, max_scan_duration, and header_tolerance
  /// match sensor::IngestConfig. makeLidarTimeSession copies those fields
  /// and require_point_time into the runtime lock.
  struct Sensor {
    struct Lidar {
      std::string time_field = "auto";
      std::string time_unit = "auto";
      std::string time_ref = "auto";
      double max_scan_duration = 0.5;
      double header_tolerance = 0.5;
      bool require_point_time = false;
      double max_range_m = 0.0;  // Zero disables the input far-range cutoff.
    } lidar;
    ImuConfig imu;
  } sensor;

  /// Optional TUM dump written once when the ROS node exits.
  /// An empty trajectory_dir disables it. The file is the pose-graph map
  /// trajectory.
  /// trajectory_file is relative to that directory and may include dataset
  /// folders, for example "sapphire/S3E_Dormitory_1/alpha.tum".
  struct Output {
    std::string trajectory_dir;
    std::string trajectory_file = "trajectory.tum";
  } output;
};

}  // namespace sapphire

// Register PCL point type so pcl::fromROSMsg / toROSMsg work automatically
// and small_gicp can consume it directly.
POINT_CLOUD_REGISTER_POINT_STRUCT(sapphire::Point, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp))
