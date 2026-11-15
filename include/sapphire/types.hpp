#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
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
};

/// IMU navigation state at a specific timestamp.
struct NavigationState {
  double stamp = 0.0;
  Isometry3d T_world_imu = Isometry3d::Identity();
  Eigen::Vector3d v_world = Eigen::Vector3d::Zero();
  bool valid = false;
};

/// Map-frame pose returned by latestResult() / latestPropagatedResult().
/// The stored frontend sample is an OdomFrameSample and is not premultiplied
/// by T_map_odom.
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

/// IMU stationary initialization parameters
struct ImuInitConfig {
  double convergence_gyro_std = 0.005;  // rad/s
  double convergence_accel_std = 0.05;  // m/s²
  double timeout_sec = 5.0;             // max calibration duration
  double check_interval_sec = 0.5;      // how often to check convergence
  int min_samples = 100;                // min ~0.5s at 200 Hz
  double gravity_mag = 9.80665;         // m/s²
};

/// IMU noise parameters for equivariant preintegration.
/// Defaults are for BMI088 (Mid-360 built-in IMU).
/// Sources: datasheet §4.1 noise density + DLIO defaults for random walk.
struct ImuNoiseConfig {
  double accel_noise_density = 0.000196;  // m/s²/√Hz  (BMI088: 175 μg/√Hz, rounded)
  double gyro_noise_density = 0.000152;   // rad/s/√Hz (BMI088: 0.014 °/s/√Hz ≈ 0.000244, DLIO uses 0.000152)
  double accel_random_walk = 0.000165;    // m/s²·√s  (DLIO default)
  double gyro_random_walk = 0.0000655;    // rad/s·√s  (DLIO default)
  double accel_bias_sigma = 1e-6;         // initial bias uncertainty
  double gyro_bias_sigma = 1e-5;
  double accel_bias_rw_sigma = 0.0;  // bias random walk (zero: assume stationary init gives good bias)
  double gyro_bias_rw_sigma = 0.0;
};

/// GICP registration parameters.
/// Defaults match the validated DLIO Mid-360 profile.
struct RegistrationConfig {
  int max_iterations = 32;
  double transformation_epsilon = 0.01;  // m
  double rotation_epsilon = 0.01;        // rad
  double max_correspondence_dist = 0.5;  // m
  int k_correspondences = 16;
  int min_num_points = 64;
  /// Sensor displacement of C * T_prior, in meters. Not ||C.translation()||:
  /// a yaw about the sensor far from the origin puts the lever arm there.
  double max_correction_trans = 1.0;
  double max_correction_rot_deg = 20.0;  // deg — reject if larger
};

/// Registration result returned by Registration::align()
struct RegistrationResult {
  Eigen::Isometry3d T_correction;   // correction: T_prior → T_world_lidar
  Eigen::Isometry3d T_world_lidar;  // world-frame pose after correction
  bool converged = false;
  size_t source_points = 0;
  size_t num_inliers = 0;
  size_t iterations = 0;
  double inlier_ratio = 0.0;
  double solver_error = 0.0;
  bool accepted = false;  // false if correction was rejected
  /// Solver correction before the accept/reject gate. Identity is stored in
  /// T_correction when the gate rejects, so these keep the attempt visible.
  /// attempted_trans_m is ||C.translation()|| (lever arm about the world
  /// origin). attempted_sensor_trans_m is the displacement of the sensor
  /// pose and is what max_correction_trans compares.
  double attempted_trans_m = 0.0;
  double attempted_sensor_trans_m = 0.0;
  double attempted_rot_deg = 0.0;
  /// Final GICP information matrix from small_gicp, order [rx,ry,rz,tx,ty,tz].
  Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Zero();
  bool hessian_valid = false;
  /// Diagnostics copied from the final Hessian. The combined matrix is
  /// retained above for the existing ESKF path; these blockwise values are
  /// scale-separated and are not yet used to alter the update.
  Eigen::Matrix<double, 6, 1> hessian_eigenvalues = Eigen::Matrix<double, 6, 1>::Zero();
  double rotation_min_eigenvalue = -1.0;
  double translation_min_eigenvalue = -1.0;
  double rotation_condition = -1.0;
  double translation_condition = -1.0;
  bool conditioning_valid = false;
};

/// IMU initialization and noise, loaded from the TOML `imu` table.
struct ImuConfig {
  ImuInitConfig init;
  ImuNoiseConfig noise;
};

/// Error coordinates of the 15D frontend ESKF.
///
/// Euclidean keeps today's world-frame additive velocity and position.
/// Gal3Right is ξ = log(χ̄⁻¹ χ) on Gal(3), time slot fixed at 0, biases additive.
enum class EskfChart {
  Euclidean,
  Gal3Right,
};

/// Configuration for the Approach-A ESKF frontend fusion.
/// loadConfig writes this struct directly; chart is already an EskfChart.
struct EskfConfig {
  /// Default stays on the Euclidean chart. Gal3Right replaces Φ, Q, z, R,
  /// H, the retraction, and the reset Jacobian together, and forces
  /// inject_full_pose=false, inject_directional_pose=false,
  /// velocity_correction_gain=0, bias_update_scale=1.
  EskfChart chart = EskfChart::Euclidean;
  /// Fallback ICP measurement std when Hessian is unused / invalid.
  /// Keep small: the current GICP frontend is already accurate without ESKF,
  /// so the filter should mostly follow ICP for pose.
  double sigma_rotation = 0.01;     // rad
  double sigma_translation = 0.01;  // m
  /// Inflate H_gicp^{-1} when use_hessian is enabled.
  double icp_covariance_scale = 25.0;
  /// If true, build R from small_gicp Hessian; otherwise diagonal fallback.
  bool use_hessian = false;
  /// Numerical and degeneracy controls for the GICP information matrix.
  double hessian_min_information = 1e-6;
  double hessian_max_condition = 1e4;
  double hessian_degenerate_sigma = 1.0;
  double hessian_max_sigma = 10.0;
  /// χ² threshold for 6-DOF innovation. Set <= 0 to disable the gate.
  /// With high ICP trust / inject_full_pose, prefer hard registration gates
  /// only: a tight Mahalanobis gate rejects good ICP and opens an IMU-only
  /// death spiral within seconds.
  double mahalanobis_threshold = -1.0;
  /// Pose-injection modes — interaction semantics
  /// ============================================
  /// correctAt() evaluates three gain-modifying flags in a fixed order.
  /// Because later blocks unconditionally overwrite rows of K_effective,
  /// the flags follow an explicit priority hierarchy:
  ///
  ///    inject_full_pose  >  inject_directional_pose  >  Kalman rows
  ///   (highest priority)                              (lowest priority)
  ///
  /// ── Priority table ─────────────────────────────────────────────────
  /// | velocity_correction_gain? | inject_directional? | inject_full? | Result                             |
  /// |---------------------------|---------------------|--------------|------------------------------------|
  /// | 0                         | false               | false        | Pure Kalman on all 15 states       |
  /// | >0                        | false               | false        | Kalman orientation + biases;       |
  /// |                           |                     |              | velocity via position-observer     |
  /// | any                       | true                | false        | Kalman velocity + biases; pose     |
  /// |                           |                     |              | rows projected by Hessian obs.     |
  /// | any                       | any                 | true         | Pose = ICP measurement (identity   |
  /// |                           |                     |              | gain); velocity/biases via Kalman  |
  /// |---------------------------|---------------------|--------------|------------------------------------|
  ///
  /// Note: inject_full_pose overwrites the rot/pos rows that may have been
  /// set by inject_directional_pose or the baseline Kalman gain.  Enabling
  /// both directional and full pose injection simultaneously is wasteful
  /// (directional rows are computed but immediately discarded).
  ///
  /// Note: inject_directional_pose requires use_hessian = true and a valid
  /// Hessian to produce meaningful observability weights.  Without those,
  /// directional_observability remains identity and the flag is a no-op.
  ///
  /// velocity_correction_gain also interacts with inject_directional_pose
  /// when use_hessian is enabled: the velocity observer is projected through
  /// position observability extracted from the Hessian, so weak directions
  /// receive proportional rather than full velocity updates.
  ///
  /// ── When to use each mode ──────────────────────────────────────────
  ///   • Pure Kalman (all false): highest theoretical accuracy when ICP
  ///     covariance is well-calibrated and biases are known.
  ///   • velocity_correction_gain > 0: recommended when ICP gives sharp
  ///     pose jumps that would otherwise mislead velocity differencing.
  ///     The observer acts as a low-pass on velocity while avoiding
  ///     Jacobian-based coupling that may be too weak in practice.
  ///   • inject_directional_pose: useful when ICP is only partially
  ///     observable (e.g. long corridors) — lets ICP correct the strong
  ///     eigen-directions while the IMU prior stabilizes the weak ones.
  ///   • inject_full_pose: simplest / most robust.  ICP owns pose; Kalman
  ///     only refines velocity, accel bias, and gyro bias.  This matches
  ///     the observer frontend's correction semantics and is the default.
  ///
  /// If true, accepted updates set R/p exactly to the ICP pose. Velocity and
  /// biases still come from the Kalman update (bias optionally damped).
  bool inject_full_pose = true;
  /// If true (and Hessian is enabled), accepted pose increments use the
  /// Hessian observability projector rather than the statistical Kalman
  /// pose rows. Strong directions follow GICP; weak directions retain IMU.
  bool inject_directional_pose = false;
  /// Scale applied to Kalman bias increments (1 = full ESKF coupling).
  double bias_update_scale = 0.25;
  /// Optional position-innovation velocity correction [1/s]. This avoids
  /// differentiating noisy ICP poses while keeping scan-to-scan prediction
  /// responsive. Set 0 to use the unconstrained Kalman velocity rows.
  /// See the pose-injection priority table above for interaction with
  /// inject_directional_pose / inject_full_pose.
  double velocity_correction_gain = 0.0;
  double accel_bias_max = 10.0;  // m/s²
  double gyro_bias_max = 0.5;    // rad/s
  /// Initial orientation / velocity / position std used in P0.
  double init_sigma_theta = 0.1;     // rad
  double init_sigma_velocity = 1.0;  // m/s
  double init_sigma_position = 1.0;  // m
  /// gal3_right only. Added in quadrature to the calibrated accelerometer
  /// density, on the same right-Jacobian input channel. This is model
  /// error of the specific force, not a rewrite of the sensor calibration.
  /// Zero keeps the datasheet Q, whose steady-state pose gain collapses.
  double specific_force_model_density = 0.0;  // m/s²/√Hz
  /// gal3_right only. Variance rate on the right-error position block.
  /// Pose-transport residual that is not a velocity error. It is not
  /// multiplied by an extra Φ. Zero leaves position diffusion to the
  /// input channel's O(dt³) term.
  double position_model_diffusion = 0.0;  // m²/s
};

/// Top-level configuration — will be loaded from TOML
struct Config {
  struct Extrinsics {
    /// Maps LiDAR-frame points into the IMU frame.
    Isometry3d T_imu_lidar = Isometry3d::Identity();
  } extrinsics;

  struct Odometry {
    /// Leaf size used to downsample each deskewed scan before GICP.
    double voxel_size = 0.25;
    /// Frontend fusion backend: "observer" (default) or "eskf".
    std::string fusion = "observer";
    /// Axis-aligned box in LiDAR frame; points inside are removed (robot body).
    struct CropBox {
      double min_x = -1.0;
      double min_y = -1.0;
      double min_z = -1.0;
      double max_x = 1.0;
      double max_y = 1.0;
      double max_z = 1.0;
    } crop_box;
    struct Submap {
      double splitting_distance = 2.0;
      double splitting_rotation = 0.7853981633974483;  // 45 degrees
      int max_keyframes = 10;
      double voxel_size = 0.25;
      /// Reset the local registration map after this many consecutive
      /// rejected scans. Zero disables recovery.
      int recovery_after_rejections = 0;
    } submap;
    struct Observer {
      // DLIO geometric-observer gains.
      double position_gain = 4.5;
      double velocity_gain = 11.25;
      double orientation_gain = 4.0;
      double accel_bias_gain = 2.25;
      double gyro_bias_gain = 1.0;
      double accel_bias_max = 10.0;  // m/s², absolute total bias
      double gyro_bias_max = 0.5;    // rad/s, absolute total bias
    } observer;
    EskfConfig eskf;
  } odometry;

  struct Registration {
    std::string type = "GICP";  // GICP | VGICP
    RegistrationConfig gicp;
    struct Vgicp {
      double voxel_resolution = 0.5;
    } vgicp;
  } registration;

  struct Pgo {
    bool enabled = false;
    double keyframe_distance = 0.5;
    double keyframe_rotation = 0.3;
    double loop_min_time_separation = 30.0;
    double loop_min_travel_distance = 30.0;
    double loop_max_rotation = 3.14;
    /// Radius for spatial loop candidates. Up to five historical frames
    /// inside this radius are tried, nearest first; that count is fixed
    /// in the backend, not a toml field.
    double loop_search_radius = 30.0;
    int loop_search_stride = 2;
    int target_frame_count = 50;
    double source_voxel_size = 0.4;
    double target_voxel_size = 0.4;
    /// Odometry travel around the query whose keyframe clouds are
    /// transformed into the query frame before GICP. The graph node
    /// spacing stays keyframe_distance. Zero keeps the query scan alone.
    double source_travel_radius_m = 10.0;
    /// Drop a point farther than this from the scan that observed it.
    /// The gate is in that scan's body frame, before the odometry
    /// transform into the query. NodeBuilder uses the same 20 m.
    double source_max_range_m = 20.0;
    /// Cap on the query window. A stationary spin never leaves the
    /// travel radius, so the frame count would otherwise grow without
    /// bound. Same limit as NodeBuilder::max_history_frames.
    int source_max_frames = 256;
    /// GICP loop acceptance. Point covariances use 20 neighbors.
    /// Translation and rotation convergence epsilons stay 0.01 in code.
    int lidar_min_inliers = 64;
    int lidar_max_iterations = 32;
    double lidar_max_correspondence_m = 2.0;
    double lidar_min_overlap = 0.5;
    double lidar_max_fitness_m2 = 0.5;
    /// Diagonal floors added to the GICP covariance before it is
    /// inverted into the loop factor. Hessian order is GTSAM Pose3:
    /// rotation first, translation second.
    double lidar_translation_sigma_m = 0.2;
    double lidar_rotation_sigma_rad = 0.1;
    /// Drop a loop trial when any binary edge, after optimization on an
    /// ISAM2 copy, exceeds this multiple of that edge's standard
    /// deviation. Translation uses the largest absolute component;
    /// rotation uses the geodesic angle. Intern name:
    /// pose_graph.max_constraint_error_sigma.
    double max_constraint_error_sigma = 3.0;
    double update_period_sec = 1.0;
    int map_frame_stride = 3;
    double map_voxel_size = 0.8;

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

  ImuConfig imu;

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
    } lidar;

    /// Acceleration unit of the raw IMU message.
    /// auto: |mean| in [0.5, 1.5] is g, [5, 15] is m/s².
    /// g and m_s2 skip the magnitude test.
    struct Imu {
      std::string accel_unit = "auto";
    } imu;
  } sensor;

  /// Optional TUM dump written once when the ROS node exits.
  /// An empty trajectory_dir disables it. The file is the pose-graph map
  /// trajectory, so a non-empty trajectory_dir requires pgo.enabled.
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
