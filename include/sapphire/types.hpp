#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
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
    PCL_ADD_POINT4D;       // x, y, z as floats, data[3]=1
    float intensity;
    double timestamp;       // seconds from scan start (for deskew)

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using PointCloud = pcl::PointCloud<Point>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;

// ═══════════════════════════════════════════════════════════════════
//  Math Types
// ═══════════════════════════════════════════════════════════════════

using Vector3f = Eigen::Vector3f;
using Vector3d = Eigen::Vector3d;
using Isometry3f = Eigen::Isometry3f;
using Isometry3d = Eigen::Isometry3d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

/// Voxel index
using Voxel = Eigen::Vector3i;

// ═══════════════════════════════════════════════════════════════════
//  Sensor Data
// ═══════════════════════════════════════════════════════════════════

/// Raw IMU measurement (single sample)
struct ImuData {
    double stamp = 0.0;              // seconds
    Eigen::Vector3d accel;           // m/s^2, in IMU frame
    Eigen::Vector3d gyro;            // rad/s, in IMU frame
};

/// IMU navigation state at a specific timestamp.
struct NavigationState {
    double stamp = 0.0;
    Isometry3d T_world_imu = Isometry3d::Identity();
    Eigen::Vector3d v_world = Eigen::Vector3d::Zero();
    bool valid = false;
};

/// Odometry pipeline output per LiDAR scan
struct OdometryResult {
    Isometry3d T_world_lidar;        // current pose in world frame
    Eigen::Vector3d v_world = Eigen::Vector3d::Zero();
    double stamp = 0.0;
    bool converged = false;
};

// ═══════════════════════════════════════════════════════════════════
//  Configuration
// ═══════════════════════════════════════════════════════════════════

/// IMU stationary initialization parameters
struct ImuInitConfig {
    double convergence_gyro_std  = 0.005;   // rad/s
    double convergence_accel_std = 0.05;    // m/s²
    double timeout_sec           = 5.0;     // max calibration duration
    double check_interval_sec    = 0.5;     // how often to check convergence
    int    min_samples           = 100;     // min ~0.5s at 200 Hz
    double gravity_mag           = 9.80665; // m/s²
};

/// IMU noise parameters for equivariant preintegration.
/// Defaults are for BMI088 (Mid-360 built-in IMU).
/// Sources: datasheet §4.1 noise density + DLIO defaults for random walk.
struct ImuNoiseConfig {
    double accel_noise_density  = 0.000196;  // m/s²/√Hz  (BMI088: 175 μg/√Hz, rounded)
    double gyro_noise_density   = 0.000152;  // rad/s/√Hz (BMI088: 0.014 °/s/√Hz ≈ 0.000244, DLIO uses 0.000152)
    double accel_random_walk    = 0.000165;  // m/s²·√s  (DLIO default)
    double gyro_random_walk     = 0.0000655; // rad/s·√s  (DLIO default)
    double accel_bias_sigma     = 1e-6;      // initial bias uncertainty
    double gyro_bias_sigma      = 1e-5;
    double accel_bias_rw_sigma  = 0.0;       // bias random walk (zero: assume stationary init gives good bias)
    double gyro_bias_rw_sigma   = 0.0;
};

/// Desired number of deskew time slices.
/// Higher = more precise motion compensation but more compute.
/// Mid-360 with ~20K unique timestamps benefits from group-by (adaptive slicing).
struct DeskewConfig {
    // Mid-360 offset_time is already relative to the message reference stamp.
    // Enable only for drivers whose header must be aligned to the first retained point.
    bool time_offset = false;
};

/// GICP registration parameters.
/// Defaults match the validated DLIO Mid-360 profile.
struct RegistrationConfig {
    int    max_iterations            = 32;
    double transformation_epsilon    = 0.01;      // m
    double rotation_epsilon          = 0.01;      // rad
    double max_correspondence_dist   = 0.5;       // m
    int    k_correspondences         = 16;
    int    min_num_points            = 64;
    double max_correction_trans      = 1.0;       // m — reject if larger
    double max_correction_rot_deg    = 20.0;      // deg — reject if larger
};

/// Registration result returned by Registration::align()
struct RegistrationResult {
    Eigen::Isometry3d T_correction;  // correction: T_prior → T_world_lidar
    Eigen::Isometry3d T_world_lidar; // world-frame pose after correction
    bool    converged      = false;
    size_t  num_inliers    = 0;
    bool    accepted       = false;   // false if correction was rejected
    /// Final GICP information matrix from small_gicp, order [rx,ry,rz,tx,ty,tz].
    Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Zero();
    bool    hessian_valid  = false;
};

/// Sapphire sensor suite config — mirrors odometry.yaml.
/// Designed for zero-compromise: every parameter has a clear source and unit.
struct ImuConfig {
    ImuInitConfig init;
    ImuNoiseConfig noise;
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
            double max_x =  1.0;
            double max_y =  1.0;
            double max_z =  1.0;
        } crop_box;
        struct Submap {
            double splitting_distance = 2.0;
            double splitting_rotation = 0.7853981633974483;  // 45 degrees
            int max_keyframes = 10;
            double voxel_size = 0.25;
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
        struct Eskf {
            double sigma_rotation = 0.01;       // rad
            double sigma_translation = 0.01;    // m
            double icp_covariance_scale = 25.0;
            bool use_hessian = false;
            /// Absolute information floor used before inverting the GICP
            /// Hessian. Eigen-directions weaker than max_eigen /
            /// hessian_max_condition are treated as degenerate.
            double hessian_min_information = 1e-6;
            double hessian_max_condition = 1e4;
            /// Correction-tangent standard deviation assigned to a
            /// degenerate Hessian direction, capped at this maximum.
            double hessian_degenerate_sigma = 1.0;
            double hessian_max_sigma = 10.0;
            double mahalanobis_threshold = -1.0;  // disabled; hard gates only
            bool inject_full_pose = true;
            /// Use Hessian observability itself as the accepted pose gain:
            /// observable directions follow GICP, degenerate ones keep IMU.
            bool inject_directional_pose = false;
            double bias_update_scale = 0.25;
            double velocity_correction_gain = 0.0;
            double accel_bias_max = 10.0;
            double gyro_bias_max = 0.5;
            double init_sigma_theta = 0.1;      // rad
            double init_sigma_velocity = 1.0;   // m/s
            double init_sigma_position = 1.0;   // m
        } eskf;
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
        double loop_search_radius = 15.0;
        int loop_search_stride = 2;
        int target_frame_count = 50;
        double source_voxel_size = 0.4;
        double target_voxel_size = 0.4;
        double fitness_threshold = 0.3;
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
    DeskewConfig deskew;
};

}  // namespace sapphire

// Register PCL point type so pcl::fromROSMsg / toROSMsg work automatically
// and small_gicp can consume it directly.
POINT_CLOUD_REGISTER_POINT_STRUCT(sapphire::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp))
