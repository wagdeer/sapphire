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

using Vector3d = Eigen::Vector3d;
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

/// Per-LiDAR-frame health and performance diagnostics.
struct OdometryDiagnostics {
    double preprocess_ms = 0.0;
    double downsample_ms = 0.0;
    double submap_rebuild_ms = 0.0;
    double deskew_timeline_ms = 0.0;
    double deskew_integration_ms = 0.0;
    double deskew_transform_ms = 0.0;
    double deskew_total_ms = 0.0;
    double registration_ms = 0.0;
    double fitness_score = 0.0;
    size_t num_inliers = 0;
    size_t iterations = 0;
    size_t source_points = 0;
    size_t target_points = 0;
    size_t keyframe_count = 0;
    size_t stored_keyframe_points = 0;
    size_t deskew_timestamp_groups = 0;
    size_t deskew_imu_intervals = 0;
    size_t deskew_pim_copies = 0;
    bool registration_accepted = false;
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
};

/// Odometry pipeline output per LiDAR scan
struct OdometryResult {
    Isometry3d T_world_lidar;        // current pose in world frame
    Eigen::Vector3d v_world = Eigen::Vector3d::Zero();
    OdometryDiagnostics diagnostics;
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
    double  fitness_score  = 0.0;
    double  elapsed_ms     = 0.0;
    size_t  num_inliers    = 0;
    size_t  iterations     = 0;
    bool    accepted       = false;   // false if correction was rejected
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
    } odometry;

    struct Registration {
        std::string type = "GICP";  // GICP | VGICP | CUDA_VGICP
        RegistrationConfig gicp;
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
    } pgo;

    struct Cuda {
        bool enabled = false;
    } cuda;

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
