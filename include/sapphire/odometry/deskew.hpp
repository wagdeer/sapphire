#pragma once

#include <sapphire/types.hpp>
#include <sapphire/ring_buffer.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <vector>

namespace sapphire {

inline constexpr std::size_t kImuBufferCapacity = 500;
using ImuBuffer = RingBuffer<ImuData, kImuBufferCapacity>;

enum class DeskewStatus {
    Success,
    EmptyScan,
    NoPointTimestamps,
    EmptyImuBuffer,
    InvalidTimeRange,
    InvalidImuOrder,
    InsufficientImuCoverage,
};

struct DeskewMetrics {
    double timeline_ms = 0.0;
    double integration_ms = 0.0;
    double transform_ms = 0.0;
    double total_ms = 0.0;
    size_t timestamp_groups = 0;
    size_t imu_intervals = 0;
    size_t pim_copies = 0;
};

/// Result of per-point motion compensation
struct DeskewResult {
    PointCloudPtr cloud;                    // motion-corrected points in world frame
    Isometry3d T_world_lidar_ref = Isometry3d::Identity();
    Eigen::Vector3d v_world_ref = Eigen::Vector3d::Zero();
    double reference_stamp = 0.0;
    DeskewMetrics metrics;
    DeskewStatus status = DeskewStatus::EmptyScan;
    bool converged = false;                 // true only when per-point deskew succeeded
};

/// Deskew a LiDAR scan into the world frame using Gal(3) IMU preintegration.
///
/// Algorithm:
///   1. Sort points by timestamp
///   2. Extract unique timestamps → build per-group indices
///   3. Integrate IMU exactly to each unique timestamp
///   4. Recover the world IMU state using Gamma * T_i * Upsilon
///   5. Transform each point with T_world_imu(t) * T_imu_lidar
///
/// T_A_B maps coordinates in frame B into frame A.
///
/// @param scan              input scan; point timestamps are relative seconds
/// @param scan_stamp        absolute timestamp corresponding to timestamp zero
/// @param imu_buf           bias-corrected IMU samples, oldest to newest
/// @param prev_stamp        absolute time of the supplied initial state
/// @param T_world_imu_prev  IMU pose in world at prev_stamp
/// @param v_world_prev      IMU velocity in world at prev_stamp
/// @param T_imu_lidar       calibrated LiDAR-to-IMU extrinsic
/// @param gravity_world     world gravity acceleration, normally (0,0,-g)
/// @param noise             IMU noise parameters
/// @param time_offset       if true, align the earliest retained point to scan_stamp
DeskewResult deskew(
    const PointCloudConstPtr& scan,
    double scan_stamp,
    const ImuBuffer& imu_buf,
    double prev_stamp,
    const Isometry3d& T_world_imu_prev,
    const Eigen::Vector3d& v_world_prev,
    const Isometry3d& T_imu_lidar,
    const Eigen::Vector3d& gravity_world,
    const ImuNoiseConfig& noise,
    bool time_offset = false
);

}  // namespace sapphire
