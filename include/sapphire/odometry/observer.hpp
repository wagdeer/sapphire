#pragma once

#include <sapphire/types.hpp>

namespace sapphire {

/// Result of one geometric-observer correction at a LiDAR reference time.
struct ObserverUpdate {
    NavigationState state;
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
};

/// Apply a DLIO-style geometric observer update.
///
/// The prior state is the IMU-integrated state at the current LiDAR reference
/// timestamp. The measurement pose is the GICP-corrected IMU pose at that same
/// timestamp. Rejected measurements and non-positive intervals leave the
/// state and biases unchanged.
ObserverUpdate applyGeometricObserver(
    const NavigationState& prior_state,
    const Isometry3d& T_world_imu_measurement,
    double previous_state_stamp,
    const Eigen::Vector3d& accel_bias,
    const Eigen::Vector3d& gyro_bias,
    const Config::Odometry::Observer& config,
    bool measurement_accepted);

}  // namespace sapphire
