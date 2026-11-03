# Design

Sapphire is a LiDAR-inertial odometry library. The C++17 core has no ROS
dependency. `sapphire_ros1` and `sapphire_ros2` convert messages, load a TOML
profile, and publish the result.

## Pipeline

1. A short stationary period initializes gyroscope bias, accelerometer bias, and gravity alignment.
2. Each scan is deskewed with Gal(3) preintegration. IMU propagation between scans uses the same preintegration, so deskew and state prediction stay on one motion model.
3. The deskewed cloud is voxel-filtered and registered to a local submap. The profile selects GICP or VGICP.
4. The registration residual updates velocity and IMU bias. The profile selects the update: `odometry.fusion` is `eskf` or `observer`. The profiles in `cfg/` use `eskf`.
5. Keyframes refresh the submap. When pose-graph optimization is enabled, an asynchronous ISAM2 graph runs beside the front end and applies a map correction. The front end keeps integrating in the odometry frame.

Deskewed clouds are expressed in the world frame. With pose-graph optimization disabled, the map frame and the odometry frame are the same frame.

The derivation of the ESKF update is in [eskf_on_gal3.md](eskf_on_gal3.md).

## Layout

```text
include/sapphire/
├── odometry/     pipeline, deskew, registration, submap, ESKF, observer
├── backend/      pose graph
├── mapping/      occupancy grid
├── config.hpp    TOML profiles
└── imu_init.hpp
cfg/              dataset profiles
```

## Dependencies

Eigen 3.4, PCL, OpenMP, GTSAM, small_gicp, spdlog, and toml++. CMake also
looks up an MPI C library while configuring PCL, because VTK references
`MPI::MPI_C`. The odometry code does not call MPI. toml++ 3.4 is downloaded
by CMake when it is not already installed.
