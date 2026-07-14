# Sapphire

LiDAR-inertial odometry using Gal(3) equivariant IMU preintegration.

[中文说明](README.zh.md)

Sapphire is a feature-free, scan-to-submap LIO pipeline. Its main design
choice is to use the same Gal(3) state-recovery model for per-point deskew
and IMU-rate propagation. Pose and velocity therefore come from one
preintegrated state instead of two independent motion models.

The core is a C++17 static library with no ROS dependency. ROS 2 support
lives in the separate
[`sapphire_ros2`](https://github.com/wagdeer/sapphire_ros2) wrapper.

## Pipeline

```text
IMU
  -> stationary initialization (MAD outlier rejection + variance convergence)
  -> bias correction and timestamp validation
  -> fixed-capacity IMU buffer
  -> Gal(3) propagation

LiDAR
  -> NaN/crop-box filtering
  -> Gal(3) per-point deskew
  -> voxel downsampling
  -> GICP or VGICP scan-to-submap registration
  -> geometric observer and online bias correction
  -> frontend keyframe submap
  -> asynchronous ISAM2 pose graph and loop closure
```

For an integration interval, Sapphire recovers the Gal(3) state as

```text
Xi(t) = Gamma_ij(t) * Xi_ref * Upsilon_ij(t)
```

`Gamma_ij` contains the gravity/time evolution, while `Upsilon_ij` is the
preintegrated local IMU increment. Pose and velocity are extracted from
the recovered `Xi(t)`. The vendored implementation under
`external/preintegration/` is based on Fornasier et al.'s
`aau-cns/equivariant-preintegration`.

## Features

- Stationary IMU initialization with configurable convergence and timeout
- Non-finite, non-monotonic, and large-gap IMU input diagnostics
- Gal(3) deskew and propagation with online accelerometer/gyroscope bias
  correction
- Custom parallel voxel filter
- GICP and VGICP registration through `small_gicp`
- Distance/rotation-gated frontend submaps
- Asynchronous GTSAM ISAM2 pose graph with ICP loop verification
- Validated TOML configuration
- Ten CTest targets covering configuration, deskew, preintegration,
  pipeline, observer, registration, submaps, pose graph, voxel filtering,
  and the IMU ring buffer

The frontend operates in a local odometry frame. The pose-graph backend
maintains a separate `T_map_odom` correction and composes it onto published
poses without rewriting frontend deskew or observer state.

## Requirements

- CMake 3.16+
- A C++17 compiler
- Eigen3
- spdlog
- PCL
- OpenMP
- GTSAM
- small_gicp
- tomlplusplus 3.4+ (downloaded by CMake when a system package is absent)

The ROS wrapper additionally requires ROS 2, `rclcpp`, `sensor_msgs`,
`nav_msgs`, `diagnostic_msgs`, `visualization_msgs`, `tf2_ros`,
`tf2_eigen`, and `pcl_ros`.

GTSAM and small_gicp installation differs by platform. Install versions
that export CMake package configurations discoverable by `find_package`
before configuring Sapphire.

## Build and test the core

```bash
git clone https://github.com/wagdeer/sapphire.git
cd sapphire

cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

## Build with ROS 2

Keep the core and wrapper as sibling directories. The wrapper detects the
core and adds it to its build automatically.

```bash
mkdir -p ~/sapphire_ws/src
cd ~/sapphire_ws/src
git clone https://github.com/wagdeer/sapphire.git
git clone https://github.com/wagdeer/sapphire_ros2.git

cd ~/sapphire_ws
export ROS_DISTRO=humble  # Change to the ROS 2 distribution you installed.
source "/opt/ros/${ROS_DISTRO}/setup.bash"
colcon build --symlink-install --packages-select sapphire_ros2
colcon test --packages-select sapphire_ros2
colcon test-result --all
source install/setup.bash
```

## Docker

The wrapper repository includes a parameterized development image. Build
it from the `sapphire_ros2` repository root and select the ROS distribution
with `ROS_DISTRO`:

```bash
cd ~/sapphire_ws/src/sapphire_ros2

# ROS 2 Humble / Ubuntu 22.04
docker build -f docker/Dockerfile \
  --build-arg ROS_DISTRO=humble \
  -t sapphire:humble .

# ROS 2 Jazzy / Ubuntu 24.04
docker build -f docker/Dockerfile \
  --build-arg ROS_DISTRO=jazzy \
  -t sapphire:jazzy .
```

The default base variant is `desktop-full`. It can also be selected
explicitly with `--build-arg ROS_IMAGE_VARIANT=desktop-full`. A chosen
combination must exist as an `osrf/ros` image, and the ROS and GTSAM
repositories must provide packages for its Ubuntu release.

Mount the workspace into the container:

```bash
docker run -it --rm --network host \
  -v ~/sapphire_ws:/workspace \
  sapphire:humble

colcon build --symlink-install --packages-select sapphire_ros2
source install/setup.bash
```

## Run

The default launch profile targets a Livox Mid-360:

```bash
ros2 launch sapphire_ros2 sapphire.launch.py
```

Common overrides:

```bash
ros2 launch sapphire_ros2 sapphire.launch.py \
  lidar_topic:=/points \
  imu_topic:=/imu/data \
  odom_topic:=/sapphire/odometry \
  acc_scale:=1.0 \
  config_file:=/absolute/path/to/sapphire_mid360.toml
```

Launch defaults:

- LiDAR input: `/livox/lidar`
- IMU input: `/livox/imu`
- Odometry output: `/sapphire/odometry`
- IMU acceleration scale: `9.80665` (Mid-360 messages report acceleration
  in units of g)
- Parent/child frames: `map` and `lidar`

Published interfaces:

- `/sapphire/odometry` — `nav_msgs/msg/Odometry`, normally at IMU rate
- `/sapphire/deskewed` — `sensor_msgs/msg/PointCloud2`, at LiDAR rate
- `/sapphire/pgo/map` — subscriber-driven pose-graph map
- `/sapphire/pgo/graph` — subscriber-driven pose-graph markers
- TF — `map -> lidar` by default; frame names are configurable

## Sensor assumptions

Check these before using a new sensor or dataset:

1. Keep the platform stationary while IMU initialization is running.
2. Supply monotonically increasing LiDAR and IMU timestamps from the same
   clock domain.
3. Convert acceleration to m/s². Set `acc_scale:=1.0` if the ROS message
   already uses SI units.
4. Convert each point's relative timestamp to seconds in the ROS adapter.
5. Set `T_imu_lidar` correctly. It maps LiDAR-frame points into the IMU
   frame:

   ```text
   p_imu = R_imu_lidar * p_lidar + t_imu_lidar
   ```

6. Verify that the configured crop box does not remove valid near-field
   geometry for the sensor mounting position.

The IMU buffer is fixed-capacity. If LiDAR processing stalls long enough
for required history to be overwritten, Sapphire rejects that scan instead
of deskewing it with incomplete IMU coverage.

## Configuration

The default profile is `cfg/sapphire_mid360.toml`:

```toml
[imu]
[imu.init]
[imu.noise]
[deskew]
[odometry]
[odometry.crop_box]
[odometry.observer]
[odometry.submap]
[registration]
[registration.gicp]
[registration.vgicp]
[pgo]
[extrinsics.T_imu_lidar]
```

Configuration is validated at startup. Changes take effect after restart;
no recompilation is required.

## Repository layout

```text
sapphire/
├── cfg/                         Default sensor configuration
├── docs/                        Design and review documents
├── external/                    Vendored preintegration and Lie groups
├── include/sapphire/
│   ├── backend/pose_graph.hpp
│   ├── odometry/
│   │   ├── deskew.hpp
│   │   ├── observer.hpp
│   │   ├── pipeline.hpp
│   │   ├── registration.hpp
│   │   ├── submap.hpp
│   │   └── voxel_filter.hpp
│   ├── imu_init.hpp
│   ├── ring_buffer.hpp
│   └── types.hpp
├── src/                         Core implementation
└── tests/                       Unit and integration-style tests
```

## Credits

- [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry):
  pipeline, observer, and submap design reference
- [Equivariant IMU Preintegration with Biases](https://github.com/aau-cns/equivariant-preintegration):
  Gal(3) preintegration theory and implementation
- [small_gicp](https://github.com/koide3/small_gicp): GICP and VGICP
- [GTSAM](https://github.com/borglab/gtsam): incremental pose-graph
  optimization

## License

MIT.
