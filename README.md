# Sapphire

LiDAR-inertial odometry using Gal(3) equivariant IMU preintegration.

[中文说明](README.zh.md)

For repeatable HILTI and S3E visualization runs in the ROS 1 and ROS 2 Docker
environments, see [docs/DATASET_REPLAY.md](docs/DATASET_REPLAY.md).

Sapphire is a feature-free, scan-to-submap LIO pipeline. Its main design
choice is to use the same Gal(3) state-recovery model for per-point deskew
and IMU-rate propagation. Pose and velocity therefore come from one
preintegrated state instead of two independent motion models.

The core is a C++17 static library with no ROS dependency. ROS 2 support
lives in the separate
[`sapphire_ros2`](https://github.com/wagdeer/sapphire_ros2) wrapper.

## Benchmark snapshot

Across the six robust S3E scenes, 17 trajectories cover **10.42 km** of
ground-truth path with **1.17 m mean ATE** and **1.05 m median ATE**
(unweighted).
HILTI retains five primary sparse-ATE runs over **346.4 m** of
control-point reference, with **0.058 m mean ATE** and **0.052 m median ATE**.
Newer College completes all five experiments over **5.94 km** of continuous
ground truth, with **0.285 m mean ATE** and **0.182 m median ATE**.

ATE is SE(3)-Umeyama-aligned translation APE without scale correction. Each
S3E robot is reported separately. GT length is the reference path, while
estimated travel is the full TUM path. Laboratory is omitted from this ATE
table because its GT supports only endpoint error.

| Dataset | Sequence | Agent | GT length (m) | Estimated travel (m) | Metric | Result (m)
|---|---|---|---:|---:|---|---:
| HILTI | Site 1 · Handheld 1 | reference | 81.32 | 205.5 | sparse ATE | 0.030
| HILTI | Site 1 · Handheld 2 | reference | 84.39 | 163.9 | sparse ATE | 0.052
| HILTI | Site 1 · Handheld 3 | reference | 81.89 | 165.2 | sparse ATE | 0.108
| HILTI | Site 1 · Handheld 4 | reference | 52.88 | 300.0 | sparse ATE | 0.053
| HILTI | Site 1 · Handheld 5 | reference | 45.94 | 150.1 | sparse ATE | 0.045
| S3E | Campus Road 3 | Alpha | 983.3 | 940.0 | ATE | 0.876
| S3E | Campus Road 3 | Carol | 986.9 | 948.9 | ATE | 1.050
| S3E | Dormitory 1 | Alpha | 727.0 | 724.0 | ATE | 0.478
| S3E | Dormitory 1 | Bob | 719.3 | 722.6 | ATE | 1.378
| S3E | Dormitory 1 | Carol | 721.9 | 738.0 | ATE | 0.804
| S3E | Library 1 | Alpha | 507.6 | 520.4 | ATE | 0.988
| S3E | Library 1 | Bob | 517.2 | 530.6 | ATE | 1.671
| S3E | Library 1 | Carol | 498.9 | 508.1 | ATE | 1.339
| S3E | Library 2 | Alpha | 521.6 | 505.0 | ATE | 1.238
| S3E | Library 2 | Bob | 523.9 | 505.5 | ATE | 1.683
| S3E | Library 2 | Carol | 520.3 | 505.8 | ATE | 0.923
| S3E | Square 1 | Alpha | 546.0 | 542.7 | ATE | 1.166
| S3E | Square 1 | Bob | 496.5 | 495.9 | ATE | 0.748
| S3E | Square 1 | Carol | 529.2 | 531.7 | ATE | 0.866
| S3E | Square 3 | Alpha | 487.4 | 463.7 | ATE | 1.835
| S3E | Square 3 | Bob | 569.6 | 559.8 | ATE | 1.932
| S3E | Square 3 | Carol | 563.8 | 541.0 | ATE | 0.912
| Newer College | Short Experiment | reference | 1609.4 | 1422.7 | ATE | 0.502
| Newer College | Long Experiment | reference | 3063.4 | 2603.4 | ATE | 0.494
| Newer College | Quad with Dynamics | reference | 479.0 | 476.0 | ATE | 0.148
| Newer College | Dynamic Spinning | reference | 97.20 | 71.48 | ATE | 0.100
| Newer College | Parkland Mound | reference | 695.7 | 654.8 | ATE | 0.182

Excluded from the headline comparison: HILTI Site 2 scenes with high error,
invalid degeneracy, or too few control points; S3E Campus Road 3 Bob as the
remaining ATE outlier; S3E Teaching Building 1 and Tunnel 1 due to GT/path
anomalies. Laboratory 1-4 are omitted because their GT supports only endpoint
error, not continuous ATE.

### Trajectories vs ground truth

Each image is one benchmark result. Solid is Sapphire, dashed is GT, and
crosses mark HILTI control points.

| Trajectory result | Trajectory result |
|---|---|
| ![HILTI Site 1 Handheld 1](docs/benchmarks/trajectories/hilti_site1_handheld_1.png) | ![HILTI Site 1 Handheld 2](docs/benchmarks/trajectories/hilti_site1_handheld_2.png) |
| ![HILTI Site 1 Handheld 3](docs/benchmarks/trajectories/hilti_site1_handheld_3.png) | ![HILTI Site 1 Handheld 4](docs/benchmarks/trajectories/hilti_site1_handheld_4.png) |
| ![HILTI Site 1 Handheld 5](docs/benchmarks/trajectories/hilti_site1_handheld_5.png) | ![S3E Campus Road 3 Alpha](docs/benchmarks/trajectories/s3e_campus_road_3_alpha.png) |
| ![S3E Campus Road 3 Carol](docs/benchmarks/trajectories/s3e_campus_road_3_carol.png) | ![S3E Dormitory 1 Alpha](docs/benchmarks/trajectories/s3e_dormitory_1_alpha.png) |
| ![S3E Dormitory 1 Bob](docs/benchmarks/trajectories/s3e_dormitory_1_bob.png) | ![S3E Dormitory 1 Carol](docs/benchmarks/trajectories/s3e_dormitory_1_carol.png) |
| ![S3E Library 1 Alpha](docs/benchmarks/trajectories/s3e_library_1_alpha.png) | ![S3E Library 1 Bob](docs/benchmarks/trajectories/s3e_library_1_bob.png) |
| ![S3E Library 1 Carol](docs/benchmarks/trajectories/s3e_library_1_carol.png) | ![S3E Library 2 Alpha](docs/benchmarks/trajectories/s3e_library_2_alpha.png) |
| ![S3E Library 2 Bob](docs/benchmarks/trajectories/s3e_library_2_bob.png) | ![S3E Library 2 Carol](docs/benchmarks/trajectories/s3e_library_2_carol.png) |
| ![S3E Square 1 Alpha](docs/benchmarks/trajectories/s3e_square_1_alpha.png) | ![S3E Square 1 Bob](docs/benchmarks/trajectories/s3e_square_1_bob.png) |
| ![S3E Square 1 Carol](docs/benchmarks/trajectories/s3e_square_1_carol.png) | ![S3E Square 3 Alpha](docs/benchmarks/trajectories/s3e_square_3_alpha.png) |
| ![S3E Square 3 Bob](docs/benchmarks/trajectories/s3e_square_3_bob.png) | ![S3E Square 3 Carol](docs/benchmarks/trajectories/s3e_square_3_carol.png) |
| ![Newer College Short Experiment](docs/benchmarks/trajectories/newer_college_01_short_experiment.png) | ![Newer College Long Experiment](docs/benchmarks/trajectories/newer_college_02_long_experiment.png) |
| ![Newer College Quad with Dynamics](docs/benchmarks/trajectories/newer_college_05_quad_with_dynamics.png) | ![Newer College Dynamic Spinning](docs/benchmarks/trajectories/newer_college_06_dynamic_spinning.png) |
| ![Newer College Parkland Mound](docs/benchmarks/trajectories/newer_college_07_parkland_mound.png) |  |

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
- Eleven CTest targets covering configuration, sensor ingest, deskew,
  preintegration, pipeline, observer, registration, submaps, pose graph,
  voxel filtering, and the IMU ring buffer

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
  config_file:=/absolute/path/to/sapphire_mid360.toml
```

Launch defaults:

- LiDAR input: `/livox/lidar`
- IMU input: `/livox/imu`
- Odometry output: `/sapphire/odometry`
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
3. Acceleration may arrive in g or m/s². Sapphire converts a stationary
   magnitude near 1 g into m/s². Set `[sensor.imu] accel_unit` to `g` or
   `m_s2` only when that check fails. Angular velocity stays in rad/s.
4. Point timestamps enter the pipeline as absolute seconds on the IMU
   clock. The ROS adapter classifies the cloud; set `[sensor.lidar]` only
   when that classification fails.
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
├── docs/                        Design, replay, and benchmark evidence
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
│   ├── sensor/cloud_ingest.hpp
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
