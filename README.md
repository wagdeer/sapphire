# Sapphire

LiDAR-inertial odometry using Gal(3) equivariant IMU preintegration.

[中文说明](README.zh.md) · [Design](docs/DESIGN.md) ·
[Dataset replay](docs/DATASET_REPLAY.md)

Sapphire estimates a 6-DoF LiDAR-inertial trajectory, removes scan distortion
with the same Gal(3) motion model used for IMU propagation, and builds a
globally consistent map through asynchronous pose-graph optimization. The core
is standalone C++17; ROS 1 and ROS 2 are provided by thin wrappers.

## Demos

| S3E | HILTI 2023 | Newer College 2020 |
|---|---|---|
| ![S3E RViz mapping](docs/sapphire.gif) | ![HILTI 2023 RViz mapping](docs/hilti-2023.gif) | ![Newer College RViz mapping](docs/newer_college.gif) |

## Benchmark snapshot

| Dataset | Valid runs | GT coverage | Mean ATE | Median ATE |
|---|---:|---:|---:|---:|
| HILTI 2023 | 5 | 346.4 m control path | 0.058 m | 0.052 m |
| S3E | 17 | 10.42 km | 1.17 m | 1.05 m |
| Newer College 2020 | 5 | 5.94 km | 0.285 m | 0.182 m |

ATE is translation RMSE after timestamp association (≤0.1 s) and SE(3)
Umeyama alignment, without scale correction. HILTI uses sparse control-point
GT; S3E and Newer College use continuous GT. Known data/GT anomalies are
excluded from the summary and retained in the detailed table.

Reproduction:

1. Build the core and ROS wrapper with the commands below.
2. Replay a sequence at 1× and export its TUM trajectory using
   [DATASET_REPLAY.md](docs/DATASET_REPLAY.md).
3. Associate estimate and GT timestamps, apply SE(3) alignment without scale,
   and compute translation RMSE.

[Full per-sequence results](docs/benchmarks/benchmark_table.md) ·
[All trajectory plots](docs/benchmarks/trajectory_gallery.md)

## Build

```bash
git clone https://github.com/wagdeer/sapphire.git
cd sapphire
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

The core requires CMake, a C++17 compiler, Eigen3, PCL, OpenMP, GTSAM,
small_gicp, and spdlog. The ROS wrapper additionally requires the matching
ROS 2 toolchain.

## Run

Keep `sapphire` and
[`sapphire_ros2`](https://github.com/wagdeer/sapphire_ros2) as sibling
repositories, then build the wrapper:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sapphire_ros2
source install/setup.bash
```

Launch with a dataset-specific TOML profile:

```bash
ros2 launch sapphire_ros2 sapphire.launch.py \
  config_file:=/absolute/path/to/profile.toml \
  lidar_topic:=/points \
  imu_topic:=/imu
```

Published topics include `/sapphire/odometry`, `/sapphire/deskewed`,
`/sapphire/trajectory`, and `/sapphire/pgo/map`.

## Documentation

- [Architecture and design](docs/DESIGN.md)
- [Dataset replay and visualization](docs/DATASET_REPLAY.md)
- [Detailed benchmark table](docs/benchmarks/benchmark_table.md)
- [Trajectory overlays](docs/benchmarks/trajectory_gallery.md)

## Credits

- [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry):
  pipeline and submap design reference
- [Equivariant IMU Preintegration with Biases](https://github.com/aau-cns/equivariant-preintegration):
  Gal(3) preintegration theory and implementation
- [small_gicp](https://github.com/koide3/small_gicp): GICP and VGICP
- [GTSAM](https://github.com/borglab/gtsam): incremental pose-graph
  optimization

## License

MIT.
