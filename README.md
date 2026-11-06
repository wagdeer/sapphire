# Sapphire

LiDAR-inertial odometry with Gal(3) equivariant IMU preintegration.

[中文说明](README.zh.md)

Sapphire estimates a 6-DoF trajectory from LiDAR and IMU. Scan deskew and IMU
propagation share one Gal(3) motion model, and an asynchronous pose graph
keeps the map consistent. The core is standalone C++17. ROS 1 and ROS 2 are
thin wrappers.

## Demos

**S3E**

![S3E RViz mapping](docs/sapphire.gif)

**HILTI 2022**

![HILTI 2022 construction stairs RViz mapping](docs/hilti-2022.gif)

**HILTI 2023**

![HILTI 2023 RViz mapping](docs/hilti-2023.gif)

**Newer College 2020**

![Newer College RViz mapping](docs/newer_college.gif)

## Benchmarks

Each number below is the unweighted mean or median of per-run translation ATE:
RMSE after SE(3) alignment, with no scale correction. Runs are not weighted by
distance.

| Dataset | Runs in the mean | What the coverage column counts | Mean ATE | Median ATE |
|---|---:|---|---:|---:|
| HILTI 2022 · complete results | 9 | 86 matched control points | 0.2257 m | 0.0615 m |
| HILTI 2023 Site 1 handheld | 5 | 346.4 m of control-point polylines | 0.0683 m | 0.0617 m |
| S3E | 18 | 11.39 km of ground-truth path | 1.9394 m | 0.9808 m |
| Newer College 2020 | 5 | 5.94 km of ground-truth path | 0.2632 m | 0.1587 m |

HILTI uses sparse control points and a 2 s association window. S3E and Newer
College use a 0.1 s window. Partial runs, sequences that were not replayed,
and runs withheld from the mean are listed on the benchmark page and are not
folded into this table.

The HILTI 2022 row contains the 9 runs with complete sparse-control coverage
in the current replay. `exp03`, `exp10`, and `exp16` remained partial and are
reported separately on the benchmark page rather than folded into the mean.
[Per-sequence results and trajectory plots](docs/benchmarks/benchmark_table.md).

## Build

```bash
git clone https://github.com/wagdeer/sapphire.git
cd sapphire
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j"$(nproc)"
cmake --build build --target test
```

Requires CMake 3.16 or newer, a C++17 compiler, Eigen 3.4, PCL 1.11 or newer,
OpenMP, GTSAM 4.3, small_gicp, and spdlog. Configuring PCL also requires an
MPI C library, because VTK's imported target references `MPI::MPI_C`. Sapphire
itself does not call MPI. toml++ 3.4 is used when it is already installed;
otherwise CMake downloads that release. The test target above works on CMake
3.16; `ctest --test-dir` needs CMake 3.20.

## Install

```bash
cmake --install build --prefix /opt/sapphire
```

A downstream project finds the static library with
`find_package(sapphire REQUIRED)` and links the `sapphire` target. Put the
prefix on `CMAKE_PREFIX_PATH`. The consumer also needs Eigen, PCL, GTSAM,
OpenMP, small_gicp, spdlog, toml++ 3.4, and an MPI C library; this package
does not ship them. toml++ downloaded while building Sapphire is not part of
the install. The ROS wrappers below use a sibling checkout and
`add_subdirectory`, and do not need this prefix.

## ROS

ROS 1 and ROS 2 are separate packages. Put the core beside the wrapper you
need:

```text
ws/src/
├── sapphire/
└── sapphire_ros2/    or sapphire_ros1/
```

ROS 2 Humble, from the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sapphire_ros2
source install/setup.bash

ros2 launch sapphire_ros2 sapphire.launch.py \
  config_file:=/absolute/path/to/sapphire/cfg/sapphire_mid360.toml \
  lidar_topic:=/your_lidar \
  imu_topic:=/your_imu
```

`config_file` is required. If you omit the topic arguments, the launch file
subscribes to `/front_lidar` and `/front_lidar/imu`. The node always publishes
`/sapphire/odometry` and `/sapphire/deskewed`. `/sapphire/trajectory`,
`/sapphire/pgo/map`, and `/sapphire/pgo/graph` are published only when the
profile sets `pgo.enabled` to true.

Dependency installation, Docker, and bag playback are in
[`sapphire_ros2`](https://github.com/wagdeer/sapphire_ros2) and
[`sapphire_ros1`](https://github.com/wagdeer/sapphire_ros1). Dataset profiles
live in `cfg/`. A replay walkthrough is in
[docs/DATASET_REPLAY.md](docs/DATASET_REPLAY.md).

## Documentation

| Document | What it is |
|---|---|
| [docs/benchmarks/benchmark_table.md](docs/benchmarks/benchmark_table.md) | Per-run ATE, including runs left out of the table above |
| [docs/DATASET_REPLAY.md](docs/DATASET_REPLAY.md) | How to replay a bag and write a TUM trajectory |

## Credits

- [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry): pipeline and submap design
- [Equivariant IMU Preintegration with Biases](https://github.com/aau-cns/equivariant-preintegration): Gal(3) preintegration
- [small_gicp](https://github.com/koide3/small_gicp): GICP and VGICP
- [GTSAM](https://github.com/borglab/gtsam): incremental pose-graph optimization

## License

MIT.
