# Sapphire

LiDAR-inertial odometry. Deskew and IMU propagation use the same Gal(3) preintegration. The library is C++17 and has no ROS dependency.

[中文说明](README.zh.md)

## Demos

**S3E**

![S3E RViz mapping](docs/sapphire.gif)

**HILTI 2022**

![HILTI 2022 construction stairs RViz mapping](docs/hilti-2022.gif)

**HILTI 2023**

![HILTI 2023 RViz mapping](docs/hilti-2023.gif)

**Newer College 2020**

![Newer College RViz mapping](docs/newer_college.gif)

## Dependencies

| Package | Version |
|---|---|
| CMake | 3.16+ |
| Compiler | C++17 |
| Eigen | 3.4 |
| PCL | 1.12+ |
| OpenMP | |
| GTSAM | 4.3 |
| small_gicp | 1.0.1 |
| spdlog | |
| toml++ | 3.4, downloaded automatically if missing |

For ROS, use the Dockerfile in the wrapper repository. Both images already contain Eigen, PCL, OpenMP, spdlog, GTSAM, and small_gicp. The Noetic image also installs toml++ 3.4. The Humble image does not, so CMake downloads it when it is missing.

- [sapphire_ros1](https://github.com/wagdeer/sapphire_ros1): ROS 1 Noetic, `docker/Dockerfile`
- [sapphire_ros2](https://github.com/wagdeer/sapphire_ros2): ROS 2 Humble, `docker/Dockerfile`

To build only the core on Ubuntu 22.04:

```bash
sudo apt update
sudo apt install -y build-essential cmake git software-properties-common \
  libeigen3-dev libpcl-dev libomp-dev libspdlog-dev
sudo add-apt-repository -y ppa:borglab/gtsam-release-4.3
sudo apt install -y libgtsam-dev
```

Those packages are Eigen 3.4, PCL 1.12, OpenMP, spdlog, and GTSAM 4.3.

small_gicp 1.0.1:

```bash
sudo rm -rf /tmp/small_gicp
git clone --depth 1 --branch v1.0.1 https://github.com/koide3/small_gicp.git /tmp/small_gicp
cmake -S /tmp/small_gicp -B /tmp/small_gicp/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_BENCHMARKS=OFF \
  -DBUILD_WITH_MARCH_NATIVE=OFF
cmake --build /tmp/small_gicp/build -j"$(nproc)"
sudo cmake --install /tmp/small_gicp/build
```

## Build

```bash
git clone https://github.com/wagdeer/sapphire.git
cd sapphire
cmake -S . -B build
cmake --build build -j"$(nproc)"
```

Tests:

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j"$(nproc)"
cmake --build build --target test
```

## Install

```bash
cmake --install build --prefix /opt/sapphire
```

Add `/opt/sapphire` to `CMAKE_PREFIX_PATH`, then `find_package(sapphire REQUIRED)` and link `sapphire`.

## ROS

Put the core next to the wrapper. The wrapper compiles that `sapphire` directory.

```text
ws/src/
├── sapphire/
└── sapphire_ros1/    or sapphire_ros2/
```

Sensor profiles are in `cfg/`. Replaying a recorded dataset is described in [docs/DATASET_REPLAY.md](docs/DATASET_REPLAY.md).

## Credits

- [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry): pipeline and submap design
- [Equivariant IMU Preintegration with Biases](https://github.com/aau-cns/equivariant-preintegration): Gal(3) preintegration
- [small_gicp](https://github.com/koide3/small_gicp): GICP and VGICP
- [GTSAM](https://github.com/borglab/gtsam): pose-graph optimization

## License

MIT.
