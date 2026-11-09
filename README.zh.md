# Sapphire

激光-惯性里程计。点云去畸变和 IMU 传播使用同一套 Gal(3) 预积分。核心库是 C++17，不依赖 ROS。

[English](README.md)

## 演示

**S3E**

![S3E RViz 建图](docs/sapphire.gif)

**HILTI 2022**

![HILTI 2022 施工楼梯 RViz 建图](docs/hilti-2022.gif)

**HILTI 2023**

![HILTI 2023 RViz 建图](docs/hilti-2023.gif)

**Newer College 2020**

![Newer College RViz 建图](docs/newer_college.gif)

## 依赖

| 依赖 | 版本 |
|---|---|
| CMake | 3.16+ |
| 编译器 | C++17 |
| Eigen | 3.4 |
| PCL | 1.12+ |
| OpenMP | |
| GTSAM | 4.2 |
| small_gicp | 1.0.1 |
| spdlog | |
| toml++ | 3.4，没有会自动下载 |

用 ROS 时，直接用封装仓库里的 Dockerfile。两个镜像里都有 Eigen、PCL、OpenMP、spdlog、GTSAM 和 small_gicp。Noetic 镜像还会装上 toml++ 3.4。Humble 镜像没有，缺的时候由 CMake 下载。

- [sapphire_ros1](https://github.com/wagdeer/sapphire_ros1)：ROS 1 Noetic，`docker/Dockerfile`
- [sapphire_ros2](https://github.com/wagdeer/sapphire_ros2)：ROS 2 Humble，`docker/Dockerfile`

只在 Ubuntu 22.04 上编译核心库：

```bash
sudo apt update
sudo apt install -y build-essential cmake git software-properties-common \
  libeigen3-dev libpcl-dev libomp-dev libspdlog-dev
sudo add-apt-repository -y ppa:borglab/gtsam-release-4.2
sudo apt install -y libgtsam-dev
```

这几个包对应 Eigen 3.4、PCL 1.12、OpenMP、spdlog 和 GTSAM 4.2。

small_gicp 1.0.1：

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

## 编译

```bash
git clone https://github.com/wagdeer/sapphire.git
cd sapphire
cmake -S . -B build
cmake --build build -j"$(nproc)"
```

跑测试：

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j"$(nproc)"
cmake --build build --target test
```

## 安装

```bash
cmake --install build --prefix /opt/sapphire
```

把 `/opt/sapphire` 加到 `CMAKE_PREFIX_PATH`，然后 `find_package(sapphire REQUIRED)` 并链接 `sapphire`。

## ROS

把核心和封装放在同一个工作空间。封装会直接编译旁边的 `sapphire`。

```text
ws/src/
├── sapphire/
└── sapphire_ros1/    或 sapphire_ros2/
```

传感器配置在 `cfg/`。公开数据集的回放步骤见 [docs/DATASET_REPLAY.md](docs/DATASET_REPLAY.md)。

## 致谢

- [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry)：管线与子图
- [Equivariant IMU Preintegration with Biases](https://github.com/aau-cns/equivariant-preintegration)：Gal(3) 预积分
- [small_gicp](https://github.com/koide3/small_gicp)：GICP 与 VGICP
- [GTSAM](https://github.com/borglab/gtsam)：位姿图优化

## 许可证

MIT.
