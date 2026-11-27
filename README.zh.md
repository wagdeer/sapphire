# Sapphire

激光惯性 SLAM。前端采用点到平面匹配与滚动体素地图，后端异步执行子图 BA 和位姿图优化。核心库使用 C++17，无 ROS 依赖。

[English](README.md)

## 依赖

| 依赖 | 版本 |
|---|---|
| CMake | 3.16+ |
| 编译器 | C++17 |
| TBB | |
| Eigen | 3.4 |
| PCL | 1.12+ |
| OpenMP | |
| GTSAM | 4.3 |
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
  libeigen3-dev libpcl-dev libomp-dev libspdlog-dev libtbb-dev
sudo add-apt-repository -y ppa:borglab/gtsam-release-4.3
sudo apt install -y libgtsam-dev
```

这几个包对应 Eigen 3.4、PCL 1.12、OpenMP、spdlog 和 GTSAM 4.3。

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
git clone --branch method_b https://github.com/wagdeer/sapphire.git
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

传感器配置在 `cfg/cfg_b/`。公开数据集的回放步骤见 [docs/DATASET_REPLAY.md](docs/DATASET_REPLAY.md)。

## 致谢

- [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry)：管线与子图
- [small_gicp](https://github.com/koide3/small_gicp)：近邻索引和并行排序
- [GTSAM](https://github.com/borglab/gtsam)：位姿图优化

## 许可证

见[组件许可证说明](licenses/NOTICE.md)。

## Method B

[Method B 独立版变更说明](docs/METHOD_B_CHANGES.md)

`methodB` 沿用 `sapphire::OdometryPipeline` 的 `pushImu`、`pushLidar` 和地图查询入口。点到平面关联使用前端的滚动八叉体素地图和迭代滤波。建图线程用激光惯性 BA 优化子图内部，压缩已通过边界检查的子图，并用 iSAM2 求解回环。重访时，前端可以读取后台发布的已验证历史平面，继续更新当前地图。

用 `cfg/cfg_b/nclt.toml` 载入 NCLT 的传感器外参和 Method B 参数。每次从空图启动时使用一个新的 `io.map_directory`，该目录保存子图和检索数据。前端用 `odometry.crop_box` 裁剪车体，可用 `sensor.lidar.max_range_m` 设置远距上限。子图打包的取点范围由 `submap.max_range_m` 设置。停止送入传感器数据后，先请求地图和图快照，再调用 `finishMapping()` 封存最后一组关键帧，随后读取已请求的结果及 `mapTrajectory()`。

A/B 使用 `main` 和 `method_b` 各自编译的程序，回放同一组雷达和 IMU 输入。评分方法见[回放说明](benchmarks/README.md)，历史测试结果见[对比表](benchmarks/method_b.md)。
