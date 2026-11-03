# Sapphire

基于 Gal(3) 等变 IMU 预积分的 LiDAR-惯性里程计。

[English](README.md) · [设计文档](docs/DESIGN.md) ·
[数据回放](docs/DATASET_REPLAY.md)

Sapphire 从 LiDAR 与 IMU 输入估计 6-DoF 轨迹，用同一套 Gal(3) 运动模型完成
逐点去畸变和 IMU 传播，并通过异步位姿图优化构建全局一致地图。核心库为独立
C++17 实现，ROS 1 和 ROS 2 由轻量封装接入。

## 演示

| S3E | HILTI 2023 | Newer College 2020 |
|---|---|---|
| ![S3E RViz 建图](docs/sapphire.gif) | ![HILTI 2023 RViz 建图](docs/hilti-2023.gif) | ![Newer College RViz 建图](docs/newer_college.gif) |

## 基准快照

| 数据集 | 纳入轨迹 | 纳入 GT 覆盖 | 平均 ATE | ATE 中位数 |
|---|---:|---:|---:|---:|
| HILTI 2023 | 5 | 346.4 m 控制点路径 | 0.058 m | 0.052 m |
| S3E | 17 | 10.42 km | 1.17 m | 1.05 m |
| Newer College 2020 | 3 | 1.27 km | 0.143 m | 0.148 m |

ATE 是完成时间关联（误差不超过 0.1 s）和 SE(3) Umeyama 对齐后的平移 RMSE，
不做尺度修正。HILTI 使用稀疏控制点 GT；S3E 与 Newer College 使用连续 GT。
已知数据或 GT 异常的结果不进入摘要，但保留在详细表中。Newer College 另外
剔除两条约 0.5 m 的较长轨迹；保留的三条 ATE 为 0.10~0.18 m。

复现步骤：

1. 按下方命令编译核心库和 ROS 封装。
2. 按 [数据回放说明](docs/DATASET_REPLAY.md) 以 1× 回放并导出 TUM 轨迹。
3. 对估计与 GT 做时间关联，执行无尺度修正的 SE(3) 对齐，并计算平移 RMSE。

[完整逐序列结果与轨迹图](docs/benchmarks/benchmark_table.zh.md)

## 编译

```bash
git clone https://github.com/wagdeer/sapphire.git
cd sapphire
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

核心库需要 CMake、C++17 编译器、Eigen3、PCL、OpenMP、GTSAM、small_gicp
和 spdlog；ROS 封装还需要对应的 ROS 2 工具链。

## 运行

将 `sapphire` 与
[`sapphire_ros2`](https://github.com/wagdeer/sapphire_ros2) 放在同级目录，
然后编译封装：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sapphire_ros2
source install/setup.bash
```

使用数据集对应的 TOML 配置启动：

```bash
ros2 launch sapphire_ros2 sapphire.launch.py \
  config_file:=/absolute/path/to/profile.toml \
  lidar_topic:=/points \
  imu_topic:=/imu
```

主要输出话题为 `/sapphire/odometry`、`/sapphire/deskewed`、
`/sapphire/trajectory` 和 `/sapphire/pgo/map`。

## 文档

- [架构与设计](docs/DESIGN.md)
- [数据回放与可视化](docs/DATASET_REPLAY.md)
- [完整基准结果与轨迹图](docs/benchmarks/benchmark_table.zh.md)

## 致谢

- [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry)：
  管线与子图设计参考
- [Equivariant IMU Preintegration with Biases](https://github.com/aau-cns/equivariant-preintegration)：
  Gal(3) 预积分理论与实现
- [small_gicp](https://github.com/koide3/small_gicp)：GICP 和 VGICP
- [GTSAM](https://github.com/borglab/gtsam)：增量位姿图优化

## License

MIT.
