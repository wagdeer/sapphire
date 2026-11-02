# Sapphire

基于 Gal(3) 等变 IMU 预积分的 LiDAR-惯性里程计。

[English](README.md)

Sapphire 是一个无特征、scan-to-submap 的 LIO 管线。核心设计选择是用同一套
Gal(3) 状态恢复模型同时做逐点去畸变和 IMU 频率状态传播——位姿和速度来自同一
个预积分状态，而不是两套独立的运动模型。

核心库是 C++17 静态库，不依赖 ROS。ROS 2 支持在独立的
[`sapphire_ros2`](https://github.com/wagdeer/sapphire_ros2) 封装仓库中。

## 基准结果

在 6 个稳健的 S3E 场景中，17 条轨迹覆盖 **10.42 km** 真值路径，平均 ATE
为 **1.17 m**，中位数为 **1.05 m**（按轨迹等权统计）。HILTI 保留 5 条主要
稀疏 ATE 结果，覆盖 **346.4 m** 控制点参考路径，平均 ATE 为 **0.058 m**，
中位数为 **0.052 m**。
Newer College 的 5 个实验全部完成，覆盖 **5.94 km** 连续真值路径，平均 ATE
为 **0.285 m**，中位数为 **0.182 m**。

ATE 为 SE(3) Umeyama 对齐后的平移 APE，不做尺度修正。S3E 三个机器人分别
列出，不合并计算。GT 长度是参考路径长度，估计轨迹长度是完整 TUM 轨迹长度；
Laboratory 的 GT 只有首尾位置，只能计算端点误差，因此不列入本 ATE 主表。

| 数据集 | 序列 | 机器人 | GT 长度 (m) | 估计轨迹长度 (m) | 指标 | 结果 (m)
|---|---|---|---:|---:|---|---:
| HILTI | Site 1 · Handheld 1 | reference | 81.32 | 205.5 | 稀疏 ATE | 0.030
| HILTI | Site 1 · Handheld 2 | reference | 84.39 | 163.9 | 稀疏 ATE | 0.052
| HILTI | Site 1 · Handheld 3 | reference | 81.89 | 165.2 | 稀疏 ATE | 0.108
| HILTI | Site 1 · Handheld 4 | reference | 52.88 | 300.0 | 稀疏 ATE | 0.053
| HILTI | Site 1 · Handheld 5 | reference | 45.94 | 150.1 | 稀疏 ATE | 0.045
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

以下异常结果未进入主比较：HILTI Site 2 中误差异常、GT 退化或控制点不足的
序列；S3E Campus Road 3 的 Bob 离群结果；S3E Teaching Building 1 和 Tunnel 1
的 GT/路径异常。Laboratory 1~4 的 GT 只能计算端点误差，不能与连续 ATE 混算，
因此不列入主表。

### 数据集可视化

| HILTI 2023 · 手持设备建图 | Newer College 2020 · 户外建图 |
|---|---|
| ![HILTI 2023 RViz 建图](docs/hilti-2023.gif) | ![Newer College RViz 建图](docs/newer_college.gif) |

### 轨迹与真值对比

每张图片对应一条结果。实线为 Sapphire，虚线为 GT，叉号表示 HILTI 控制点。

| 轨迹结果 | 轨迹结果 |
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

## 管线

```text
IMU
  -> 静止初始化 (MAD 离群值剔除 + 方差收敛)
  -> bias 校正与时间戳校验
  -> 定长 IMU 缓冲
  -> Gal(3) 状态传播

LiDAR
  -> NaN/裁剪框过滤
  -> Gal(3) 逐点去畸变
  -> 体素降采样
  -> GICP 或 VGICP scan-to-submap 配准
  -> 几何观测器与在线 bias 修正
  -> 前端关键帧子图
  -> 异步 ISAM2 位姿图与回环检测
```

对于一个积分区间，Sapphire 按以下方式恢复 Gal(3) 状态：

```text
Xi(t) = Gamma_ij(t) * Xi_ref * Upsilon_ij(t)
```

`Gamma_ij` 包含重力/时间演化，`Upsilon_ij` 是预积分的局部 IMU 增量。位姿和
速度从恢复后的 `Xi(t)` 中提取。vendored 实现位于 `external/preintegration/`
下，基于 Fornasier 等人的 `aau-cns/equivariant-preintegration`。

## 功能

- 静止 IMU 初始化，可配置收敛判据和超时
- IMU 输入的非有限值、非单调、大间隔诊断
- Gal(3) 去畸变与传播，含在线加速度计/陀螺仪 bias 修正
- 定制并行体素滤波器
- 通过 `small_gicp` 实现的 GICP 和 VGICP 配准
- 距离/旋转门控的前端子图
- 异步 GTSAM ISAM2 位姿图，带 ICP 回环验证
- 带校验的 TOML 配置
- 逐帧耗时、配准、去畸变和关键帧诊断
- 11 个 CTest 目标，覆盖配置、传感器摄取、去畸变、预积分、管线、观测器、
  配准、子图、位姿图、体素滤波和 IMU 环形缓冲

前端在局部 odometry 坐标系中运行。位姿图后端维护独立的 `T_map_odom` 修正量，
组合到发布位姿上，不往回改写前端的 deskew 或 observer 状态。

## 依赖

- CMake 3.16+
- C++17 编译器
- Eigen3
- spdlog
- PCL
- OpenMP
- GTSAM
- small_gicp
- tomlplusplus 3.4+（无系统包时由 CMake 自动下载）

ROS 封装额外需要 ROS 2、`rclcpp`、`sensor_msgs`、`nav_msgs`、
`visualization_msgs`、`tf2_ros`、`tf2_eigen` 和 `pcl_ros`。

GTSAM 和 small_gicp 的安装方式因平台而异。安装能导出 CMake package
configuration、可被 `find_package` 发现的版本后，再配置 Sapphire。

## 编译与测试核心库

```bash
git clone https://github.com/wagdeer/sapphire.git
cd sapphire

cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

## 配合 ROS 2 编译

将核心库和封装库放在同级目录下。封装库会自动检测核心库并加入构建。

```bash
mkdir -p ~/sapphire_ws/src
cd ~/sapphire_ws/src
git clone https://github.com/wagdeer/sapphire.git
git clone https://github.com/wagdeer/sapphire_ros2.git

cd ~/sapphire_ws
export ROS_DISTRO=humble  # 替换为你安装的 ROS 2 发行版。
source "/opt/ros/${ROS_DISTRO}/setup.bash"
colcon build --symlink-install --packages-select sapphire_ros2
colcon test --packages-select sapphire_ros2
colcon test-result --all
source install/setup.bash
```

## Docker

封装仓库包含参数化的开发镜像。从 `sapphire_ros2` 仓库根目录构建，用
`ROS_DISTRO` 选择 ROS 发行版：

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

默认基础镜像变体是 `desktop-full`。也可以用
`--build-arg ROS_IMAGE_VARIANT=desktop-full` 显式指定。所选组合必须作为
`osrf/ros` 镜像存在，且 ROS 和 GTSAM 仓库必须为其 Ubuntu 版本提供包。

将工作空间挂载到容器中：

```bash
docker run -it --rm --network host \
  -v ~/sapphire_ws:/workspace \
  sapphire:humble

colcon build --symlink-install --packages-select sapphire_ros2
source install/setup.bash
```

## 运行

默认 launch 配置面向 Livox Mid-360：

```bash
ros2 launch sapphire_ros2 sapphire.launch.py
```

常用参数覆盖：

```bash
ros2 launch sapphire_ros2 sapphire.launch.py \
  lidar_topic:=/points \
  imu_topic:=/imu/data \
  odom_topic:=/sapphire/odometry \
  config_file:=/absolute/path/to/sapphire_mid360.toml
```

Launch 默认值：

- LiDAR 输入：`/livox/lidar`
- IMU 输入：`/livox/imu`
- 里程计输出：`/sapphire/odometry`
- 父/子坐标系：`map` 和 `lidar`

发布的接口：

- `/sapphire/odometry` — `nav_msgs/msg/Odometry`，通常以 IMU 频率发布
- `/sapphire/deskewed` — `sensor_msgs/msg/PointCloud2`，LiDAR 频率
- `/sapphire/pgo/map` — 订阅者驱动的位姿图地图
- `/sapphire/pgo/graph` — 订阅者驱动的位姿图标记
- TF — 默认 `map -> lidar`；坐标系名称可配置

## 传感器使用前提

使用新传感器或数据集前，确认以下事项：

1. IMU 初始化期间保持平台静止。
2. LiDAR 和 IMU 时间戳来自同一时钟域且单调递增。
3. 加速度可以是 g 或 m/s²。静止模长接近 1 时，Sapphire 会把它换成 m/s²。
   只有这次判断失败时才写 `[sensor.imu] accel_unit` 为 `g` 或 `m_s2`。
   角速度保持 rad/s。
4. 进入 pipeline 的点时间是绝对秒，和 IMU 同一个钟。ROS 适配器负责分类；
   只有分类失败时才需要写 `[sensor.lidar]`。
5. 正确设置 `T_imu_lidar`。它将 LiDAR 系点映射到 IMU 系：

   ```text
   p_imu = R_imu_lidar * p_lidar + t_imu_lidar
   ```

6. 确认配置的裁剪框不会移除传感器安装位置附近的有效几何结构。

IMU 缓冲是定长的。如果 LiDAR 处理停滞时间足够长以至于所需历史数据被覆盖，
Sapphire 会拒绝该帧而非用不完整的 IMU 覆盖去做去畸变。

## 配置

默认配置为 `cfg/sapphire_mid360.toml`：

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

配置在启动时校验。修改后重启生效，无需重新编译。

## 仓库结构

```text
sapphire/
├── cfg/                         默认传感器配置
├── docs/                        设计与审查文档
├── external/                    Vendored 预积分与 Lie 群库
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
├── src/                         核心实现
└── tests/                       单元与集成式测试
```

## 参考与致谢

- [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry)：
  管线、观测器和子图设计参考
- [Equivariant IMU Preintegration with Biases](https://github.com/aau-cns/equivariant-preintegration)：
  Gal(3) 预积分理论与实现
- [small_gicp](https://github.com/koide3/small_gicp)：GICP 和 VGICP
- [GTSAM](https://github.com/borglab/gtsam)：增量位姿图优化

## 许可证

MIT。
