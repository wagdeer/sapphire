# Sapphire LiDAR SLAM

`core/` 是独立算法库，入口为 `core/CMakeLists.txt`，产物为 `sapphire_core.so`。
`ros2_warpper/` 是 ROS 2 输入输出适配和可执行程序，链接 core，不生成自定义消息。
里程计和建图通过进程内有界队列交接冻结的 Gaussian 点云，core 不依赖 ROS。
对外入口为 `slam.hpp`：`SlamPipeline::Callbacks` 接收结果和运行通知，`SlamPipeline::ContinuationProgress` 表示处理进度；里程计与建图流程位于 `core/include/pipeline/` 和 `core/src/pipeline/`，配置和回调由 `SlamPipeline` 持有。
回调在工作线程同步执行，不同线程的回调可能并发；状态引用仅在调用期间有效，点云可通过共享只读指针保留。回调应尽快返回，不应调用阻塞式停止、等待或关闭接口。
`Memory` 负责节点驻留与数据库交接，图和位姿由 `Topograph` 持有；`Statistics` 只保存标量、时间和节点 ID。
`MappingParameters` 在配置入口解析完成，运行模块直接使用结构体。参数字符串仅用于配置边界和数据库历史记录；历史记录不会覆盖启动参数。
统计日志目录由 `Mapping::init()` 的目录参数指定，未指定时使用数据库所在目录；不再读取 `PDR_WORKING_DIRECTORY`。
建图调参使用 `[mapping.*]`，原 `[backend.*]` 需改名；无消费者的 `compression.detect.*`、`compression.merge.*` 已移除。
位姿群使用 `SE3f` / `SE3d`，IMU 状态仍为 `StateGroup`；原 `backend` 和 `backend::io` 命名空间已并入 `sapphire`。
`captureReady()` 返回与位姿、修正同时读取的 `ContinuationProgress`，其中 `map_id`、`generation`、`ready_revision`、`last_completed` 用于识别地图和输出版本。

雷达点定义位于 `sensor/lidar_type.hpp`；ROS 只解码字段、偏移和字节序。原生双雷达入口为 `sensor/lidar_sync.hpp` 的 `LidarSync::push()` / `pop()`，输入是拥有数据的 `TimedLidarPoint` 数组，保留绝对时间戳和双精度解码值。时间窗口、过滤和后雷达到参考雷达的变换均在 core；单路滤波使用 `LidarHandler`。
`common/point.hpp` 保留算法中点的协方差、聚合和操作语义；通用 `rtree.hpp` 保留。
子图由 `NodeBuilder` 按行驶距离构建，最多保留 256 帧轨迹后切分，输入结束时提交尾部。Gaussian 数量没有人为硬上限，也不按点数截断或切分。归档保持原格式，仅校验几何、实际长度和矩阵格式可表示范围；解压按实际产出增长内存。`NodeBuilder` 直接构造拥有几何、锚点和时间戳的 `Node`，以独占所有权通过 `Mapping::process_node()` 交给 `Memory::admit_node()`；接纳时分配地图身份，过程中不重新构造节点。构建结果另携带锚点之后的轨迹样本，提交序号由运行流程维护。续建匹配拒绝时原节点保持未接纳状态，等待重试。

`tools/parallel_executor.hpp` 使用 Pandora 当前的通用线程池，体素地图和局部优化共用前端持有的实例。
局部优化将 IMU 与雷达分块交给 `parallel_invoke`，全部结束后按固定分块顺序归并；不再保留独立的 range executor。
计时统一使用 `tools::Timer`，诊断日志通过 `SAPPHIRE_PROFILE=1` 启用。

## 构建、测试和安装

以下命令从工作区目录执行（本机为 `/home/user/code/sapphire_git`）。
需要 C++17、CMake、CUDA Toolkit、Eigen、Boost、OpenCV core、small_gicp、
spdlog、toml++、SQLite、zlib；测试脚本使用 Python 3 和 NumPy。
GTSAM 最小源码集合及 3D-BBS 从 `core/thirdparty/` 编译。
CUDA 架构默认 89，可用 `-DCMAKE_CUDA_ARCHITECTURES=87` 指定 Orin NX；编译成功不代表实机验收。

只构建 core，无需 source ROS：

```bash
cmake -S src/core -B build/core -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON -DCMAKE_INSTALL_PREFIX="$PWD/install/core"
cmake --build build/core -j4
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 ctest --test-dir build/core --output-on-failure
cmake --install build/core
```

构建 ROS 程序和全部常规测试（自动编译 core）：

```bash
source /opt/ros/humble/setup.bash
cmake -S src/ros2_warpper -B build/ros2 -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON -DCMAKE_INSTALL_PREFIX="$PWD/install/ros2"
cmake --build build/ros2 -j4
cmake --install build/ros2
source install/ros2/share/sapphire_ros2/local_setup.bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=217 \
  ctest --test-dir build/ros2 --output-on-failure
ros2 launch sapphire_ros2 launch_sapphire.py
```

也可用 `colcon build --base-paths src/ros2_warpper --packages-select sapphire_ros2`。
目录移动后使用新的构建目录；不要继续沿用指向已删除源码目录的 CMake 缓存。
`SAPPHIRE_ENABLE_LTO` 默认 ON。`SAPPHIRE_BUILD_G3REG_EXPERIMENTS` 默认 OFF，
仅用于隔离的研究目标，不是 SLAM 运行依赖。

安装后的原生库调用验证，不链接 ROS：

```bash
cmake -S src/tests/package_consumer -B build/consumer \
  -DSAPPHIRE_INSTALL_PREFIX="$PWD/install/core"
cmake --build build/consumer -j2
ctest --test-dir build/consumer --output-on-failure
```

## 配置和输入

地图保存到 `map.database_path`；未指定时使用 `general.save_path` 下的会话目录。
旧 `general.save_map` 开关已删除，关闭 `pose_graph.enabled` 时不创建地图数据库或空会话目录。

只有三份运行配置：

- `core/config/sensors.toml`：安装、话题、型号、外参、预处理和同步。
- `core/config/params.toml`：初始化、里程计、子图、配准、图优化和存储参数。
- `ros2_warpper/config/ros.yaml`：ROS 输出话题、坐标系、显示开关和 QoS。

`lidar0` 是参考雷达，增加 `lidar1` 自动启用双雷达，无独立 single/dual 开关。
型号数字：0=Livox、1=RoboSense/Airy、2=Hesai XT、3=Velodyne，也接受对应名称。
单雷达支持这四种格式，双雷达当前只支持两路 RoboSense/Airy。
默认文件是原 Mid360 标定；其他安装必须填写对应外参。

两路外参都将雷达坐标转到同一个 IMU：`p_imu = R * p_lidar + t`。
平移单位米，旋转按行排列。程序计算
`T_lidar0_lidar1 = inverse(T_imu_lidar0) * T_imu_lidar1`。
每路点/扫描时间为设备时间加 `time_offset` 秒，IMU 是共同时间基准。
未知参数、无效旋转、缺失外参、重复话题和不支持的双雷达型号会拒绝启动。

```bash
ros2 launch sapphire_ros2 launch_sapphire.py \
  sensor_config:=/absolute/path/sensors.toml \
  algorithm_config:=/absolute/path/params.toml rviz:=false
```

`ros_config` 可指定 ROS YAML。
ROS 订阅 PointCloud2 和 Imu；没有 Livox CustomMsg 订阅。

| 类型 | PointCloud2 逐点时间 |
| --- | --- |
| Livox | `offset_time: UINT32`，相对 header 的纳秒；保留旧 `timestamp/time` 相对纳秒输入 |
| RoboSense / Airy | `timestamp: FLOAT64`，绝对秒 |
| Hesai XT | `timestamp: FLOAT64`，绝对秒 |
| Velodyne | `time: FLOAT32`，相对 header 的秒，可为负值 |

除旧 Livox 时间输入外，扫描边界由全部原始点的时间范围确定，然后再抽点、过滤。
解码检查字段、大小端和行填充。非有限时间、超长扫描或过滤后空扫描会拒收。
双雷达默认按 10 Hz 的 `[begin,end)` 时间窗融合；缺少覆盖时丢窗，不静默退化为单雷达。
`sync.lidar_merge_hz` 只控制软件时间窗，不修改硬件转速；0 表示使用 lidar0 帧窗口。
固定窗口频率不超过 `imu.rate_hz / 5`，窗口长度不超过 `preprocess.max_scan_duration`。
等待、消息缓存和时间缺口限制见 `[sync]`，实际 IMU 数据不足仍会跳过更新。

## 输出与持久化

标准输出只有 Odometry、TF 和 PointCloud2。应用默认话题可由 `ros.yaml` 覆盖。
`publish.scan` / `publish.local_map` 控制可选显示，不影响算法或数据库。
显示点云直接编码 XYZ+intensity，不依赖 PCL。

地图位姿和 map→odom TF 使用内部一致的提交版本，发布失败保留有限重试。
本地发布成功不保证订阅者已经收到。`finish=true`、SIGINT 和 SIGTERM 都先停止输入，
排空已接受的工作，再发布最终结果、关闭数据库和 ROS context；失败返回非零。

当前数据库 schema 为 14；旧版本明确拒绝，新建模式拒绝覆盖已有地图。
Gaussian 几何保存在 `Node.lidar_geometry`，优化位姿保存在 `Admin`，逐帧轨迹保存在 `Trajectory`。
离线导出使用同一只读快照中的优化结果，不能拿原始 `Node.pose` 当作优化结果。
两个脚本只支持 schema 14，对损坏数据、缺失优化结果和未对齐的独立图分量报错：

```bash
python3 src/scripts/build_global_map.py /path/map.db -o /path/map.ply
python3 src/scripts/build_odom_tum.py /path/map.db -o /path/trajectory.tum
```

PLY 默认导出 Gaussian 均值，可选 sparse/dense 采样。TUM 使用
`T_map_body = T_map_anchor * T_anchor_body`，是优化后的全局轨迹。
输出成功后才替换目标文件；失败不会把已有导出文件替换为部分结果。
