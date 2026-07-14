# Sapphire — LiDAR-Inertial SLAM

## What Sapphire Is

Sapphire 是一个 **LiDAR-inertial SLAM 系统**，定位于 KISS-SLAM 做不到的事：

| KISS-SLAM 的限制 | Sapphire 的答案 |
|---|---|
| LiDAR-only（无 IMU） | 紧耦合 IMU：equivariant 预积分 |
| Python（嵌入式慢） | C++17，零 Python 运行时 |
| 无 GPU 加速 | CUDA 一等公民（cmake 开关，同仓库） |
| MapClosures 回环 | 可插拔 LoopDetector 接口 |
| g2o 批量优化 | GTSAM ISAM2 增量优化 |

名字来源：Ti:Sapphire 激光器（LiDAR 本质是激光）+ 莫氏硬度 9（工程鲁棒）。
原名 Breeze（微风），2026-07-09 改为 Sapphire。

## 工程架构

```
sapphire/          (STATIC lib, C++17 + PCL + Eigen)
  ├── include/sapphire/
  │   ├── types.hpp              Point, PointCloud, Config 类型定义
  │   ├── config.hpp             TOML 配置加载 + 验证
  │   ├── ring_buffer.hpp        定长环形缓冲区
  │   ├── imu_init.hpp           ImuInitializer
  │   ├── backend/
  │   │   └── pose_graph.hpp     异步 ISAM2 PGO + map←odom 快照
  │   └── odometry/
  │       ├── pipeline.hpp       OdometryPipeline (pushLidar/pushImu → OdometryResult)
  │       ├── deskew.hpp         Gal3 预积分去畸变
  │       ├── registration.hpp   small_gicp scan-to-scan 配准
  │       └── submap.hpp         SubmapManager k-NN 关键帧管理
  ├── src/
  │   ├── config.cpp             TOML 解析实现
  │   ├── imu_init.cpp           IMU 静止校准
  ├── src/odometry/
  │       ├── pipeline.cpp       管道状态机 (ImuState, preprocess, deskew, reg, submap, observer)
  │       ├── deskew.cpp         Gal3 积分 + 世界系输出
  │       ├── registration.cpp   GICP align + 拒绝逻辑
  │       └── submap.cpp         k-NN 最近关键帧选择 + VoxelGrid 重建
  ├── cfg/
  │   └── sapphire_mid360.toml   Mid-360 默认配置
  ├── tests/
  │   ├── deskew_test.cpp        7 cases (static, velocity, yaw, lever-arm, fallback...)
  │   ├── config_test.cpp        4 cases (project config, normalization, validation)
  │   └── pipeline_test.cpp      2 cases (IMU init, first-scan deskew)
  └── external/                  vendor deps (preintegration, lie, nanoflann)

sapphire_cuda/      (-DSAPPHIRE_CUDA=ON, future)
sapphire_cli/       (standalone MCAP 直读, future)

sapphire_ros2/      (ament_cmake wrapper)
  ├── include/sapphire_ros2/
  │   └── sapphire_ros.hpp      ROS ↔ core bridge + pipeline owner
  ├── src/
  │   ├── main.cpp               init + spin
  │   └── sapphire_ros.cpp       raw PointCloud2 → pcl::PointCloud<sapphire::Point>
  └── launch/
      └── sapphire.launch.py     Livox defaults (/livox/lidar, /livox/imu)

## 数据类型

核心点类型: `sapphire::Point` — PCL 原生类型:
```cpp
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;       // float x, y, z
    float intensity;
    double timestamp;       // seconds from scan start (for deskew)
};
POINT_CLOUD_REGISTER_POINT_STRUCT(sapphire::Point, ...)
```

`PointCloud = pcl::PointCloud<Point>`, `PointCloudPtr/ConstPtr` 别名。
small_gicp 可直接消费，零转换。ROS wrapper 从 PointCloud2 raw fields 提取
x/y/z/intensity/timestamp 构建此类型。
```

## 数据流

```
sensor_msgs::PointCloud2
  → raw field parsing (x/y/z/intensity/offset_time)
    → pcl::PointCloud<sapphire::Point>
      → pipeline_->pushLidar(stamp, cloud) → preprocess (NaN+crop)
        → deskew (Gal3 → world frame) → registration (WIP)
          → publish_odometry() → nav_msgs::Odometry

sensor_msgs::Imu
  → sapphire::ImuData { stamp, accel, gyro }
    → pipeline_->pushImu(imu) → ImuInitializer (first N s)
      → bias-corrected → RingBuffer<ImuData, 500>
      → ImuState { T_world_imu, v_world } for next deskew
```

### ROS 层与 Core 的边界

**铁律：ROS wrapper 只做 ROS 消息格式转换，不参与算法。** 用户明确纠正过——"ros层除了ros什么都不应该知道"。

| 层 | 知道什么 | 不应该知道 |
|----|---------|-----------|
| `sapphire_ros2` | sensor_msgs ↔ sapphire 数据搬运 | 传感器型号、deskew 算法、IMU 初始化 |
| `sapphire` core | PCL 点云、预积分、配准 | ROS message 格式、QoS、topic 名 |

传感器差异（Livox `offset_time` ns vs Velodyne `time` s）在 wrapper 内以
auto-detect 处理——按 `offset_time` → `t` → `timestamp` → `time` 顺序尝试字段名，
匹配后统一转成 `sapphire::Point::timestamp`（double seconds）。
不引入传感器抽象层，不做类型分支。以后加传感器再加字段检测逻辑。

## Code Style (User Iron Rule)

**Comments: English only.** Never Chinese in code files (.hpp, .cpp, .py, .txt, .xml,
CMakeLists.txt). Documentation outside code may use Chinese. This is a hard rule —
user corrected all Chinese comments in a single sweep (2026-07-09).

**Dependency discipline:** Only add `find_package` + linker entry when the
corresponding `.cpp` actually uses it. Adding GTSAM::gtsam preemptively broke the
build because the PPA's CMake target name didn't match. Lesson: skeleton = minimal,
add deps incrementally as modules are implemented.

**INTERFACE → STATIC:** As soon as the core library has `.cpp` files, switch from
`add_library(sapphire INTERFACE)` to `add_library(sapphire STATIC ...)`.

参考 GLIM（koide3）：`sapphire` 是普通 CMake STATIC 库，`sapphire_ros2` 是 ament_cmake 包。

colcon 自动处理依赖顺序，先编 sapphire 再编 sapphire_ros2。

sapphire_ros2 的 CMakeLists.txt 用 add_subdirectory 在 workspace 内查找 sapphire：

```cmake
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../sapphire/CMakeLists.txt")
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../sapphire ${CMAKE_BINARY_DIR}/sapphire)
else()
  find_package(sapphire REQUIRED)
endif()
```

**Pipeline 接口：** core 通过 `pushLidar()` / `pushImu()` 接收数据（void return），
结果通过 `latestResult()` 轮询获取。push 和 get 解耦，为后续多线程异步处理留空间。

```cpp
// sapphire::OdometryPipeline (core)
void pushLidar(double stamp, const PointCloudConstPtr& points);  // PCL shared_ptr
void pushImu(const ImuData& imu);
const OdometryResult& latestResult() const;
```

// sapphire_ros2::SapphireRos (wrapper, ROS ↔ core)
// 构造函数中创建 pipeline_，callbacks 中调用 pushXxx()
```

## ROS2 Launch 约定

默认话题（Livox Mid-360）：
```bash
ros2 launch sapphire_ros2 sapphire.launch.py
# lidar_topic:=/livox/lidar  (default)
# imu_topic:=/livox/imu       (default)
# odom_topic:=/sapphire/odometry (default)
```

可重映射：
```bash
ros2 launch sapphire_ros2 sapphire.launch.py lidar_topic:=/velodyne_points
```

## 核心依赖

**apt 直接装:**
- libeigen3-dev, libtbb-dev
- libspdlog-dev, libnanoflann-dev, libomp-dev

**sapphire core 当前已 link 的 CMake targets:**
- `Eigen3::Eigen`, `spdlog::spdlog`, `${PCL_LIBRARIES}`
- TBB/GTSAM/tomlplusplus — 已安装但未 link，等对应模块实现时再加

**PPA（不再源码编译）:**
- GTSAM: `ppa:borglab/gtsam-develop` → `libgtsam-dev`（支持 Jammy 22.04）

**源码编译安装到 /usr/local（容器内，非 vendored）:**
- small_gicp（koide3，GICP 配准后端）→ `find_package(small_gicp REQUIRED)` 可用
- tomlplusplus（marzer，header-only TOML 解析）— ⚠️ `libtomlplusplus-dev` **不存在于 Ubuntu 22.04**

**ROS2:**
- rclcpp, sensor_msgs, nav_msgs, tf2_ros, tf2_eigen, pcl_ros
- PCL: 由 pcl_ros 传递，但 CMake 需显式 `find_package(PCL REQUIRED)` 才能拿到 `${PCL_LIBRARIES}`

### 依赖陷阱

1. **`libtomlplusplus-dev` 在 Ubuntu 22.04 不存在。** 必须从 GitHub marzer/tomlplusplus 源码编译。Ubuntu 23.10+ 才有 apt 包。
2. **GTSAM 无原生 apt 包。** 必须通过 `ppa:borglab/gtsam-develop` → `libgtsam-dev`。
3. **骨架阶段只加实际用到的依赖，不要"预留"。** 编译不过的直接原因是未来模块的依赖被提前声明导致 CMake target 找不到。等实现对应模块时再加对应的 find_package。这是今天踩过的坑——加了 GTSAM::gtsam 但 PPA 的 target 名不匹配，直接炸了。
4. **`<depend>libpcl-all-dev</depend>` 和 `<depend>Eigen3</depend>` 不是合法的 ROS package.xml 标签。** PCL 由 pcl_ros 的 rosdep 链传递，Eigen 由 eigen3_cmake_module 覆盖。
5. **CMake 中不要混用 `target_link_libraries` 和 `ament_target_dependencies` 处理同一个 ROS 包。** ROS 依赖全走 ament_target_dependencies，target_link_libraries 只用于非 ROS 目标（sapphire、PCL_LIBRARIES）。
6. **small_gicp 需要 `libomp-dev`。** 不加的话 `find_package(OpenMP)` 在 Docker 里可能静默失败。
7. **small_gicp CMake target 是 `small_gicp::small_gicp`，不是 `${small_gicp_LIBRARIES}`。** `find_package(small_gicp)` 后 `target_link_libraries(... small_gicp::small_gicp)`。
7. **core 从 INTERFACE 改 STATIC 后，sapphire_ros2 的 target_link_libraries 要同步更新。** INTERFACE 时只需头文件，STATIC 后要链接 .a 文件。
8. **PCL 模板括号解析陷阱。** `boost::make_shared<PointCloud>()`、`boost::const_pointer_cast<PointCloud>()` 在 GCC 下会因为 `PointCloud` 展开为 `pcl::PointCloud<sapphire::Point>` 导致嵌套 `>>` 解析失败。解决方法：
   - 用 `std::make_shared<PointCloud>()` 替代 `boost::make_shared`
   - 避免 `const_pointer_cast`，直接用 `PointCloudConstPtr` 成员接受赋值
   - 或使用中间 typedef：`using Ptr = typename PointCloud::Ptr;`

### 库设计陷阱

1. **不要在 library constructor 里调 `spdlog::set_level()`。** 它会改全局日志级别，污染所有链接 Sapphire 的工程。日志级别是应用的职责，不是库的。
2. **`std::optional` 成员函数不加检查直接解引用是安全炸弹。** 加 `assert()` 或 `if (!has_value())`。
3. **数据结构（ImuData）不应跟算法（OdometryPipeline）同文件。** 拆到 types.hpp，消除循环依赖可能。
4. **Config 里用临时构造读默认值是假解耦。** 真正的解耦是把 Config 传入构造函数，读取 `config_.imu.xxx`。
5. **魔法数字尤其是 IMU 噪声参数必须有来源注释。** 将来换传感器（如 ICM-20948 vs BMI088），不知道这些值从哪来的就是定时炸弹。
6. **`assert()` 不能替代 runtime guard — 已修复 (a17fa44)。** 三处 assert 改为 `throw std::logic_error`。内部生命周期违规不应静默吞掉——抛异常更合理，避免空 optional 解引用 UB。`<cassert>` 已从 pipeline.cpp 移除。✅
7. **TOML 配置运行时接入 — 已修复 (40d6664)。** `SapphireRos` 通过 `ament_index_cpp::get_package_share_directory("sapphire_ros2")` 找到 TOML，`declare_parameter("config_file")` 支持 launch 文件覆盖。CMakeLists.txt 把 `sapphire/cfg/` install 到 ros2 包的 share 目录。✅
8. **`requireFiniteNonnegative` 允许零值。** `config.cpp:69` 只拒绝 `< 0`，零噪声密度会产生退化协方差矩阵（零信息 = 无限不确定性），虽然不会崩溃但滤波行为异常。应该 warn 或拒绝零点。

## 文件组织约定

- `sapphire_ros2/src/main.cpp` — 唯一入口，只做 init + make_shared + spin
- `sapphire_ros2/include/sapphire_ros2/sapphire_ros.hpp` — ROS 数据收发类 + pipeline owner
- `sapphire_ros2/src/sapphire_ros.cpp` — subscribers, publishers, QoS, PCL 转换, pipeline pushXxx()
- `sapphire/include/sapphire/odometry/pipeline.hpp` — core pipeline 接口（pushLidar/pushImu）
- `sapphire/src/odometry/pipeline.cpp` — pipeline 实现（imu_init → deskew → 降采样 → 配准 → submap）
- 核心算法在 `sapphire::OdometryPipeline` 里，`SapphireRos` 只负责 ROS ↔ sapphire 数据转换

**Logger 约定：** core 用 spdlog (`[pipeline] ...`)，ROS2 层用 RCLCPP_INFO（仅启动信息）。
诊断输出（LiDAR/IMU 数据确认）走 pipeline 的 spdlog，见上方 Logging 约定。

**IMU buffer:** `sapphire::ImuBuffer`，底层为定长
`RingBuffer<ImuData, kImuBufferCapacity>`（`include/sapphire/ring_buffer.hpp`）。
固定容量、零堆分配、O(1) 随机访问、支持反向迭代（rbegin/rend 用于 deskew 时间插值）。
满了自动覆盖最老元素，无需手动 pop_front。

为什么自己写而不引入外部库：
- `boost::circular_buffer` — DLIO 原来用的，但我们已决定不用 Boost
- `rigtorp/SPSCQueue` (MIT, 1.8k stars) — 纯 FIFO，不支持 deskew 需要的随机访问
- Abseil — 无 deque/ring-buffer 替代（btree_map/InlinedVector/…都不行）
- `std::deque` — 堆分配 chunk，不如定长 ring buffer 干净

RingBuffer 本身非线程安全。Pipeline 使用 `imu_mutex_` 保护读写；deskew
同时锁定状态和 IMU 缓冲完成一致快照，随后释放锁执行预积分和点云变换，
避免阻塞 IMU callback。

### C++ 头文件陷阱

**用了容器就要 `#include` 它。** 缺少头文件会报看起来无关的错误（如 `not declared in this scope`），实际是类型模板未实例化导致整行解析失败。

### GCC 编译器陷阱

**GCC 11 不支持嵌套 struct 在类内作为默认参数。**
`class Foo { struct Bar { int x = 1; }; explicit Foo(const Bar& b = Bar{}); }`
会报 "default member initializer required before the end of its enclosing class"。
GCC 12+ 已修复。解决：将 Config struct 提到类外（如 `ImuInitConfig` 替代 `ImuInitializer::Config`）。

### Logging 约定

**核心原则：只打一次性事件，不打 per-call 数据流。** 高频回调（LiDAR 10Hz, IMU 200Hz）里不应该有 per-call 日志。初始化期间有进度条（flush style），其他所有信息通过最终结果 API 或按需查询获取。

| 场景 | 方式 | 频率 | 说明 |
|------|------|------|------|
| 构造函数 | `spdlog::info` | 一次 | 配置信息 |
| 初始化完成 | `spdlog::info` | 一次 | bias, gravity, quality |
| pushLidar skipped | `spdlog::debug` + static skip counter | 每 50 帧 | 避免 10Hz 刷屏 |
| pushImu 数据流 | 不打 | - | 200Hz 绝对不打 |
| IMU 初始化进度 | `fprintf(stderr, "\r...")` + `fflush` | 每 0.5s 刷新 | flush style, 同一行覆盖 |
| 最终结果 | `spdlog::info` | 收敛后一次 | 清晰的 block 输出 |
| 异常/错误 | `spdlog::warn` / `spdlog::error` | 按需 | empty cloud, timeout 等 |

**Per-scan pipeline 日志（v0.1 调试期）：**

前 5 帧 + 每 20 帧打一次 info 级摘要，中间帧静默。格式：

```cpp
static int scan_idx = 0;
scan_idx++;
const bool verbose = (scan_idx <= 5) || (scan_idx % 20 == 0);

if (verbose) {
    spdlog::info("[pipeline] ┌─ scan #{:<4d}  stamp={:.3f}  points={}", ...);
}
// ... deskew ...
if (verbose) spdlog::info("[pipeline] │ deskew  converged={}  pos=[...]");
// ... registration ...
if (verbose) spdlog::info("[pipeline] │ GICP    converged={}  inliers={}  iter={}  pos=[...]");
if (verbose) spdlog::info("[pipeline] └─ done");
```

用树形线（`┌─`, `│`, `└─`）做层级，每帧一个逻辑块。

**flush-style 进度显示模式（用户偏好：优雅的终端输出）：**
```cpp
// In-place progress — overwrites same line with \r
static const char spinner[] = "|/-\\";
static int spin_idx = 0;
std::fprintf(stderr,
    "\r[imu_init] %c %.1fs | %zu samples | "
    "gyro_σ=%.4f/%.4f  accel_σ=%.3f/%.3f  ",
    spinner[(spin_idx++) % 4], elapsed, samples_.size(),
    gyro_std, cfg_.convergence_gyro_std,
    accel_std, cfg_.convergence_accel_std);
std::fflush(stderr);
// On completion: fprintf(stderr, "\n"); then spdlog::info(...)
```

**终端美学偏好（用户强要求）：**
- 进度显示用 flush style（`fprintf(stderr, "\r...")` + `fflush`），不要逐行 spdlog
- 最终结果用 box-drawing 字符（`┌─┐│└─┘` 单线框，不要 `╔═╗║╚═╝` 双线）做 hacker-style banner
- 数据用侧边栏两列布局、固定宽度对齐、固定小数点（不要科学计数法）
- 大段输出用单个 `spdlog::info(R"(...)")` raw string，不要拆成多个 `spdlog::info` 调用
- 有 logo/品牌时要在输出里展示（`◆ SAPPHIRE ◆`）
- 参考实现：`references/imu-init-banner.cpp`

**反模式 — 不要在 per-call 回调里打日志：**
```cpp
// WRONG — 10Hz 刷屏
void pushLidar(...) {
    spdlog::info("pushLidar: {} points", points.size());  // NO
}

// WRONG — 200Hz 刷屏
void pushImu(...) {
    spdlog::debug("pushImu: accel=...");  // NO
}

// RIGHT — 节流
void pushLidar(...) {
    static int skip_count = 0;
    if (++skip_count % 50 == 0) {
        spdlog::debug("skipped {} scans", skip_count);  // ~5s interval
    }
}
```

## Docker 开发环境

**镜像策略：只装依赖，不含源码。** 源码通过 volume 挂载，改代码不需要重 build 镜像。

工作目录：`/workspace`（容器默认路径）。数据集挂载：`/home/bindeer/qx/bags` → `/data`（只读）。

```bash
# 构建镜像（一次性）
cd /home/bindeer/git/sapphire_ws
docker build -t sapphire:humble .

# 挂载源码 + 数据集启动
docker compose run --rm sapphire bash

# 容器内首次构建（已在 /workspace）
rosdep install --from-paths src -y -i --skip-keys sapphire
colcon build --symlink-install

# 改代码后只需重复 colcon build
colcon build --symlink-install --packages-select sapphire_ros2
```

**docker-compose.yml 关键配置：**
- GPU: `runtime: nvidia`（不是 `deploy.resources`，非 swarm 下不稳定）
- 挂载: `./src:/workspace/src` + `/home/bindeer/qx/bags:/data:ro`
- `--symlink-install` 让 install 指向源码

基础镜像 `osrf/ros:humble-desktop-full`，GTSAM 走 PPA，small_gicp + tomlplusplus 源码编译。完整 Dockerfile 在 `/home/bindeer/git/sapphire_ws/Dockerfile`。

## v3 关键设计决策

- **PCL 原生 Point type**: `sapphire::Point` (PCL_ADD_POINT4D + POINT_CLOUD_REGISTER_POINT_STRUCT)，替代自定义 Point3d。core 依赖 PCL——做 SLAM 的都用 PCL，没问题。small_gicp 零转换直接消费。
- **传感器适配边界**: ROS wrapper 只做原始 PointCloud2 bytes → `sapphire::Point` 数据搬运，不含传感器类型判断。不同传感器的 timestamp 字段名差异在 wrapper 内以 auto-detect 处理（offset_time/t/timestamp/time），不引入传感器抽象层。
- **Deskew 输出坐标系**: World frame。新版 deskew 使用完整 Gal3 恢复（`Gamma_ij() * Xi_init * Upsilon()`），直接输出世界系点云 + 世界系位姿 + 速度。两个 scan 都在世界系时 scan-to-scan GICP 的 correction 就是 inter-scan motion。
- **VoxelMap**: KISS-SLAM 移植（规划中）
- **Submap 分割**: 距离 100m + 旋转 90° 双阈值
- **RegistrationBackend::setTarget(raw PointCloud)**: 后端自预处理（KD-tree/GaussianVoxelMap/NDT grid）
- **KD-tree 回环**: 用 geometric_centroid 而非 origin（大 submap 必漏检）
- **v0.1 不需要 ISAM2Ext**: GPU 因子只在前端，不入因子图
- **PGO 前后端隔离**: 前端始终在连续 odom 系运行；PGO 独立门控关键帧并在后台维护 ISAM2，只通过线程安全的 `T_map_odom` 快照修正最终输出，不回写 deskew、GICP、observer 或 IMU propagation。独立门控有意对齐 DLIO/SimpleLoopClosure：PGO 默认以 0.5 m/0.3 rad 采样，而 Mid-360 前端 submap 以 1.5 m/45° 采样；前者需要更密的图节点和回环查询，不能直接等同于局部地图关键帧。
- **PGO 回环配准**: 候选在 `loop_search_radius` 内经时间、累计行程和姿态差过滤，随后用单帧 source 对目标帧邻域地图做 PCL ICP。为容纳大圈轨迹末端的累计里程计漂移，最大对应距离与 DLIO 一致设为 `2 * loop_search_radius`；diagnostics 提供候选数、ICP 拒绝数、最近 fitness 和当前 `T_map_odom` 平移/旋转修正量。
- **统一对外里程计**: PGO 关闭时 `T_map_odom=I`；开启时对外结果为 `T_map_lidar=T_map_odom*T_odom_lidar`，ROS topic 和消息类型不变。
- **按需 PGO 可视化**: `/sapphire/pgo/graph` 发布优化节点、里程计边和回环边，并已在默认 `display.rviz` 中以 Reliable + Transient Local QoS 启用；`/sapphire/pgo/map` 发布优化关键帧拼接的稀疏地图。无订阅者时不构建地图，地图拼接在 PGO worker 中异步完成。
- **alignment_risk**: SuperLoc 启发的退化预测

## ROS2 QoS 陷阱

**RViz 默认用 RELIABLE，但 ROS2 传感器数据偏好 BEST_EFFORT。** 两者不兼容时 QOS 不匹配静默丢弃消息。

```cpp
// ❌ RViz 看不到点云
auto sensor_qos = rclcpp::SensorDataQoS();  // BEST_EFFORT
deskewed_pub_ = create_publisher<PointCloud2>("sapphire/deskewed", sensor_qos);

// ✅ 显式用 reliable_qos 给可视化 topic
auto reliable_qos = rclcpp::QoS(10).reliable();
deskewed_pub_ = create_publisher<PointCloud2>("sapphire/deskewed", reliable_qos);
```

规则：传感器订阅保持 BEST_EFFORT（丢帧优于阻塞），可视化发布用 RELIABLE（RViz 兼容）。

### ROS2 Wrapper 陷阱

1. **`publish_deskewed()` frame_id 应为 "odom"，不是 "lidar"。** deskew 输出是世界系（odom frame）点云，用 "lidar" 会导致 RViz 在错误位置显示。DLIO 的 `deskewed_pub` 也是 odom frame。

## Docker X11 可视化（WSL2/WSLg）

WSL2 自带 WSLg（`DISPLAY=:0`, `/tmp/.X11-unix/X0`），Docker 里跑 RViz：

```yaml
# docker-compose.yml
environment:
  - DISPLAY=${DISPLAY:-:0}
  - QT_X11_NO_MITSHM=1
volumes:
  - /tmp/.X11-unix:/tmp/.X11-unix:ro
```

`QT_X11_NO_MITSHM=1` 是 Qt/RViz 在 Docker 内运行的必须项——共享内存路径不兼容。

## VoxelGrid 降采样（2026-07-11 — ✅ 已实现，77315ac）

**实现：** `downsamplePoints()` 在 deskew 后、GICP 前对 source 使用 0.25m VoxelGrid。
keyframe 保留全分辨率点云，`latest_deskewed_` 发布降采样版本（与 GICP 视角一致）。
submap target 已有 0.25m VoxelGrid，source/target 同分辨率匹配更稳定。

w=1 修复照搬 submap 模式，保证 small_gicp 兼容。

**关键设计：** 降采样后检查 min_points，稀疏场景提前跳过而非空 feed GICP。

## propagateState（2026-07-11 — ✅ 已实现，69db9a7）

**实现：** 独立 Gal(3) propagation pim，与 deskew 同源的 Gamma*Xi*Upsilon 恢复。
`pushImu()` 中 `propagateStateLocked()` 实时积分，维护 `propagated_state_`。
`rebasePropagation()` 在 GICP 修正后重置 baseline 并重放 buffered IMU，
保证 propagation 不因 LiDAR 延迟丢失 IMU 数据。ROS wrapper 在 imu_callback 中
发布 propagated odometry 实现 200Hz 高频输出。

**线程安全：** `state_mutex_` 独立于 `imu_mutex_`，deskew 先拷贝 baseline 再访问 buffer。

**与 deskew 的关系：** 两条独立路径，共享同一套 Gal(3) 公式。deskew 仍从 reference state
+ 缓存 IMU 独立进行 timeline 积分（不依赖 propagated state 的 prior），保证精确性。

## 在线 Bias 估计：几何观测器路线（2026-07-11 纠正）

**DLIO 的在线 bias 不通过 Jb() 更新，而是几何观测器：**
- GICP 位置残差 → `Kab * position_error` → 更新 accel bias
- GICP 姿态残差 → `Kgb * rotation_error` → 更新 gyro bias
- 配合 Kp/Kv/Kq/Kab/Kgb 增益和 bias 限幅

**Jb() 的正确用途：** 适合 ESKF、因子图或严格预积分重线性化方案——需要精确时间对齐和残差管理。直接接入当前 pipeline 容易出现时间基准不一致、残差和 bias 重复修正。

**推荐路线：**
1. VoxelGrid 降采样（P0）
2. propagateState + Kp/Kv/Kq 完整状态观测器
3. Kab/Kgb 在线 bias 更新及限幅
4. 最后评估是否需要基于 Jb() 的 ESKF/优化方案

## 当前进度（2026-07-11）

✅ Docker 环境可编译可运行
✅ 数据链路端到端通：PointCloud2 raw bytes → pcl::PointCloud<sapphire::Point> → pipeline
✅ LiDAR (~20K pts/10Hz) + IMU (200Hz) 数据正常流入
✅ RingBuffer<T,N> 自研完成
✅ PCL 原生 Point type：`sapphire::Point`，small_gicp 零转换
✅ 基础骨架：sapphire (STATIC) + sapphire_ros2 (ament_cmake)
✅ Launch 文件：Mid-360 默认话题 + 可配置世界系/雷达系/acc_scale

✅ IMU 预积分 + Lie 群数学库
- `external/preintegration/` + `external/lie/` — header-only (SO3/SEn3/Gal3/TG)

✅ ImuInitializer — MAD滤波+方差收敛+重力对齐+hacker banner
✅ Config + ImuNoiseConfig + DeskewConfig + RegistrationConfig + Extrinsics + CropBox + SubmapConfig + ObserverConfig

✅ Deskew — Gal3 `Gamma * Xi * Upsilon` 恢复，世界系输出，速度传播
✅ TOML config 系统 — `config.cpp` + `cfg/sapphire_mid360.toml`，tomlplusplus
✅ Registration（small_gicp scan-to-submap GICP）— `registration.cpp`，三重拒绝逻辑
✅ SubmapManager — k-NN 最近关键帧，VoxelGrid 目标重建，w=1 修复
✅ Geometric Observer — DLIO 风格速度修正 `v += dt * Kv * position_error`
✅ Pipeline 闭环 — first-scan target → deskew → GICP → observer → submap → 下一帧
✅ PreintegrationParams 参数顺序 Bug 修复 — gyroNoise 在前、accelNoise 在后
✅ 测试 — deskew_test (7 cases), config_test (4 cases), pipeline_test (2 cases), submap_test (1 case)

✅ ROS2 wrapper — MultiThreadedExecutor, callback groups, datatype-aware 逐点时间戳解析
✅ TF 广播 (world_frame → lidar_frame) + deskewed 点云发布 + odom 发布
✅ rclcpp::Time 用 llround 替代旧版 int32/uint32 split

✅ VoxelGrid source downsampling (0.25m) before GICP (2026-07-11, 77315ac)
✅ propagateState — IMU-rate Gal(3) propagation between LiDAR corrections, same Gamma*Xi*Upsilon as deskew (2026-07-11, 69db9a7)
📋 待实现：在线 bias 估计（几何观测器 Kp/Kv/Kq → Kab/Kgb，不用 Jb()）→ 数据集 benchmark
📋 重构：ImuPropagator 独立模块（buffer+propagate+rebase 从 pipeline 抽出，当前 pipeline ~500 行）

📋 代码审查完成（2026-07-11）：无阻塞性 bug。见 `docs/code-review/REVIEW_2026-07-11.md`
📋 baseline 审查完成（2026-07-11）：submap+observer+registration 闭环正确。见 `references/baseline-review-2026-07-11.md`

## Novelty & Academic Publishing (2026-07-11)

### Gal(3) Equivariant Preintegration: First Full Deployment in LiDAR SLAM

Sapphire 将 Fornasier et al. (2023-2024) 的 Gal(3) 等变预计分理论首次完整工程化落地到 LiDAR-inertial SLAM 系统中。

**与 DLIO 的关系澄清（重要纠正）：两条独立技术路线，不是"DLIO 引入但降级"。**
- DLIO (Kenny Chen, NASA JPL, ICRA 2023): SE(3) continuous-time 轨迹 + slerp 插值 deskew，**不使用** Fornasier 库
- Fornasier (U. Klagenfurt, 2023-2024): Gal(3) 群上的 equivariant 预积分理论 + C++ 开源库 `aau-cns/equivariant-preintegration`
- Sapphire: 采用 Fornasier 的库，在完整 LiDAR-inertial odometry pipeline 中落地完整 Gal(3) 恢复

**Novelty 定位：**
```
aau-cns/equivariant-preintegration  176★ 25 forks (全部为学习镜像)
         ↓
  任何 fork 都未集成到完整 LiDAR SLAM 管线
         ↓
  DLIO → ❌ 未使用 Fornasier 库（独立 SE(3) 预积分路线）
  FAST-LIO(2) → ❌ ESKF 欧拉积分
  LIO-SAM → ❌ GTSAM 传统预积分
  其他所有开源 LIO → ❌
         ↓
  Sapphire → ✅ 唯一完整部署 Gamma*Xi*Upsilon 的 LiDAR SLAM
```

### 发表策略

**推荐靶场：RA-L + ICRA**（审稿 2-4 月，中了可 ICRA presentation）

**Core Contribution:**
1. 首个完整使用 Gal(3) equivariant preintegration 的 LiDAR-inertial odometry
2. Deskew 和 IMU propagation 统一在 Gal(3) 群上的同源框架
3. 速度状态在 deskew 中保留 → 消除 deskew/propagation 不一致性

**必做实验：**
- 公开数据集: KITTI, Hilti 2021/2022, Newer College, M2DGR
- **消融研究（最核心）**: slerp deskew vs Gal(3) deskew，同一 pipeline，只换 deskew 方式
- 对比 SOTA: DLIO, FAST-LIO2 → ATE/RPE

**加分实验：**
- 高动态场景速度一致性分析（Hilti 激烈运动 — Gal(3) 优势最明显）
- 计算开销分析（Gal(3) vs slerp CPU cost）

**论文题目草稿：**
> "Sapphire: Equivariant Preintegration for Unified Deskew and Propagation in Direct LiDAR-Inertial Odometry"

详细调研和对比数据见 `references/novelty-and-publishing.md`。

## IMU 初始化（2026-07-10 讨论）

**决定：先做 IMU 初始化再做 deskew。** 没有 bias 校正和重力对齐的 IMU 数据直接用于 deskew 会产生系统性误差。

详细文献调研：`references/imu-initialization-survey.md`

### 设计决策

| 决策 | 选择 | 理由 |
|------|------|------|
| 初始化模式 | 静止初始化（同 DLIO） | Mid-360 内置 BMI088，上电 bias 稳定 |
| 收敛判据 | 方差驱动（替代固定 3s） | gyro_std < 0.005, accel_std < 0.05 |
| 异常值处理 | MAD 滤波 | 防止校准期间偶尔碰撞污染估计 |
| 重力对齐 | `FromTwoVectors(accel_avg, [0,0,g])` | 业界标准，DLIO/LIO-SAM/OpenVINS 均用 |
| 超时策略 | 5s 超时用已有数据 | 比 DLIO 的无限等待更健壮 |
| 质量报告 | gyro_std, accel_std, quality level | 用户可见的可信度指标 |

### 数学本质

静止时：
```
accel_meas = R_world_imu^T * [0, 0, g]^T + bias_accel + noise
gyro_meas  = 0 + bias_gyro + noise
```

重力方向和 accel bias 耦合，DLIO/业界的处理：
1. 假设 bias << g（BMI088 accel bias < 0.1 m/s²，g = 9.8）→ 成立
2. `gravity_dir = normalize(mean(accel))`
3. `bias_accel = mean(accel) - gravity_dir * |g|`
4. 后续 Geo Observer / ESKF 在线修正 bias
   （详见 `references/dlio-online-bias-estimation.md` — DLIO 的 Phase 2
    在线 bias 估计，Sapphire v0.1 暂未实现，等 Registration 就位后加入）

### ImuInitializer 接口（已实现）

```cpp
// sapphire/include/sapphire/imu_init.hpp

/// Configuration (extracted to avoid GCC 11 nested-struct default-init bug)
struct ImuInitConfig {
    double convergence_gyro_std  = 0.005;
    double convergence_accel_std = 0.05;
    double timeout_sec           = 5.0;
    double check_interval_sec    = 0.5;
    int    min_samples           = 100;
    double gravity_mag           = 9.80665;
};

class ImuInitializer {
public:
    enum class State { WAITING, CALIBRATING, CONVERGED, TIMEOUT };
    struct Result { /* gyro_bias, accel_bias, q_gravity, quality */ };
    explicit ImuInitializer(const ImuInitConfig& config = ImuInitConfig{});
    std::optional<Result> feedImu(double stamp,
                                   const Eigen::Vector3d& accel,
                                   const Eigen::Vector3d& gyro);
};
```

### Pipeline 状态机（已实现）

```cpp
// pipeline.cpp — pushImu()
if (!initialized_) {
    auto result = imu_initializer_.feedImu(imu.stamp, imu.accel, imu.gyro);
    if (result.has_value()) {
        imu_init_result_ = std::move(result);
        // Create gravity-aligned, stationary ImuState
        imu_state_.stamp = imu.stamp;
        imu_state_.T_world_imu.linear() = imu_init_result_->q_gravity.toRotationMatrix();
        imu_state_.v_world.setZero();
        imu_state_.valid = true;
        // Start deskew buffer with bias-corrected samples only
        imu_buffer_.clear();
        imu_buffer_.push_back(correctImu(imu));
        initialized_ = true;
    }
    return;
}
imu_buffer_.push_back(correctImu(imu));

// pipeline.cpp — pushLidar()
if (!initialized_ || !points || points->empty()) return;
auto preprocessed = preprocessPoints(points);  // NaN filter + crop box
if (preprocessed->size() < min_points) return;
DeskewResult deskewed = deskewPointcloud(stamp, preprocessed);
if (deskewed.status != DeskewStatus::Success) return;

if (!has_prev_scan_) {
    // First scan: set as registration target, seed ImuState
    latest_deskewed_ = deskewed.cloud;
    prev_scan_ = deskewed.cloud;
    registration_.setTarget(prev_scan_);
    imu_state_.stamp = deskewed.reference_stamp;
    imu_state_.T_world_imu = deskewed.T_world_lidar_ref * T_imu_lidar.inverse();
    imu_state_.v_world = deskewed.v_world_ref;
    has_prev_scan_ = true;
    return;
}
// Registration stage (WIP — next iteration)
```

### v0.1 简化

- 假设 IMU 和 LiDAR 同轴（不处理外参），因为 Mid-360 内置 IMU 物理上就在 LiDAR 壳体里
- 不处理 IMU 坐标变换（无 `transformImu`），后续加外参支持时再补
- 单一 `ImuInitializer` 实例在 pipeline 构造函数中创建

### ImuInitializer 陷阱

1. **`computeResult()` 里重复调了 `rejectOutliers()`。** `checkConvergence()` 已经调用过一次并修改了 `samples_`（移除离群点），`computeResult()` 又调用一次——第二次 MAD 在已过滤数据上运行，找不到新离群点，纯浪费 O(N)。解法：删掉 `computeResult()` 开头的 `rejectOutliers()`，或加 `if (state_ != CONVERGED)` guard。

---

## Registration 设计（2026-07-10 — 开始实现）

### 模块接口

```cpp
// sapphire/include/sapphire/odometry/registration.hpp

class Registration {
public:
    using GicpType = small_gicp::RegistrationPCL<Point, Point>;

    explicit Registration(const RegistrationConfig& config = RegistrationConfig{});
    void setSource(const PointCloudConstPtr& cloud);
    void setTarget(const PointCloudConstPtr& cloud);
    RegistrationResult align(const Isometry3d& T_prior);
};
```

### RegistrationResult

```cpp
struct RegistrationResult {
    Eigen::Isometry3d T_correction;   // T_prior → T_world_lidar
    Eigen::Isometry3d T_world_lidar;  // world-frame pose
    bool    converged      = false;
    double  fitness_score  = 0.0;
    size_t  num_inliers    = 0;
    size_t  iterations     = 0;
    bool    accepted       = false;   // false if correction rejected
};
```

### RegistrationConfig（DLIO 兼容默认值）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| max_iterations | 64 | GICP 最大迭代 |
| transformation_epsilon | 0.0005 | 平移收敛阈值 (m) |
| rotation_epsilon | 0.0005 | 旋转收敛阈值 (rad) |
| max_correspondence_dist | 1.0 | 最大对应距离 (m) |
| k_correspondences | 20 | 最近邻对应数 |
| min_num_points | 100 | 最小点数 |
| max_correction_trans | 1.0 | 拒绝阈值 (m) |
| max_correction_rot_deg | 20.0 | 拒绝阈值 (deg) |

### 拒绝逻辑

修正量超过平移 1.0m 或旋转 20° 时拒绝 GICP 结果，回退到 IMU 先验位姿。
日志输出拒绝原因和修正量。

### v0.1 简化

- scan-to-submap GICP（SubmapManager 管理 k-NN 关键帧目标）
- 拒绝帧不回填子图，IMU 先验作为 fallback 位姿
- 完整调试记录：`references/registration-debugging.md`

### 陷阱

1. **`small_gicp::RegistrationResult` 字段名是 `error` 不是 `fitness`。** API 里没有 `fitness` 成员，用 `reg_result.error` 获取最终配准误差。
2. **`reg_result.iterations` 是 `size_t`，不需要判零。** 直接赋值即可。
3. **CMake target 名是 `small_gicp::small_gicp`。** `find_package(small_gicp REQUIRED)` 后 link `${small_gicp_LIBRARIES}` 不行（未定义），必须用 `small_gicp::small_gicp`。
4. **`align()` 不要传 guess — 照搬 DLIO 原版模式。** 使用 `gicp_.align(*aligned)` 不加显式 guess 参数。small_gicp 内部用上次 `getFinalTransformation()` 作初始猜测——scan-to-scan 场景下连续帧位移小、这个默认值就是好初值。传显式 guess 的行为跟 small_gicp 的内部状态管理不同导致性能异常（9950X 上 20K 点 GICP 从 sub-second 变成 10 秒）。
5. **T_correction 是增量变换，需要组合而非替换。** `latest_result_.T_world_lidar = reg_result.T_correction * latest_result_.T_world_lidar`，不是 `= reg_result.T_world_lidar`。

## IMU Bias 估计架构（2026-07-10 讨论）

### DLIO 的两层 bias 估计

DLIO 的 IMU bias 处理分两层，Sapphire ImuInitializer 只实现了第一层：

```
① 静止校准 (ImuInitializer — Sapphire ✅ 已实现)
   imu_calibrate_: 前几秒静止采样 → 均值作为初始 bias

② 在线几何观测器 (Sapphire ❌ 待实现)
   State { p, q, v, b }  其中 b = { gyro, accel }
   GICP 位姿残差 → Kp/Kv/Kq 修正 p/q/v → Kab/Kgb 修正 bias
   配合限幅防发散
```

**重要纠正（2026-07-11）：DLIO 的在线 bias 通过几何观测器更新，不是 Jb()。**
- 位置残差 → accel bias，姿态残差 → gyro bias
- Jb() 更适合 ESKF/因子图重线性化方案，直接接入当前 pipeline 容易出现时间基准、残差和 bias 重复修正
- 推荐先实现 Kp/Kv/Kq → Kab/Kgb，跑通后再评估是否真的需要 Jb()

### EquivariantPreintegration 与 Online Bias 的互补设计

预积分库的 `Jb()` Jacobian 和 `Upsilon_corrected()` 方法就是为在线 bias 修正设计的：

```
t₀                              t₁  扫描到达 → GICP → Geo Observer 更新 bias_new
├──── 预积分(旧 biasHat) ──────┤      │
│  X_, Cov_, Jxi_               │      ▼
│                               │   Upsilon_corrected(bias_new)
│                               │   = exp(Jb() * (bias_new - biasHat)) * Upsilon
│                               │         ↑ Jacobian 预积分时就算好了, O(1) 修正
│                               │
│                               │   resetIntegrationAndSetBias(bias_new)
│                               │   下一个区间用新 bias 从零开始积
├───────────────────────────────┼──────→ t₂
```

没有 Jb() Jacobian 的话，bias 更新后必须重新积分这段时间的所有 IMU 数据 — O(N)。
有 Jb() 则 O(1) 矩阵乘法即可修正预积分结果。

### 分工

| EquivariantPreintegration | Geo Observer |
|---------------------------|--------------|
| 解决"怎么高效算" | 解决"bias 应该是多少" |
| 积分 + 协方差传播 | 从 LiDAR 误差反推 bias |
| Jb Jacobian (免费) | 增益控制收敛速度 |
| bias 一阶修正 O(1) | 限幅防发散 |

当前 gap：预积分库代码在 `external/` 下但**尚未接入 pipeline**。
等 Registration 跑通 → Geo Observer 有误差信号 → 预积分库接入 → 完整闭环。

完整 DLIO deskew 分析：`references/dlio-deskew-analysis.md`

**最终实现（2026-07-11 重写，对齐 DLIO Gal3 预积分）：**
1. std::sort points by timestamp
2. 提取唯一时间戳 → timestamps[] + unique_time_indices[]
3. Gal3 预积分：`partial = pim; partial.integrateMeasurement(accel, gyro, partial_dt)` per unique timestamp
4. Gal3 恢复：`Gamma_ij() * initial_state * Upsilon()` → 世界系 IMU 位姿 + 速度
5. 逐 group 变换到世界系：`T_world_imu(t) * T_imu_lidar * p_lidar`
6. 支持 `time_offset` 补偿 header→首点延迟

输出：World frame，`T_world_lidar_ref` + `v_world_ref` 给 registration 和 propagate。
与 DLIO 差异：无 Boost（手写 adjacent_filtered）、无线程池（v0.1）。

接口：
```cpp
DeskewResult deskew(
    const PointCloudConstPtr& scan, double scan_stamp,
    const RingBuffer<ImuData, 500>& imu_buf,
    double prev_stamp, const Isometry3d& T_world_imu_prev,
    const Eigen::Vector3d& v_world_prev,
    const Isometry3d& T_imu_lidar,
    const Eigen::Vector3d& gravity_world,
    const ImuNoiseConfig& noise, bool time_offset = false
);
```

### Deskew 陷阱

1. **Gal3 恢复模式：`Gamma_ij() * Xi_init * Upsilon()`，不是 slerp。** 旧实现用 slerp 做 SE(3) 插值，会丢失速度状态。正确做法是完整 Gal3 恢复——`Gamma_ij()` 补偿初始状态差异，`Upsilon()` 提取预积分位姿，乘法得到世界系状态。
2. **Fallback 也要做世界系变换。** 没有 per-point timestamp 或 IMU 覆盖不足时，用 `T_world_imu * T_imu_lidar` 做单一位姿变换（`makeFallback`）。此时 `DeskewResult::converged = false`，但点云仍在世界系（非雷达系），保证下游 registration 的输入一致。
3. **PreintegrationParams 参数顺序：gyro 在前，accel 在后。** DLIO 构造器签名是 `(gravity, gyroNoiseSigma, accNoiseSigma, virtualVelNoise, virtualTimeScaleNoise, gyroBiasNoise, accBiasNoise, ...)`。旧代码把 accel_noise_density 塞到 gyroNoise 位置——gyro/accel 噪声密度互换，协方差矩阵缩放方向错误。虽然 BMI088 两者量级接近（~2e-4）未暴露，但系统性错误必须修正。
4. **预积分 bias 设零。** IMU 数据在入 buffer 前已完成 bias 校正，deskew 内的 `pim_->resetIntegrationAndSetBias(Zero)` 是正确的——不要重复减 bias。
5. **PointCloud2 timestamp 字段按 datatype 读，不要假定宽度。** `timestamp(8)` = FLOAT64（8字节），按 `float`（4字节）读会得到垃圾值导致 deskew fallback。详见 `references/pointcloud2-field-parsing.md`。
6. **`integrateMeasurement` 参数类型：IMU 数据是 `Vector3f`，但 `PreintegrationParams<double>` 期望 `Vector3d`。** 隐式转换能过编译但丢精度，旧代码用 `.cast<double>()` 显式转换更好。
7. **`integrateMeasurement(accMeas, gyroMeas, dt)` 参数命名是陷阱。** 函数签名第一个参数叫 `accMeas` 第二个叫 `gyroMeas`，但内部构造 `Input u(gyroMeas, accMeas)` 把它们互换了。Sapphire 传 `(measurement.accel, measurement.gyro, dt)` 经内部交换后 `Input(gryo, accel)` 结果是正确的——但读代码的人会被命名误导。不要改 vendored 库的签名，但在此处留下注释说明已验证正确。

🔗 GitHub: https://github.com/wagdeer/sapphire

## Repository Management

### Two-Repo Structure (Iron Rule)

Sapphire is **two independent git repositories**, NOT a monorepo:

```
wagdeer/sapphire.git        ← Core algorithm (C++17 STATIC lib)
  27 files: CMakeLists.txt, include/, src/, tests/, cfg/, external/, docs/

wagdeer/sapphire_ros2.git   ← ROS2 wrapper (ament_cmake)
  9 files: CMakeLists.txt, include/, src/, launch/, package.xml
```

**The workspace (`sapphire_ws/`) is NOT a git repo.** It contains:
- `src/sapphire/` — independent git repo (core)
- `src/sapphire_ros2/` — independent git repo (wrapper)
- `Dockerfile`, `docker-compose.yml`, `docker/` — infrastructure only (unversioned)

Each subdirectory has its own `.git/`. `git` commands inside `src/sapphire/`
target the core repo; inside `src/sapphire_ros2/` target the wrapper repo.
There is no git context at workspace root — that is correct.

### Force-Push Recovery

When a GitHub repo is polluted (e.g., workspace structure accidentally pushed
to `wagdeer/sapphire.git`), recover with:

```bash
cd src/sapphire
git init
git remote add origin git@github.com:wagdeer/sapphire.git
git add -A
git commit -m "feat: baseline — ..."
git push --force origin master
```

This overwrites the remote with clean content. **Never push workspace-level
commits** to either sapphire or sapphire_ros2 — they break the core/wrapper boundary.

### Pitfall: Dual Git Roots

If `git rev-parse --show-toplevel` inside `src/sapphire/` returns
`/home/bindeer/git/sapphire_ws` instead of `.../src/sapphire`, that means
`sapphire/` has no `.git/` of its own and is using the workspace's git.
The fix is `git init` inside `src/sapphire/` to create the independent repo.

## 项目文件

- `docs/ARCHITECTURE.md` — v3 最终架构
- `docs/CROSS_REVIEW.md` — 交叉审查报告（12 个问题）
- `docs/code-review/REVIEW_2026-07-11.md` — 全面代码审查（2 HIGH / 4 MEDIUM / 4 LOW）
- `docs/SURVEY_v2.md` — 技术调研
- `docs/KISS_SLAM_ANALYSIS.md` — KISS-SLAM 源码分析
- `Dockerfile` — 开发容器
- `references/code-review-2026-07-11.md` — 审查要点速查（算法验证 + bug 清单）
