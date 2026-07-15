# Sapphire 全量代码审查 — 2026-07-16

审查日期: 2026-07-16 | 范围: `sapphire/` (core) + `sapphire_ros2/` (wrapper)
审查人: 米西 (AI review, full codebase walkthrough)

---

## Executive Summary

Sapphire 是一个架构清晰的 production-grade LIO 系统。核心设计决策——无 ROS 依赖的纯 C++17 库 + 薄 ROS2 wrapper、异步 PIMPL PGO 后端、Gal(3) 等变预积分——都是正确的工程选择。经过上一轮 AI-cruft 清理（-302 行日志/诊断代码），代码处于良好状态。

**本次审查发现的主要问题：**
- 死代码：`integrate_measurement.hpp`（模板从未实例化）、`external/preintegration/`（除 lie/ 外的完整 PIM 库未使用）
- GTSAM 硬构建依赖——即使用户禁用 PGO，仍需安装 GTSAM
- Deskew 的 `(void)noise` 信号不清晰
- Registration 构造函数日志仍然冗长

**Verdict: 代码质量高，架构合理。P0 项只有 GTSAM 条件构建一项；其余均为 P1/P2 清理和改进。**

---

## 一、架构映射

```
┌─────────────────────────────────────────────────────────────┐
│  sapphire_ros2 (ROS wrapper)                                │
│  ┌─────────┐  ┌────────┐  ┌──────────┐  ┌───────────────┐ │
│  │ LiDAR cb│  │ IMU cb │  │ Odom pub │  │ PGO viz timer │ │
│  └────┬────┘  └───┬────┘  └────▲─────┘  └──────▲────────┘ │
│       │ msg→Point │            │               │          │
│  ┌────▼───────────▼────────────┴───────────────┴────────┐ │
│  │              SapphireRos::pipeline_                   │ │
│  └───────────────────────┬───────────────────────────────┘ │
└──────────────────────────┼─────────────────────────────────┘
                           │
┌──────────────────────────▼─────────────────────────────────┐
│  sapphire core (pure C++17 + Eigen)                        │
│                                                            │
│  pushImu ──► [ImuInitializer] ──► bias-corrected buffer    │
│                  │ (once)                                  │
│  pushLidar ──► preprocessPoints ──► deskewPointcloud       │
│                      │                   │                 │
│               crop + time range    Gal(3) preintegration   │
│                      │                   │                 │
│                      └──► downsampled ───┘                 │
│                              │                             │
│                     runScanRegistration                    │
│                        (GICP/VGICP)                        │
│                              │                             │
│              ┌───────────────┼───────────────┐             │
│              ▼               ▼               ▼             │
│     geometricObserver   submapManager   PGO backend        │
│     (bias+pose+vel)     (kf sliding win) (ISAM2+ICP loop)  │
│              │                              │              │
│              └──── rebasePropagation ───────┘              │
│                         │                                  │
│                    latestResult()                           │
│                    (T_map_odom applied)                     │
└────────────────────────────────────────────────────────────┘

Thread model:
  - IMU callback  (imu_cb_group_): pushImu → propagateStateLocked
  - LiDAR callback (lidar_cb_group_): pushLidar → full pipeline
  - PGO worker     (std::thread): processPending loop @ 1 Hz
  - PGO viz timer  (ROS timer): publish_pgo_visualization @ 1 Hz

Lock ordering:
  imu_mutex_ → state_mutex_  (deskew, rebase, IMU push)
  imu_mutex_ → output_mutex_ (latestPropagatedResult)
  output_mutex_ (standalone: latestResult, latestDeskewed)

PGO backend has its own input_mutex_ + output_mutex_ — fully isolated from frontend.
```

### 数据流统计

| Pipeline Stage | Input | Output | Allocations per scan |
|---|---|---|---|
| Preprocess | raw PointCloud (~20K pts) | crop-filtered cloud | 1× PointCloud + 1× vector reserve |
| Deskew | filtered cloud + IMU buffer | world-frame deskewed cloud | 1× PointCloud + N_ts× TimedState |
| Voxel downsample | deskewed cloud (~20K pts) | downsampled (~5K pts) | 1× vector<IndexedVoxel> (N pts) + 1× PointCloud |
| Registration | source + submap target | T_correction + accepted T | 1× PointCloud (aligned) |
| Submap rebuild | N_kf× downsampled clouds | voxelized submap | 1× merged cloud + 1× downsample |
| PGO addFrame | cloud_odom + T_odom_lidar | — | 1× InputFrame copy (light) |
| PGO loop ICP | source + target clouds | T_loop + fitness | 2× voxelized (source+target) |

### 模块清单

| 模块 | 文件 | 行数 | 职责 |
|---|---|---|---|
| Types + Config | `types.hpp`, `config.hpp/cpp` | 216 + 488 | 类型定义、TOML加载与验证 |
| RingBuffer | `ring_buffer.hpp` | 180 | 固定容量环形缓冲（header-only） |
| ImuInitializer | `imu_init.hpp/cpp` | 85 + 261 | 静止IMU初始化（MAD outlier rejection + 方差收敛） |
| Deskew | `deskew.hpp/cpp` | 68 + 354 | Gal(3)预积分运动补偿 |
| VoxelFilter | `voxel_filter.hpp/cpp` | 17 + 147 | 确定性体素降采样（OpenMP并行） |
| Registration | `registration.hpp/cpp` | 49 + 120 | small_gicp GICP/VGICP wrapper |
| Observer | `observer.hpp/cpp` | 29 + 74 | DLIO风格几何观测器 |
| Submap | `submap.hpp/cpp` | 56 + 108 | 关键帧滑动窗口局部地图 |
| Pipeline | `pipeline.hpp/cpp` | 181 + 556 | 里程计前端主控制器 |
| PoseGraph | `pose_graph.hpp/cpp` | 79 + 620 | 异步PGO后端（ISAM2 + ICP回环） |
| MeanOnlyIntegrator | `detail/mean_only_gal3_integrator.hpp` | 90 | 轻量Gal(3)积分器（无协方差） |
| integrateMeasurement | `detail/integrate_measurement.hpp` | 32 | **死代码** — 通用PIM模板，从未实例化 |
| ROS2 Wrapper | `sapphire_ros.cpp/hpp` | 413 + 57 | 消息转换 + ROS生命周期 |

---

## 二、死代码与未使用依赖

### 2.1 `integrate_measurement.hpp` — 完全未使用

```cpp
// include/sapphire/odometry/detail/integrate_measurement.hpp
template <typename Pim>
void integrateMeasurement(Pim& pim, const ImuData& measurement, double dt) { ... }
```

该模板调用 `pim.integrateMeasurementMeanOnly()`，设计用于完整的 `EquivariantPreintegration` 类。但实际上：
- `MeanOnlyGal3Integrator` 有自己的 `integrate()` 方法
- `EquivariantPreintegration` 从未在 sapphire 中被实例化
- `pipeline.cpp` 不 include 此文件
- `deskew.cpp` 不 include 此文件

**结论：死代码，可删除。**

### 2.2 `external/preintegration/` — 仅 lie/ 被使用

vendored 的完整等价预积分库：
- `preintegration.hpp` (319行) — EquivariantPreintegration 类
- `state.hpp` — Gal3TG × Vec10 状态类型
- `params.hpp` — IMU 噪声参数
- `input.hpp` — 加速度+角速度输入类型
- **`lie/Gal3.hpp` (594行)** — 被 `mean_only_gal3_integrator.hpp` 和 `deskew.cpp` 使用 ✅
- **`lie/SO3.hpp`** — Gal3 依赖 ✅
- **`lie/SEn3.hpp`** — Gal3 依赖 ✅
- **`lie/TG.hpp`** — Gal3 依赖 ✅

CMakeLists 中 `external/preintegration` 被加入 include path：
```cmake
target_include_directories(sapphire PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/preintegration>
)
```

但 `preintegration.hpp` 的 include 路径为 `#include <preintegration.hpp>`（尖括号），CMake 配置使其可见，然而实际上从未被任何 sapphire 源文件 include。`mean_only_gal3_integrator.hpp` 确实 `#include <preintegration.hpp>`，但该文件定义的是 `detail::MeanOnlyGal3Integrator`，它使用 `lie::Gal3`（来自 `external/lie/Gal3.hpp`，通过 `preintegration.hpp` → `state.hpp` → `Gal3.hpp` 链引入），但从不使用 `EquivariantPreintegration` 类本身。

实际上 `mean_only_gal3_integrator.hpp` 的 `#include <preintegration.hpp>` 是通过 include path 找到 `external/preintegration/preintegration.hpp`，然后这个文件 include `state.hpp` → `Gal3.hpp`。所以这个 include 链是间接获取 Gal3 的路径。

**问题：include 意图不清晰。** `mean_only_gal3_integrator.hpp` 真正需要的只有 `lie::Gal3`，但它通过 `preintegration.hpp`（完整PIM库入口）间接获取，引入了不必要的编译依赖。

**建议：** `mean_only_gal3_integrator.hpp` 直接 `#include <lie/Gal3.hpp>`，然后可以：
- 从 CMake include path 中移除 `external/preintegration`（如果不再需要）
- 或者保留 include path 但让代码意图更清晰

### 2.3 `(void)noise` — 未使用参数信号

```cpp
// deskew.cpp:298
DeskewResult deskew(..., const ImuNoiseConfig& noise, ...) {
    (void)noise;  // <-- 接受但立即丢弃
```

`noise` 参数是完整 EquivariantPreintegration 所需（构建 `Params`），但 `MeanOnlyGal3Integrator` 不需要。该参数保留在公共 API 中，但实现中显式丢弃。

**建议：** 保留参数签名（API 稳定性），但将 `(void)noise` 替换为文档注释说明为何保留。

### 2.4 GTSAM 无条件构建依赖

```cmake
# CMakeLists.txt:11
find_package(GTSAM REQUIRED)
```

即使 `pgo.enabled = false`，GTSAM 仍需安装。这对只想用前端里程计的用户是额外负担。

**建议 (P0):** 参考 `DESIGN.md` 中的条件编译方案，或至少标记为可选：
```cmake
find_package(GTSAM QUIET)
if(GTSAM_FOUND)
    target_compile_definitions(sapphire PUBLIC SAPPHIRE_HAS_GTSAM)
endif()
```

### 2.5 重复的 `kMaxIntegrationStepSec` 定义

```cpp
// integrate_measurement.hpp:11
constexpr double kMaxIntegrationStepSec = 0.02;

// mean_only_gal3_integrator.hpp:37 (类内部)
constexpr double kMaxIntegrationStepSec = 0.02;
```

两者的值相同但定义位置不同。`integrate_measurement.hpp` 是模板封装（死代码），`mean_only_gal3_integrator.hpp` 是实际使用的副本。

**结论：如果删除 `integrate_measurement.hpp`，此问题自动消失。**

---

## 三、CPU 热点分析

### 3.1 Deskew 积分 — 已知瓶颈

已有专门审查 `docs/code-review/REVIEW_2026-07-14_pim.md` 详细分析。要点：
- Release 20K ts: ~7 ms/帧 → 可接受
- Debug 20K ts: ~1.5 s/帧 → 不能跑
- O(N_unique_timestamps) 精确积分 vs DLIO 的 O(N_imu_intervals) + 区间内插值
- 优化方案已排优先级：时间分片 (P1) > 轻量积分器 (P2)

### 3.2 体素滤波器分配模式

每个 deskew 扫描调用 `deterministicVoxelDownsample`，分配：
- `std::vector<IndexedVoxel>` — N × 48 bytes (5× int64 + size_t + bool)
- 20K 点 → ~960 KB
- parallel sort (small_gicp::quick_sort_omp) 后串行 centroid 归约

**调用于：**
1. `downsamplePoints` — 每帧 LiDAR 一次（将 deskewed cloud 降采样用于 registration）
2. `SubmapManager::rebuildTarget` — 仅在关键帧集合变化时触发
3. `PoseGraphBackend::voxelized` — PGO 回环搜索时对 source/target 各一次
4. `PoseGraphBackend::rebuildGlobalMap` — 按需触发

**评估：** 分配量大但模式合理。`IndexedVoxel` 比 naive pair<key, index> 更紧凑，sort 后单次遍历，无重复分配。

### 3.3 Submap 重建

```cpp
// submap.cpp:93-106
void SubmapManager::rebuildTarget() {
    auto merged = std::make_shared<PointCloud>();
    size_t point_count = 0;
    for (const size_t index : active_indices_) {
        point_count += keyframes_[index].cloud_world->size();
    }
    merged->reserve(point_count);
    for (const size_t index : active_indices_) {
        *merged += *keyframes_[index].cloud_world;  // 逐点拷贝
    }
    target_ = deterministicVoxelDownsample(*merged, config_.voxel_size);
}
```

每次关键帧集合变化时重建整个 submap（合并 N_kf 个点云 + 降采样）。10 帧 × 5K 点 = 50K 点合并 + 降采样 → ~2.4 MB 分配。

**优化空间 (P2):** 增量更新方案——仅重新合并变化的帧，但对 10 Hz LiDAR 来说 10 帧 submap 足够小，非瓶颈。

### 3.4 PGO 回环搜索

```cpp
// pose_graph.cpp:311-346
int searchLoopTarget(size_t query_id, const Isometry3d& query_pose) const {
    // O(candidate_count) 线性搜索，candidate_count 随运行时间增长
    for (size_t candidate = 0; candidate < candidate_count; ++candidate) {
```

每次回环检测对所有历史帧做 O(N) 搜索。`loop_search_stride = 1` 时每帧都搜索。对于 >1000 帧的场景，搜索成本变为 O(N²)。可以用 KD-tree 空间索引优化。

**现状：** 每 1 秒运行一次，目前帧数较少时无问题。

### 3.5 PCL ICP for Loop Registration

```cpp
// pose_graph.cpp:370
pcl::IterativeClosestPoint<Point, Point> icp;
icp.setMaximumIterations(50);
icp.setMaxCorrespondenceDistance(config_.loop_search_radius * 2.0);
```

回环检测使用 PCL 原生 ICP（点到点），而非 frontend 使用的 small_gicp（GICP/VGICP）。这是一个有意的工程选择——ICP 更快，适合大范围回环搜索。但与 frontend 使用不同算法，需注意精度差异。

---

## 四、PGO 实现质量评估

### 4.1 架构决策 ✅

| 决策 | 评价 |
|---|---|
| PIMPL 隔离 GTSAM | ✅ 公共头文件无需引入 GTSAM |
| 异步 worker thread | ✅ 前端不受 PGO 延迟影响 |
| 独立的 keyframe selection | ✅ PGO keyframe 策略独立于前端 submap |
| 增量 ISAM2 | ✅ 不每次重建整个因子图 |
| T_map_odom 校正模式 | ✅ 前端保持 odom 帧，PGO 发布校正 |

### 4.2 因子图构建

```cpp
// pose_graph.cpp:271-309
bool buildOdometryGraph(gtsam::NonlinearFactorGraph& graph, gtsam::Values& initial) {
    // 第一帧: PriorFactor (tight)
    // 相邻帧: BetweenFactor with odom_noise_
    // 初值: last_optimized * delta_odom
```

- PriorFactor variance: `1e-12`（极紧，将第一帧固定在世界原点）
- Odom noise: translations `1e-6`, rotations `1e-4`（信任前端姿态 > 位置）
- 每个 worker cycle 增量添加新的 odometry 边

### 4.3 回环检测与验证

多级过滤 → ICP 验证 → Cauchy robust noise model：

1. 空间距离 < search_radius
2. 时间分离 > min_time_separation
3. 旅行距离 > min_travel_distance
4. 角度差异 < max_rotation（排除反向回环）
5. ICP fitness < fitness_threshold
6. Cauchy robust kernel 降权异常值

**质量评价：** 过滤链完整，robust kernel 保护正确。✅

### 4.4 潜在问题

1. **回环边使用 ICP 而非 GICP** — 精度可能低于 frontend 的 GICP/VGICP 配准，但搜索速度更快，是合理的工程折衷。
2. **loop_noise 权重基于 fitness** — `variances.setConstant(std::max(fitness, 1e-9))`。fitness 是 ICP 的均方距离，作为信息矩阵的逆方差是常见做法，但 scaling 可能需要根据传感器调优。
3. **无回环一致性检查** — 接受的回环边直接加入因子图，没有二次几何验证（如检查回环边与 odometry 边的残差一致性）。

---

## 五、代码质量细节

### 5.1 ✅ 优秀的实践

| 实践 | 位置 | 说明 |
|---|---|---|
| RAII 贯穿始终 | 全局 | `unique_ptr`, `shared_ptr`, 无 raw new/delete |
| 锁顺序文档化 | `pipeline.hpp` | `imu_mutex_ → state_mutex_` 注释清晰 |
| PIMPL 隔离 | `pose_graph.hpp:75` | `std::unique_ptr<Impl>` 隐藏 GTSAM |
| 确定性体素滤波 | `voxel_filter.cpp` | point_index tie-breaker 保证可复现 |
| 配置验证完整 | `config.cpp:314-486` | 所有字段均有边界检查 |
| 异常安全 | `pipeline.cpp:180` | throw + 文档化前提条件 |
| 自文档化错误码 | `deskew.hpp:15-23` | 7 种 DeskewStatus 枚举 |
| `std::scoped_lock` 避免死锁 | `pipeline.cpp:89,179,530` | 多处双锁原子获取 |
| MID-360 crop box 特殊处理 | `types.hpp:151` | 注释解释为何移除 body points |

### 5.2 ⚠️ 需要改进

| 问题 | 位置 | 严重度 | 建议 |
|---|---|---|---|
| fprintf + spdlog 混用 | `imu_init.cpp:52,69,76,183` vs `L238` | LOW | 进度条 (`\r` flush) 需要 fprintf；最终结果用 spdlog。添加注释解释 |
| Registration 构造日志 | `registration.cpp:27-34` | LOW | 5 行 info 打印所有 GICP 参数。已清理 pipeline 构造日志，此处应同步精简 |
| `(void)noise` 无注释 | `deskew.cpp:298` | LOW | 替换为 `// noise unused: MeanOnlyGal3Integrator does not require IMU noise params` |
| 必填字段散落 | `config.cpp` | LOW | `requiredNumber` 用于少数字段（gravity），其余字段有默认值并可选。不一致——某些字段是半必需的 |
| CMake TODO 注释 | `CMakeLists.txt:143` | LOW | `# TODO: add as modules are implemented: find_package(TBB)` — 清理 |
| `target_set_` 冗余 | `registration.hpp:46` | LOW | 可改为检查 `gicp_.getInputTarget()` |

### 5.3 线程安全审计

| 操作 | 锁 | 安全？ |
|---|---|---|
| `latestResult()` 读 + PGO 校正 | `output_mutex_` → 释放 → `impl_->correction()` (自己的锁) | ✅ 非原子但时序正确 |
| `latestPropagatedResult()` | `scoped_lock(state_mutex_, output_mutex_)` | ✅ 双锁原子获取 |
| `rebasePropagation()` | `scoped_lock(imu_mutex_, state_mutex_)` | ✅ 双锁原子获取 |
| `pushImu()` 后初始化路径 | `scoped_lock(imu_mutex_, state_mutex_)` | ✅ 双锁原子获取 |
| PGO `addFrame()` / `correction()` | `input_mutex_` + `output_mutex_` (独立锁集) | ✅ 与前端完全隔离 |
| `propagateStateLocked()` | 调用者持有 `state_mutex_` | ✅ 文档化前提 |

**审计结论：** 无死锁风险，锁顺序一致，文档化清晰。✅

---

## 六、依赖分析

### 6.1 构建依赖

| 依赖 | 使用位置 | 必需？ | 备注 |
|---|---|---|---|
| Eigen3 | 全局（类型、变换） | ✅ 必需 | |
| spdlog | 全局（日志） | ✅ 必需 | |
| PCL | types.hpp (PointCloud), deskew, submap, pipeline, pose_graph | ✅ 必需 | 用于点云类型和 transform |
| GTSAM | pose_graph.cpp | ⚠️ 仅 PGO 需要 | 应条件构建 |
| OpenMP | voxel_filter, deskew | ✅ 必需 | 并行降采样和 deskew |
| small_gicp | registration.cpp | ✅ 必需 | GICP/VGICP 配准 |
| tomlplusplus | config.cpp | ✅ 必需 | TOML 配置 |
| Threads | pose_graph.cpp (worker thread) | ✅ 必需 | |

### 6.2 ROS2 依赖（wrapper 层）

`sapphire_ros2` 依赖：`rclcpp`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`, `tf2_ros`, `tf2_eigen`, `pcl_ros`, `ament_index_cpp`

所有这些依赖都是 ROS wrapper 层的合理需求，无冗余。

---

## 七、测试覆盖

| 测试 | 覆盖模块 | 类型 |
|---|---|---|
| `config_test` | TOML 加载 + 验证 | 单元 |
| `deskew_test` | deskew 算法 | 单元 |
| `ring_buffer_test` | RingBuffer 容器 | 单元 |
| `preintegration_test` | Mean-only vs 完整积分等价性 | 单元 |
| `mean_only_gal3_integrator_test` | Gal3 积分器 | 单元 |
| `pipeline_test` | 完整 pipeline (IMU init + LiDAR) | 集成 |
| `pose_graph_test` | PGO 后端 | 单元 |
| `observer_test` | 几何观测器 | 单元 |
| `registration_test` | GICP 配准 | 单元 |
| `submap_test` | 关键帧管理 | 单元 |
| `voxel_filter_test` | 体素降采样 | 单元 |

**覆盖评价：** 11 个测试覆盖所有模块，包括集成测试。✅

待补：无 `imu_init` 独立单元测试（初始化逻辑在 `pipeline_test` 中间接覆盖）。

---

## 八、与 DLIO 的关键差异

| 维度 | DLIO | Sapphire |
|---|---|---|
| 预积分 | 传统 Forster 9 维 | Gal(3) 等变（10 维 + 紧耦合 bias） |
| 去畸变策略 | 区间内插值 (slerp+lerp) | 逐时间戳精确 Gal(3) 恢复 |
| 后端 | 无（纯前端） | ISAM2 + ICP 回环 |
| 体素滤波 | PCL VoxelGrid | 自研确定性 centroid filter |
| 配准 | small_gicp | small_gicp（同） |
| 观测器 | 几何观测器 | 几何观测器（同） |
| 依赖 | ROS 1 | ROS 2（核心库无 ROS 依赖） |

Sapphire 在保持 DLIO 精度的同时，增加了 PGO 回环和更好的跨平台性。

---

## 九、优化优先级总览

```
┌──────────┬─────────────────────────────────────────────────────────┐
│ P0 立刻  │ GTSAM 条件构建 — find_package(GTSAM QUIET)             │
├──────────┼─────────────────────────────────────────────────────────┤
│ P1 清理  │ 删除 integrate_measurement.hpp（死代码）                │
│          │ Registration 构造日志精简                               │
│          │ (void)noise → 文档注释                                  │
│          │ mean_only_gal3_integrator.hpp 直接 #include <lie/Gal3> │
├──────────┼─────────────────────────────────────────────────────────┤
│ P2 改进  │ Deskew 时间分片（见 PIM 审查）                          │
│          │ PGO loop search 空间索引 (KD-tree)                      │
│          │ 清理 CMakeLists.txt TODO 注释                           │
│          │ 补 imu_init 独立单元测试                                │
├──────────┼─────────────────────────────────────────────────────────┤
│ P3 远期  │ PGO loop registration 升级为 GICP                       │
│          │ Submap 增量更新                                         │
│          │ 回环一致性检查（二次几何验证）                          │
└──────────┴─────────────────────────────────────────────────────────┘
```

---

## 十、一句话总结

**高质量的 production-grade LIO 系统，核心架构决策正确，锁策略严谨，测试覆盖完整。主要待改进项是死代码清理和 GTSAM 条件构建——其余都是锦上添花的优化。**

---

## 附录: 文件统计

| 类别 | 文件数 | 总行数 |
|---|---|---|
| 头文件 (include/) | 11 | ~1,250 |
| 源文件 (src/) | 9 | ~3,200 |
| 外部库 (external/) | 8 | ~2,800 |
| 测试 (tests/) | 11 | ~2,500 (估算) |
| 配置 (cfg/) | 1 | 92 |
| ROS2 wrapper | 4 | ~600 |
| **总计** | **44** | **~10,400** |

### 相关审查文档

- `REVIEW_2026-07-11.md` — 初始审查（架构 + 首批问题）
- `REVIEW_2026-07-11_v2.md` — 第二轮（mean-only 验证 + profiling）
- `REVIEW_2026-07-14.md` — AI-cruft 清理审查（日志/诊断/死代码，-302 行）
- `REVIEW_2026-07-14_pim.md` — PIM/预积分性能深度分析（Debug vs Release + deskew 优化）
- `REVIEW_2026-07-16_comprehensive.md` — 本文档（全量代码审查）
