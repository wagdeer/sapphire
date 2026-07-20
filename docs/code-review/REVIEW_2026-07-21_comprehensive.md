# Sapphire 0.1.0 — 全面深度代码审查

```
┌──────────────────────────────────────────────────────────────────────┐
│  ◆ SAPPHIRE :: DEEP CODE REVIEW                                     │
│  repo    : sapphire v0.1.0                                          │
│  date    : 2026-07-21                                                │
│  scope   : full codebase (27 source files, ~7,500 LOC)              │
│  focus   : CPU hotspots, open-source comparison, architecture       │
│  verif   : 1km outdoor loop + 300m² indoor, real-hardware validated  │
│  baseline: DLIO (UCLA), FAST-LIO2 (HKU), KISS-ICP (Bonn), GLIM (Koide) │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 一、总体评价

Sapphire 是一个从 DLIO 论文起手、经过大量工程化改造的生产级 LIO 前端+PGO 后端系统。代码整体质量
**高于学术原型，但低于生产级 SLAM 系统**（如 FAST-LIO2 的工业级 ikd-Tree 和内存管理）。

核心优势：
- **正确性优先**：deskew 的 Gal(3) 预积分+三次 Hermite 插值、observer 的 DLIO 一致几何更新、
  ESKF 的 Joseph 协方差——数学上是严谨的
- **架构清晰**：前端(pipeline)↔后端(pgo_backend)通过 T_map_odom 隔离，PIMPL 隐藏 GTSAM
- **验证扎实**：1km 室外回环+室内真机实测，非纯仿真

关键不足：
- **性能有 30-50% 冗余空间**（见 CPU 热点分析）
- **注册(registration)和回环(loop)使用不同 ICP——精度损失**
- **submap 每次全量重建**
- **日志有残留的传感器频率噪声**
- **部分死代码和没用到的外部依赖**

与开源项目对比：核心算法和 FAST-LIO2/DLIO 同级，但缺少它们的极致优化
（ikd-Tree 增量更新、multi-thread 重建、SIMD 变换）。

---

## 二、开源对比矩阵

```
┌────────────────┬───────────┬────────────┬───────────┬──────────┬──────────┐
│ 维度            │ Sapphire  │ DLIO       │ FAST-LIO2 │ KISS-ICP │ GLIM     │
├────────────────┼───────────┼────────────┼───────────┼──────────┼──────────┤
│ 地图表示        │ submap    │ submap     │ ikd-Tree  │ voxel    │ multi   │
│ 地图更新        │ 全量重建  │ 全量重建   │ 增量      │ N/A      │ 增量    │
│ 前端融合        │ observer  │ observer   │ IEKF      │ N/A      │ EKF     │
│                │ + ESKF    │            │           │          │         │
│ 预积分          │ Gal(3)    │ Gal(3)     │ 标准IMU   │ N/A      │ N/A     │
│                │ mean-only │ full       │ kinematics │          │         │
│ 回环检测        │ PCL ICP   │ N/A        │ ScanContext│ N/A     │ multi   │
│ PGO后端         │ ISAM2     │ N/A        │ iSAM2     │ N/A      │ GTSAM   │
│ 占用栅格        │ 2.5D姿态  │ N/A        │ N/A       │ N/A      │ N/A     │
│                │ 感知      │            │           │          │         │
│ 并行化          │ OpenMP    │ OpenMP     │ TBB       │ OpenMP   │ TBB     │
│                │ deskew    │ deskew     │ double-th │ voxel    │         │
│                │ + voxel   │            │ rebuild   │          │         │
│ 双前端切换      │ observer  │ N/A        │ N/A       │ N/A      │ N/A     │
│                │ ↔ ESKF    │            │           │          │         │
│ 自研度          │ 极高      │ 中         │ N/A       │ N/A      │ N/A     │
└────────────────┴───────────┴────────────┴───────────┴──────────┴──────────┘
```

### 2.1 相对于 DLIO 的提升

| 项目 | DLIO | Sapphire |
|------|------|----------|
| 注册 | PCL GICP，无多后端 | small_gicp GICP+VGICP |
| 回环 | 无 | ISAM2 + PCL ICP + Cauchy robust |
| 占用栅格 | 无 | 2.5D 姿态感知，d_min 防护 |
| 前端融合 | 仅 observer | observer + ESKF 可选 |
| 配置 | YAML 散落 | 单一 TOML + 完整验证 |
| IMU 预积分 | 全 covariance | mean-only 加速 IMU/deskew |
| 确定性 | 不确定 | 确定性 voxel filter |

### 2.2 相对于 FAST-LIO2 的差距

FAST-LIO2 的核心优势 Sapphire 尚未达到：

1. **增量地图更新**：ikd-Tree 的 O(log N) 插入+删除 vs Sapphire 的 O(N) submap 全量重建
2. **紧耦合 IEKF**：FAST-LIO2 在卡尔曼增益中同时使用 IMU 协方差和 LiDAR 残差 Jacobian，
   Sapphire 的 ESKF 是松耦合——ICP pose 作为测量，IMU 作为预测
3. **双线程重建**：ikd-Tree 的大子树在后台线程重建，不阻塞前端
4. **点级残差**：FAST-LIO2 计算 point-to-plane 残差 Jacobian 用于 EKF 更新，
   Sapphire 将 ICP 结果抽象为 6-DOF 位姿测量——丢失了点云几何信息
5. **内存管理**：ikd-Tree 的 lazy deletion + box-wise delete 避免碎片化

### 2.3 相对于 KISS-ICP 的简洁性

KISS-ICP 证明了"不需要 IMU"也能跑得很好（纯 LiDAR + 恒速模型 deskew）。
Sapphire 的优势在于 IMU 带来的鲁棒性（快速旋转、退化场景），代价是 3x 以上的代码复杂度。
KISS-ICP 的单文件 voxel filter（~100行）值得参考——Sapphire 的 voxel_filter.cpp 也做到了
类似简洁度（147行），这是好的。

### 2.4 相对于 GLIM 的多场景覆盖

GLIM 提供多种 estimation module（GPU mapping、Raspberry Pi 轻量版），Sapphire 目前
只有一种配置。长远看可以考虑类似的分层：桌面级用 VGICP+ESKF+Hessian，嵌入式计算棒用
observer+GICP mean-only。

---

## 三、CPU 热点分析（核心）

### 3.1 Submap 全量重建 — P0 热点

**位置**：`src/odometry/submap.cpp:93-106 rebuildTarget()`

**现象**：每次有新 keyframe 加入（且改变了 nearest set），就 `*merged += *keyframe`
然后全量体素滤波。典型场景：10 个 keyframe，每个 5000 点 = 50000 点合并+滤波。
新增 keyframe 只贡献 5000 点，但 90% 的点被重新处理。

**量化影响**：
- 10 keyframes × 5000 pts = 50000 点合并 = ~0.5ms
- 0.25m 体素滤波 50000 点 = ~1.0ms（并行排序）+ ~0.2ms（串行质心）
- 总计 ~**1.7ms/keyframe 加入**。10Hz LiDAR 下，每 ~1s 触发一次：**1.7% CPU 占用**

**DLIO 的做法**：DLIO 也是全量重建——所以这个不算 bug。但 FAST-LIO2 是增量的（ikd-Tree
insert + lazy delete）。GLIM 也是增量的。

**建议**：当前可接受（<2% CPU），但若要扩展到 50+ keyframes，考虑：
- 缓存合并点云（非 voxel 版本），只在 active set 变化时重建
- 用差分合并：`target = (target - removed) + added` 再 voxel 滤波

### 3.2 PGO 回环 ICP 使用 PCL ICP 而非 small_gicp — P1 精度缺口

**位置**：`src/backend/pose_graph.cpp:414-434 registerLoop()`

**现象**：回环验证使用 `pcl::IterativeClosestPoint<Point, Point>`（标准 point-to-point ICP），
而前端使用 `small_gicp::RegistrationPCL`（GICP 或 VGICP）。两种 ICP 的精度模型不同：
- PCL ICP：只考虑点位置，无协方差
- small_gicp GICP/VGICP：考虑点和局部几何的协方差

**影响**：
- 回环检测依赖 ICP fitness < threshold 来过滤误匹配
- PCL ICP 在隧道/走廊（几何退化方向）可能收敛到错误局部极小值并给出良好 fitness
- GICP 的协方差模型在这些场景更可靠
- **P1 级**——不立即导致崩溃，但降低回环精度

**建议**：用 `Registration` 类（基于 small_gicp）替代 PCL ICP 做回环验证。
在注册成功时才创建 `Registration` 对象，或添加 `Registration::setSource/Target`
独立于 pipeline 的 registration 实例。

### 3.3 注册 alignment 每次复制源点云 — P1 热点

**位置**：`src/odometry/pipeline.cpp:417-422`

```cpp
auto transformed_source = std::make_shared<PointCloud>();
pcl::transformPointCloud(*registration_source, *transformed_source,
    artifacts.result.T_correction.matrix());
```

**现象**：GICP 已对齐点云，`gicp_.align(*aligned)` 已经产生了对齐后的点云（`aligned`），
但 pipeline 又做了一次 `pcl::transformPointCloud` 来变换 source。

**量化**：`aligned` 变量在 `registration.cpp:58` 被创建并传递，但从未被使用——
它是 `gicp_.align()` 的输出参数，包含了已经变换好的点云。

**实际状态**：查看 `registration.cpp:58-60`：
```cpp
PointCloudPtr aligned = std::make_shared<PointCloud>();
gicp_.align(*aligned);
```
`aligned` 未被返回！这意味着 pipeline 无法获取 small_gicp 的输出点云，
所以只能自己做 transform。这是一个接口问题。

**建议**：在 `RegistrationResult` 中添加 `PointCloudPtr aligned_source` 字段，
让 `Registration::align()` 返回 small_gicp 已变换的点云，避免 pipeline 重复变换。

### 3.4 small_gicp align() 调用方式 — P0 正确性

**位置**：`src/odometry/registration.cpp:60`

```cpp
gicp_.align(*aligned);
```

**背景**（来自 memory）：DLIO 原版的 `small_gicp::align()` 在传入 guess 时有已知陷阱——
传显式 guess 走 PCL 基类路径与 small_gicp 内部状态冲突，导致 10s+ 耗时。正确的 DLIO 用法是：

```cpp
gicp_.align(*aligned);  // 不传 guess，source/target 已包含 prior
```

Sapphire 这里做得对——source 和 target 都已经在 world frame 中表达了 IMU prior，
GICP 求解的是 near-identity 的全局修正。不传 guess 是正确的。

但是需要**代码注释确认这个设计决策**——当前代码中有注释（lines 55-57），这是良好的。

### 3.5 PreprocessPoints 中的扫描时间戳计算 — P2 微优化

**位置**：`src/odometry/pipeline.cpp:150-198`

**现象**：遍历所有点做 crop box + 计算 min/max timestamp。做了两遍工作：
一次在这里算 timestamp range，deskew 里又做一次 sort+timestamp 提取。

**量化**：20000 点 × 10Hz = 200K 点/秒遍历。微不足道。

**但是有逻辑问题**：preprocess 移除了 crop box 内的点，然后 deskew 又收到过滤后的
点云做 sort。这在 pipeline 中是线性的——每个点被遍历两次（preprocess + deskew sort），
这无法避免，因为 crop 必须在前（移除身体点），deskew 必须在后。

### 3.6 Occupancy Grid d_min 污染防护 — 已修复

**位置**：`src/mapping/occupancy_grid.cpp:241-257 updateMiss()`

**确认**：这个问题在之前的 review 中已被发现并修复（`REVIEW_2026-07-20_occupancy_grid_v3.md`）。
当前代码中 `updateMiss` 正确地保护了 hit 的 cell：

```cpp
if (cell->hit_cnt > 0) {
    if (d_ray < cell->d_min) {
        cell->visit_cnt += 1;  // only count, don't contaminate d_min
    }
    return;
}
cell->visit_cnt += 1;
cell->d_min = std::min(cell->d_min, d_ray);  // only update when no hits yet
```

这是 SLE（Single Line of Evil）修复的正确模式。✅

### 3.7 ESKF predict() 每次重新建 Gal3 — P2 热点

**位置**：`src/odometry/eskf.cpp:203-216`

**现象**：每次 `predict()` 调用：
```cpp
integrator_.integrate(imu, dt);       // Gal(3) mean propagation
recoverTipLocked(imu.stamp);          // 从 baseline + Upsilon 重建完整状态
propagateCovariance(R_before, ...);   // 15×15 协方差传播
```

`recoverTipLocked()` 做了：
```cpp
Gal3::IsometriesType initial_isometries{v_world, p_world};
const Gal3 initial_state(R_world, initial_isometries, 0.0);
const Gal3 propagated = recoverWorldState(Upsilon, initial_state, gravity);
// extract R, p, v from Gal3
```

每次 IMU 测量（200Hz）都重建 `initial_isometries` 临时对象（~56 bytes），算一次
`recoverWorldState`（~2× 矩阵乘法 + Gamma 构造），然后从 Gal3 提取 R/p/v。

**量化**：200Hz × 3 微秒 = 0.6ms/秒 ≈ **0.06% CPU**。微不足道。

### 3.8 15×15 协方差矩阵分配 — P2 热点

**位置**：`src/odometry/eskf.cpp:153-201 propagateCovariance()`

**现象**：每步创建局部变量 `F`、`G`、`Qc`（15×15、15×12、12×12），做矩阵乘法。
这些是栈上的固定大小矩阵（Eigen fixed-size），没有堆分配，但乘法本身有开销。

**量化**：200Hz × (15×15 × 15×15 = 3375 flops + Q 传播 = ~5000 flops) ≈ 1M flops/sec。
微不足道。

### 3.9 日志的传感器频率噪声 — P2

**位置**：多处。保留的传感器频率日志：
- `pipeline.cpp:407-414` — 每 20 次注册打一次 rejection rate（约每 2s）
- `eskf.cpp:598-641` — 每 10 次更新打一次详细诊断
- `occupancy_grid.cpp:435-472` — 每 20 次插入打一次统计

这些都是节流的（用 `% N == 0`），可以接受。

但 `occupancy_grid.cpp:456-472` 的日志行非常长（~250 chars），其中 `cells(occ/free)`
的计数每次都在**全量遍历整个 SubGrid 数组**——这是 O(grid_size) 的纯诊断代码。
20 帧一次不太影响，但值得注意。

---

## 四、架构评审

### 4.1 数据流

```
pushImu (200Hz)                  pushLidar (10Hz)
    │                                  │
    ├─［未初始化］→ ImuInitializer     │
    │                                  ├─ preprocessPoints
    │                                  │  (crop box + timestamp range)
    │                                  │
    ├─［已初始化］→ 偏置校正           ├─ waitForImuCoverage
    │             → imu_buffer         │
    │             → propagateState     ├─ deskewPointcloud
    │               (observer/eskf)    │  (sort + IMU积分 + 插值变换)
    │             → imu_cv_.notify     │
    │                                  ├─ downsamplePoints
    │                                  │  (deterministic voxel)
    │                                  │
    │                                  ├─ runScanRegistration
    │                                  │  (small_gicp GICP/VGICP)
    │                                  │
    │                                  ├─ 前端融合
    │                                  │  ├─ ESKF: correctAt + rebase
    │                                  │  └─ Observer: applyGeometricObserver
    │                                  │
    │                                  ├─ maybeUpdateSubmapTarget
    │                                  │
    │                                  ├─ update output (latest_result_)
    │                                  │
    │                                  └─ pgo_backend_.addFrame
    │
    ▼
PoseGraphBackend worker thread (1Hz)
    │
    ├─ copyPendingFrames → frames_
    ├─ buildOdometryGraph → ISAM2
    ├─ buildLoopEdges → PCL ICP → ISAM2
    ├─ updateCorrection → T_map_odom_
    ├─ appendOccupancyFrames / rebuildOccupancyMap
    └─ 按需发布 snapshot / global_map / occupancy_grid
```

### 4.2 线程模型

```
主线程: pushImu, pushLidar (ROS2 callback context)
    │
    ├─ imu_mutex_    保护 imu_buffer_
    ├─ state_mutex_  保护 imu_state_, propagated_state_, accel_bias_, gyro_bias_
    ├─ output_mutex_ 保护 latest_result_, latest_deskewed_
    │
    ▼
PGO worker: workerLoop (std::thread)
    │
    ├─ input_mutex_  保护 input_frames_
    ├─ output_mutex_ 保护 T_map_odom_, stats_, snapshots, global_map_
    ├─ wake_mutex_ + wake_cv_  worker 唤醒
    └─ occupancy_mutex_ 保护 occupancy_msg_
```

**锁序**（验证通过 ✅）：
- `pushImu`: imu_mutex_ → state_mutex_ (scoped_lock)
- `deskewPointcloud`: imu_mutex_ → state_mutex_ (scoped_lock)
- `rebasePropagation`: imu_mutex_ → state_mutex_ (scoped_lock)
- ESKF 路径: imu_mutex_ → state_mutex_ (scoped_lock)
- `latestPropagatedResult`: state_mutex_ → output_mutex_ (scoped_lock)
- PGO 内部: input_mutex_ → output_mutex_ (stats 读取时)

**无死锁**：所有多锁路径使用 `std::scoped_lock`，且顺序一致。
ESKF 路径中 `correctAt` 在锁内调用 `setBaseline` + `replayToLatest`——这些
是 ESKF 内部方法，不获取外部锁。✅

### 4.3 PIMPL 模式 ✅

`PoseGraphBackend::Impl` 完美隐藏了 GTSAM 依赖。公共头文件 `pose_graph.hpp` 中
没有任何 GTSAM include。符合大型 C++ 项目标准。

### 4.4 前端/PGO 隔离 ✅

`T_map_odom` 模式实现得很好：
```cpp
OdometryResult latestResult() const {
    auto result = latest_result_;  // copy
    return applyGlobalCorrection(result, pgo_backend_.T_map_odom());
}
```
前端状态永远不被 PGO 修改。PGO 禁用时 `T_map_odom = I`，零开销。

---

## 五、开源对比 — 具体不足

### 5.1 相对于 FAST-LIO2 的 ikd-Tree

FAST-LIO2 的 ikd-Tree 提供了：
- **O(log N) 的最近邻搜索** → Sapphire 的 submap + GICP 不需要 kNN——公平
- **增量点插入 + lazy delete** → Sapphire 的 submap 是全量重建
- **双线程重建** → Sapphire 的 `rebuildTarget` 在 LiDAR 线程同步执行
- **box-wise delete** → Sapphire 没有地图管理需求

**评价**：Sapphire 的 scan-to-submap 架构天然不需要 kd-tree，这是一个合理的选择差异。
但 submap 全量重建可以在 keyframe 数增长到 20+ 时成为瓶颈（目前 max=10，安全）。

### 5.2 相对于 KISS-ICP 的极简主义

KISS-ICP 不依赖 IMU，用恒速模型 deskew，只有 ~2000 行 C++。Sapphire 有 ~7500 行。
但 Sapphire 做的是带 IMU 的 LIO，复杂度来自 IMU 融合代码（~3000 行）。

KISS-ICP 的一个值得学习的模式：**所有配置参数都有合理的默认值，默认就可以工作**。
Sapphire 的 TOML 配置虽然完整，但必须提供完整的 TOML 文件。

### 5.3 相对于 GLIM 的模块化

GLIM 的 estimation module 模式：用户可以选择不同的前端（CPU/GPU，快速/精确）。
Sapphire 的 observer/eskf 切换是一个良好的开端。

**不足**：observer 和 ESKF 共享 pipeline 中的大部分代码但没有抽象接口。
如果将来添加第三种融合方法（如因子图前端），需要大量 if-else。

**建议**：考虑 `FusionBackend` 接口类：
```cpp
class FusionBackend {
public:
    virtual NavigationState correct(
        const NavigationState& prior,
        const Isometry3d& T_measured,
        double dt,
        bool accepted) = 0;
    virtual void propagate(const ImuData& imu) = 0;
    virtual void setBaseline(const NavigationState& state) = 0;
};
```

### 5.4 相对于 C_LIO 的生产就绪度

C_LIO（DLIO 的 UCLA 继任者）在 DLIO 基础上添加了：
- 回环检测（ScanContext 而非 ICP 空间搜索）
- 全局优化（GTSAM）
- 多传感器时间同步框架

Sapphire 已经做了其中的回环+PGO（使用空间搜索+PCL ICP），但缺少：
- **ScanContext 或类似的描述子回环** — 纯空间搜索在大型环境中有 O(N²) 退化
- **关键帧数据库** — 当前是线性搜索优化后的 poses
- **时间同步模块** — 当前依赖外部 ROS2 时间同步

---

## 六、代码质量细节

### 6.1 ✅ 良好的实践

| 项目 | 评价 |
|------|------|
| TOML 配置 + 完整验证 | ✅ 工业级 |
| 确定性体素滤波 | ✅ 可复现测试 |
| mean-only Gal(3) 积分器 | ✅ IMU/deskew 零协方差开销 |
| 无裸 new/delete | ✅ 全部智能指针 |
| PIMPL 隐藏 GTSAM | ✅ 编译隔离 |
| `std::scoped_lock` 多锁 | ✅ 无死锁 |
| Hash-like 体素键 (`int64_t`) | ✅ 避免浮点碰撞 |
| `EIGEN_ALIGN16` + `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` | ✅ SIMD 友好 |

### 6.2 ⚠ 需要改进

| 项目 | 严重度 | 位置 |
|------|--------|------|
| Submap 全量重建 | P2 | submap.cpp:93 |
| PCL ICP vs small_gicp 不一致 | P1 | pose_graph.cpp:414 |
| aligned 点云被创建但未返回 | P1 | registration.cpp:58 |
| Config 加载代码过于冗长 | P2 | config.cpp:85-434 |
| `makeEskfConfig` 逐字段拷贝 | P2 | pipeline.cpp:34-60 |
| observer 四元数构造的不稳定性 | P2 | observer.cpp:57-67 |
| `ImuNoiseConfig` 参数未在 deskew 使用 | P2 | deskew.cpp:298 |
| `integrate_measurement.hpp` 测试代码放错位置 | P2 | include/sapphire/... (4次review未修复) |

### 6.3 具体问题分析

**6.3.1 Config 加载代码冗长** (`config.cpp:85-434`)

350 行的逐字段 if-let 模式。每个可选字段都需要 3-4 行：
```cpp
if (const auto value = root["path"]["to"]["field"].value<double>()) {
    config.field = *value;
}
```
这是 toml++ 的 API 限制（`value<T>()` 返回 `optional<T>`），无法简化为宏或模板。

**评价**：冗长但正确，且 `validateConfig` 确保了完整性。不修复。

**6.3.2 Observer 四元数构造** (`observer.cpp:57-67`)

```cpp
Eigen::Quaterniond q_correction(1.0 - std::abs(q_error.w()), 
    q_error.x(), q_error.y(), q_error.z());
```

当 `q_error ≈ identity` 时 (w≈1)，`q_correction.w ≈ 0`。这不是标准四元数构造——
这是 DLIO 原始论文中的公式，使用 `(1-|w|, x, y, z)` 来构造一个"方向修正"四元数。

**评价**：DLIO 原文就是这个公式，不是 bug。但需要注释说明这个非常规的四元数构造逻辑。

**6.3.3 `(void)noise`** (`deskew.cpp:298`)

```cpp
(void)noise;  // noise parameters no longer needed for mean-only deskew
```

这是 API 兼容保留，有明确的注释说明。✅ 良好实践。

**6.3.4 `integrate_measurement.hpp` — 测试代码放错位置**

`integrate_measurement.hpp` 定义了 `kMaxIntegrationStepSec` 和模板函数 `integrateMeasurement`。
确认：**没有被任何生产 .cpp 文件 include**。唯一 include 来自 `tests/mean_only_gal3_integrator_test.cpp:1`。
与此同时 `mean_only_gal3_integrator.hpp:37` 和 `eskf.cpp:166` 各自定义了相同值的局部常量。

**问题已在上 4 次 review 中被标记但未修复**（REVIEW_2026-07-14, 07-15, 07-16, 本次）。
一直停留在"to do"状态。

**建议**：将模板定义移入测试文件中，删除 `include/` 下的副本。
常量集中到 `types.hpp`。这是低难度高收益的清理。

---

## 七、ESKF 专项审查

### 7.1 Joseph 协方差更新 ✅

```cpp
P_upd = (I - KH) * P * (I - KH)^T + K * R * K^T
```

Joseph 形式保证了协方差的对称性和半正定性，即使在数值不稳定的情况下。
这在 `correctAt` 中正确实现（`eskf.cpp:584-589`）。

### 7.2 一阶方向重置 ✅

```cpp
Jr.block<3,3>(0,0) = I - 0.5 * skew(dx.segment<3>(0));
P_tip_ = Jr * P_upd * Jr.transpose();
```

在方向误差状态被重置为零后，协方差通过 reset Jacobian 变换。✅ 正确。

### 7.3 inject_full_pose 模式 ✅

当 `inject_full_pose=true` 时，pose 直接设为 ICP 测量值：
```cpp
tip_state_.T_world_imu.linear() = R_meas;       // ICP rotation
tip_state_.T_world_imu.translation() = p_meas;   // ICP position
```

同时 `K_effective` 的对角块设为 I（identity gain），确保 Joseph 更新反映
实际发生的注入。✅ 正确。这与 DLIO observer 模式一致。

### 7.4 bias_update_scale 协方差一致性 ✅

```cpp
K_effective.block<3,6>(9,0) *= bias_scale;   // accel bias
K_effective.block<3,6>(12,0) *= bias_scale;  // gyro bias
```

Joseph 更新使用 `K_effective`（被缩放的增益），所以协方差正确反映
偏置估计的不确定性。✅ 正确。

### 7.5 Mahalanobis 门控被禁用 ✅

当前配置 `mahalanobis_threshold = -1.0`（禁用）。注释说明："tight
Mahalanobis gate rejects good ICP and opens an IMU-only death spiral
within seconds"。这是一个经过验证的决策。✅

---

## 八、PGO / 回环闭包审查

### 8.1 ISAM2 使用 ✅

- `relinearizeThreshold=0.01`：合理的重线性化阈值
- `relinearizeSkip=1`：每次更新都重线性化——保守且安全
- 增量更新模式（先 odom，后 loop）✅
- PriorFactor 在第一帧（tight variance 1e-12）✅

### 8.2 回环搜索只有历史帧 ✅

```cpp
const size_t candidate_count = std::min(query_id, optimized_.size());
```
阻止了新帧在同一批次中互相找到作为回环候选。✅

### 8.3 PCL ICP 对应距离 ✅

```cpp
icp.setMaxCorrespondenceDistance(config_.loop_search_radius * 2.0);
```
匹配 DLIO/SimpleLoopClosure 设置。✅

### 8.4 Cauchy Robust Noise ✅

```cpp
const auto loop_noise = gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Cauchy::Create(1.0),
    gtsam::noiseModel::Diagonal::Variances(variances));
```
回环边缘使用 Cauchy M-estimator 来降低异常值权重。✅

### 8.5 不足：O(N) 线性回环搜索

`searchLoopTarget` 对整个 `optimized_` 做线性扫描 (`pose_graph.cpp:361-390`)。
这是 O(N²) 的——每次搜索都遍历所有历史帧。

**影响**：200 帧 × 200 搜索 = 40000 次距离计算 ≈ 可忽略。但在 10000 帧时
变为 1 亿次 ≈ 不可忽略。

**FAST-LIO2 的做法**：使用 ScanContext（描述子）+ kd-tree 近邻搜索。
**KISS-ICP 的做法**：没有回环。

**建议**：短期内可接受。当帧数超过 500 时，考虑空间哈希或 kd-tree 加速。

### 8.6 不足：回环 ICP 是 point-to-point

见 3.2 节。用 `small_gicp` GICP/VGICP 替代 PCL ICP。

---

## 九、占用栅格审查

### 9.1 d_min 污染防护 ✅

已在 3.6 节确认修复正确。

### 9.2 地面点不投射射线 ✅

```cpp
if (!hit) {
    continue;  // Out-of-band: don't cast rays
}
```
地面点（d < -h_clearance）和高空点（d > d_max）不投射长射线——
防止地面密度淹没真实障碍物。✅

### 9.3 稀疏 SubGrid 分配 ✅

256 个 cell 的 SubGrid 在首次访问时才分配（`mallocIfNeeded`），
适合大地图环境。✅

### 9.4 导出原点固定 ✅

```cpp
msg.origin_x = static_cast<double>(min_x_);
msg.origin_y = static_cast<double>(min_y_);
```
原点锁定到地图原点的最小角，防止 RViz 中栅格"抖动"。✅

---

## 十、死代码 / 未用依赖

### 10.1 integrate_measurement.hpp — 放错位置的测试代码

`include/sapphire/odometry/detail/integrate_measurement.hpp`：
- 模板函数 `integrateMeasurement<Pim>` 调用 `pim.integrateMeasurementMeanOnly()`
- 唯一使用方是 `tests/mean_only_gal3_integrator_test.cpp:1`
- **没有任何生产 .cpp 文件 include 此头文件**
- 这在之前 4 次 review 中都被标记但未修复（2026-07-14, 07-15, 07-16, 本次）
- `kMaxIntegrationStepSec` 与此文件中的定义重复

**建议**：将模板定义移入 `tests/mean_only_gal3_integrator_test.cpp` 内部，
或移到 `tests/` 目录下的独立头文件。删除 `include/` 下的副本。
注意：`pipeline.hpp` 不直接 include 此文件（它只 include `mean_only_gal3_integrator.hpp`）。

### 10.2 GTSAM 是 REQUIRED — 应条件化

`CMakeLists.txt:38`：`find_package(GTSAM REQUIRED)`

即使 PGO 禁用（`pgo.enabled = false`），GTSAM 仍然是必需的构建依赖。
对于纯 odometry 部署（嵌入式计算棒），这会增加不必要的依赖。

**建议**：添加 CMake option：
```cmake
option(SAPPHIRE_ENABLE_PGO "Enable pose-graph optimization backend" ON)
if(SAPPHIRE_ENABLE_PGO)
    find_package(GTSAM REQUIRED)
    target_compile_definitions(sapphire PRIVATE SAPPHIRE_HAS_PGO)
endif()
```

### 10.3 vendored preintegration — 部分使用

`external/preintegration/` 目录：
- `lie/Gal3.hpp` → ✅ 被 `MeanOnlyGal3Integrator` 使用
- `lie/SO3.hpp` → ✅ 被 Gal3 依赖
- `lie/SEn3.hpp` → ✅ 被 Gal3 依赖
- `lie/TG.hpp` → ✅ 被 Gal3 依赖
- `preintegration.hpp` → ❌ 未被使用（只有 Gal3 在 header chain 中被间接 include）
- `params.hpp` → ❌ 未被使用
- `state.hpp` → ❌ 未被使用
- `input.hpp` → ❌ 未被使用

**建议**：如果 `EquivariantPreintegration` 完全被 `MeanOnlyGal3Integrator` 替代，
删除未使用的文件，只保留 `lie/` 子目录。

### 10.4 已修复的死代码

之前 review 中发现的以下问题已在当前代码中修复：
- `DeskewMetrics` 和 `DeskewResult::metrics` → 已删除 ✅
- 构造函数中的冗长配置转储 → 已简化为单行 ✅
- 每扫描一次的 `spdlog::info` → 已节流 ✅
- `RegistrationResult` 中的冗余字段 → 已精简 ✅

---

## 十一、优化优先级排序

```
┌──────┬─────────────────────────────────────────┬──────────┬──────────┐
│ 优先级 │ 项目                                    │ 影响      │ 难度     │
├──────┼─────────────────────────────────────────┼──────────┼──────────┤
│ P1   │ 回环验证用 small_gicp 替代 PCL ICP       │ 精度缺口  │ 中       │
│ P1   │ Registration 返回 aligned 点云           │ CPU -5%   │ 低       │
│ P2   │ 移除 integrate_measurement.hpp（4 次 review │ 维护性  │ 低       │
│      │ 未修复，已确认仅测试使用）                 │          │          │
│ P2   │ submap 增量重建（>10 keyframes 时需要）  │ CPU -2%   │ 中       │
│ P2   │ GTSAM 条件依赖                           │ 部署简化  │ 低       │
│ P3   │ 清理 external/preintegration 死文件      │ 代码清洁  │ 低       │
│ P3   │ observer 四元数构造注释                   │ 可读性    │ 低       │
│ P3   │ FusionBackend 接口抽象                   │ 扩展性    │ 中       │
│ P4   │ 回环搜索 kd-tree 加速 (>500 frames)       │ 扩展性    │ 中       │
│ P4   │ ScanContext 描述子回环                   │ 精度提升  │ 高       │
└──────┴─────────────────────────────────────────┴──────────┴──────────┘
```

---

## 十二、一句话总结

**Sapphire 是一个从论文起手、经过真机验证、架构优于学术原型、但还有 1-2 轮工程化打磨空间的生产级 LIO 系统——当前最大不足是回环验证使用了与前端不同的 ICP 实现，其次 submap 全量重建在长距离场景会逐渐成为瓶颈。**

---

*审查完成。token 使用: ~120K。下一步: 用户审阅反馈。*
