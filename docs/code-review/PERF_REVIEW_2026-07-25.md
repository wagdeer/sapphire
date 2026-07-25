# Sapphire 性能缺陷审查报告

> **日期**: 2026-07-25  
> **范围**: 全库 ~4000 行 C++，聚焦运行时性能缺陷  
> **重点**: 热路径（IMU 200Hz / LiDAR 10Hz）中的不必要开销

---

## 一、严重程度分类

| 等级 | 定义 | 影响 |
|------|------|------|
| 🔴 Critical | 热路径中重复计算、重复分配或重复变换 | 直接增加单帧延迟 |
| 🟡 High | 热路径中可预计算/可缓存的计算 | 累积开销明显 |
| 🟢 Medium | 非关键路径或已接近最优的微优化 | 边际收益 |

---

## 二、性能缺陷详情

### ✅ 缺陷 1：ESKF 路径点云双重变换（已修复，2026-07-25）

**文件**: `src/odometry/pipeline.cpp`  
**位置**: 行 466-473 和 行 572-588

`processLidarScan` 在 ESKF 模式下对同一帧点云执行了两次完整变换。第一次将降采样后的 source 从 IMU 先验位姿变换到 GICP 优化后的位姿（DLIO 路径需要）。第二次再将其从 GICP 位姿对齐到 ESKF 融合位姿。对于 Mid-360 扫描（~24000 点），每次 `transformPointCloud` 做 4x4 矩阵乘每个 3D 点，总计 ~96k 标量乘加运算。在 10 Hz 扫描频率下，这相当于每秒多做一次 240000 点的变换。

**修复**: 数学上 `T_align × T_correction = T_world_lidar_out × T_world_lidar_ref⁻¹`，`T_correction` 可消掉。两处修改：

1. **L466**: 第一次变换增加 `&& !useEskf()` 条件，ESKF 路径跳过该变换，`artifacts.corrected_source` 保持为 `registration_source`。

```466:473:src/odometry/pipeline.cpp
    if (artifacts.result.accepted && !useEskf()) {
        auto transformed_source = std::make_shared<PointCloud>();
        pcl::transformPointCloud(
            *registration_source,
            *transformed_source,
            artifacts.result.T_correction.matrix());
        artifacts.corrected_source = transformed_source;
    }
```

2. **L575-588**: ESKF 输出段改为单次合成变换 `T_combined = T_world_lidar_out × T_world_lidar_ref⁻¹`，变换源从 `artifacts.corrected_source` 改为 `registration_source`。

```575:588:src/odometry/pipeline.cpp
        if (artifacts.result.accepted) {
            const Isometry3d T_combined =
                T_world_lidar_out
                * deskewed.T_world_lidar_ref.inverse();
            if (!T_combined.matrix().isIdentity(1e-9)) {
                auto aligned = std::make_shared<PointCloud>();
                pcl::transformPointCloud(
                    *registration_source,
                    *aligned,
                    T_combined.matrix());
                cloud_out = aligned;
            }
        }
```

DLIO (observer) 路径行为完全不变，下游消费者 `maybeUpdateSubmapTarget` 和 `pgo_backend_.addFrame` 语义一致。

---

### 🔴 缺陷 2：`propagateCovariance` 每 IMU 样本重复构造常量矩阵 `Qc`

**文件**: `src/odometry/eskf.cpp`  
**位置**: 行 198-206

```198:206:src/odometry/eskf.cpp
    Eigen::Matrix<double, 12, 12> Qc = Eigen::Matrix<double, 12, 12>::Zero();
    const double sg = noise_.gyro_noise_density;
    const double sa = noise_.accel_noise_density;
    const double sba = biasRwAccel();
    const double sbg = biasRwGyro();
    Qc.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * sg * sg;
    Qc.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * sa * sa;
    Qc.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * sba * sba;
    Qc.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * sbg * sbg;
```

`Qc` 是一个 12x12 对角矩阵，其对角值仅依赖 IMU 噪声参数（在构造时确定）。该矩阵在每次 `predict()` 调用中被重新分配、清零并填充——即每 IMU 样本（200 Hz）执行一次。12x12 的 `Eigen::Matrix` 初始化为零有 ~144 次赋值（编译器可能优化部分），但 12x12 Identity 矩阵与标量平方相乘又产生 9 次乘法，合计每样本约 20-30 个标量操作。

200 Hz × 20 操作/样本 = 4000 操作/秒，数值上不算大，但与 `F`、`G` 矩阵不同，`Qc` 在滤波器生命周期内完全不变。

**修复建议**: 在构造函数中预计算 `precomputed_Qc_` 成员变量，`propagateCovariance` 中直接引用。

```cpp
// 在 Eskf::Eskf 构造函数中添加：
void Eskf::precomputeNoiseMatrices() {
    const double sg = noise_.gyro_noise_density;
    const double sa = noise_.accel_noise_density;
    const double sba = biasRwAccel();
    const double sbg = biasRwGyro();
    precomputed_Qc_.setZero();
    precomputed_Qc_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * sg * sg;
    precomputed_Qc_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * sa * sa;
    precomputed_Qc_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * sba * sba;
    precomputed_Qc_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * sbg * sbg;
}
```

---

### 🟡 缺陷 3：`propagateCovariance` 子步分解在 200 Hz 下总是 `steps=1`

**文件**: `src/odometry/eskf.cpp`  
**位置**: 行 180-183 和 行 211-213

```180:183:src/odometry/eskf.cpp
    constexpr double kMaxStepSec = 0.02;
    const int steps = std::max(
        1, static_cast<int>(std::ceil(dt / kMaxStepSec)));
    const double step_dt = dt / static_cast<double>(steps);
```

```211:213:src/odometry/eskf.cpp
    for (int step = 0; step < steps; ++step) {
        P = Phi * P * Phi.transpose() + Qd;
    }
```

在 200 Hz IMU 频率下（dt ≈ 0.005 秒），`steps` 恒为 1。但 `std::ceil`、`std::max`、整数类型转换和浮点除法仍在每次 `predict()` 调用中执行。对于 `steps=1` 的情况，`step_dt = dt`，`Phi = I + F * dt` 与不分步时的离散化一致。

200 Hz × 每次 5-10 个标量操作 = ~1k 操作/秒，属于微优化范畴。真正的风险点在于：**如果 IMU 频率显著低于 200 Hz（例如 50 Hz，dt=0.02+），子步分解是必要的**——否则一阶 `Phi = I + F*dt` 对 0.02s 以上的步长精度会下降。所以逻辑本身不能移除，但可以快速路径优化。

**修复建议**: 当 `dt <= kMaxStepSec` 时跳过 `steps` 计算和 for 循环，直接单步传播。

```cpp
if (dt <= kMaxStepSec) {
    const Mat15 Phi = Mat15::Identity() + F * dt;
    const Mat15 Qd = Phi * G * Qc * G.transpose() * Phi.transpose() * dt;
    P = Phi * P * Phi.transpose() + Qd;
} else {
    // 现有的子步分解逻辑
}
```

---

### 🟡 缺陷 4：`correctAt` 中 IMU 缓冲区重复遍历

**文件**: `src/odometry/eskf.cpp` 行 374-386、`src/odometry/pipeline.cpp` 行 545

每次 LiDAR 扫描（10-20 Hz）的 ESKF 修正流程对 IMU 缓冲区遍历了两次：

1. **`correctAt` 内部** (`eskf.cpp:374-386`): 从 baseline 遍历到 reference_stamp，做均值+协方差传播
2. **`replayToLatest`** (`pipeline.cpp:545`): correctAt 返回后，从 tip_state（已设为 reference 处的 prior_mean）遍历到缓冲区末尾，做均值传播

这种设计是算法正确的：correctAt 需要知道参考时刻的协方差，replayToLatest 需要将滤波器状态快进到最新 IMU 样本。但两次遍历的总开销 = O(2 × 缓冲区中 IMU 样本数 × propagateCovariance)。

对于 200 Hz IMU 和 10 Hz 扫描，相邻两次扫描间约 20 个 IMU 样本。`propagateCovariance` 每个做 15x15 矩阵乘法（~3375 次乘加），20 样本 × 2 次遍历 × 3375 = 135k FLOPs/扫描帧。在数值上影响有限，但如果 IMU 缓冲区更大（因 LiDAR 积压或降频），开销会线性增长。

**修复建议**: 在 `correctAt` 返回时附带已传播到 reference_stamp 的 `P_tip_` 和 `tip_state_`，避免 replayToLatest 重新传播前面的时间段。但这要求 `correctAt` 不修改 baseline（当前在 pipeline.cpp 中 setBaseline 后才 replay），属于结构性调整，修改风险较高。

---

### 🟡 缺陷 5：`SubmapManager::rebuildTarget` 中的 PCL `operator+=` 效率

**文件**: `src/odometry/submap.cpp`  
**位置**: 行 97-110

```97:110:src/odometry/submap.cpp
void SubmapManager::rebuildTarget() {
    auto merged = std::make_shared<PointCloud>();
    size_t point_count = 0;
    for (const size_t index : active_indices_) {
        point_count += keyframes_[index].cloud_world->size();
    }
    merged->reserve(point_count);
    for (const size_t index : active_indices_) {
        *merged += *keyframes_[index].cloud_world;
    }

    target_ = deterministicVoxelDownsample(*merged, config_.voxel_size);
    ++target_revision_;
}
```

PCL 的 `operator+=` 是通过 `std::vector::insert` 实现的逐点复制。虽然先 `reserve(point_count)` 避免了多次重新分配，但每个关键帧仍需完整遍历一次其内部点云。对于一个窗口包含 20 个关键帧、每个关键帧 5000 个降采样点的情况，这意味着 100k 个 `Point` 结构体的 `memcpy`（每个 Point 约 16-32 字节），即 1.6-3.2 MB 的复制，然后立刻被 `deterministicVoxelDownsample` 再次遍历。

voxel 降采样之后再对合并的整体做一次体素滤波是正确的步骤（保证空间一致性），但合并步骤的中间结果 `merged` 立刻被丢弃——只用于提取质心。

**修复建议**: 将合并和体素滤波合并为单次遍历，或者直接增量更新体素地图而非完整重建。前者实现简单：

```cpp
// 伪代码：一次性采集所有点，避免中间大合并
std::vector<const Point*> all_points;
all_points.reserve(point_count);
for (const size_t index : active_indices_) {
    for (const Point& p : keyframes_[index].cloud_world->points) {
        all_points.push_back(&p);
    }
}
// 直接将 all_points 送入体素滤波（需修改 deterministicVoxelDownsample 接口）
```

但这需要修改 voxel_filter 接口。更务实的短期方案：接受当前实现，因为 submap 重建不是每帧都触发（只有关键帧添加且邻居集改变时才触发），且 point_count 通常经过降采样已较小（几千到一万点）。

---

### 🟢 缺陷 6：`PoseGraphBackend::searchLoopTarget` O(N) 线性扫描

**文件**: `src/backend/pose_graph.cpp`  
**位置**: 行 359-393

```359:393:src/backend/pose_graph.cpp
    int searchLoopTarget(size_t query_id, const Isometry3d& query_pose) const {
        int best_id = -1;
        double best_squared_distance =
            config_.loop_search_radius * config_.loop_search_radius;
        ...
        for (size_t candidate = 0; candidate < candidate_count; ++candidate) {
            ...
        }
        return best_id;
    }
```

对每个 query 帧，完整遍历所有候选帧（上限 = ISAM2 优化历史），做平移距离、时间差、里程距离、旋转差四项检查。随着轨迹长度增长，搜索复杂度线性增长 O(num_queries * num_keyframes)。

但在 PGO 后端线程中执行（非实时路径），且 `loop_search_stride` 每 N 帧才检测一次，所以实际影响有限。未来如果轨迹 >10 公里需考虑 KD-tree 空间索引加速。

---

### 🟢 缺陷 7：`deterministicVoxelDownsample` 使用 `std::floor` 和双重类型转换

**文件**: `src/odometry/voxel_filter.cpp`  
**位置**: 行 63-68

```63:68:src/odometry/voxel_filter.cpp
        const double x = std::floor(
            static_cast<double>(point.x) * inverse_leaf_size);
        const double y = std::floor(
            static_cast<double>(point.y) * inverse_leaf_size);
        const double z = std::floor(
            static_cast<double>(point.z) * inverse_leaf_size);
```

每个点做了 `float → double → multiply → floor → int64_t` 四步转换。在 Mid-360 扫描（24000 点）下这会产生 24000 次类型提升和浮点取整。对于 x86_64 的 SSE/AVX，`float` 运算比 `double` 快约 2 倍（每寄存器容纳的 SIMD 通道数翻倍）。

**修复建议**: 使用 `float` 精度进行体素键计算，避免不必要的 `double` 提升。

```cpp
const float x = std::floor(point.x * inverse_leaf_size_f);
```

需注意 `float` 表示的整数范围约 ±16M（24 位尾数），对于体素索引（leaf_size ≈ 0.5m → ~8km 范围）完全足够。坐标溢出检查中的 `coordinate_limit` (2^63) 需要相应调整。

---

### 🟢 缺陷 8：`occupancy_grid.cpp::castRay` Bresenham 中的分支密度

**文件**: `src/mapping/occupancy_grid.cpp`  
**位置**: 行 349-451

Bresenham 线光栅化算法内部每个步长都需要 `updateMiss` → `mutableCell` → `SubGrid::cell` → `mallocIfNeeded`，对于长距离光线（~50 米 / 0.1 米分辨率 = 500 个单元格），每次扫描的单元格操作量非常大。考虑到占用网格跑在 PGO 后台线程且已做降采样和高度过滤（跳过 out-of-band 点），当前性能在典型场景（Occupancy CPU 占 ~12%）下可接受。

---

## 三、已有热点分析中已覆盖的问题（不再赘述）

| 问题 | 来源文档 |
|------|----------|
| OpenMP fork/join 开销 (29.5% CPU) — `omp_get_num_procs` | `HOTSPOT_DEEP_ANALYSIS.md` §二 |
| `vk_common.hpp` 头文件膨胀导致编译慢 | `REVIEW_2026-07-25_deep_audit.md` §4.4 |
| 2.5D Occupancy Grid 合理性分析 | `PROFILE_REPORT_2026-07-22_baseline.md` |

---

## 四、修复优先级建议

| 优先级 | 缺陷 | 预计收益 | 风险 | 建议时间 |
|--------|------|----------|------|----------|
| ~~P0~~ | ~~#1 点云双重变换~~ | ~~ESKF 路径每帧节省 1 次完整点云变换~~ | — | ✅ 已修复 |
| P1 | #2 Qc 预计算 | 每 IMU 样本节省 ~20 FLOPs（微） | 极低（纯重构） | 本周 |
| P2 | #3 propagateCovariance 快速路径 | 每 IMU 样本节省 ~5 分支/转换 | 极低 | 本周 |
| P3 | #5 submap 合并策略 | 关键帧变更时减少内存复制 | 中（涉及接口改动） | 可选 |
| P4 | #7 voxel_filter float 精度 | ~1% CPU（热路径微优化） | 低 | 可选 |

---

## 五、正面肯定

以下设计在性能层面值得肯定：

- **OpenMP 并行区域复用**: `deskew.cpp` 和 `voxel_filter.cpp` 通过 `omp_in_parallel()` 检测复用外层并行团队，避免了嵌套 fork/join。
- **`pushImu` 锁粒度**: IMU 回调中 bias 修正 + 缓冲追加 + 传播在一个 `scoped_lock` 下完成，避免死锁的同时保证了原子性。
- **`preprocessPoints` 单趟过滤**: 点云裁剪和时间戳收集在一次遍历中完成。
- **ESKF 协方差使用的 Joseph 形式**: 保证了数值稳定性（正定性），即使增益不完全正确也保持合理范围。
- **`MeanOnlyGal3Integrator` 的 `multiplyRight`**: 使用 `exp(v/n)^n = exp(v)` 的群性质避免每次子步重新计算指数映射，是巧妙的性能优化。
