# Sapphire 低风险/易修改优化清单

> **日期**: 2026-07-25
> **范围**: 全库 ~4000 行 C++
> **筛选标准**: 修改风险极低、改动量小、收益明确

---

## 一、热路径优化

### 优化 1：`propagateCovariance` 快速路径

**文件**: `src/odometry/eskf.cpp`，行 192-218
**风险**: 无 | **难度**: 低 | **估计行数**: +10/-5

**现状**：200Hz IMU 下 dt ≈ 0.005s，`kMaxStepSec = 0.02`，`steps` 恒为 1。但每次 `predict()` 仍然执行 `std::ceil`、`std::max`、`int/double` 类型转换和 `for` 循环分支。

```cpp
// 现状 (eskf.cpp:192-217)
constexpr double kMaxStepSec = 0.02;
const int steps = std::max(
    1, static_cast<int>(std::ceil(dt / kMaxStepSec)));
const double step_dt = dt / static_cast<double>(steps);

const Mat15 Phi = Mat15::Identity() + F * step_dt;
const Mat15 Qd = Phi * G * Qc * G.transpose() * Phi.transpose() * step_dt;
for (int step = 0; step < steps; ++step) {
    P = Phi * P * Phi.transpose() + Qd;
}
```

**修改**：

```cpp
// 修改后
constexpr double kMaxStepSec = 0.02;
// F, G 矩阵构造保持不变 ...
const auto& Qc = precomputed_Qc_;

if (dt <= kMaxStepSec) {
    // 快速路径：99%+ 调用走这里（200Hz IMU）
    const Mat15 Phi = Mat15::Identity() + F * dt;
    const Mat15 Qd = Phi * G * Qc * G.transpose() * Phi.transpose() * dt;
    P = Phi * P * Phi.transpose() + Qd;
} else {
    // 慢路径：低频率 IMU 仍需子步分解
    const int steps = static_cast<int>(std::ceil(dt / kMaxStepSec));
    const double step_dt = dt / static_cast<double>(steps);
    const Mat15 Phi = Mat15::Identity() + F * step_dt;
    const Mat15 Qd = Phi * G * Qc * G.transpose() * Phi.transpose() * step_dt;
    for (int step = 0; step < steps; ++step) {
        P = Phi * P * Phi.transpose() + Qd;
    }
}
P = 0.5 * (P + P.transpose());   // ← 注意：对称化需放在 if/else 之后
```

**注意**：当前 `P = 0.5 * (P + P.transpose());` 在函数末尾（行 218），快速路径分支中无需重复，保持原位即可。

**收益**：每 IMU 样本（200Hz）节省 ~5 次不必要的整数/浮点转换和分支判断。

---

### 优化 2：`preprocessPoints` 消除 `have_time` 每点分支

**文件**: `src/odometry/pipeline.cpp`，行 161-186
**风险**: 极低 | **难度**: 低 | **估计行数**: +25/-10

**现状**：循环中每个有效点都检查 `have_time` 标志。只有第一个有效点走 if 分支初始化 `min_time`/`max_time`，之后所有点走 else 分支。Mid-360 扫描 ~24000 点，即每帧 ~24000 次多余分支预测。

```cpp
// 现状 (pipeline.cpp:161-186)
for (const Point& point : points->points) {
    if (!std::isfinite(point.x)
        || !std::isfinite(point.y)
        || !std::isfinite(point.z)) {
        continue;
    }
    const bool inside_crop_box =
        point.x >= box.min_x && point.x <= box.max_x
        && point.y >= box.min_y && point.y <= box.max_y
        && point.z >= box.min_z && point.z <= box.max_z;
    if (inside_crop_box) {
        continue;
    }
    output->points.push_back(point);
    if (!have_time) {
        min_time = point.timestamp;
        max_time = point.timestamp;
        have_time = true;
    } else {
        min_time = std::min(min_time, point.timestamp);
        max_time = std::max(max_time, point.timestamp);
    }
}
```

**修改**：

```cpp
// 修改后
auto it = points->points.begin();
const auto end = points->points.end();

// 第一遍：找到首个有效点并初始化 min_time/max_time
while (it != end) {
    if (!std::isfinite(it->x) || !std::isfinite(it->y) || !std::isfinite(it->z)) {
        ++it;
        continue;
    }
    const bool inside_crop_box =
        it->x >= box.min_x && it->x <= box.max_x
        && it->y >= box.min_y && it->y <= box.max_y
        && it->z >= box.min_z && it->z <= box.max_z;
    if (inside_crop_box) {
        ++it;
        continue;
    }
    output->points.push_back(*it);
    min_time = max_time = it->timestamp;
    have_time = true;
    ++it;
    break;
}

// 后续点：无需 have_time 分支
const auto& box_alias = box;  // lambda 捕获用
for (; it != end; ++it) {
    if (!std::isfinite(it->x) || !std::isfinite(it->y) || !std::isfinite(it->z)) {
        continue;
    }
    if (it->x >= box_alias.min_x && it->x <= box_alias.max_x
     && it->y >= box_alias.min_y && it->y <= box_alias.max_y
     && it->z >= box_alias.min_z && it->z <= box_alias.max_z) {
        continue;
    }
    output->points.push_back(*it);
    min_time = std::min(min_time, it->timestamp);
    max_time = std::max(max_time, it->timestamp);
}
```

**收益**：每 LiDAR 帧（10Hz）消除约 24000 次分支预测失败。

---

### 优化 3：`integrateTimeline` 消除 `target_stamps[target_idx]` 重复访问

**文件**: `src/odometry/deskew.cpp`，行 227-244
**风险**: 极低 | **难度**: 极低 | **估计行数**: +3/-4

**现状**：`target_stamps[target_idx]` 在同一迭代中外层 if 判断访问一次，内层 while 条件又访问一次，while 体内再取一次值。

```cpp
// 现状 (deskew.cpp:227-234)
if (target_idx < target_stamps.size()
    && target_stamps[target_idx] <= interval_end) {
    while (target_idx < target_stamps.size()
           && target_stamps[target_idx] <= interval_end) {
        const double target_stamp = target_stamps[target_idx];
```

**修改**：

```cpp
// 修改后
while (target_idx < target_stamps.size()) {
    const double target_stamp = target_stamps[target_idx];
    if (target_stamp > interval_end) {
        break;
    }
    const double alpha = (target_stamp - integrated_until) / interval_dt;
    states.push_back({
        interpolateState(
            prev_interval_end_state, interval_end_state,
            alpha, interval_dt),
        target_stamp,
    });
    ++target_idx;
}
```

**收益**：消除外层 if 的冗余检查，代码更紧凑。

---

## 二、`push_back` → `emplace_back` 批量替换

**风险**: 极低 | **难度**: 极低 | **影响文件**: 5 个

`push_back({a, b, c})` 会先构造临时对象再移动/拷贝到容器。`emplace_back(a, b, c)` 直接在容器内原地构造，省去临时对象。以下 7 处可安全替换：

| # | 文件 | 行号 | 当前写法 | 修改为 |
|---|------|------|----------|--------|
| 1 | `pose_graph.cpp` | 144 | `input_frames_.push_back({cloud_odom, T_odom_lidar, stamp, travel_distance})` | `input_frames_.emplace_back(cloud_odom, T_odom_lidar, stamp, travel_distance)` |
| 2 | `pose_graph.cpp` | 259 | `frames_.push_back({...})` | `frames_.emplace_back(...)` |
| 3 | `pose_graph.cpp` | 503 | `loop_edges_.push_back({query_id, static_cast<size_t>(target_id)})` | `loop_edges_.emplace_back(query_id, static_cast<size_t>(target_id))` |
| 4 | `submap.cpp` | 47 | `keyframes_.push_back({T_world_lidar, cloud_world, stamp})` | `keyframes_.emplace_back(T_world_lidar, cloud_world, stamp)` |
| 5 | `deskew.cpp` | 234 | `states.push_back({interpolateState(...), target_stamp})` | `states.emplace_back(interpolateState(...), target_stamp)` |
| 6 | `imu_init.cpp` | 57 | `samples_.push_back({stamp, accel, gyro})` | `samples_.emplace_back(stamp, accel, gyro)` |
| 7 | `occupancy_grid.cpp` | 576 | `mapped.push_back({static_cast<float>(px), ...})` | `mapped.emplace_back(static_cast<float>(px), ...)` |

**注意**：第 5 项（`deskew.cpp`）使用圆括号必须改为 `emplace_back`，因为花括号初始化与类构造函数的重载解析可能不同。其余项语义完全等价。

**收益**：消除热路径中临时对象的构造和移动开销。

---

## 三、`selectNearest` 合并两次排序

**文件**: `src/odometry/submap.cpp`，行 77-93
**风险**: 低 | **难度**: 低 | **估计行数**: +5/-3

**现状**：先 `partial_sort` 按距离排序，再把 top-k 的 `.second`（索引）提取到新 vector，再做 `std::sort` 排序索引。

```cpp
// 现状 (submap.cpp:77-93)
std::partial_sort(
    distances.begin(),
    distances.begin() + static_cast<std::ptrdiff_t>(count),
    distances.end(),
    [](const auto& lhs, const auto& rhs) { ... });
std::vector<size_t> selected;
selected.reserve(count);
for (size_t i = 0; i < count; ++i) {
    selected.push_back(distances[i].second);
}
std::sort(selected.begin(), selected.end());
```

**修改**：直接在 `distances` 的前 count 个元素上按 `.second` 排序，避免中间 `selected` 向量：

```cpp
// 修改后
std::partial_sort(
    distances.begin(),
    distances.begin() + static_cast<std::ptrdiff_t>(count),
    distances.end(),
    [](const auto& lhs, const auto& rhs) {
        if (lhs.first != rhs.first) return lhs.first < rhs.first;
        return lhs.second < rhs.second;
    });

std::sort(
    distances.begin(),
    distances.begin() + static_cast<std::ptrdiff_t>(count),
    [](const auto& lhs, const auto& rhs) {
        return lhs.second < rhs.second;
    });

std::vector<size_t> selected;
selected.reserve(count);
for (size_t i = 0; i < count; ++i) {
    selected.push_back(distances[i].second);
}
```

**收益**：消除中间 vector 分配和一次额外遍历。非热路径，但代码更紧凑。

---

## 四、优先级汇总

| 优先级 | 优化 | 文件 | 收益 | 风险 |
|--------|------|------|------|------|
| **P0** | #1 propagateCovariance 快速路径 | `eskf.cpp` | 200Hz 热路径每调用省 ~5 操作 | 无 |
| **P0** | #2 preprocessPoints 分支消除 | `pipeline.cpp` | 10Hz 每帧省 ~24k 分支 | 极低 |
| **P1** | #3 integrateTimeline 重复访问 | `deskew.cpp` | 代码清洁度 | 极低 |
| **P1** | #4 emplace_back 批量替换 | 5 个文件 | 省 7 处临时对象构造 | 极低 |
| **P2** | #5 selectNearest 合并排序 | `submap.cpp` | 非热路径，代码紧凑 | 低 |

**建议**：P0+P1 共涉及 6 个文件，总计改动约 55 行。P2 可顺带修改。

---

## 五、已修复项（不在本次范围）

| 缺陷 | 状态 | 来源 |
|------|------|------|
| ESKF 点云双重变换 (`pipeline.cpp`) | ✅ 已修复 | `PERF_REVIEW_2026-07-25.md` #1 |
| `Qc` 预计算 (`eskf.cpp/.hpp`) | ✅ 已修复 | `PERF_REVIEW_2026-07-25.md` #2 |
| voxel_filter float 精度 (`voxel_filter.cpp`) | ✅ 已修复 | `PERF_REVIEW_2026-07-25.md` #7 |
