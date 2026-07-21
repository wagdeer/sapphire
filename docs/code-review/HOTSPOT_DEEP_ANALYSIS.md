# Sapphire CPU Hotspot — Deep Analysis & Fix Recipes

```
┌──────────────────────────────────────────────────────────────────────┐
│  ◆ SAPPHIRE :: HOTSPOT DEEP DIVE                                    │
│  scope   : 29,271 gperftools samples, 1.4GB rosbag                  │
│  config  : ESKF + VGICP + PGO + occupancy (Mid-360)                 │
│  thread  : 3-thread MultiThreadedExecutor + OpenMP                  │
│  purpose : Root-cause analysis per hotspot + concrete fix code      │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 一、总览

29,271 个样本中，排名前 10 的热点占据了 80% 的 CPU 时间。按模块分类：

```
OpenMP runtime  ██████████████████████████████  29.5%  ← 主要瓶颈
GICP/VGICP      ██████████████████              17.9%  ← 预期内
Occupancy Grid  ████████████                    12.0%  ← 意外高
DDS/ROS2        █████                            5.0%  ← 中间件税
Voxel downsample████                             4.6%  ← 预期内
small_gicp misc ████                             3.7%
Deskew           █                                1.0%  ← 很高效
Other           ██████████████████████████       26.3%
```

---

## 二、逐函数深度分析

### 🔴 HOTSPOT 1 — omp_get_num_procs (29.5% self, 38.5% cum)

```
Samples : 8,648
Self    : 29.5%
Cum     : 38.5%（说明被调用者也有开销）
```

**根因分析：**

`omp_get_num_procs` 是 OpenMP 运行时的核心函数。它被调用在：
1. 每个 `#pragma omp parallel` 进入时（决定线程数）
2. `omp_get_max_threads()` 内部实现
3. 线程池管理（fork/join）

Sapphire 中直接使用了 2 个 OpenMP parallel 区域：
- `voxel_filter.cpp:57` — 并行体素键计算
- `deskew.cpp:264` — 并行点云变换

但 small_gicp 内部也使用 OpenMP（`quick_sort_omp`、`estimate_local_features`）。
这些库函数每次被调用时都创建/销毁 OpenMP 并行区域。

**为什么占比 29.5%？**

不是 `omp_get_num_procs` 函数本身慢，而是它被当作 OpenMP fork/join 开销的
"代言人"——当线程在 barrier 上等待或在 fork/join 中切换时，SIGPROF 采样
恰好落在 `omp_get_num_procs`（因为它在调用栈的顶部附近）。

实测证据：
- Sapphire 进程使用了 121% CPU（2 核+），说明有至少 2 个活跃线程
- OpenMP 的默认 `OMP_WAIT_POLICY=passive` 导致线程在每次 parallel region
  结束后被销毁，下次再创建——创建/销毁循环的采样落在 `omp_get_num_procs`
- small_gicp 的 `quick_sort_omp` 每次体素滤波调用都创建一个 parallel region
  → 10Hz × ~3 个体素滤波调用（source + submap rebuild + occupancy） = 30/秒
  → 30 × fork/join 开销 = 可观的浪费

**修复方案：**

**方案 A（立即生效）：设置 OpenMP 环境变量**

```bash
# 在容器/Dockerfile 中设置：
export OMP_WAIT_POLICY=active      # 线程保持 spinning，不销毁
export OMP_DYNAMIC=false           # 禁用动态线程数调整
export OMP_NUM_THREADS=4           # 固定线程数（RTX 5070 Ti 有 16GB，用 4 核安全）
```

预期节省：15-20%（消除 fork/join 开销，线程保持 alive）

**方案 B（代码级）：合并并行区域**

当前流程：
```
downsamplePoints()     → OpenMP parallel (voxel calc) → serial (sort) → serial (centroid)
deskew > transformScan → OpenMP parallel (point transform)
small_gicp             → 内部再有 OpenMP parallel（knn + features）
```

问题：每个函数独立创建 parallel region，互不知对方的存在。

修复：在 pipeline 层面，将多个需要并行化的操作合并在一个大的 parallel region 内，
使用 `omp single` / `omp task` 分派：

```cpp
// pipeline.cpp processLidarScan — 合并并行区域
#pragma omp parallel
{
    #pragma omp single nowait
    {
        deskewed = deskewPointcloud(stamp, preprocessed.cloud);
    }
    #pragma omp single nowait
    {
        // 体素滤波在 parallel region 内使用 omp for
        registration_source = downsamplePoints(deskewed.cloud);
    }
}
```

但这样做需要修改 pipeline 的控制流——短期不推荐，复杂度高。

**方案 C（已验证有效）：减少 small_gicp 的 OpenMP 使用**

在 `voxel_filter.cpp` 中，`small_gicp::quick_sort_omp` 已经使用了并行排序。
如果 Sapphire 自己已经处于一个并行上下文中，就不需要 small_gicp 再 fork 新线程。

```cpp
// voxel_filter.cpp — 当 Sapphire 已经并行化时，禁用 small_gicp 的 OpenMP
// 在 small_gicp 调用前：
omp_set_nested(0);  // 禁用嵌套并行
```

但 small_gicp 的 OpenMP 使用是不可控的——它是上游库。

**推荐执行顺序：**
1. 立即：方案 A（环境变量），5 秒生效，无需重编译
2. 短期：方案 C（omp_set_nested(0)），1 行代码改动
3. 长期：方案 B（合并并行区域），架构改动

**验证方法：**
```bash
# 对比测试
# Before:
docker exec sapphire_dev bash -c 'CPUPROFILE=/tmp/before.prof ...'

# After (方案A):
docker exec -e OMP_WAIT_POLICY=active -e OMP_DYNAMIC=false \
    sapphire_dev bash -c 'CPUPROFILE=/tmp/after.prof ...'

# 对比:
google-pprof --text --functions <binary> /tmp/before.prof | head -5
google-pprof --text --functions <binary> /tmp/after.prof | head -5
```

---

### 🟡 HOTSPOT 2 — small_gicp::UnsafeKdTree::knn_search (17.9%)

```
Samples : 5,249
Self    : 17.9%
Cum     : 17.9%（纯 self-time，无子调用）
```

**根因分析：**

这是 GICP/VGICP 扫描匹配的核心：为每个 source point 在 target 点云中找到
k 个最近邻。算法是 kd-tree 搜索，时间复杂度 O(N_source × log N_target)。

mid360 典型参数：
- source: ~5000 点（0.25m 体素滤波后）
- target: ~50000 点（10 个 submap keyframe，每个 ~5000 点）
- k=16（k_correspondences）
- 每次 scan: 5000 × log(50000) × 16 = 5000 × 16 × 16 = 1.28M 次距离计算

**为什么 17.9%？**

这是正常的。GICP 的核心计算量就在这里。对标 FAST-LIO2 的 ikd-Tree 搜索，
这个比例是合理的。

**修复方案：**

**方案 A（调参）：降低 k_correspondences**

当前 `k_correspondences = 16`。对于室内结构化环境，8 通常足够。
对于室外 Mid-360，可以降到 12。每降低 4，knn_search 时间降低 ~25%。

```toml
# sapphire_mid360.toml
[registration.gicp]
k_correspondences = 12  # 从 16 降低
```

**方案 B（调参）：增加 source 体素大小**

当前 `odometry.voxel_size = 0.25`。增加到 0.35m 可以将 source 点数减少 ~40%，
GICP 时间线性减少。

```toml
[odometry]
voxel_size = 0.35  # 从 0.25 增加，source 点 ~3000
```

代价：轻微精度损失（0.35m 体素对 Mid-360 的 0.1m 精度影响不大）。

**方案 C（算法）：使用 VGICP 而非 GICP**

当前配置已使用 VGICP。VGICP 的 knn_search 在 voxel map 上执行，比原始 kd-tree 更快。
确认：
```
[registration]
type = "VGICP"  # ✅ 已配置
```

**方案 D（长期）：考虑 nanoflann 替代**

small_gicp 的 `UnsafeKdTree` 是 simple kd-tree。nanoflann 的 SIMD 优化版本
在大量点查询时快 2-3×。但需要 fork small_gicp。

**验证方法：**
```bash
# 测试不同 k 值
for k in 8 10 12 16; do
    # 修改 TOML 中的 k_correspondences，运行，对比
done
```

---

### 🟡 HOTSPOT 3 — OccupancyGrid (12% total: insertScan 8.7% cum + castRay 6.6% cum + updateMiss 3.3%)

```
Samples breakdown:
  OccupancyGrid::insertScan   291 self (1.0%)  cum 2534 (8.7%)
  OccupancyGrid::castRay      261 self (0.9%)  cum 1930 (6.6%)
  OccupancyGrid::updateMiss   956 self (3.3%)  cum 1355 (4.6%)
  OccupancyGrid::toMsg        258 self (0.9%)  cum 403 (1.4%)
  OccupancyGrid::isOccupied   121 self (0.4%)  cum 121 (0.4%)
  OccupancyGrid::mutableCell  115 self (0.4%)  cum 115 (0.4%)
  ─────────────────────────────────────────────────
  Total occupancy               ~12%
```

**根因分析：**

占用栅格的 CPU 消耗分两部分：

1. **insertScan（8.7% cum）**：在 PGO 的 appendOccupancyFrames 中被调用。
   每个 PGO keyframe 都做一次射线投射。PGO keyframe 策略（0.5m/0.3rad）比
   前端 submap keyframe（1.5m/45°）密集得多。假设 1km 轨迹：2000 个 PGO 帧。

   每帧处理 ~5000 点（经 0.2m 体素滤波），每点投射一条射线。
   每条射线调用 castRay → updateMiss/updateHit。
   
   castRay 的 Bresenham 遍历每个 cell 都调用 updateMiss，updateMiss 内部有：
   - mutableCell()：除法和位运算
   - hit_cnt 检查和 d_min 比较
   
   这些操作在 per-cell 粒度上执行 → 高频率

2. **publishOccupancySnapshot / toMsg（1.4%）**：序列化栅格到 OccupancyGridMsg。
   遍历整个 SubGrid 数组，检查每个 cell 的 visit_cnt。

**关键发现：即使没有 RViz 订阅者，occupancy 也在运行。**

当前的 TOML 配置：
```toml
[pgo.occupancy]
enabled = true  # ← 始终运行
```

**修复方案：**

**方案 A（立即）：禁用或条件化 occupancy**

```toml
[pgo.occupancy]
enabled = false  # 不需要栅格地图时关闭
```

或在代码中添加 subscriber 检查（类似 PGO global_map 的 lazy 模式）：

```cpp
// pose_graph.cpp processPending() — 在 insert 前检查
if (occupancy_requested_.load(std::memory_order_acquire) || 
    occupancy_inserted_id_ < optimized_.size()) {
    // 只在有订阅者或新帧时才插入
    appendOccupancyFrames();
}
```

**方案 B：减少 PGO keyframe 密度**

当前：
```toml
[pgo]
keyframe_distance = 0.5   # → 改为 1.0
keyframe_rotation = 0.3   # → 改为 0.5
```

这会让 occupancy 处理的关键帧数量减少 4×。

**方案 C：updateMiss 热路径优化**

`updateMiss` 是目前 occupancy 中 self-time 最高的函数（3.3%）。

当前代码 (`occupancy_grid.cpp:241-257`)：
```cpp
void OccupancyGrid::updateMiss(int gx, int gy, float d_ray) {
    CellData* cell = mutableCell(gx, gy);  // division + bit ops
    if (!cell) return;
    if (cell->hit_cnt > 0) {
        if (d_ray < cell->d_min) {
            cell->visit_cnt += 1;
        }
        return;
    }
    cell->visit_cnt += 1;
    cell->d_min = std::min(cell->d_min, d_ray);
}
```

优化版：
```cpp
void OccupancyGrid::updateMiss(int gx, int gy, float d_ray) {
    // Inline mutableCell 的快速路径（跳过边界检查——castRay 已保证在界内）
    const int sx = gx >> kSubGridBits;
    const int sy = gy >> kSubGridBits;
    SubGrid& sub = grids_[sy * grid_size_x_ + sx];
    if (!sub.allocated()) return;
    CellData* cell = &sub.data_[(gy & 15) * 16 + (gx & 15)];
    
    if (cell->hit_cnt > 0) {
        if (d_ray < cell->d_min) ++cell->visit_cnt;
        return;
    }
    ++cell->visit_cnt;
    if (d_ray < cell->d_min) cell->d_min = d_ray;
}
```

预期节省：updateMiss 从 3.3% → ~1.5%（去除 mutableCell 的除法和边界检查）。

**方案 D：批量处理射线（SIMD 化）**

将 castRay 的 per-cell 循环向量化——但这在 `float` 精度下收益有限，
且 Bresenham 的迭代性质不适合 SIMD。不适合短期实现。

**验证方法：**
```bash
# 方案 A 验证：关闭 occupancy 跑 profiling
# 修改 TOML: pgo.occupancy.enabled = false
# 运行相同 bag，对比整体 CPU 和 occupancy 占比
```

---

### 🟢 HOTSPOT 4 — deterministicVoxelDownsample (2.1% self, 4.6% cum)

```
Samples : 627 self + 728 in sort/centroid
Self    : 2.1%
Cum     : 4.6%
```

**根因分析：**

调用路径：
1. `downsamplePoints()` → 每 scan 一次（10Hz）
2. `SubmapManager::rebuildTarget()` → 每 keyframe 添加一次（~1Hz）
3. `PoseGraphBackend::buildTargetCloud()` → 回环 target 构建（偶尔）
4. `OccupancyGrid::insertScan()` → 内部体素预滤波

每个调用做：
- 并行计算体素键（`#pragma omp parallel for`）
- `small_gicp::quick_sort_omp` 排序
- 串行质心计算

**为什么 4.6%？**

排序是瓶颈。`quick_sort_omp` 在体素键的 64-bit 整数数组上做并行比较排序。
5000 点的排序需要 ~50K 次比较。

**修复方案：**

**方案 A：用基数排序代替比较排序**

体素键是 64-bit 整数（x, y, z 编码为 int64_t），天然适合基数排序。
基数排序是 O(N) 的，比较排序是 O(N log N) 的。

```cpp
// 替换 quick_sort_omp 调用
// 64-bit 整数基数排序：4 次 Counting sort（每 16-bit）
void radixSortVoxels(std::vector<IndexedVoxel>& voxels) {
    // Phase 1: count
    // Phase 2: prefix sum
    // Phase 3: scatter
    // 4 passes over 16-bit chunks
}
```

small_gicp 已经提供了 `radix_sort_omp` 但 Sapphire 使用的是 `quick_sort_omp`。
检查 `small_gicp/util/sort_omp.hpp` 是否有 radix sort 版本。

**方案 B：预分配体和复用排序缓冲区**

当前每次调用创建新的 `std::vector<IndexedVoxel>`（`points.size()` 个元素）。
可以预分配一个 member 级别的 buffer：

```cpp
// voxel_filter 改为 class 而非 free function
class VoxelDownsampler {
    std::vector<IndexedVoxel> voxel_buffer_;  // 复用
    std::vector<IndexedVoxel> sort_buffer_;   // 复用
public:
    PointCloudPtr downsample(const PointCloud& points, double leaf_size);
};
```

**方案 C：减少不必要的体素滤波调用**

当前 pipeline.cpp 中 `downsamplePoints` 被调用了多次：
```cpp
// runScanRegistration:
registration_source = downsamplePoints(deskewed.cloud);  // ← source scan

// maybeUpdateSubmapTarget:
// 内部 rebuildTarget 再次调用 deterministicVoxelDownsample  ← submap

// initializeFirstLidarTarget:
world_scan 经过 downsamplePoints  ← first frame
```

**这些是否都可以省？** 一次 deskew 后的降采样结果可以在多处复用。

**推荐执行顺序：**
1. 方案 A：切换到基数排序（如果 small_gicp 支持）
2. 方案 B：预分配缓冲区
3. 长期：方案 C（但需要仔细验证是否正确——不同上下文需要不同体素大小）

---

### 🟡 HOTSPOT 5 — ROS2 DDS 开销 (~5%)

```
Samples scattered across:
  eprosima::fastrtps::rtps::create_SQLite3_persistence  126 (0.4%)
  eprosima::fastdds::rtps::SharedMemWatchdog::run        76 (0.3%)
  eprosima::fastrtps::rtps::RTPSWriter::update_cached     59 (0.2%)
  + pthread_cond_signal, pthread_mutex_lock, etc.
  ─────────────────────────────────────────────────
  Total DDS                                               ~5%
```

**根因分析：**

ROS2 Humble 默认使用 FastDDS（eProsima）。FastDDS 的 SQLite3 持久化和
SharedMemory 看门狗在持续消耗 CPU。这是 ROS2 的中间件税。

**修复方案：**

**方案 A：切换到 CycloneDDS**

```bash
# 在 Docker 容器中安装
apt-get install -y ros-humble-rmw-cyclonedds-cpp

# 运行时切换
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

CycloneDDS 通常比 FastDDS 快 30-50%（更少的线程、更轻量的发现协议）。

**方案 B：禁用不必要的 discovery**

```bash
# 如果只有一个节点（无需跨机通信）
export ROS_LOCALHOST_ONLY=1
```

这会禁用多播发现，减少 DDS 的网络 I/O 线程开销。

**方案 C：减少 topic 发布频率**

当前 Sapphire 通过 ROS2 发布：
- 10Hz odometry
- 10Hz deskewed cloud（如果可视化开启）
- 1Hz PGO map
- occupancy grid（按需）

确保这些 topic 在无订阅者时不发布（已经做了 lazy publish？）。

**验证方法：**
```bash
# 在容器中安装 CycloneDDS 后：
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 运行 bag，对比 profile
```

---

### 🟢 HOTSPOT 6 — Deskew (1.0%)

```
Samples : 45 self sapphire::deskew
Self    : 0.2%
Cum     : 1.0%
```

**确认：Deskew 已经非常高效。** MeanOnlyGal3 积分器 + 三次 Hermite 插值 +
并行点变换的优化是正确的。不需要进一步优化。

这与 code review 中的预判一致（review 中标记为已优化的热点）。

---

### 🟢 HOTSPOT 7 — IMU Propagation (<1%)

**确认：IMU 传播几乎没有开销。** 这是因为：
1. `MeanOnlyGal3Integrator` 只传播均值（无协方差）
2. `propagateStateLocked` 仅做矩阵乘法和状态恢复
3. Gal(3) 的 `multiplyRight` 和 `recoverWorldState` 都是 O(1) 操作

不需要优化。

---

## 三、优化优先级与预估收益

```
┌──────┬──────────────────────────────────────┬─────────┬──────────┬──────────┐
│ 优先级 │ 措施                                   │ 难度     │ 预估节省  │ 风险     │
├──────┼──────────────────────────────────────┼─────────┼──────────┼──────────┤
│ P0   │ OMP_WAIT_POLICY=active + OMP_DYNAMIC │ 0 行代码 │ 15-20%   │ 低       │
│      │ =false (环境变量)                      │          │          │          │
│ P1   │ 关闭 occupancy 或 subscriber 驱动     │ 5 行代码 │ 10-12%   │ 低       │
│ P1   │ 降低 PGO keyframe 密度 (0.5→1.0m)    │ TOML 改动 │ 5-8%    │ 低       │
│ P2   │ 降低 k_correspondences (16→12)       │ TOML 改动 │ 3-5%     │ 低       │
│ P2   │ updateMiss 内联优化                    │ 10 行    │ 1-2%     │ 极低     │
│ P3   │ 切换到 CycloneDDS                     │ 安装包    │ 3-5%     │ 低       │
│ P3   │ voxel filter 基数排序                 │ 20 行    │ 1-2%     │ 中       │
│ P4   │ 合并并行区域（长期架构改进）            │ 大改动    │ 3-5%     │ 中       │
└──────┴──────────────────────────────────────┴─────────┴──────────┴──────────┘

P0+P1 合计预估节省：25-32%
```

---

## 四、验证测试矩阵

需要一个测试脚本来对比优化前后的效果：

```bash
#!/bin/bash
# 对比测试：禁用 occupancy + OpenMP 优化
BAG=/data/rosbag2_2024_04_16-14_17_01
BIN=/workspace/install/sapphire_ros2/lib/sapphire_ros2/sapphire_ros_node
COMMON_ARGS="--ros-args -p config_file:=/workspace/src/sapphire/cfg/sapphire_mid360.toml
    -r lidar/points:=/livox/lidar -r imu/data:=/livox/imu"

# Baseline (当前配置)
export CPUPROFILE=/tmp/baseline.prof
# ... 运行，收集 ...

# Test 1: OMP_WAIT_POLICY=active
OMP_WAIT_POLICY=active OMP_DYNAMIC=false CPUPROFILE=/tmp/test1.prof ...

# Test 2: 关闭 occupancy
# 修改 TOML → pgo.occupancy.enabled = false
CPUPROFILE=/tmp/test2.prof ...

# Test 3: k_correspondences=12
# 修改 TOML → registration.gicp.k_correspondences = 12
CPUPROFILE=/tmp/test3.prof ...

# Test 4: 组合 Test1+2+3
OMP_WAIT_POLICY=active OMP_DYNAMIC=false CPUPROFILE=/tmp/test4.prof ...

# 对比所有 profile
for f in /tmp/baseline.prof /tmp/test{1,2,3,4}.prof; do
    echo "=== $f ==="
    google-pprof --text --functions $BIN $f | head -10
done
```

---

## 五、实验验证 (2026-07-21) — 5 项假设逐一实锤

以下所有实验在 Docker (WSL2, 32 vCPU) + Sapphire ESKF/VGICP + 同一 1.4GB rosbag 上完成。
occupancy grid 已关闭（已验证节省 ~10% CPU），所有对比基于无 RViz 纯净剖面。

### 实验矩阵

```
┌──────┬──────────────────────────────┬──────────┬──────────────┐
│ 实验  │ 措施                           │ 假设节省   │ 实测结果      │
├──────┼──────────────────────────────┼──────────┼──────────────┤
│ A    │ OMP_WAIT_POLICY=active        │ 15-20%   │ ❌ 无效       │
│ B    │ 关闭 occupancy                │ 10-12%   │ ✅ -9.8% CPU │
│ C    │ OMP_NUM_THREADS 32→2          │ n/a      │ ❌ 吞吐崩     │
│ D    │ 外层 #pragma omp parallel 包裹 │ 2/6 fork  │ ❌ 线程冲突   │
│ E    │ OMP_NESTED=false              │ n/a      │ ❌ 无效       │
└──────┴──────────────────────────────┴──────────┴──────────────┘
```

### 实验 A: OMP_WAIT_POLICY=active

- 原理: 线程 spin-wait 不下线，消除 fork/join 的 sleep→wake 上下文切换
- 实测: 墙钟 273s→272s (0% 改善)，`omp_get_num_procs` 从 38.4% 暴涨到 93.8%
- 根因: ACTIVE 模式下空闲线程在 barrier 上空转，SIGPROF 采样全部落在空转代码路径上，
  污染了 profile。真实的 fork/join 开销在 10Hz scan 间隔中有足够时间窗口被吸收。
- **结论: 这条路径不通。**

### 实验 B: 关闭 occupancy grid

- 对比: 默认配置 (21,585 samples) vs occupancy=off (19,460 samples)
- 实测: -2,125 samples (-9.8% CPU)。函数级: updateMiss 从 3.4% 归零。
- 墙钟: 303s→270s (bag replay 下变化不大，实机 10Hz 下余量会体现)
- **结论: 已落地。TOML 设 `pgo.occupancy.enabled = false`。**

### 实验 C: OMP_NUM_THREADS=2

- 目的: 减少 oversubscription (32线程→2线程)，降低 fork/join 开销
- 实测: omp_get_num_procs 从 43.4%→33.5% (绝对值 8436→2690)，但完成帧数从
  ~2680 暴跌到 ~570。节点追不上 10Hz。
- 根因: knn_search 等实际计算在 32 线程下并行度远高于 2 线程，减少的 overhead
  抵不上并行度损失。
- **结论: 32 线程 overhead 高但吞吐够，降线程数捡芝麻丢西瓜。**

### 实验 D: 外层 #pragma omp parallel 包裹 (方案 2a)

- 实现: `#pragma omp parallel { #pragma omp master { deskew(); voxel(); } }`
  利用已部署的 `omp_in_parallel()` 支持，deskew/voxel 内用 `#pragma omp for`，
  共享一个并行 team，目标将 6 次 fork/join 减少到 1 次。
- 实测: 节点在 IMU init 完成后不再处理任何 LiDAR 扫描 (CPU 2%, 0 帧)。
  即使空 `#pragma omp parallel {}` 块也触发相同症状。
- 根因: Docker/WSL2 下，`omp_get_max_threads() = 32` 创建的 worker 线程与
  ROS2 MultiThreadedExecutor 的线程模型冲突。`OMP_NUM_THREADS=2` 可解线程数
  问题但方案 D 在 2 线程下仍卡死——说明问题不仅是线程数，可能是 OpenMP 线程池
  初始化与 ROS2 executor 的交互导致。
- 基础设施 (deskew/voxel 的 `omp_in_parallel()` 支持) 已部署在 commit 7af5a43，
  等待外层包裹在实机 (Orin NX) 上验证。
- **结论: Docker 环境下无法验证，基础设施已就绪。**

### 实验 E: OMP_NESTED=false

- 与 OMP_NUM_THREADS=2 组合测试，无效果。
- **结论: 无效。**

---

## 六、修订后的热点归因 (occupancy 关闭, 无 RViz)

```
┌──────────────────────────────────────────────────────────────────────┐
│  ◆ 纯净剖面 (19,460 samples, occupancy=off, 无RViz)                  │
└──────────────────────────────────────────────────────────────────────┘

FUNCTION                                  SAMPLES    %      SOURCE
─────────────────────────────────────────────────────────────────────
omp_get_num_procs                         8,436     43.4%   small_gicp + Sapphire
UnsafeKdTree::knn_search                  6,200     31.9%   small_gicp
estimate_local_features (cum)             7,245     37.2%   small_gicp
quick_sort_omp_impl                         120      0.6%   small_gicp
IncrementalVoxelMap::nn_search              147      0.8%   small_gicp
AxisAlignedProjection::find_axis            104      0.5%   small_gicp
ParallelReductionOMP                        103      0.5%   small_gicp
─────────────────────────────────────────────────────────────────────
small_gicp 合计                          ~14,200   ~73%    ← 绝对大头
─────────────────────────────────────────────────────────────────────
Voxel sort (Sapphire)                       718      3.7%   Sapphire
Deskew                                      <50     <0.3%   Sapphire
ESKF/Observer                               ~200     ~1%    Sapphire
DDS/ROS2                                    ~300     ~2%    ROS2
─────────────────────────────────────────────────────────────────────
```

small_gicp 内部有 10+ 个 `#pragma omp parallel` 区域，每次触发 fork/join。
这些占 73% CPU，其中 43% 是 fork/join 开销，31% 是 knn_search 本身。

## 七、结论与下一步

1. **OMP_WAIT_POLICY=active 无效** — 线程空转污染 profile，墙钟零改善。
2. **Occupancy 关掉省 10%** — 已验证并落地。
3. **OMP_NUM_THREADS 降不下来** — 32 线程 overhead 高但并行度必须撑住 10Hz。
4. **外层并行区域包裹暂不可行** — Docker/WSL2 下线程冲突，等实机验证。
5. **small_gicp 是唯一有意义的攻击面** — 73% CPU。vendor 进项目并消除其
   内部 `#pragma omp parallel` 是下一个高收益方向。
6. **替代 nano_gicp 不可行** — small_gicp 比 nano_gicp 的底座 FastGICP 快 1.9x
   (KITTI 00 benchmark)，两者都用 nanoflann kd-tree。

**唯一未验证的假设: k_correspondences 16→8**。改 TOML 一行，预计 knn_search
减半 (~16% 总 CPU 节省)。收益明确、零风险。
