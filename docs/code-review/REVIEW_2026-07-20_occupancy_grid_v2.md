┌─ DEEP AUDIT ──────────────────────────────────────────────────────────┐
│  ◆ PROJECT  :: Sapphire Occupancy Grid — 第二轮深审                    │
│  ◆ DATE     :: 2026-07-20                                              │
│  ◆ FOCUS    :: 线程、锁、计算效率、算法正确性                            │
│  ◆ VERDICT  :: 1 个算法 bug（d_min 污染），若干 P1/P2                   │
└──────────────────────────────────────────────────────────────────────┘

一、找到的算法 Bug —— d_min 污染

1.1 问题

updateMiss() 在 hit_cnt ≤ 3 时会无条件更新 d_min：
    cell->d_min = std::min(cell->d_min, d_ray);

d_min 初始值 1e4。如果一条地面射线（d ≈ -2.0）在墙体射线（d ≈ 0.5）之前
到达同一 xy 栅格：

    Step 1: 地面射线 → updateMiss (hit_cnt=0 ≤ 3)
             d_min = min(1e4, -2.0) = -2.0    ← 污染

    Step 2: 墙体射线 → updateHit
             d_min = min(-2.0, 0.5) = -2.0    ← 被污染的值覆盖了墙高

    Step 3: 4 次 hit 之后 hit_cnt > 3，d-aware 模式激活
             地面射线 d_ray ≈ -2.0，检查 d_ray < d_min？
             -2.0 < -2.0？NO → visit_cnt 永远不增长

    → 墙体消失后，栅格永远无法清除，ratio = hit_cnt/visit_cnt 永久锁死

1.2 为什么不是 corner case

在`makeWallCloud()`测试中，地面点在前、墙点在后（test 里恰好是这个顺序）。
虽然测试通过（因为只测 occupied，不测 clear），但实际场景中点云顺序不可控：

- Livox Mid-360 是 non-repetitive 扫描，点顺序取决于扫描模式
- VLP-16 线序是固定的，地面线通常在前（下方激光线先出）
- 任何雷达的俯视扫描都倾向于"先扫到地面，后扫到障碍物"

这不是理论 bug，是生产环境必然触发的问题。

1.3 修复

改动极小，只改 updateMiss() 的条件：

```cpp
// Before (occupancy_grid.cpp:239-252):
void OccupancyGrid::updateMiss(int gx, int gy, float d_ray) {
    CellData* cell = mutableCell(gx, gy);
    if (!cell) return;
    if (cell->hit_cnt > 3) {
        if (d_ray < cell->d_min) {
            cell->visit_cnt += 1;
        }
    } else {
        cell->visit_cnt += 1;
        cell->d_min = std::min(cell->d_min, d_ray);  // ← 污染源
    }
}

// After:
void OccupancyGrid::updateMiss(int gx, int gy, float d_ray) {
    CellData* cell = mutableCell(gx, gy);
    if (!cell) return;
    if (cell->hit_cnt > 3) {
        if (d_ray < cell->d_min) {
            cell->visit_cnt += 1;
        }
    } else {
        cell->visit_cnt += 1;
        // Only track d_min from miss rays when no obstacle has been observed.
        // Once a hit occurs, d_min comes exclusively from hits (updateHit).
        if (cell->hit_cnt == 0) {
            cell->d_min = std::min(cell->d_min, d_ray);
        }
    }
}
```

语义变化：d_min 现在只表示"最低障碍物底面"，不再被地面射线污染。
地面射线在无 hit 的栅格中可以设置 d_min（后续 hit 拿这个做 baseline），
但一旦有 hit 就锁定由 hit 管理。

二、线程与锁 — 审计通过

2.1 锁获取顺序

整个系统涉及三把锁：

┌──────────────────────┬────────────────┬─────────────────────────────────┐
│ Mutex                │ 持有者          │ 保护对象                         │
├──────────────────────┼────────────────┼─────────────────────────────────┤
│ OccupancyGrid::mutex_│ PGO worker 线程 │ grids_, min/max, revision        │
│ output_mutex_        │ PGO worker 线程 │ T_map_odom_, occupancy_msg_ 等   │
│                      │ 任意调用线程    │ (latestOccupancyGrid 读取)        │
│ input_mutex_         │ LiDAR callback  │ input_frames_ (生产者)            │
│                      │ PGO worker 线程 │ copyPendingFrames (消费者)        │
└──────────────────────┴────────────────┴─────────────────────────────────┘

嵌套获取顺序（仅 PGO worker）：
    publishOccupancySnapshot():
        OccupancyGrid::mutex_  →  output_mutex_

    appendOccupancyFrames():
        OccupancyGrid::mutex_  (insertScan 内) → 释放
        OccupancyGrid::mutex_  (toMsg 内)      → 释放 → output_mutex_

Pipeline 侧（latestOccupancyGrid）只获取 output_mutex_，不碰
OccupancyGrid::mutex_。

→ 无死锁风险。✓

2.2 数据竞争检查

- occupancy_inserted_id_: PGO worker 独占读写 → 无线程安全问题
- frames_: PGO worker 独占（copyPendingFrames 从 input_frames_ 拷贝后私有）
- optimized_: PGO worker 独占
- occupancy_msg_: output_mutex_ 保护读写 → 安全
- occupancy_: 构造后不 reassign，PGO worker 独占调用 → 安全
- 原子变量 (stop_, snapshot_requested_, map_requested_, occupancy_requested_):
  标准的 release/acquire 语义 → 安全

→ 无线程安全问题。✓

三、计算效率

3.1 insertScan 点云双趟变换

第一趟：计算 scan bounds → resizeTo
第二趟：ray casting

每趟都对每个点做 T_map_lidar * p_body。Mid-360 ~20K 点，开销可忽略。
但如果上 128 线雷达（~300K 点）：两趟 × 300K × 30 flops ≈ 18M flops，
配合 mutex 锁持有时间变长。

建议：第一趟缓存 p_map（用 vector<Eigen::Vector3d> 或直接存进临时 cloud），
第二趟复用。改造成本低，可以在 TODO 里标记。

3.2 toMsg() 全量遍历

toMsg() 遍历全部 SubGrid（含未分配的），skipping via allocated() check。
15K SubGrid × allocated() 检查 → ~15K 次 → <1ms。内部循环只对已分配的执行。
总体 toMsg() 开销主要是 data vector 的分配和填充。

对于 200m × 200m @ 0.1m：4M cells × 1 byte = 4MB，分配 + memset 约 ~2ms。
每 1 秒触发一次（PGO worker 周期内），可接受。

3.3 castRay 浮点精度

d_step = (d_end - d_sensor) / dy
j * d_step 累加 2000 步（200m / 0.1m），float32 累积误差 ~4e-7。
对 d_min 的判断 (d_ray < d_min) 不会有实际影响。
→ 精度足够。✓

四、其他问题

4.1 d_sensor 恒为 0

occupancy_grid.cpp:342-343：
    sensor = t → sensor - t = 0 → d_sensor ≡ 0
变量存在但从未贡献非零值。删除或改名 + 注释。

4.2 castRay 端点 clamping 的 d 值错误

当 castRay 的端点超出地图边界时，代码 clamp 到地图边缘，但 d_step 仍使用
原始端点的 d_end 值。clamp 后的实际 d 值与原始 d_end 不同，导致边缘附近
栅格的 d_ray 插值错误。

触发条件：地图扩容前、或 resizeTo 的 margin 不够打时。
当前流程中 resizeTo 先于 ray casting 调用且带 margin，所以很难触发。
但代码防御性不足。

修复：clamp 后重新计算 clamped_d_end。

4.3 ensureBounds 死 API

occupancy_grid.hpp:42 声明的 public ensureBounds() 无外部调用者。
insertScan 内直接调 resizeTo。建议删除或改成 private。

4.4 rebuildOccupancyMap 日志无限流

pose_graph.cpp:602 spdlog::info 无节流，每次回环触发。
连续多边回环时刷屏。建议加 revision % 5 == 0 或降到 debug。

4.5 测试缺口

补三个关键测试：
    1. d_min 污染回归：地面射线先于障碍物射线到达 → 障碍物消失后栅格应能清除
    2. 非零姿态：roll/pitch 场景下 z_hat 倾斜 → 过滤正确
    3. resizeTo：新旧 SubGrid 数据偏移正确性

五、汇总

┌──────┬──────────────────────────────────────────┬──────────┬────────┐
│ Pri  │ Item                                     │ Severity │ Effort │
├──────┼──────────────────────────────────────────┼──────────┼────────┤
│  P0  │ d_min 污染：updateMiss 污染导致栅格无法清除 │ 算法 bug │  1 行  │
│  P1  │ castRay clamp 后 d 值错误                  │ 防御性   │  3 行  │
│  P1  │ insertScan 双趟变换                        │ 性能     │ 小     │
│  P2  │ d_sensor 恒为 0                            │ 可读性   │ 微小   │
│  P2  │ ensureBounds 死 API                        │ 死代码   │ 微小   │
│  P2  │ rebuild 日志无限流                          │ 噪音     │ 微小   │
│  P2  │ 测试缺口 × 3                               │ 质量保障 │ 中     │
└──────┴──────────────────────────────────────────┴──────────┴────────┘

一句话：d_min 污染是真实算法 bug，其余项是防御性和可维护性问题。
线程和锁经过逐条路径分析，无死锁、无数据竞争。
