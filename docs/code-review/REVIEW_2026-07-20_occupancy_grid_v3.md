┌─ DEEP AUDIT v3 — Independent ─────────────────────────────────────────┐
│  ◆ PROJECT  :: Sapphire Occupancy Grid                                │
│  ◆ DATE     :: 2026-07-20                                             │
│  ◆ METHOD   :: State-machine trace, all input orderings                │
│  ◆ SCOPE    :: occupancy_grid.{hpp,cpp} + config + test                │
│  ◆ PRIOR    :: v1 (surface), v2 (d_min contamination, thread)          │
│  ◆ VERDICT  :: v2 fix incomplete — 1 P0 gap + 1 P0 threshold bug      │
└──────────────────────────────────────────────────────────────────────┘

一、v2 审计修复验证 — 为什么它不完整

v2 提议的修复（仅改 updateMiss）：

    if (cell->hit_cnt == 0) {
        cell->d_min = std::min(cell->d_min, d_ray);  // ← gate on hit_cnt
    }

v2 假设这可以防止 d_min 污染。但它只解决了问题的一半。d_min 还通过
updateHit 被污染：

    void updateHit(int gx, int gy, float d) {
        cell->d_min = std::min(cell->d_min, d);  // ← 无可避免地取污染值
        ...
    }

完整的状态轨迹（即使应用 v2 修复）：

    Cell 初始状态: hit_cnt=0, d_min=1e4

    Ray A (d_end=-1.5, in-band but below plane):
      ├─ intermediate cell X 处 d_ray=-0.8
      ├─ updateMiss: hit_cnt==0 → d_min = min(1e4, -0.8) = -0.8  ← 已污染
      └─ visit_cnt=1

    Ray B (d_end=0.5, 来自不同角度的同一 cell X):
      └─ updateHit: d_min = min(-0.8, 0.5) = -0.8  ← STILL CONTAMINATED
         hit_cnt=1, visit_cnt=2

    所有后续 miss 射线穿过 cell X:
      ├─ d_ray >= -0.8? 几乎所有都 >= -0.8（射线从传感器出发，d≈0 递增）
      ├─ d_ray < -0.8? NO
      └─ → visit_cnt 永不增长 → 永远 occupied → 永久幽灵障碍物

触发条件：d 在 [-h_clearance, 0) 范围内的 in-band point 在真障碍物之前
到达同一 cell。Mid-360 默认 h_clearance=2.0，意味着所有 -2.0 <= d < 0
的点都算 in-band 并发出射线插值为负 d_ray 的 miss 射线。

这不是 corner case——非重复扫描的 Lidar 会在障碍物射线之前以不可预测的
顺序将地面射线送入同一栅格单元。

二、d_min 污染的完整修复

需要两处改动：

    --- updateMiss (行 241-257) ---

    // BEFORE:
    if (cell->hit_cnt > 0) {
        if (d_ray < cell->d_min) { cell->visit_cnt += 1; }
        return;
    }
    cell->visit_cnt += 1;
    cell->d_min = std::min(cell->d_min, d_ray);  // ← 污染源 #1

    // AFTER:
    if (cell->hit_cnt > 0) {
        if (d_ray < cell->d_min) { cell->visit_cnt += 1; }
        return;
    }
    cell->visit_cnt += 1;
    // d_min only from misses BEFORE first hit. First hit resets it.
    if (cell->hit_cnt == 0) {
        cell->d_min = std::min(cell->d_min, d_ray);
    }

    --- updateHit (行 229-239) ---

    // BEFORE:
    cell->d_min = std::min(cell->d_min, d);  // ← 污染源 #2

    // AFTER:
    if (cell->hit_cnt == 0) {
        cell->d_min = d;  // first hit RESETS d_min
    } else {
        cell->d_min = std::min(cell->d_min, d);
    }

语义：d_min 现在表示"最低观测到的障碍物底面"。第一次 hit 重置为实际
障碍物高度，覆盖之前 miss 的任何地平面追踪。hit 绝不被地面射线污染。

    修复后的轨迹：
    Ray A (d=-0.8 miss): hit_cnt=0, d_min=-0.8    [via updateMiss]
    Ray B (d=0.5 hit):  hit_cnt==0, d_min=0.5     [RESET by updateHit]
    Ray C (d=-0.8 miss): hit_cnt>0, d_ray=-0.8 < d_min=0.5 → visit_cnt++ ✓
    → 障碍物消失时，地面射线正确清除它。

三、hit_cnt 阈值 bug — 代码与文档不一致

updateMiss 注释和 docs/occupancy_grid.md 都说 >3：

    // occupancy_grid.cpp:249 (注释):
    // "before hit_cnt>3 washes walls/trees out of the map."

    // docs/occupancy_grid.md:283:
    // "cell.hit_cnt > 3 区分'已确认障碍物'和'可能是噪声'"

但实际代码使用 >0：

    if (cell->hit_cnt > 0) {  // ← 应该是 >3

影响：
  >0: 1 次 hit 后保护立即激活。一次噪声 hit 锁死栅格。
  >3: 4 次 hit 后才进入 d-aware 模式，对传感器噪声更鲁棒。

建议：如果 >0 是故意的（户外场景 hit 稀疏，希望尽快保护），更新注释和
文档。否则改回 >3。

四、insertScan 跳过地面射线 — 与设计文档矛盾

docs/occupancy_grid.md:506-512 明确说：

    "对所有点：ray cast 从传感器原点到该点 → 沿途栅格按 d-aware 规则更新"

    "地面点的射线标 free 不标 occupied"

但实际代码（行 413-418）直接跳过所有 out-of-band 点：

    if (!hit) {
        // Out-of-band returns (ground / high canopy) must NOT cast long
        // free rays.
        continue;  // ← 完全跳过，无 free-space 清除
    }

这意味着地面/天花板点不发出任何射线。无 free-space 雕刻。

后果（非 d_min 污染——另一个问题）：如果某一区域仅被障碍物射线侧扫到
（作为 miss 的中间 cell），则该区域仍被标记为 unknown（浅灰色），而非
free（白色），因为从未有射线特意经过该区域标 free。

这对导航 costmap 重要吗？如果规划器将 unknown 视为不可通行=是，当做
障碍物=否，当做 free=否→ 取决于 costmap 的 interpretation。

代码注释称这是有意的权衡（Mid-360 地面密度）。如果这确实是正确的行为，
请更新文档以反映实际逻辑。

五、castRay clamp d 值错误

当射线终点超出地图边界时，代码将终点 clamp 到地图边缘：

    if (!worldToGlobalIndex(end_x, end_y, x1, y1)) {
        const double cx = std::clamp(end_x, ..., ...);
        const double cy = std::clamp(end_y, ..., ...);
        // x1/y1 现在是 clamp 后的值
        mark_hit = false;
    }

但 d_step 仍使用原始 d_end（未 clamp 的障碍点）：

    const float d_step = (d_end - d_sensor) / N;  // N = clamp 后的步数

clamp 后的端点处实际 d 值与原始 d_end 不同。错配导致地图边缘约 5-10 个
栅格的 d_ray 插值错误。

当前难以触发（insertScan 在 castRay 之前做 resizeTo + margin），但在
margin 不足或动态障碍物超出地图时可能发生。

修复：cramp 后重新计算 clamped_d_end：

    const double cx = std::clamp(...);
    const double cy = std::clamp(...);
    const float clamped_d_end = static_cast<float>(
        z_hat.dot(Eigen::Vector3d(cx, cy, 0) - t));  // 近似，需要 p_map.z

或者更简单：当端点 clamp 时，基于 clamp 后的世界坐标重新计算 d。

六、isOccupied 阈值

代码：    hit_cnt < 2 → 不可 occupied（最少 2 次 hit）
文档：    visit_cnt > 3 → 不评估比值（最少 3 次 visit）

代码从不检查 visit_cnt > 3。如果 visit_cnt=1 且 hit_cnt=2（如何做到的？），
ratio=2.0>0.3 → occupied。不一致，但仅在理论构造中可触发。

七、updateMiss 中的 d_min 更新与 hit_cnt 检查顺序

当前代码：

    if (cell->hit_cnt > 0) {
        if (d_ray < cell->d_min) {
            cell->visit_cnt += 1;
        }
        return;  // ← 保护模式：无 d_min 更新
    }
    cell->visit_cnt += 1;
    cell->d_min = std::min(cell->d_min, d_ray);  // ← 未保护模式

逻辑：hit_cnt==0 → 无保护。hit_cnt>0 → 已保护。

应用 P0 修复 #1（d_min 污染）后，这个结构仍然是正确的。只是阈值应该
从 >0 变为 >3 以匹配设计意图。

八、汇总

┌──────┬──────────────────────────────────────────────┬──────────┬────────┐
│ Pri  │ Item                                         │ Severity │ Lines  │
├──────┼──────────────────────────────────────────────┼──────────┼────────┤
│  P0  │ updateHit 用 min 取污染 d_min                  │ 算法 bug │  1 行  │
│      │ → 第一次 hit 时重置 d_min                      │          │        │
│  P0  │ hit_cnt 阈值 mismatched (>0 vs >3)            │ 算法 bug │  1 行  │
│      │ → 改回 >3 或更新所有注释/文档                   │          │        │
│  P1  │ insertScan 跳过非带内点，无 free 射线            │ 设计鸿沟 │  N/A   │
│      │ → 修复代码或更新文档                             │          │        │
│  P1  │ castRay clamp 后 d 插值错误                     │ 防御性   │ ~5 行  │
│  P2  │ d_sensor 恒为 0 — 变量名有误导性                 │ 可读性   │  2 行  │
│  P2  │ ensureBounds 死 API                            │ 死代码   │  删除  │
│  P2  │ toMsg 全尺寸分配（暂不处理）                      │ 效率     │  -     │
│  P2  │ 测试缺口：姿态/d-aware/resize/contamination       │ 质量保障 │  中    │
└──────┴──────────────────────────────────────────────┴──────────┴────────┘

九、一句话总结

v2 找到的 d_min 污染是对的但修复不完整——updateHit 的 std::min 仍然
在第一次 hit 前被地面射线污染时取到已污染的值。添加首次 hit 重置即可
修复。另外代码用 >0 但所有文档说 >3——要么是 bug 要么文档未更新。
