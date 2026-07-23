┌─ DEEP CODE REVIEW ───────────────────────────────────────────────────┐
│  ◆ PROJECT  :: Sapphire Occupancy Grid                               │
│  ◆ DATE     :: 2026-07-23                                            │
│  ◆ SCOPE    :: occupancy_grid.{hpp,cpp} + pose_graph.cpp integration │
│  ◆ BASELINE :: ea5cd17 (world-Z) + 40df126 (miss d_min) + 27e34f1    │
│  ◆ TRIGGER  :: 用户反馈：无回环时栅格不对齐、抖动分层、CPU 占用高       │
│  ◆ PRIOR    :: v1–v3 occupancy reviews；本文件已纠偏（见文末勘误）     │
│  ◆ VERDICT  :: 真 P0 是 updateHit 的 d_min 污染 + h_clearance 语义； │
│                world-Z 不是三问题共同根因，勿盲目回退                   │
└──────────────────────────────────────────────────────────────────────┘

一、总体评价

OccupancyGrid 的数据结构（两级 SubGrid 懒分配 + hit/visit 比值）和
Bresenham 射线遍历合理；pose_graph 侧 odom 追加 / loop 重建的策略清晰，
线程与锁顺序经多轮审查无阻塞问题。

当前实现的主要缺口：

1. **算法**：`updateHit` 首次 hit 仍用 `std::min`，近地 in-band 点可永久压低
   `d_min`，障碍消失后难清除（幽灵 / 分层感）。`40df126` 只修了 miss 侧。
2. **参数语义**：设计文档把 `h_clearance` 写成底盘间隙（0.1–0.3 m），配置与
   `types.hpp` 写成传感器离地高度（≈2.0 m）。这才是平地大量近地点进 band
   的主开关，与 body-z / world-Z 选择基本无关。
3. **过滤轴权衡**：`ea5cd17` 的 world-Z 有明确动机（跨 keyframe 的 `d`/`d_min`
   可比）；设计文档的姿态平面更利于斜坡分类。二者是张力，不是单边「改错」。
4. **文档/代码不一致**：`hit_cnt>0` vs 文档 `>3`；out-of-band skip vs 文档
   「地面也 cast free」——后者是实现刻意权衡（防 Mid-360 地面刷白），不是漏写。

二、三个用户现象 → 根因（纠偏后）

═══════════════════════════════════════════════════════════════════════
  问题 1：栅格与扫描点云不对齐（无回环）
═══════════════════════════════════════════════════════════════════════

根因 A [场景依赖] — world-Z 在斜坡上会误判，但不是平地主因

  斜坡上 world-Z（`d = p.z - t.z`）确实会把远端抬高的坡面点收成 HIT：

        世界 Z=4m  ──●  ← 坡面点
                    /│
        世界 Z=3m  / │  ← d = 3-1 = 2.0 ≥ -h_clearance → HIT
        世界 Z=1m ●──┘  ← sensor
        世界 Z=0m ───────

  姿态平面（`d = z_hat·(p-t)`）在斜坡上更符合「能不能走过去」的语义。

  但平地、小 pitch/roll 时：
    body-z:  d = z_hat·(p-t) = p_body.z
    world-Z: d = p.z - t.z
  两者几乎相等。传感器颠簸 10 cm 对两种定义的影响同量级。
  **平地 bag 上「不对齐 / 地面像障碍」不能归因于 ea5cd17。**

根因 A2 [P0 配置] — h_clearance=2.0 把近地带进 HIT

  配置注释与 `types.hpp`：`h_clearance ≈ sensor height`（2.0 m）。
  设计文档典型值：底盘间隙 0.1–0.3 m。

  在 h=2.0、传感器离地 ≈2 m 时，地面点 d≈-2.0 落在阈值上，大量近地点
  成为 in-band HIT（body-z 与 world-Z 相同）。这比「换过滤轴」更能解释
  地面误占与 CPU。

根因 B [P1 结构] — 只追加不回看

  `appendOccupancyFrames()` 只向前插入新 keyframe；历史位姿微调不会重投。
  回环才 `rebuildOccupancyMap()`。

  无回环时 ISAM2 对早期位姿的漂移通常很小（毫米～厘米），可造成**微分层**，
  但不是「整墙错位」的主因。不宜夸张成「每轮重线性化全体位姿大跳变」。

═══════════════════════════════════════════════════════════════════════
  问题 2：抖动 → 墙壁多层
═══════════════════════════════════════════════════════════════════════

根因 C [P0] — updateHit 的 d_min 污染（真算法 bug）

  `40df126` 已从 `updateMiss` 去掉 `d_min` 写入。当前污染路径在 **hit 侧**：

    updateHit:
      cell->d_min = std::min(cell->d_min, d);  // 首次也应 RESET

  在 h_clearance=2.0 下，近地 in-band 点先到同一 cell：

    Step 1: 近地点 HIT，d≈-1.5
            updateHit: d_min = -1.5, hit_cnt=1
    Step 2: 墙 HIT，d≈0.5
            d_min = min(-1.5, 0.5) = -1.5  ← 锁死
    Step 3: 墙消失后 free 射线 d_ray 很难 < d_min
            visit_cnt 不涨 → ratio 锁在 occupied

  这才是「幽灵障碍 / 一碰就锁死」的可靠机制。
  **首次 hit 应 `d_min = d`，之后再 `min`。**

根因 C2 [设计权衡] — hit_cnt>0 vs 文档 >3

  代码：`if (cell->hit_cnt > 0)` 进入 d-aware 保护。
  文档：`>3` 才保护「强障碍」。

  `>0` 更保守（更难被刷白）；`>3` 更易清幽灵、也更易被密集 free 冲掉。
  **不是单纯写错，需二选一并同步文档**，不要无脑改回 >3。

根因 D [P1] — XY/yaw 位姿差 + 分辨率，不是过滤轴

  墙在栅格上叠多层，主因是相邻 keyframe 的 xy/yaw 差、0.1 m 分辨率、
  Bresenham 涂抹，以及上面的 d_min 锁死。
  高度过滤轴只决定谁进 band，**不决定墙投到哪个 XY cell**。
  「姿态扰动经 world-Z 传导到栅格分层」不成立。

  不推荐对 ISAM2 位姿做低通再投栅格（会与优化轨迹不一致）。
  优先修 d_min；必要时再做位姿版本化重投 / 回环重建。

═══════════════════════════════════════════════════════════════════════
  问题 3：CPU 高
═══════════════════════════════════════════════════════════════════════

根因 E [P0 参数 / P1 算法] — in-band 点数由 h_clearance 与 skip 策略主导

  错误对比（已废弃，勿再引用）：
    「body-z 地面 ≈-2.1 → skip；world-Z 地面 =-2.0 → HIT；30% vs 70–90%」
  平地上两者 d 同值，该对比人为拉开阈值。

  实际杠杆：
  - h_clearance=2.0 → 近地大量 in-band（主）
  - out-of-band 已 skip（实现刻意，减轻地面刷白，也减少 free 射线）
  - cloud_voxel_size、usable_range、Bresenham 步数

  回退 body-z ** alone 不会在平地 magically 砍半 CPU**。

根因 F [P1] — insertScan 双趟 T_map * p

  第一趟算 bounds，第二趟 ray cast，各做一遍变换。可缓存 p_map。

根因 G [P2] — toMsg 成本（表述需准确）

  已对未分配 SubGrid `continue`，不是「4M cell 全扫」。
  仍有：顶层 125×125 指针扫描 + 已分配块的 16×16 遍历 +
  按 visited max 分配整幅 `data`（可至数 MB）。可维护已分配列表优化。

根因 H [P2] — Bresenham 浮点乘：微优化，后置。

根因 I [P2] — voxel `max_threads=1`

  注释意图是避免与前端 VGICP/ESKF 抢核，不只是「嵌套 OpenMP」。
  是否放开需实测，不能断言「无意义」。

三、world-Z vs body-z（勿当已定案）

┌────────────────┬────────────────────────────┬────────────────────────────┐
│                │ body-z (z_hat·(p-t))        │ world-Z (p.z-t.z)          │
├────────────────┼────────────────────────────┼────────────────────────────┤
│ 斜坡分类         │ 跟姿态，坡面不易变障碍       │ 远端坡面易进 HIT            │
│ 跨 KF 的 d 可比  │ 各帧平面不同，d_min 难比     │ 高度带一致，d-aware 跨帧合理 │
│ 平地行为         │ ≈ world-Z                 │ ≈ body-z                   │
│ ea5cd17 动机     │ —                         │ 修跨帧 d 不可比 + 减姿态耦合 │
└────────────────┴────────────────────────────┴────────────────────────────┘

结论：斜坡场景再认真选轴；平地优先修 d_min 与 h_clearance 语义。
**不要把「回退 world-Z」写成解决三问题的第一刀。**

四、未修 / 待决策清单

┌──────┬─────────────────────────────────────────────┬─────────┬──────────┐
│ 状态  │ 问题                                        │ 严重度  │ 位置     │
├──────┼─────────────────────────────────────────────┼─────────┼──────────┤
│  ❌  │ updateHit 首次 hit 应 RESET d_min            │ P0 算法 │ occ:236  │
│  ⚖️  │ hit_cnt>0 vs 文档>3：择一并同步文档           │ P1 权衡 │ occ:250  │
│  ⚖️  │ h_clearance：底盘间隙 vs 传感器高度，写清语义 │ P0 配置 │ cfg/toml │
│  ⚖️  │ out-of-band skip vs 文档「地面也 cast free」 │ P1 权衡 │ occ:412  │
│      │   （当前 skip 是刻意防地面刷白，勿当漏实现）   │         │          │
│  ⚖️  │ world-Z vs body-z（见第三节）                 │ P1 架构 │ occ:406  │
│  ❌  │ castRay clamp 后 d 插值未按夹点重算           │ P2 防御 │ occ:276  │
│  ❌  │ ensureBounds 对外死 API                      │ P2 死码 │ hpp:44   │
│  ❌  │ toMsg 可维护已分配 SubGrid 列表               │ P2 效率 │ occ:474  │
│  ❌  │ 测试：d_min 污染回归 / 斜坡过滤 / resize      │ P2 测试 │ test     │
└──────┴─────────────────────────────────────────────┴─────────┴──────────┘

`isOccupied`：代码要求 `hit_cnt >= 2`；设计文档写 `visit_cnt > 3`。
需统一文档与实现，属一致性而非独立崩溃路径。

五、位姿分层（结构债）

append-only 在无回环下会有微分层。可选方向：

  A. 回环（或位姿漂移超阈值）时全量 rebuild — 已有 loop 路径
  B. keyframe 位姿版本号：超分辨率阈值则重投该帧
  C. 显示层用 T_map_odom 吸收，栅格仍用插入时位姿（仅缓解观感）

不推荐：对 ISAM2 位姿指数平滑后再 insertScan。

六、性能量级（定性，非精确 profile）

Profile 基线（见 PROFILE_REPORT）：occupancy 约占总 CPU ~12%
（insertScan + castRay + updateMiss）。

主要成本在 in-band 射线的 Bresenham / mutableCell，而非「world-Z 特有」。
双趟变换约几个百分点；toMsg 按导出频率摊薄。
精确 80–200 ms/帧等数字需本机实测，本文不采用未经 profile 的臆测表。

七、修复优先级（纠偏后）

┌──────┬──────────────────────────────────────────────┬──────────┬──────┐
│ Pri  │ Item                                         │ Impact   │ Fix  │
├──────┼──────────────────────────────────────────────┼──────────┼──────┤
│ P0-1 │ updateHit：首次 hit 重置 d_min               │ 幽灵/分层│ 3行  │
│ P0-2 │ 厘 h_clearance 语义并调参 / 文档对齐         │ 地面/CPU │ 配置 │
│ P1-1 │ hit_cnt 阈值：选定 >0 或 >3，同步全部文档     │ 一致性   │ 小   │
│ P1-2 │ 缓存 p_map，去掉双趟变换                       │ 性能小   │ 10行 │
│ P1-3 │ 斜坡需求明确后再选 body-z / world-Z / 混合     │ 斜坡对齐 │ 设计 │
│ P1-4 │ out-of-band free：若要做，用稀疏/短距/降权    │ 幽灵     │ 设计 │
│      │   （禁止无保护地全量地面 cast）                 │          │      │
│ P2-1 │ toMsg 已分配列表；Bresenham 整数化；补测试    │ 性能/质量│ 中   │
│ P2-2 │ ensureBounds 删除或改 private                 │ 死码     │ 1行  │
└──────┴──────────────────────────────────────────────┴──────────┴──────┘

八、建议修复顺序

第一轮：
  1. `updateHit` first-hit reset + 回归测试（近地 hit → 墙 hit → 可清除）
  2. 理清并写入 `h_clearance` / out-of-band / hit_cnt 的真实语义

第二轮：
  3. 缓存 p_map
  4. 若有斜坡数据，再评估过滤轴；无斜坡证据不回退 world-Z

第三轮：
  5. toMsg / Bresenham / 测试补强
  6. 位姿版本化重投（仅当微分层仍不可接受）

九、一句话总结

真 P0 是 **updateHit 的 d_min 污染** 与 **h_clearance=传感器高度导致近地大量
in-band**；walls 分层主因是 XY 位姿差 + d_min 锁死，不是 world-Z。
`ea5cd17` 用 world-Z 换跨帧 d 可比，与姿态平面斜坡优势构成权衡——
**回退 5 行 z_hat 不能同时解决对齐、分层与 CPU。**

十、勘误（相对本文初稿 / 外部 LLM 稿）

| 原主张 | 更正 |
|--------|------|
| world-Z 是三问题共同根因 | 否；真 P0 为 d_min + h_clearance 语义 |
| 平地 body-z 30% / world-Z 70–90% in-band | 平地两者 d 近似，对比无效 |
| 回退 world-Z 今天五行使三问题消失 | 否；斜坡另议，平地收益小且损跨帧 d 可比 |
| 姿态扰动经 world-Z 导致墙分层 | 否；分层在 XY/yaw + d_min |
| out-of-band 也 cast free 为必做 P1 | 否；当前 skip 是防地面刷白的刻意行为 |
| ISAM2 每轮全体位姿大漂移 | 过述；无回环多为微分层 |
| 低通 ISAM 位姿修抖动 | 不推荐 |
| toMsg 遍历全部 4M cell | 未分配 SubGrid 已 skip；成本在指针扫描+导出缓冲 |
| voxel max_threads=1 无意义 | 有与前端抢核的意图，需实测再动 |
| 80–200 ms/帧等无 profile 数字 | 不以精确值写入；以 ~12% CPU 基线为准 |
