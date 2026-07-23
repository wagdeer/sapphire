┌─ FULL AUDIT ──────────────────────────────────────────────────────────┐
│  ◆ PROJECT  :: Sapphire + sapphire_ros2                                │
│  ◆ DATE     :: 2026-07-22                                              │
│  ◆ METHOD   :: 对照当前工作区源码独立复核（不采信过期 review 结论）      │
│  ◆ SCOPE    :: odometry / ESKF / PGO / occupancy / ROS2 / config / test │
│  ◆ BRANCHES :: sapphire=`grid_map`@d026c81+；sapphire_ros2=`nano_gicp` │
│  ◆ VERDICT  :: 1 个集成 P0 + 若干 P1；先前 deep_v2 两条 CRITICAL 已过期 │
└───────────────────────────────────────────────────────────────────────┘

一、审查前提与工作区真相

当前工作区两个仓库**不在同一功能线上**：

| 仓库 | 分支 | HEAD 要点 |
|------|------|-----------|
| `src/sapphire` | `grid_map`（ahead 1） | occupancy + ESKF binary search + P0 增量 append |
| `src/sapphire_ros2` | `nano_gicp` | **无** occupancy 发布；可视化定时器无 revision 门控 |

因此：核心库已具备栅格能力，但 ROS 包装层当前**接不上**；用户开 `enabled=true`
也看不到 `/sapphire/pgo/occupancy_grid`。这不是算法问题，是集成断裂。

对既有文档的纠偏（必须先读）：

| 文档断言 | 当前代码 | 判定 |
|----------|----------|------|
| `REVIEW_2026-07-22_deep_v2.md` CRITICAL：`mapping/mapping/` 重复文件仍在 | `4284d96` 已删，源码树只剩一份 | **doc stale** |
| 同文档 CRITICAL：`correctAt` O(N²) IMU 扫描仍在 | `d026c81` 已用 `findFirstAfter` 二分 + 区间回放 | **doc stale** |
| occupancy「v2/v3 污染已彻底修好」 | `hit_cnt>0` 后隔离了，**首次 hit 前 miss 仍可压低 d_min**；`updateHit` 仍 `min` | **半修** |
| 头文件/文档：带外点仍 cast free rays | `insertScan` 对 `!hit` 直接 `continue` | **doc/code 不一致** |

---

二、架构速览（当前）

```
LiDAR/IMU callbacks (MutuallyExclusive 两组)
        │
        ▼
OdometryPipeline
  preprocess → deskew(Gal3) → voxel → GICP/VGICP → ESKF|Observer
  → SubmapManager → PoseGraphBackend::addFrame
        │
        ▼
PGO worker (独立 std::thread, period=update_period_sec)
  ISAM2 odom / loop ICP → T_map_odom
  occupancy: odom→append / loop→full rebuild / export→toMsg only
        │
        ▼
ROS: odom + deskewed + (若 PGO) map/graph
     ✗ occupancy 发布未接线（sapphire_ros2@nano_gicp）
```

线程与锁（复核结论：**未发现确认死锁/数据竞争**）：

- Pipeline：`imu_mutex_` + `state_mutex_`（`scoped_lock` 同序）、`output_mutex_`
- PGO：`input_mutex_` / `output_mutex_` / 独立 `occupancy_mutex_` / `wake_mutex_`
- OccupancyGrid 自带 `mutex_`：当前仅 PGO worker 调用，属防御性加锁

---

三、P0 — 现在就该处理

3.1 仓库分支错配 + occupancy 功能断链
───────────────────────────────────
**证据**
- `sapphire_ros2` 全文无 `OccupancyGrid` / `requestOccupancyGrid` / `occupancy_grid_pub_`
- `sapphire` 侧 `PoseGraphBackend::{request,latest}OccupancyGrid` 与增量/全量策略已就绪
- 配置 `cfg/sapphire_mid360_directional_eskf.toml` 当前 `enabled = false`

**影响**：开栅格以为「在建图侧有输出」，实际要么算了发不出去，要么关掉后以为算法坏了。
此前「开 occupancy 建图变差」已用 A/B 坐实是 **PGO worker 被重活拖死**；
`pose_graph` 已改为 odom 增量 / 仅回环全量，但 ROS 侧若回到旧接线且无 revision 门控，
风险会回来。

**建议**
1. 将 `sapphire_ros2` 切到与 `grid_map` 对齐的分支/提交，接回 occupancy 发布
2. 发布路径必须：`get_subscription_count()>0` + **按 PGO revision 请求**（同 map）
3. 可视化 callback group 与 IMU/LiDAR 隔离（3 线程 executor），避免稠密 `OccupancyGrid`
   序列化堵传感器回调

3.2 `sapphire/pgo/map` 每秒无条件 `requestPoseGraphMap`
────────────────────────────────────────────────
**位置**：`sapphire_ros2/src/sapphire_ros.cpp:383-394`

```cpp
if (pgo_map_pub_->get_subscription_count() > 0) {
    pipeline_->requestPoseGraphMap();   // 每 1s 定时器都请求
    ...
}
```

有订阅时，PGO worker 可能每秒 `rebuildGlobalMap()`（合并多关键帧 + voxel）。
与 occupancy 历史问题同类：**可视化请求抢 worker**，拖慢 `T_map_odom` / 回环。

**建议**：与 revision 门控对齐（`poseGraphStats().revision` 变化才 request）。

---

四、P1 — 高影响（算法 / 资源 / 正确性）

4.1 SubmapManager::keyframes_ 永不淘汰
────────────────────────────────
**位置**：`src/odometry/submap.cpp:37-58`，`selectNearest` 扫全历史

`addKeyframe` 只 `push_back`，无环形裁剪。`max_keyframes` 只限制**活跃 target 子集**，
不限制内存。长航时：点云线性涨；每次选近邻对全部历史算距离。

**建议**：保留最近 N 个（或距离窗）物理裁剪；或与 PGO `frames_` 共享生命周期策略。

4.2 回环噪声：ICP fitness 直接当地平 6-DoF 方差
──────────────────────────────────────────
**位置**：`pose_graph.cpp:489-493`

```cpp
variances.setConstant(std::max(fitness, 1e-9));
```

`fitness` 量纲 ≈ m²，却同时塞进旋转轴。走廊/隧道易「自信错误回环」。
另：回环用 PCL 点对点 ICP，前端用 small_gicp GICP/VGICP——优化问题不一致。

**建议**：平移/旋转分尺度方差；回环配准改用 small_gicp（或至少分轴标定）。

4.3 Occupancy `d_min` 首次 hit 前仍可被 miss 压低
──────────────────────────────────────────
**位置**：`occupancy_grid.cpp:229-257`

- `hit_cnt==0`：miss 仍 `d_min = min(d_min, d_ray)`
- `updateHit`：始终 `min`，无法抬高已被压低的值
- 配置 `h_clearance=2.0` 时，近地 in-band 射线会产生负 `d_ray`，污染路径真实

**建议**（择一）：
- 首次 `updateHit` **重置** `d_min = d`（不再与 miss 历史取 min）
- 或 miss **永不写** `d_min`（只由 hit 维护）

并同步：`occupancy_grid.md` / 头文件注释（`hit_cnt>3`、带外仍 cast）与实现。

4.4 PointCloud2 解析缺边界与 datatype 校验
─────────────────────────────────────
**位置**：`sapphire_ros2/src/sapphire_ros.cpp:143-192`

- 未校验 `data.size() >= n * point_step`
- x/y/z/intensity 假定 FLOAT32；timestamp 有 dtype 分支，xyz 没有

畸形消息 → 越界读或静默错坐标。

**建议**：assert/校验 size 与 field datatype；intensity 用 `memcpy` 替代非对齐
`reinterpret_cast`。

4.5 Mahalanobis 门禁用 + 全姿态注入
──────────────────────────────
配置 `mahalanobis_threshold = -1`，硬门限（平移/转角）兜底。退化场景下错误但
「看起来收敛」的 GICP 仍可写入状态。属**已知权衡**，但 NIS 已算却几乎只打日志。

**建议**：阈值仍可关，但 NIS 超限时升频 warn / 诊断计数，便于现网发现退化。

4.6 `request*` 的 `notify_one` 实际叫不醒 worker
──────────────────────────────────────────
**位置**：`pose_graph.cpp` `wake_cv_.wait_for` 谓词只看 `stop_`

`requestOccupancyGrid` / map / snapshot 的 notify 变成空转，最多晚一个
`update_period_sec`。

**建议**：谓词加入对应 `*_requested_` 原子量。

---

五、P2 — 可维护性 / 配置 / 性能债

| # | 项 | 说明 |
|---|----|------|
| 5.1 | `accel_bias_sigma` / `gyro_bias_sigma` | `types.hpp` 有字段，`config.cpp` **不解析**，TOML 写了无效 |
| 5.2 | `inject_full_pose` ∩ `inject_directional_pose` | 同时 true 时后者被静默覆盖；缺校验 |
| 5.3 | OpenMP 热点 | Profile：`omp_*` + small_gicp 内并行占大头；Sapphire 自身 `omp_in_parallel` 规避正确，根因多在库 fork/join |
| 5.4 | `ensureBounds` 死 API / `d_sensor≡0` | 可读性清理 |
| 5.5 | castRay clamp 后仍用原始 `d_end` | 防御性；`resizeTo+margin` 下难触发 |
| 5.6 | IMU ring 500 | ~2.5s@200Hz；deskew 等更久会 EmptyImuBuffer——边界需文档化 |
| 5.7 | `EskfConfig` vs `Config::Odometry::Eskf` 双结构 | 手写拷贝，易漏字段 |
| 5.8 | 文档债 | `DESIGN.md` 引用不存在的 ARCHITECTURE 等；`REVIEW_*_deep_v2` 未回写已修项 |
| 5.9 | GTSAM REQUIRED | PGO 关也硬依赖，贡献门槛 |
| 5.10 | 测试缺口 | 无 `imu_init` 单测；无 ROS 解析单测；无 PGO↔occupancy 集成测；无 TSan CI |

---

六、Occupancy 专项（与近期会话对齐）

已落地（`pose_graph`，应保留）：

```
odom_updated  → appendOccupancyFrames()     // 只打新关键帧
loop_updated  → rebuildOccupancyMap()       // 全量
export        → 至多 catch-up append + toMsg // 禁止因 dirty 全量重打
```

仍未收口：

1. ROS 发布与 revision 门控（本审查 P0）
2. `d_min` 首次 hit 重置（P1）
3. `h_clearance=2.0` 使近地大量 in-band：栅格观感/地面误占风险（设计权衡，启用前要调）
4. 文档与实现：`hit_cnt>0` vs 文档 `>3`；带外点 skip vs 文档「地面也 cast free」

默认 `enabled=false` 时上述算法项**不影响建图**；一开就必须同时满足调度 + ROS 接线，
否则会复现「建图变差 / 看不见栅格」两类故障。

---

七、测试与质量门

现有：13 个自研可执行测试（ESKF 覆盖最好，occupancy 3 例，PGO 2 例）。

优先补：

1. occupancy：miss→hit 顺序下 `d_min` 不被负射线锁死；障碍消失后可清除
2. pose_graph：odom 只增 `inserted_id`、loop 才 `revision` 大跳 + rebuild 日志
3. ROS：畸形 PointCloud2 size/datatype 拒绝
4. CI：至少 Release + `ctest`；可选 ASan/TSan job

---

八、优先级汇总

┌──────┬────────────────────────────────────────────────┬──────────┬────────┐
│ Pri  │ Item                                           │ 类型     │ Effort │
├──────┼────────────────────────────────────────────────┼──────────┼────────┤
│  P0  │ 对齐 sapphire_ros2 与 grid_map；接回 occupancy │ 集成     │ 中     │
│  P0  │ pgo/map（及 occupancy）按 revision 请求        │ 调度     │ 小     │
│  P1  │ Submap keyframes_ 裁剪                         │ 资源     │ 小     │
│  P1  │ 回环噪声模型 +（可选）small_gicp 回环          │ 算法     │ 中     │
│  P1  │ d_min 首次 hit 重置 / miss 不写 d_min          │ 算法     │ 微小   │
│  P1  │ PointCloud2 边界与 dtype 校验                  │ 稳健性   │ 小     │
│  P1  │ wake_cv 谓词包含 requested 标志                │ 正确性   │ 微小   │
│  P2  │ 配置隙缝、文档同步、OpenMP/库热点、测试/CI     │ 债       │ 不等   │
└──────┴────────────────────────────────────────────────┴──────────┴────────┘

已关闭（勿再排期）：重复 `mapping/mapping/` 文件；`correctAt` O(N) 全表扫描。

---

九、一句话

全库算法骨架扎实，锁模型干净；当前最大风险是 **grid_map 核心与 nano_gicp ROS 包装错配**，
以及可视化路径仍可能每秒抢 PGO worker。Occupancy 调度 P0 在 core 已改对，但要
**接线 + revision 门控 + d_min 收口** 后才能安全默认开启。
