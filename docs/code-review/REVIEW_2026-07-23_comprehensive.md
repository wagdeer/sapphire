┌────────────────────────────────────────────────────────────────────┐
│  ◆ SAPPHIRE  ::  COMPREHENSIVE CODE REVIEW                        │
│  2026-07-23  /  branch: grid_map  /  ~9,185 LOC (core only)       │
│  commits reviewed: ea5cd17..4a0d8f5 (6 occupancy + prior)         │
│  scope: sapphire core + sapphire_ros2 wrapper                     │
└────────────────────────────────────────────────────────────────────┘

Scope
─────
| Area            | Files | LOC    | Status                   |
|-----------------|-------|--------|--------------------------|
| Odometry core   | 8     | ~3,500 | Stable, observer+ESKF     |
| PGO backend     | 1     | ~726   | Async ISAM2, loop closure |
| Occupancy grid  | 2     | ~700   | 6 recent fixes, hardening |
| Voxel filter    | 2     | ~280   | Parallel, deterministic   |
| Config/TOML     | 2     | ~740   | Full load+validate        |
| RingBuffer      | 1     | ~180   | Header-only, iterators    |
| ROS2 wrapper    | 3     | ~583   | Thin, format conversion   |
| Tests           | 13    | ~3,300 | 13 executables            |

一、总体评价
───────────
架构干净的分层 LIO 前端 + PGO 后端。ODOM pipeline: IMU push → deskew → voxel downsample
→ GICP registration → observer/ESKF fusion → submap → PGO worker thread。ROS wrapper
只做 sensor_msgs↔sapphire::Point 转换，不参与算法逻辑。线程模型清晰：imu_mutex_
+ state_mutex_ + output_mutex_ 三重保护，锁序一致。PIMPL 把 GTSAM 头文件隔离在
pose_graph.cpp 内。

近期 6 个 occupancy grid commit 密集修复了 P0 级 bug：d_min 污染、坐标不对齐、
strict < 不等式锁死、幽灵障碍物。质量密集但偏防御性——每个补丁都是"先有 bug 再修复"
模式，现在是时候做一轮结构化优化。

二、优化发现（按严重程度）
─────────────────────────

2.1  MEDIUM  OccupancyGrid::toMsg() 双重全网格扫描
     ─────────────────────────────────────────────
     文件: src/mapping/occupancy_grid.cpp:502–579
     现状: toMsg() 先遍历全网格找 max_gx/max_gy（517–536 行），
     再遍历全网格填充 data buffer（549–577 行）。每次导出都是
     2 × O(N_subgrids × 256 cells)，在 PGO worker 上串行。
     影响: 大场景 1000+ SubGrid → ~512K cell 访问 × 2 = ~1M 次
     影响: 每次 occupancy_requested_ 触发时阻塞 worker。RViz 1Hz
     轮询意味着每秒 ~1M cell 访问。

     方案: 在 insertScan / castRay 中维护 running_max_gx/max_gy，
     toMsg() 只用一次遍历。running max 只在原 cell 为 -1（未知）
     且新 cell 为非 -1 时更新，成本可忽略。

2.2  MEDIUM  deterministicVoxelDownsample 三路重复
     ──────────────────────────────────────────────
     文件: src/odometry/voxel_filter.cpp:61–135
     现状: in_parallel / use_parallel / serial 三路代码各 ~35 行，
     唯一的差异是 #pragma 前缀。三重维护增加了 drift 风险。
     方案: 将 voxel 索引计算提取为 lambda，pragma 用条件编译
     或模板分派。

2.3  LOW     PGO worker 空转轮询
     ────────────────────────────
     文件: src/backend/pose_graph.cpp:226–248
     现状: wake_cv_.wait_for(lock, period, ...) — predicate 只有
     stop + 三个 request flag。即使无新帧，worker 也按 update_period_sec
     频率唤醒、copyPendingFrames()（空 vector）、buildOdometryGraph()
     （added_odom_id_ == frames_.size() → false）然后立即返回。
     影响: 无功能问题，但每秒 ~1 次无意义的 mutex 争用 + 函数调用。

     方案: 在 addFrame() 中设置 input_changed_.store(true)，
     predicate 中检查 || input_changed_.exchange(false)，或者
     在 predicate 中比对 input_frames_.size() != frames_.size()。

2.4  LOW     OccupancyGrid 诊断扫描
     ──────────────────────────────
     文件: src/mapping/occupancy_grid.cpp:462–499
     现状: rev <= 3 或 rev % 20 时遍历全部 SubGrid 统计 occupied/free
     计数。网格膨胀后这是 O(all cells) × spdlog::info 格式化。
     影响: 小场景可忽略；大场景（500+ SubGrid）可能在每 20 次插入
     时产生 ~10ms 延迟。

     方案: 维护 running occupied/free 计数（增删 cell 时更新），
     或者只在 toMsg() 快速扫一遍后缓存统计。减少诊断频率
     到 rev % 100。

2.5  LOW     config.cpp 字段加载冗余
     ───────────────────────────────
     文件: src/config.cpp:85–442（360 行！）
     现状: 每一个 config 字段都有独立的 if (const auto value = ...)
     块，共约 80 个字段 × 3–4 行 = 近 300 行 boilerplate。
     方案: 可写一个模板函数 reduce，但当前代码清晰可读且一次性
     加载，不改亦可。仅风格层面的 observation。

三、架构评估
───────────

3.1  ✅  线程模型清晰
     pipeline.cpp 对三个 mutex（imu_mutex_, state_mutex_, output_mutex_）
     的锁序一致：scoped_lock(imu, state) 仅出现在 rebasePropagation
     和 ESKF correctAt 路径。output_mutex_ 保持最外层，从不与另两个
     嵌套。

3.2  ✅  PIMPL 隔离 GTSAM
     pose_graph.hpp 头文件不暴露任何 GTSAM 类型。OccupancyGridMsg 是
     POD 结构体，避免 nav_msgs 依赖传入 core。PoseGraphBackend::Impl
     藏在 .cpp 匿名 namespace + unique_ptr。

3.3  ⚠   GTSAM 是硬依赖
     CMakeLists.txt:38: find_package(GTSAM REQUIRED)
     即使 pgo.enabled = false，GTSAM 仍然被链接。pose_graph.cpp 始终
     编译。可改为 OPTIONAL + #ifdef SAPPHIRE_ENABLE_PGO。

3.4  ✅  ESKF 的 Joseph + 重置雅可比
     协方差更新使用 Joseph 形式（避免数值不对称），方向重置用一阶
     雅可比 Jr = I - 0.5*skew(dx)，考虑完备。

四、热点分析（基于静态代码阅读 + 现有 PROFILE 数据）
────────────────────────────────────────────────

| 模块              | 估计占比 | 状态                     |
|-------------------|----------|--------------------------|
| GICP (small_gicp) | 40-50%   | 外部库，不可控            |
| VoxelFilter       | 5-10%    | 已并行化 + OpenMP aware  |
| Deskew            | 2-5%     | 已 OpenMP 并行           |
| ESKF predict      | 2-3%     | 均值传播 + 协方差传播     |
| Submap merge      | 1-3%     | keyframe 变更时重建       |
| Occupancy grid    | 1-2%     | PGO worker 串行，可接受   |
| PGO ISAM2         | 1-2%     | GTSAM 可控               |
| Config load       | <0.1%    | 一次性                   |

主要瓶颈在 small_gicp 外部库，不必在此优化。VoxelFilter 的
omp_in_parallel() 感知是正确选择——避免嵌套 fork。

五、Occupancy Grid 状态机审计
─────────────────────────────

最近 6 个 commit 修复的状态机问题回顾：

  [已修复 ✅]  d_min 首次命中重置 (4a0d8f5)
  Root: d_min 无条件 min() 导致近地点永久压低
  Fix: hit_cnt==0 时 d_min=d，后续 min(d_min,d)

  [已修复 ✅]  SubGrid 坐标不对齐 (a8c658f)
  Root: min_x_/min_y_ 未 snap 到 SubGrid 边界
  Fix: 初始化和每次 expand 都 floor 到 subgrid_reso_ 倍数

  [已修复 ✅]  Strict < 不等式 (533bee5)
  Root: d_ray < d_min 导致同高度穿墙锁死
  Fix: d_ray <= d_min + clear_height_eps

  [已修复 ✅]  World-Z 替代 body-z (ea5cd17)
  Root: R.col(2) 耦合传感器姿态
  Fix: 用 p_map.z() - t.z() 全局 Z 坐标

  [已修复 ✅]  自由射线腐蚀 d_min (40df126)
  Root: updateMiss 无条件更新 d_min
  Fix: updateMiss 永远不碰 d_min

  [已修复 ✅]  PGO worker cv 漏唤醒 (d5e5908)
  Root: predicate 不包含 snapshot/map/occupancy request flag
  Fix: 三 flag 均在 predicate 中检查

残存风险：无已知 P0 问题。边界条件（scan 在 grid 边界之外时 castRay
clamp 逻辑）已被测试覆盖。

六、正面发现
───────────

  [+] RingBuffer 实现质量高 — 完整随机访问迭代器，copy/move 语义正确
  [+] 测试覆盖全面 — 13 个独立 executable 覆盖核心模块
  [+] Occupancy grid 测试精心设计 — hit/visit 比率控制、d-aware 清除路径
  [+] LTO/IPO 支持 — 两个 CMakeLists 都检查并启用
  [+] std::optional 表示未初始化状态而不依赖 sentinel 值
  [+] spdlog 诊断丰富但可控 — 关键路径只在特定频率打印（% 20, rev <= 3）
  [+] sapphire_ros2 保持 thin — ROS 到 sapphire 的转换纯格式，不混入算法

七、优先级排序
─────────────

| Pri  | Item                                        | Impact       | Effort |
|------|----------------------------------------------|--------------|--------|
| P1   | Occupancy toMsg 双重扫描消除                  | 中（大场景）  | 小     |
| P1   | VoxelFilter 三路代码去重                     | 低（维护性）  | 小     |
| P2   | PGO worker 空转消除                          | 极低          | 极小   |
| P2   | Occupancy 诊断扫描频率/方式                   | 低            | 小     |
| P3   | GTSAM 可选化                                 | 中（部署）    | 中     |
| P3   | config.cpp 模板去 boilerplate               | 低            | 中     |

八、一句话总结
─────────────
代码质量扎实，最近密集修复了 occupancy grid 的 6 个 P0 状态机 bug。
当前无已知正确性问题。两个 MEDIUM 级优化（toMsg 双重扫描、VoxelFilter
去重）投入产出比高，建议在下个 PR 一起处理。
