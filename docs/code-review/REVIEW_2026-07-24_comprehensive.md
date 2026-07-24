┌──────────────────────────────────────────────────────────────────────┐
│  ◆ SAPPHIRE  ::  COMPREHENSIVE CODE REVIEW                          │
│  DATE       ::  2026-07-24                                           │
│  BRANCH     ::  grid_map  (344561e)                                  │
│  UNCOMMITTED::  cfg/sapphire_mid360_directional_eskf.toml (config    │
│                 tweaks: GICP, 3cm grid, faster IMU init)             │
│  STASH      ::  2 stashes (grid_map + eskf profiling artifacts)      │
│  SCOPE      ::  全量源码审查 — 栅格图 + PGO 集成 + 管线架构          │
│  PRIOR      ::  P0-1 occupancy wiring was marked "done" — NOT TRUE   │
└──────────────────────────────────────────────────────────────────────┘

═══════════════════════════════════════════════════════════════════════
一、关键发现：上次审查的 P0-1 没有落地
═══════════════════════════════════════════════════════════════════════

上次审查 (REVIEW_2026-07-22_full.md) 标记的 P0-1 "occupancy wiring 断链"
——commit eae51d3 在 nano_gicp 分支上做了 sapphire_ros2 侧接线。
但当前分支是 grid_map。

验证：

  $ grep -r "occupancy\|OccupancyGrid" sapphire_ros2/src/
  → 0 hits

sapphire_ros2 当前代码里对 occupancy 的集成完全不存在。
P0-1 的修复在另一个分支上，当前分支没合进来。

这不是"栅格图有问题"——是栅格图的 ROS 侧大门压根没开。

═══════════════════════════════════════════════════════════════════════
二、栅格图模块逐段审查
═══════════════════════════════════════════════════════════════════════

文件：src/mapping/occupancy_grid.cpp (581行) + .hpp (120行)
测试：tests/occupancy_grid_test.cpp (353行)

2.1 算法正确性 — 5/6 通过，1 个重大遗漏

  ✅ 射线遍历 (castRay) — Bresenham 变体，主轴步进，无浮点除
  ✅ d-aware 更新规则 — hit/miss 分离，first-hit reset 正确
  ✅ 首次初始化 origin snapping — 防止 SubGrid 级坐标错位
  ✅ 扩展时 SubGrid shift — 数据正确迁移到新网格位置
  ✅ d_max / ground_margin / d_hit_min — 频带过滤逻辑正确

  🔴 P0  testObstacleClearsAfterDisappears 定义了但未调用
     ─────────────────────────────────────────────────────────
     定义位置：occupancy_grid_test.cpp:270-336
     main() 中：第 340-348 行只调用了 5 个测试函数
     第 6 个 testObstacleClearsAfterDisappears 完全没有被调用

     这是 P0 级：一个专门为验证"障碍物消失后栅格能清除"
     的回归测试默默躺在代码里，从未参与 CI。

     修复：main() 中加一行 testObstacleClearsAfterDisappears();

2.2 性能剖面

  🔴 P1  insertScan 两遍全量云循环
     ─────────────────────────────────────────────────────────
     第一遍 (L392-407)：遍历所有点 → 统计 XY bounds
     第二遍 (L422-459)：遍历所有点 → castRay + updateHit

     两遍都是 O(N)，对 Mid-360 ~20K 点/帧可合并省一半遍历。

  🔴 P1  toMsg 每帧 O(所有 SubGrid × 所有 Cell)
     ─────────────────────────────────────────────────────────
     toMsg() 对整个 grids_ 数组遍历 (L518-577)，包括未
     allocated 的空 SubGrid。访问 sub.allocated() 然后跳过
     — 但外层循环本身已经 O(grid_size_x * grid_size_y)。

     50m×50m 地图, 0.03m 分辨率 → ~104K 个 SubGrid
     → 每个都要检查 allocated()。可以维护一个 allocated 索引
     集合来跳过未分配的 SubGrid。

  🔴 P1  insertScan 周期日志每 20 帧 O(全量 Cell)
     ─────────────────────────────────────────────────────────
     L462-498：每 20 帧遍历所有 SubGrid 的所有 Cell 来统计
     occupied/free 计数。这是"为了打印一行日志"在做全量扫描。

     建议：维护 running 计数器，在 updateHit/updateMiss 时
     增量更新，日志只读计数器值。

  🟡 P2  castRay d_end 不匹配 clamped 路径
     ─────────────────────────────────────────────────────────
     L299-308：终点超出地图时 clamp 到边界，d_end 保持原值。
     d_step = (d_end - d_sensor) / N 用了原始 d_end，但路径
     长度已经 clamp → 插值出的 d_ray 不对。

     影响：地图边缘栅格的 visit_cnt 更新精度下降。非致命，
     因为边缘通常不是核心区域。

2.3 线程安全

  ✅ OccupancyGrid 用独立 mutex_ 保护全部 mutable 操作
  ✅ PGO 后端用 occupancy_mutex_ 隔离栅格导出，不与 T_map_odom
     竞争同一把 output_mutex_
  ✅ 无死锁风险（锁序：input_mutex_ → wake_mutex_ → 无嵌套）

2.4 代码规范

  🟡 P2  castRay 死分支
     ─────────────────────────────────────────────────────────
     L321-323：if (std::abs(dy) > std::abs(dx)) { if (dy == 0) return; }
     如果 dy==0 则 |dy| ≤ |dx|，此分支永不进入。无害但混乱。

  ✅ first-hit reset 注释详尽 (L245-248)
  ✅ 命名一致：hit_cnt / visit_cnt / d_min 语义清晰

═══════════════════════════════════════════════════════════════════════
三、PGO / 管线集成审查
═══════════════════════════════════════════════════════════════════════

3.1 架构质量

  ✅ Occupancy 与 PGO 解耦：独立 mutex，独立线程
  ✅ 增量 append vs 全量 rebuild 策略正确：
     odom 更新 → appendOccupancyFrames (增量)
     loop 更新 → rebuildOccupancyMap (全量重建)
  ✅ publishOccupancySnapshot 先 catch up 新帧 → 避免重复射线
  ✅ revision 机制正确：每次 insertScan +1，toMsg 携带 revision

  🟡 P1  回环 ICP fitness 作为全轴方差
     ─────────────────────────────────────────────────────────
     pose_graph.cpp:495：variances.setConstant(std::max(fitness, 1e-9))
     
     PCL ICP getFitnessScore() 返回值是 MSE (m²)，量纲是
     平移误差的平方。把这个值注入旋转轴方差物理上不正确：
     fitness=0.01 → sigma_rx²=sigma_tx²=0.01 → 把 10cm 平移精度
     当作 0.1rad (5.7°) 的旋转精度写入 ISAM2 的信息矩阵。

     后果：回环边在旋转轴上的信息权重严重偏高。走廊场景下
     fitness 小 → ISAM2 过度信任回环方向 → 错误回环校正时
     姿态剧烈跳变。

     已在上次审查标记 P1-2，尚未修复。

  🟡 P1  Submap keyframes 永不淘汰
     ─────────────────────────────────────────────────────────
     submap.cpp:47：keyframes_.push_back(...)，只有 add 没有 remove
     selectNearest 每帧扫描所有 keyframe (O(N))，长航时线性增长

     已在上次审查标记 P1-1，尚未修复。

3.2 资源管理

  ✅ ISAM2 每周期 update() → 正确
  ✅ 回环搜索 stride 控制 CPU 开销 → 合理
  ⚠ appendOccupancyFrames 不遵守 keyframe 距离过滤：
     PGO addFrame 有 keyframe_distance 过滤器，但 occupancy
     直接从 optimized_ 取所有帧插入。这合理（栅格图需要所有
     位姿上的点），但注释应说明为什么这里不用 PGO 的 keyframe 策略。

3.3 可扩展性

  ✅ Occupancy 与 PGO 完全解耦，未来可独立线程运行
  ✅ OccupancyGridMsg 是 ROS-free 的纯结构体 → 任何传输层可用
  ⚠ 没有 occupancy 配置热更新机制（合理，v0.1 不需要）

═══════════════════════════════════════════════════════════════════════
四、全架构横向审查
═══════════════════════════════════════════════════════════════════════

4.1 依赖审计

  ✅ CMakeLists.txt 依赖与源码匹配
     Eigen3  → 核心数学、所有模块使用
     spdlog  → 全局日志
     PCL     → 点云类型、ICP、transformPointCloud
     GTSAM   → PGO ISAM2 + 因子图
     OpenMP  → voxel downsampling 并行
     Threads → PGO worker 线程
     small_gicp → 前端 GICP 配准
     toml++  → TOML 配置解析

  无死依赖。GTSAM 只在 PGO enabled 时实际使用，但 CMake
  层面 REQUIRED — 对无 PGO 场景是一次性 overhead（可接受）。

4.2 线程模型

  ┌─ 主线程 (ROS callback) ─────────────────────────────────────┐
  │  pushLidar → processLidarScan                                │
  │    ├─ deskewPointcloud                                       │
  │    ├─ registration_.align()                                  │
  │    ├─ maybeUpdateSubmap → rebuildTarget (含 OMP 并行)        │
  │    └─ pgo_backend_.addFrame()                                │
  │  pushImu → propagateStateLocked                              │
  │  latestResult / latestPose / latestDeskewed → output_mutex_ │
  └──────────────────────────────────────────────────────────────┘
  
  ┌─ PGO worker 线程 ───────────────────────────────────────────┐
  │  workerLoop() → processPending()                             │
  │    ├─ copyPendingFrames (input_mutex_)                       │
  │    ├─ buildOdometryGraph → ISAM2 update → updateCorrection  │
  │    ├─ buildLoopEdges → ISAM2 update                         │
  │    ├─ appendOccupancyFrames / rebuildOccupancyMap            │
  │    └─ publishOccupancySnapshot (occupancy_mutex_)            │
  └──────────────────────────────────────────────────────────────┘

  锁依赖图（无环，无死锁）：
    input_mutex_  →  仅 addFrame / copyPendingFrames
    output_mutex_ →  T_map_odom + stats + snapshot
    occupancy_mutex_ → occupancy_msg_ (独立于 output_mutex_)
    wake_mutex_ + wake_cv_ → worker 唤醒

  设计正确。occupancy_mutex_ 独立于 output_mutex_ 是关键决策
  ——密集栅格拷贝不会阻塞前端 T_map_odom() 轮询。

4.3 RAII / 资源管理

  ✅ PIMPL：PoseGraphBackend 用 unique_ptr<Impl>
  ✅ ISAM2：unique_ptr，析构时自动释放
  ✅ Worker 线程：stop_ + join 正确
  ✅ OccupancyGrid：mutable mutex 保护
  ⚠ 无异常安全的 rollback：processPending 里 updateIsam 抛异常
     只打 log，不回滚 added_odom_id_。但 ISAM2 update 失败
     时状态是原子操作的（要么 succeed 要么 unchanged），所以
     实际上不需要 rollback。

4.4 边界条件

  ✅ insertScan 对空点云、非有限位姿矩阵均有保护
  ✅ worldToGlobalIndex 对空地图、越界坐标均有保护
  ✅ mutableCell 对 null SubGrid 返回 nullptr
  ✅ d_max <= 0 等效于无上限（L436 条件判断）
  ✅ 首次 resizeTo 时 origin snap 后 grid_size 至少为 1

═══════════════════════════════════════════════════════════════════════
五、优化优先级排序
═══════════════════════════════════════════════════════════════════════

🔴 P0 — 立即修复

  P0-1  testObstacleClearsAfterDisappears 未调用
        文件: occupancy_grid_test.cpp:340
        修复: 在 main() 中加一行 testObstacleClearsAfterDisappears();
        影响: 最重要的回归测试被跳过

  P0-2  occupancy 在 sapphire_ros2 未接线
        文件: sapphire_ros2/src/sapphire_ros.cpp
        状态: 修复在 nano_gicp 分支 (eae51d3)，未合入 grid_map
        修复: cherry-pick eae51d3 或手工移植 requestOccupancyGrid()
              publisher + revision 门控逻辑

🔴 P1 — 本迭代修复

  P1-1  insertScan 两遍全量云循环 → 合并为单遍
        文件: occupancy_grid.cpp:392-459
        节省: ~50% 遍历开销

  P1-2  toMsg 遍历所有 SubGrid（含未分配） → 维护 allocated 索引
        文件: occupancy_grid.cpp:518-577
        节省: 大场景下 ~80% 网格扫描开销

  P1-3  insertScan 日志全量 Cell 扫描 → 增量计数器
        文件: occupancy_grid.cpp:462-498
        节省: 每 20 帧避免一次 O(全部 Cell)

  P1-4  回环 ICP fitness 作为全轴方差 → 分轴标定
        文件: pose_graph.cpp:495
        修复: 平移 axis 用 fitness，旋转 axis 用至少 0.01
        影响: 回环对姿态信息的权重现在严重偏高

  P1-5  Submap keyframes 淘汰
        文件: submap.cpp:47
        修复: 当总数 > N*2 时淘汰最远的 50%
        影响: 长航时内存线性增长

🟡 P2 — 下个迭代

  P2-1  castRay d_end 与 clamped 路径不匹配
        文件: occupancy_grid.cpp:299-308

  P2-2  castRay 死分支清理 (dy==0)
        文件: occupancy_grid.cpp:321-323

  P2-3  appendOccupancyFrames 与 PGO keyframe_distance 策略差异
        补充注释说明为什么 occupancy 用所有帧

  P2-4  insertScan 中 d_sensor 硬编码为 0.0f
        sensor z 高度假设 = t.z()，但 d_sensor 恒为 0
        注释已正确，但变量语义可改名 d_sensor_ref

═══════════════════════════════════════════════════════════════════════
六、正面发现
═══════════════════════════════════════════════════════════════════════

  ✅ 两级索引 + 懒分配 (SubGrid 按需 malloc) — 内存效率优秀
  ✅ 首次 resizeTo origin snapping — 关键 bug fix，注释详尽
  ✅ d-aware 更新规则 + first-hit reset — 算法设计深思熟虑
  ✅ Occupancy 与 PGO 完全解耦，独立 mutex — 架构干净
  ✅ 增量 append + 回环重建策略 — 正确且高效
  ✅ 全量 validation 配置 (validateConfig) — 边界条件覆盖全面
  ✅ NO raw new/delete — 全用 unique_ptr / shared_ptr
  ✅ 线程模型无循环锁依赖 — 无死锁风险

═══════════════════════════════════════════════════════════════════════
七、一句话总结
═══════════════════════════════════════════════════════════════════════

  栅格图核心算法没有问题——数据结构、射线遍历、d-aware 模型都
  很扎实。但 ① 最重要的回归测试被跳过，② occupancy 在 ROS 侧
  根本没接线，③ 几个 P1 级性能热点（双循环、全量扫描、无淘汰）
  需要修。修完 P0 是功能可用，修完 P1 是生产级。
