┌─ DEEP REVIEW: Occupancy Grid ────────────────────────────────────────────┐
│  MODULE   :: sapphire/src/mapping/occupancy_grid.{hpp,cpp} + tests      │
│  DATE     :: 2026-07-23                                                 │
│  REV      :: v2 — 独立全量重审 (未对照 v1 文末清单逐个核验)              │
│  SCOPE    :: 算法正确性 · 性能剖面 · 线程安全 · 代码规范 · 测试覆盖      │
│  METHOD   :: 逐行读源码 → 对照设计文档 → 对照测试 → 对照类型系统         │
└──────────────────────────────────────────────────────────────────────────┘

═══════════════════════════════════════════════════════════════════════════
A. 总览
═══════════════════════════════════════════════════════════════════════════

模块概况：
  源码  :: occupancy_grid.hpp (120行) + occupancy_grid.cpp (574行)
  测试  :: occupancy_grid_test.cpp (268行, 5个用例)
  文档  :: occupancy_grid.md (611行设计说明)
  配置  :: types.hpp Occupancy (14个参数), cfg/*.toml (默认 enabled=false)
  集成  :: pose_graph.cpp (appendOccupancyFrames / rebuildOccupancyMap /
          publishOccupancySnapshot)

核心数据结构：
  两级索引 :: SubGrid (16×16 CellData) + 顶层 grids_[] 平面数组
  懒分配   :: SubGrid 首次访问才 malloc (256格 ≈ 3KB)
  单元字段 :: hit_cnt (uint32) / visit_cnt (uint32) / d_min (float)
  占据模型 :: hit_cnt/visit_cnt 比值 > occ_threshold, 非 log-odds

总体评价：数据结构简洁有力，两级索引+懒分配在大场景下内存友好。射线
遍历采用主轴斜率插值，循环内无除法。hit/visit 比值模型参数少收敛快。
d-aware 更新规则正确处理了 2.5D "高处射线不能穿透低处障碍物"问题。
线程模型清晰 (独立 mutex, 重活在外)。测试覆盖正向场景但缺口明显。

主要问题集中在三个层面：
  ① 一个算法级别的 d_min 管理 bug (P0)
  ② 多个文档/代码/配置语义不一致 (P1)
  ③ 若干性能和边界条件未覆盖 (P1-P2)

═══════════════════════════════════════════════════════════════════════════
B. 算法逐段审查
═══════════════════════════════════════════════════════════════════════════

B.1 射线遍历 (castRay, L276-344)
───────────────────────────────────────────────────────────────────────────

  [✓] 算法正确性 :: 主轴步进是标准的 Bresenham 变体。用 |dy| > |dx|
      选择主轴，沿主轴 step=±1，副轴用 lround(j*k) 插值。循环内
      一次乘法一次 lround，无除法/三角函数。

  [✓] d 插值 ::
      d_step = (d_end - d_sensor) / N  (N = |dy| 或 |dx|)
      d_ray = d_sensor + j * d_step
      增量形式正确，线性插值沿射线参数化。累积误差在 LiDAR 量程
      (40m) 内可忽略。

  [⚠] lround 变体 :: 副轴用 std::lround (round-half-away-from-zero)，
      与标准 Bresenham 的 round-half-up 有极细微差。实际无影响，
      两者在栅格 0.1m 分辨率下的最大差 < 0.5 格 — 不构成 bug。

  [⚠] 死分支 :: L314-316: |dy| > |dx| 分支内 if (dy == 0) return;
      若 dy==0 则 |dy| <= |dx|，此分支永不进入。无害死码。

  [✗] P2 :: clamp 后 d 值不一致 (L292-302)
      当终点 out-of-bounds 时 clamp 到地图边缘，mark_hit=false，
      但 d_end 保持原值。射线沿 clamped 格子走了，但 d_ray 插值
      用原 d_end → d_step 不对应 clamped 路径。
      修正：按 clamped (x1,y1) 反算实际比例 → 重算有效 d_end。

    ┌─ 示例 ──────────────────────────────────────────────────────┐
    │ sensor (0,0) → 远处点 out-of-bounds (d=5.0)                 │
    │ clamped end: (10,2)                                        │
    │ 使用的 d_step = (5.0 - 0) / N_original                     │
    │ 应使用的 d_step = (interpolated_d_at_clamp - 0) / N_clamped │
    │ 影响：地图边缘栅格的 d_ray 偏高/偏低 → visit_cnt 更新不准  │
    └─────────────────────────────────────────────────────────────┘

B.2 d-aware 更新规则 (updateHit / updateMiss)
───────────────────────────────────────────────────────────────────────────

  [✗] P0 — updateHit d_min 污染 (L240-250)
    ┌─ 问题 ──────────────────────────────────────────────────────┐
    │ current:  cell->d_min = std::min(cell->d_min, d);           │
    │ correct:  cell->d_min = (cell->hit_cnt == 0) ? d            │
    │                        : std::min(cell->d_min, d);           │
    │                                                              │
    │ 首次命中应 RESET d_min 为该障碍物的实际高度，而非与初始     │
    │ 1e4f 取 min。更关键的是：多障碍物先后命中同一 cell 时，     │
    │ d_min 应反映「当前存在的障碍物的最低高度」。                 │
    │                                                              │
    │ 场景：近地点先到 (d=-1.5, hit_cnt=0→1), d_min=-1.5          │
    │       墙点后到 (d=0.5, hit_cnt=1→2), d_min=-1.5 锁死       │
    │       → 障碍消失后 free 射线需 d_ray < -1.5+eps 才能清除   │
    │       → 几乎不可能清除 → 永久幽灵                           │
    └─────────────────────────────────────────────────────────────┘

    完整修复考虑：
      不只是 "首次 reset"，而是需要区分「同一障碍物的多次观测」
      (d 应保持 min) 和「不同障碍物命中同一 cell」(d_min 取最近
      障碍物高度)。简单的首次-reset 修了大部分 case，但如果有
      两个不同障碍物先后出现在同一 XY cell…这是极端边缘场景，
      可以后置。

  [✓] updateMiss d_min 保护 (L252-274)
      已修（40df126）：miss 不再写 d_min。逻辑：
      - hit_cnt > 0 且 d_ray <= d_min + clear_height_eps → visit_cnt++
        （射线低于或等于已确认障碍物底面 → 可以刷白）
      - hit_cnt > 0 且 d_ray > d_min + clear_height_eps → return
        （射线高于障碍物 → 不能穿透）
      - hit_cnt == 0 → visit_cnt++（无障碍物保护，直接刷白）

  [⚠] P1 — clear_height_eps 语义张力 (L266-267)
      `d_ray <= d_min + eps` 允许等高度射线穿透。这是修正多层墙的
      刻意设计。但这也意味着：一个有噪声 d_min（因上面 P0 污染）
      的 cell 更容易被清除。P0 修好后这个权衡更合理。

  [⚠] P1 — hit_cnt>0 vs 文档>3 (L265)
      代码保护从第一次 hit 就开始（hit_cnt > 0）。
      文档说 >3 才保护「强障碍物」。
      >0：更保守、更难被刷白 → 适合稀疏 LiDAR/低帧率
      >3：更激进、更易清幽灵 → 适合密集多帧观测
      两个选择都合理，但必须二选一 + 同步全部文档。

B.3 占据判定 (isOccupied / isFree, L227-238)
───────────────────────────────────────────────────────────────────────────

  [✓] hit_cnt >= 2 硬门槛 :: 防止单点噪声直接标 occupied。合理。
  [✓] occ_threshold=0.3 :: 30% 命中率即可标占据。对于 Mid-360
      密集扫描来说收敛很快；对稀疏雷达可能需要更低阈值。
  [⚠] isFree 语义 :: visit > 0 && !isOccupied → free。
      一个 visit=1 hit=0 的 cell 直接标 free。
      如果只有一条 free 射线穿过（地面点跳过不 cast），这可能是
      合理的。但如果 miss 射线很多密集→收敛快。

B.4 过滤层 (insertScan, L346-493)
───────────────────────────────────────────────────────────────────────────

  [✓] 有效性检查 :: 空点云 / 无效位姿 → early return

  [✓] voxel 下采样 :: max_threads=1，注释说明无误：在 PGO worker
      线程内避免与前端 VGICP/ESKF 抢 OpenMP 线程。合理。

  [⚠] 双趟 p_map 变换 :: bounds 扫描和 ray cast 各做一次
      T_map_lidar * p_body。可缓存 p_map，约节省 3-5% CPU。
      优先修复 P0 再考虑此优化。

  [⚠] 过滤语义与设计文档不一致 ::
      ┌─────────────────────┬───────────────────┬─────────────────┐
      │                     │ 实际代码行为       │ 设计文档         │
      ├─────────────────────┼───────────────────┼─────────────────┤
      │ world-Z vs body-z   │ p.z - t.z (world) │ z_hat·(p-t)     │
      │ out-of-band 处理    │ skip, 不 cast     │ cast free ray    │
      │ 地面点              │ 不参与任何更新     │ 参与 free 射线   │
      └─────────────────────┴───────────────────┴─────────────────┘

      world-Z 选择：使 d 值跨 keyframe 可比（多帧 d_min 一致），
      代价是斜坡场景下远端坡面点可能被误判为障碍物。这是有意识的
      权衡，不应视为 bug。

      out-of-band skip：Mid-360 地面点密度很高（每帧数千个），如果
      "地面也 cast free ray"，地面射线会以高密度刷白 in-band 障碍物
      前方的 free space → 性能开销大 + 可能过度刷白。当前 skip 策略
      是「防地面刷白」的刻意行为。代价是离开场景的障碍物残留无法被
      地面射线加速清除（但会被后续帧的 in-band free 射线慢慢清除）。

  [✓] bounds 增量扩展 :: 从 in-range 点计算 scan_bbox，排除远场噪点
      → 地图边界更紧致。L381-400 正确。

  [✗] P2 — toMsg 日志输出 (L476-492)
      条件 `revision_ <= 3 || revision_ % 20 == 0` — 每 20 帧日志一次。
      但日志中遍历所有 SubGrid 统计 occupied/free 计数的开销 ≈ 
      一次 toMsg 的一半（遍历所有 cell 但不填充 vector）。
      对性能不敏感但设计上：日志统计应复用 toMsg 内的遍历逻辑，
      或维护一个 active cell counter 增量更新。

═══════════════════════════════════════════════════════════════════════════
C. 逐数据结构审查
═══════════════════════════════════════════════════════════════════════════

C.1 SubGrid (occupancy_grid.hpp L68-83)
───────────────────────────────────────────────────────────────────────────

  [✓] 懒分配 :: data_=nullptr 初始，首次 access 时才 mallocIfNeeded()
  [✓] 移动语义 :: move ctor/assign = default → 高效 resize
  [✓] 拷贝构造 :: 深拷贝 data_ → resize 时 SubGrid 复制正确
  [⚠] cell() 重复边界检查 :: 顶层 worldToGlobalIndex 已检查全局索引
      范围，mutableCell 内部 cell() 又检查 sub_x/sub_y 范围。
      这是防御式编程 vs 性能的权衡。P2：可加 cellUnchecked() 快速路径。

C.2 CellData (hpp L62-66)
───────────────────────────────────────────────────────────────────────────

  [✓] 字段选择 :: hit_cnt/visit_cnt uint32_t (每个最大 ~4.3e9),
      d_min float (0.1m 分辨率下 float 精度足够): 每个 cell 12 bytes
  [⚠] 初始 d_min = 1e4f :: 比合理 LiDAR 量程大两个数量级。
      初次命中时 `min(1e4, d)` = d → 正确。
      但作为「未命中」标记用，语义上 INFINITY 更明确。

C.3 OccupancyGrid (hpp L30-118)
───────────────────────────────────────────────────────────────────────────

  [✓] 不可拷贝 :: 删除拷贝构造/赋值
  [✓] 序列号 :: revision_ 每次 insertScan 递增 → 外部可检测更新
  [⚠] grid_size_* 类型 :: int，grids_ 用 size_t 索引。在 32-bit 平台
      上 int 溢出前 grids_ 已经 OOM，不是实际问题。
  [✗] P2 — ensureBounds 死 API :: 公开方法但外部从未调用（pose_graph
      和测试都直接走 insertScan → resizeTo）。要么删除要么改 private。

═══════════════════════════════════════════════════════════════════════════
D. 线程安全审查
═══════════════════════════════════════════════════════════════════════════

  OccupancyGrid 内部：
  ┌─────────────────────────────────────────────────────────────────┐
  │ insertScan  :: 锁外做云过滤 → 锁内 resize+ray cast              │
  │ toMsg       :: 全锁 (扫描全量 SubGrid + 填充 vector)            │
  │ revision()  :: 轻量锁 (读一个 size_t)                           │
  │ empty()     :: 轻量锁                                           │
  └─────────────────────────────────────────────────────────────────┘

  [✓] 锁粒度 :: insertScan 的锁内段是 ray cast（主要 CPU），与 toMsg
      互斥。pose_graph 的 publishOccupancySnapshot 只在后端 worker
      上触发，不与 insertScan 并发。即使并发也只是短暂等锁。

  [✓] pose_graph 额外隔离 :: occupancy_mutex_ 保护导出消息指针，
      与 output_mutex_ 分离 → toMsg 拷贝不阻塞前端读 T_map_odom()。

  [✓] 无死锁路径 :: 单 mutex 模型，无多重锁定。

  [⚠] toMsg 锁持有时间长 :: 遍历全量 SubGrid 两次（一次找 max 索引，
      一次填充 data）。若地图很大（200m×200m, 4M cells），持有锁
      时间可到数十 ms → 阻塞 insertScan。影响仅在 pose_graph worker
      请求导出时，频率低（按需），实际不是问题。

═══════════════════════════════════════════════════════════════════════════
E. 性能剖面
═══════════════════════════════════════════════════════════════════════════

E.1 复杂度分析
───────────────────────────────────────────────────────────────────────────

  每帧 (insertScan):
    bounds 扫描      :: O(N_points) 变换+范围检查+min/max
    resize           :: O(grid_count) SubGrid 移动 (仅边界变化时)
    voxel 下采样     :: O(N_points log N) 排序 (cloud_voxel_size>0 时)
    ray cast         :: O(N_inband × avg_ray_length)

    其中 avg_ray_length = usable_range / resolution ≈ 40/0.1 = 400 cells
    N_inband ≈ N_points × in_band_ratio (取决于 h_clearance 配置)

    典型 Mid-360 ~20K points, in_band 60% → 12K rays × ~400 cells
    = ~4.8M cell accesses / frame. 每 cell 一次 mutableCell (位运算
    + SubGrid 索引 + mallocIfNeeded 检查) + updateMiss (条件分支)。

  toMsg:
    两次 SubGrid 平面扫描 = 2 × grid_size_x × grid_size_y 次指针检查
    + 已分配 SubGrid 的内部 16×16 遍历 = O(visited_cells)
    数据拷贝 = O(export_width × export_height) 字节

E.2 可优化点 (P1-P2, 不影响正确性)
───────────────────────────────────────────────────────────────────────────

  P1 缓存 p_map ─── 去重 T_map * p_body (3-5% CPU)
  P1 已分配 SubGrid 列表 ─ toMsg 跳过未分配区域 (当前已 skip，
     但顶层循环仍遍历全量 grid_size_x × grid_size_y)
  P1 cellUnchecked ── mutableCell 内部去掉重复边界检查
  P2 castRay clamp d ─ 见 B.1
  P2 active_cell_counter ─ 增量维护 occupied/free 计数，省掉日志
     遍历 (每 20 帧一次, 影响极小)
  P2 Bresenham 整数化 ── 用全整数 DDA 替代 lround/float 乘；
     精度在 0.1m 分辨率下无损。收益 ~5-15% ray cast CPU。
  P2 voxel max_threads ── 实测放开对整体吞吐的影响再决策

═══════════════════════════════════════════════════════════════════════════
F. 类型系统 / 配置一致性
═══════════════════════════════════════════════════════════════════════════

  F.1 配置默认值 (types.hpp L244-265) vs 使用语义
  ─────────────────────────────────────────────────────────────────

  h_clearance 注释 : "≈ sensor height above ground (1.5-2.5m)"
  h_clearance 默认 : 2.0
  使用语义        : d >= -h_clearance + ground_margin → HIT

  → 传感器离地 2m，地面点 d≈-2.0
  → d_hit_min = -2.0 + 0.3 = -1.7 (ground_margin=0.3)
  → 地面点 d=-2.0 < -1.7 → out-of-band → skip ✓

  此默认值在参数注释中写清为 "传感器安装高度"，与使用语义一致。
  design doc 中写 "底盘间隙 0.1-0.3m" 是不同语义下的另一套参数
  体系 — 两种语义不能混用。选择哪套取决于：是否希望 near-ground
  band 包含大部分障碍物（选 sensor height）还是窄带（选底盘间隙）。

  F.2 配置默认值 vs TOML 示例
  ─────────────────────────────────────────────────────────────────

  types.hpp 默认 vs cfg/sapphire_mid360.toml:
  ┌───────────────────────┬─────────────┬─────────────┐
  │ 参数                   │ types.hpp   │ mid360.toml │
  ├───────────────────────┼─────────────┼─────────────┤
  │ occupancy.enabled     │ false       │ false       │
  │ h_clearance           │ 2.0         │ 2.0         │
  │ ground_margin         │ 0.0         │ 0.3         │
  │ clear_height_eps      │ 0.05        │ 0.1         │
  │ d_max                 │ 3.0         │ 3.0         │
  │ occ_threshold         │ 0.3         │ 0.3         │
  │ usable_range          │ 40.0        │ 40.0        │
  │ cloud_voxel_size      │ 0.2         │ 0.2         │
  │ margin                │ 5.0         │ 5.0         │
  └───────────────────────┴─────────────┴─────────────┘

  一致，仅 ground_margin 和 clear_height_eps TOML 覆盖默认。
  P0 无关配置 — 用户看到的地面误占主因是 h_clearance 语义
  + enabled=false（栅格根本没启用）。

  F.3 类型安全
  ─────────────────────────────────────────────────────────────────

  [⚠] Options = Config::Pgo::Occupancy :: 类型别名绑定到具体嵌套类型。
      好处：API 不依赖配置文件格式。坏处：单元测试必须构造完整
      Config::Pgo::Occupancy 对象。可考虑用可注入的接口但过度设计。

  [✓] OccupancyGridMsg :: 值语义，支持移动。pose_graph 用
      shared_ptr<const OccupancyGridMsg> 传递 → 零拷贝读取。

═══════════════════════════════════════════════════════════════════════════
G. 测试覆盖审查
═══════════════════════════════════════════════════════════════════════════

G.1 已有测试
───────────────────────────────────────────────────────────────────────────

  testAttitudePlaneMarksWallOccupied       :: 基础占据+free空间
  testGroundFreeRaysDoNotEraseWall         :: out-of-band 不刷白墙
  testCeilingFilteredByDMax               :: d_max 上限过滤
  testGroundMarginSkipsNearGroundHits     :: ground_margin 效果
  testDepthJitterClearsPreviousWallCell   :: clear_height_eps 清除

  [✓] 每个测试独立构造 config，参数自包含
  [✓] 多帧叠加 (4次 insertScan) 测试收敛
  [✓] 测试通过返回值 0/1，CI 友好

G.2 缺失测试 (按严重度)
───────────────────────────────────────────────────────────────────────────

  P0 d_min 污染回归 ::
    近地点命中 (d=-1.5) → 墙命中 (d=0.5) → 墙消失 → 多帧后墙格标 free
    此测试应 FAIL 在当前代码上，修复后 PASS。

  P1 地图扩展 resize ::
    插入第一个 scan → 边界内；第二个 scan 超出边界 → 旧数据保留+新区域
    追加。验证 snapped_min 对齐和 shift 逻辑。

  P1 并发 toMsg+insertScan ::
    多线程同时调用 toMsg 和 insertScan，验证无崩溃/数据竞赛。

  P2 斜坡场景 :: 模拟 pitch=15° 姿态，验证 world-Z 下坡面点不被
    误标为 occupied（当前预期：可能误标 — 这是已知权衡）。

  P2 边界 clamp :: castRay 终点 out-of-bounds → clamp → 验证
    d 插值和 mark_hit=false 行为。

  P2 空地图 :: 未 insert 任何 scan → toMsg 返回 width=height=0

  P2 toMsg origin 稳定 :: 多次 insert + toMsg → origin_x/y 不抖动

═══════════════════════════════════════════════════════════════════════════
H. pose_graph 集成审查
═══════════════════════════════════════════════════════════════════════════

H.1 appendOccupancyFrames (L593-604)
───────────────────────────────────────────────────────────────────────────

  [✓] 追加逻辑 :: 从 occupancy_inserted_id_ 到 optimized_.size()
      逐个 insertScan。odom 更新时只追加新 keyframe，不重建。
  [✓] cloud 存储为本体系 (frames_[i].cloud_lidar 是 T_odom_lidar
      变换后的 lidar-frame cloud，即 body-frame 点云) — 正确。
      insertScan 用 ISAM2 优化的 pose 重新变换到世界系。

H.2 rebuildOccupancyMap (L606-617)
───────────────────────────────────────────────────────────────────────────

  [✓] 回环触发 :: clear() → 重置 occupancy_inserted_id_ → 全部重投
  [✓] revision 变化 :: clear 自增 revision, insertScan 自增 revision
      → 外部 publishOccupancySnapshot 可检测新数据。

H.3 publishOccupancySnapshot (L619-632)
───────────────────────────────────────────────────────────────────────────

  [✓] 先补帧后导出 :: 避免 exports 缺最新 keyframe
  [✓] shared_ptr 语义 :: 零拷贝传递，occupancy_mutex_ 保护赋值

H.4 生命周期
───────────────────────────────────────────────────────────────────────────

  [✓] occupancy_ 在 Impl 构造函数创建，worker 启动在之后
      → 初始化顺序正确
  [✓] ~Impl 先设置 stop_ → join worker → 隐式销毁 occupancy_
      → 销毁顺序正确

═══════════════════════════════════════════════════════════════════════════
I. 代码规范审查
═══════════════════════════════════════════════════════════════════════════

  [✓] 命名 :: PascalCase 类/结构体, camelCase 方法, snake_case 变量
      全模块一致
  [✓] 注释 :: 英文，关键权衡有注释（world-Z 选择、out-of-band skip、
      d_min 语义、clear_height_eps 理由）
  [✓] 头文件依赖 :: #pragma once + 最小化 include
  [✓] 匿名命名空间 :: 辅助函数不污染全局
  [⚠] 未使用 include :: <limits> (无 std::numeric_limits 额外使用),
      <memory> (仅 unique_ptr/shared_ptr — 在 hpp 已引入)
      — 极微小
  [⚠] floating-point literal :: 部分用 1e4f, 1e-3 等；部分用 0.0f,
      0.1f。风格不一致但不影响正确性。

═══════════════════════════════════════════════════════════════════════════
J. 与设计文档 (occupancy_grid.md) 的对账
═══════════════════════════════════════════════════════════════════════════

  文档描述的设计 vs 实际代码实现：

  ┌──────────────────────────┬──────────────┬──────────────┬──────────┐
  │ 项目                      │ 文档          │ 代码          │ 性质     │
  ├──────────────────────────┼──────────────┼──────────────┼──────────┤
  │ 过滤轴                    │ body-z (姿态) │ world-Z       │ 权衡差异 │
  │ out-of-band 地面处理       │ cast free     │ skip          │ 刻意简化 │
  │ h_clearance 语义          │ 底盘间隙 0.3  │ 传感器高 2.0  │ 语义漂移 │
  │ d_min 首次 hit            │ RESET d       │ std::min      │ BUG      │
  │ hit_cnt 保护阈值           │ >3           │ >0            │ 参数分歧 │
  │ isOccupied 最小 hit       │ visit_cnt>3  │ hit_cnt>=2    │ 完全重写 │
  │ Keyframe 重建             │ 全量 ray cast │ append+rebuild│ 一致     │
  │ SubGrid 懒分配            │ 16×16         │ 16×16         │ 一致     │
  │ hit/visit 比值模型         │ 比值>0.3     │ 比值>0.3      │ 一致     │
  └──────────────────────────┴──────────────┴──────────────┴──────────┘

  文档 (occupancy_grid.md) 需要大修：当前描述的是设计阶段的意图，
  代码反映的是实现过程中的权衡调整。两者都是合理的，但不一致会
  误导后续开发者。建议在文档中新增"实现偏离"章节逐条说明。

═══════════════════════════════════════════════════════════════════════════
K. 修复优先级排序
═══════════════════════════════════════════════════════════════════════════

  ┌────────┬──────────────────────────────────────────────┬─────────────┐
  │ 优先级  │ 项目                                         │ 影响         │
  ├────────┼──────────────────────────────────────────────┼─────────────┤
  │ P0-1   │ updateHit 首次 hit RESET d_min               │ 幽灵/分层    │
  │ P0-2   │ 补 d_min 污染回归测试                        │ 质量保障     │
  ├────────┼──────────────────────────────────────────────┼─────────────┤
  │ P1-1   │ 同步文档：hit_cnt>0, out-of-band skip,       │ 可维护性     │
  │        │ world-Z, h_clearance 语义                    │              │
  │ P1-2   │ 缓存 p_map（去双趟变换）                      │ 性能 ~3-5%   │
  │ P1-3   │ 补测试：resize、并发、边界 clamp              │ 质量保障     │
  ├────────┼──────────────────────────────────────────────┼─────────────┤
  │ P2-1   │ castRay clamp d 重算                         │ 边界正确性   │
  │ P2-2   │ ensureBounds 删/改 private                   │ 死码清理     │
  │ P2-3   │ cellUnchecked 快速路径                       │ 性能微优化   │
  │ P2-4   │ toMsg active list 优化                       │ 性能微优化   │
  │ P2-5   │ Bresenham 全整数化                           │ 性能 5-15%   │
  │ P2-6   │ 斜坡场景测试 (world-Z vs body-z 行为差异)     │ 场景验证     │
  └────────┴──────────────────────────────────────────────┴─────────────┘

═══════════════════════════════════════════════════════════════════════════
L. 总结
═══════════════════════════════════════════════════════════════════════════

OccupancyGrid 模块的整体架构质量高。数据结构 (两级懒分配 SubGrid)
和算法设计 (斜率插值 Bresenham + d-aware 更新) 清晰正确。

唯一算法级 bug 是 `updateHit` 的 d_min 污染（首次 hit 应 RESET
而非取 min）。这是"幽灵障碍物 / 墙壁分层 / 障碍无法清除"的可验证
根因。修复需 3 行代码 + 回归测试。

文档-代码不一致是第二大问题：多个设计文档中的描述与实际代码行为
不同（过滤轴、保护阈值、地面处理策略）。这些差异并非"代码写错了"，
而是实现过程中的有意识权衡，但文档未同步更新。

性能方面：主要 CPU 消费在 Bresenham ray casting 的 cell 遍历
（约 4.8M cell access/帧 @ Mid-360）。可优化点有多项但不影响
正确性，建议先修 P0 再逐个评估。

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  VERDICT :: 1 个 P0 算法 bug + 多个 P1 语义/文档不一致。
             数据结构设计优秀，线程模型清晰，测试有基础覆盖。
             修 P0 后整体可投入生产。
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
