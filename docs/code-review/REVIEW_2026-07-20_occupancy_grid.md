┌─ REVIEW ─────────────────────────────────────────────────────────────┐
│  ◆ PROJECT  :: Sapphire Occupancy Grid                               │
│  ◆ DATE     :: 2026-07-20                                            │
│  ◆ SCOPE    :: mapping/occupancy_grid + PGO integration + config     │
│  ◆ FILES    :: 9 source + 1 doc + 1 test                             │
│  ◆ VERDICT  :: 设计扎实，无逻辑 bug，3 个 P1 / 4 个 P2               │
└──────────────────────────────────────────────────────────────────────┘

一、总体评价

栅格图本身的算法设计很扎实。姿态感知平面过滤（z_hat = R.col(2)）避开了
地面分割在坡道上的根本困难，d-aware 更新规则正确解决了"高射线穿透低障碍物"
问题。SubGrid 懒分配 + hit/visit 比值模型选型合理，代码量小（478 行 cpp +
119 行 hpp）且无第三方依赖。

PGO 集成路径干净：keyframe 存本体系（cloud_lidar），PGO 优化后调用
insertScan 用新位姿重新投影 → 全量重建。这比 submap 拼接或双缓冲方案简单
得多。

没有发现逻辑 bug。以下问题按优先级排列。

二、P1 — 值得修但不阻塞上线

2.1 ensureBounds 是死 API
  ─────────────────────
  occupancy_grid.hpp:42 声明了 public ensureBounds()，但整个代码库中唯一的
  调用者是 insertScan() 内部的 resizeTo()（直接调用，不经过 ensureBounds）。
  ensureBounds 对外部从未被调用，且其语义（手动预扩地图）在当前 PGO 驱动的
  增量插入 + 回环重建流程中没有使用场景。

  建议：删除，或至少标记为 private 并加注释说明未来用途。

2.2 d_sensor 恒为 0 — 误导性变量名
  ─────────────────────────────────
  occupancy_grid.cpp:342-343:
    const Eigen::Vector3d sensor = t;  // lidar origin = translation
    const float d_sensor = static_cast<float>(z_hat.dot(sensor - t));
                                   // ^ always zero since sensor == t

  几何上正确（传感器原点在姿态平面 Π 上，有符号距离为 0），但变量名暗示
  它可能非零。后续 d_step = (d_end - d_sensor) / N → d_end / N，
  d_sensor 从未贡献非零值。

  建议：直接删掉 d_sensor 变量，castRay 签名改掉 d_sensor 参数，
  沿途 d 插值从 0 开始即可。或保留但加注释 "d_sensor ≡ 0 (sensor on Π)"。

2.3 insertScan 对点云做两次世界坐标变换
  ──────────────────────────────────────
  insertScan 的第一趟遍历计算 scan_min/max（用于 resizeTo），第二趟遍历
  做 ray casting。两趟都对每个点做 T_map_lidar * p_body（3D 刚体变换）。

  对于 20K 点/帧的 Mid-360，变换开销 ≈ 20K × 2 × ~30 flops ≈ 1.2M flops，
  实际可以忽略。但如果将来支持稠密激光雷达（128 线 × 100K+ 点），两趟变换
  会变成可测量的开销。

  建议：第一趟缓存 p_map，第二趟复用。当前优先级低，加 TODO 即可。

三、P2 — 小问题

3.1 rebuildOccupancyMap 的 info 日志无限流
  ─────────────────────────────────────────
  pose_graph.cpp:602-605:
    spdlog::info("[pgo] occupancy grid rebuilt from {} keyframes (rev={})",
                 occupancy_inserted_id_, occupancy_->revision());

  每次回环 closure 触发一次。如果连续检测到多个回环边（同一轮 worker 周期
  内），会刷多条。没有 throttle。

  建议：加 revision_ % 10 == 0 或类似节流，或者降到 debug。

3.2 OccupancyGridMsg 的 data 分配是全尺寸的
  ─────────────────────────────────────────
  occupancy_grid.cpp:445-447:
    msg.data.assign(width * height, -1);

  对于 200m×200m@0.1m = 4M cells × 1 byte = 4MB。一次性分配没问题，
  但 ROS2 的 OccupancyGrid 消息序列化会把整个 vector 拷贝到 DDS 层。

  建议：暂时不用动。如果以后发现 ROS2 层延迟偏高，加一个 sparse export
  （只导 occupied+free cells，其余用 run-length encoding）。

3.3 SubGrid 条件编译常量是硬编码的
  ────────────────────────────────
  kSubGridBits = 4（SubGrid 16×16）硬编码在头文件里，不可配。
  256 cells/block 对大多数场景合理，但极端稀疏场景（沙漠/海面，cell 利用率<1%）
  可能浪费内存（每个 hit SubGrid 占 256 × 12 bytes = 3KB）。

  建议：保持现状。真遇到内存问题再做成 template<size_t SubGridBits>。

3.4 测试覆盖缺口
  ──────────────
  现有测试只覆盖了：
    - 墙壁占据（垂直平面，多次 hit 后 occ_threshold 收敛）
    - 天花板过滤（d_max 上限）
  缺失：
    - 非零姿态（roll/pitch、非 Identity 位姿）—— z_hat 倾斜时过滤是否正确
    - d-aware 防穿透：障碍物上方射线不应刷白已占据栅格
    - resizeTo 扩展：map 扩张时旧 subgrid 数据是否保留在正确偏移
    - 空点云 / 单点 / 全 NaN / 全 out-of-range 边界情况
    - 多次重建（clear + append）的 revision 正确性

  建议：补 2-3 个：姿态倾斜测试（最核心创新点）、d-aware 防穿透测试、
  resize 正确性测试。

四、文档一致性

文档 docs/occupancy_grid.md 写得非常好，数学推导、设计对比、PGO 重建流程
都清晰。以下小不一致：

4.1 "Bresenham" 命名
  文档多处说 "Bresenham 射线遍历"，但代码用的是斜率法 DDA（浮点 k × 整数 j →
  lround），不是经典 Bresenham（纯整数增量误差累积）。功能等效但命名不准。
  建议统一为 "DDA" 或 "slope-based ray casting"。

4.2 重建流程伪代码与实现差异
  文档 §重建流程 描述了逐点 ray casting 循环，实际实现是 clear() +
  appendOccupancyFrames() → insertScan()。高层语义一致，但文档暗示在 PGO
  后端直接做点级操作，实际是通过 insertScan 复用。

  建议：文档加一句"实际通过 insertScan 复用现有实现"。

4.3 SubGrid** vs vector<SubGrid>
  文档用 C 风格 `SubGrid** grids_` 伪代码，代码用 `std::vector<SubGrid>`。
  正常的设计文档伪代码 vs 实际实现差异，可以接受。

五、总结

┌──────┬──────────────────────────────────────────┬─────────┬────────┐
│ Pri  │ Item                                     │ Impact  │ Effort │
├──────┼──────────────────────────────────────────┼─────────┼────────┤
│  P1  │ 删除 ensureBounds 死 API                  │  低     │ 小     │
│  P1  │ d_sensor 恒为 0 → 简化变量               │  低     │ 小     │
│  P1  │ 点云双趟变换 → 缓存 world points           │  中     │ 小     │
│  P2  │ rebuild 日志节流                          │  低     │ 微小   │
│  P2  │ 补姿态 + d-aware + resize 测试             │  中     │ 中     │
│  P2  │ 文档 "Bresenham" → "DDA"                 │  低     │ 微小   │
│  P2  │ toMsg 全尺寸分配（暂不处理）               │  低     │ -      │
└──────┴──────────────────────────────────────────┴─────────┴────────┘

一句话：算法正确，实现干净，没有 bug。P1 项都是小修小补，不影响功能正确性。
