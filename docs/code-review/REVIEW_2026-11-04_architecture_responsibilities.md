# Sapphire 架构与职责审查

> **日期**: 2026-11-04
> **模式**: Technical Debt Audit
> **范围**: `sapphire` core、`sapphire_ros1`、`sapphire_ros2`
> **基线**: `sapphire@09c8005`、`sapphire_ros1@7dbf4df`、`sapphire_ros2@243735c`
> **性质**: 只读架构审查；本文档只记录结论，不包含代码修改

## 摘要

Sapphire 的数学核心相对清晰，主要架构风险出现在编排层、artifact
所有权和 ROS 适配层。当前代码能够工作，但多项独立需求集中在少数大型
对象中，已经增加了修改波及范围和行为漂移风险。

本次审查的首要主线是收紧职责和依赖方向，而不是增加抽象层。判断标准是
一个需求能否在局部模块内完成，而不是类或接口的数量。

本次审查发现 4 项可维护性风险：

- 2 项 High；
- 2 项 Medium；
- 0 项确定性功能缺陷。

## 范围与覆盖

审查了以下内容：

- core 的公共头文件和主要实现；
- `OdometryPipeline` 的编排职责与公共 API；
- `PoseGraphBackend` 的接口、worker、回环、优化、轨迹和地图导出职责；
- `OccupancyGrid`、TUM 输出和它们与 backend 的依赖方向；
- ROS 1 与 ROS 2 wrapper 的参数、回调、轨迹保存和 PGO 可视化路径；
- 当前 `INTERFACE_CONTRACT.md` 中记录的共同契约；
- `docs/code-review/` 中已有的历史审查结论，仅作为线索；
- `pose_graph_test.cpp`、`pipeline_test.cpp`、`config_test.cpp` 等
  现有测试的结构和覆盖重点。

覆盖限制：

- 本次是结构性审查，不是逐行算法证明；
- 未重新运行完整构建和测试；
- 结论针对当前职责边界和依赖方向，不把局部代码风格偏好记为风险；
- 各仓库在审查时存在的未提交文档修改只作为契约意图参考。

## 确定性缺陷

本次架构审查范围内未发现确定性功能缺陷。

## 可维护性风险

### B-RISK-001 · `PoseGraphBackend` 承担了过多职责

- **严重程度**: High
- **类别**: responsibility
- **位置**:
  - `include/sapphire/backend/pose_graph.hpp:68`
  - `src/backend/pose_graph.cpp:302`
- **来源**: not applicable

**证据**

`PoseGraphBackend` 的公共接口同时包含：

- 关键帧输入和 odometry sample 记录；
- 回环候选搜索与 GICP；
- ISAM2 图优化与残差校验；
- map trajectory 回放；
- 稀疏全局地图；
- 可视化快照；
- 2.5D 占用栅格。

对应实现文件约 1801 行。`Impl` 同时持有输入队列、图状态、轨迹历史、
输出快照、占用栅格和 worker 生命周期。

**触发条件**

修改回环算法、占用栅格策略、轨迹语义、可视化输出或地图导出方式时，
都需要进入同一个类和同一套 worker 状态。

**影响**

- 一个功能变化会牵连多个无关功能；
- 回环、轨迹和地图导出难以独立测试；
- 公共接口无法清楚表达各子系统的变化边界；
- 线程、锁和提交状态被持续集中到单一大对象中。

**建议**

保持外部 API 暂时不变，先做内部拆分：

1. `PoseGraphEstimator`：ISAM2、因子图和优化提交；
2. `LoopClosureRegistration`：候选搜索、query/target 构建和 GICP；
3. `MapTrajectoryRecorder`：odometry sample 和 keyframe correction 回放；
4. `MapExportService`：稀疏地图、占用栅格和可视化快照；
5. `PoseGraphBackend`：只负责队列、通知和 worker 生命周期。

优先从 `MapTrajectoryRecorder` 和 `MapExportService` 开始，因为这两个
边界相对独立，且可以暂时保持现有公共 API。

**置信度**: High

### B-RISK-002 · ROS 1/ROS 2 复制了完整运行时策略

- **严重程度**: High
- **类别**: duplication
- **位置**:
  - `sapphire_ros1/src/sapphire_ros.cpp:139`
  - `sapphire_ros2/src/sapphire_ros.cpp:157`
- **来源**: not applicable

**证据**

两个 wrapper 分别实现了：

- 配置和参数解析；
- 轨迹保存和路径刷新；
- PGO snapshot 请求与轮询；
- 全局地图发布；
- 可视化和 TF 输出策略。

ROS 1 主运行时约 510 行，ROS 2 主运行时约 548 行，另有各自的
conversion 实现。退出策略也已经不同：ROS 1 保存后调用 `_Exit`，ROS 2
在保存后继续执行正常销毁。

**触发条件**

修改 TUM 输出、轨迹发布、PGO artifact 或节点退出语义时，需要同步修改
两个 wrapper 和相关接口文档。

**影响**

- wrapper 不再只是薄适配器；
- 相同契约靠人工同步，容易出现行为漂移；
- 与 ROS 无关的运行时策略无法复用和集中测试；
- ROS 1/2 的退出与 artifact 语义可能继续分叉。

**建议**

引入 ROS 无关的 `SapphireRuntime` 或 `SapphireSession`：

- 集中负责输入编排、轨迹状态、PGO snapshot 和 artifact 生命周期；
- 向 wrapper 提供稳定的领域结果；
- ROS 1/2 仅保留消息转换、QoS、callback group、executor 和日志适配。

不建议合并 ROS 包；应共享包上方的运行时逻辑。

**置信度**: High

### B-RISK-003 · `OdometryPipeline` 暴露了后端和交付层概念

- **严重程度**: Medium
- **类别**: API boundary
- **位置**:
  - `include/sapphire/odometry/pipeline.hpp:84`
- **来源**: not applicable

**证据**

除输入和里程计结果外，pipeline 还公开：

- `mapTrajectory()`；
- `poseGraphStats()`；
- `poseGraphSnapshot()`；
- `requestPoseGraphSnapshot()`；
- `requestPoseGraphMap()`；
- `requestOccupancyGrid()`。

ROS wrapper 直接通过这些接口执行“请求后轮询”的展示协议。

**触发条件**

增加非 ROS 消费者、替换地图导出方式，或者将 PGO 从同步 pipeline 中
独立出来时。

**影响**

- pipeline 同时承担算法编排和 artifact broker；
- 所有消费者都必须理解 backend worker 的轮询契约；
- 变化地图或可视化方式会影响核心 pipeline 公共 API；
- pipeline 测试容易与 backend artifact 测试混杂。

**建议**

将 artifact、可视化和地图导出 API 移到 runtime/session 层。
`OdometryPipeline` 继续只负责帧处理、状态估计和前端结果。

**置信度**: High

### B-RISK-004 · 数据类型依赖方向产生反向耦合

- **严重程度**: Medium
- **类别**: hidden dependency
- **位置**:
  - `include/sapphire/io/tum.hpp:3`
  - `include/sapphire/backend/pose_graph.hpp:3`
  - `include/sapphire/types.hpp:144`
- **来源**: not applicable

**证据**

- `io/tum.hpp` 为了使用 `MapTrajectory` 依赖整个 backend 头文件；
- `backend/pose_graph.hpp` 为了返回占用栅格依赖 mapping；
- `types.hpp` 同时保存点云、传感器数据、结果类型和整个嵌套配置树；
- 几乎所有模块头文件都直接包含 `types.hpp`。

**触发条件**

修改轨迹文件格式、地图导出、配置结构或单独使用配置/IO 模块时。

**影响**

- 输出格式、PGO 和 mapping 之间形成编译期和概念上的双向耦合；
- 配置变化会扩大重编译范围；
- 小模块难以在没有 backend 依赖的情况下被单独复用；
- 共享类型的所有权不明确。

**建议**

建立独立的最小 `model/` 或 `domain/` 层，放置：

- `MapTrajectory` 和 `MapTrajectorySample`；
- `OdometryResult`、`NavigationState` 等跨模块结果；
- 通用几何类型；
- 各子系统配置。

目标依赖方向：

```text
io       -> model
backend  -> model
mapping  -> model
odometry -> model
```

避免：

```text
io      -> backend
backend -> mapping
```

配置仍可有一个统一入口，但不需要让所有领域头文件都包含完整配置树。

**置信度**: High

## 建议的后续顺序

1. 先记录并冻结当前公共 API 和 artifact 语义；
2. 把 `MapTrajectory` 与 `MapTrajectorySample` 移到独立 model；
3. 从 `PoseGraphBackend` 抽出轨迹记录和 artifact 导出；
4. 引入 ROS 无关的 runtime/session，收拢 wrapper 中重复的运行时策略；
5. 最后再评估是否需要进一步拆分循环检测和图优化。

每一步应保持现有外部话题、参数、坐标系和 TUM 语义不变，并用现有测试和
一次短回放验证行为。优先做局部抽取，不需要立即重写 backend。

## 非目标

本轮不建议：

- 为了“架构整洁”给每个类增加接口；
- 重写 ESKF、GICP、ISAM2 或占用栅格算法；
- 合并 ROS 1 和 ROS 2 包；
- 在没有回归数据的情况下改变坐标系、时间戳或轨迹定义。
