# Sapphire engineering / architecture invariants

本文件适用于本 Git 仓库。只保存长期工程约束，不保存当前 TODO、测试成绩或迁移完成状态。
开始工作时同时阅读 [当前状态](/home/user/code/sapphire_git/src/docs/CURRENT_STATE.md) 和 [能力迁移账本](/home/user/code/sapphire_git/src/docs/MIGRATION_LEDGER.md)。事实快照与阶段边界以这些文档为入口，并核对当次源码和 Git 状态；不要把快照当永久事实。

## Project identity

- **Sapphire 是 canonical project。** Pandora-SLAM 是从 Sapphire 中途分支出去继续演进后端能力的同源项目。
- Pandora 回迁的自研代码不是 third-party。吸收单位是 capability、algorithm、architecture improvement、data-management design，不是 directory、class 或 source file。
- 保持 Sapphire 自身目录、命名、ownership 与生命周期语义。不要为迁移建立重复职责的目录层、Frame、日志系统或 compatibility wrapper。
- `thirdparty/` 只承载真正外部依赖，例如 GTSAM、Faiss、CAPE、BBS；本地修改外部算法不改变其依赖来源，也不使同源自研代码成为第三方。

## Architecture ownership

优先强化现有对象之间的 invariant、ownership 和状态转换，再考虑新增类型。不要把每个一致性约束都包装成 manager、wrapper、context 或 aggregate state-holder；新类型必须具有独立的语义职责、lifetime 或行为。

优化位姿、correction、grid 与 committed revision 一致是必须满足的 invariant，不要求新增 Snapshot abstraction。复用既有 PoseGraphBackend、NavigationGrid 和 output ownership；不得为打包这些字段引入 MapSnapshot / BackendSnapshot / RevisionSnapshot 或 latestSnapshot()。

Sapphire core 源码分为 frontend / backend / common / tools 四块。顶层 pipeline 负责组合和现有生命周期调度；ROS 保持外部适配层。

| Responsibility | Ownership |
|---|---|
| Initialization (`initialize`), estimator-specific logic (`eskf`), reusable IMU/LiDAR preprocessing/synchronization (`frontend/common`) | `frontend` |
| Visual keyframe/image attributes, mapping, retrieval, registration, pose graph, grid, persistence | `backend` |
| Shared types and camera geometry | `common` |
| Generic SIMD, profiling, thread execution and cache utilities | `tools` |
| External dependencies | `thirdparty` |

`frontend/common` 不包含具体 initialize/eskf 实现；顶层 `common` 放全局共享类型，通用工具单列在 `tools`。系统配置入口 `parameters.h/.cpp` 与 `pipeline` 位于 src 同级；配置类型本身不依赖 pipeline 实现，可由各模块包含。点观测投影属于 `pointVar` 的方法，位姿与雅可比为调用方状态，不得增加每点持久字段。

相机内外参几何方法归 `common/camera/camera.hpp` 的相机模型；不同投影模型保持明确身份，不能只因合并文件就混用几何语义。

图像处理、描述子和特征存储工具归 `backend/visual`；通用计时工具归 `tools`。SIMD 通用封装按用户要求完整保留，不能仅因当前没有调用就删除。

模块依赖为 `frontend/backend → common/tools`、`common → tools`；tools 不依赖其他业务模块或系统配置，common 不依赖前后端，前后端不互相包含实现。使用源根限定的 include，例如 `frontend/eskf/eskf.hpp`；只有顶层 pipeline 组合两者。构建目标及当前拆分边界见 [模块结构](docs/MODULE_LAYOUT.md)。

新增 abstraction 前必须说明：

1. 当前 Sapphire 没有满足同一职责的等价 abstraction。
2. 新类型有独立、明确的 semantic responsibility。
3. owner、读写者、lifetime、跨线程传递及销毁边界明确。
4. 它不是仅为兼容历史 Pandora API 而存在。
5. 它不是仅因为一种数据存在，就创建新的 `XxxFrame`。

优先吸收算法和设计到现有职责边界。不要以连续 compatibility patch 维持已错误的 abstraction，也不要以未来可能用到为由提前添加接口。

## LiDAR / Vision contract

- 当前用户选择的视觉职责是 **纹理/视角关键帧选择与子图图像属性**。新图像属性路径不执行视觉检索/PnP，不持久化视觉特征或三角化点；光流/角点仅用于运行时选帧，子图由雷达独立切换。历史视觉回环代码与旧档案兼容能力不是当前产品验收目标；实际启用路径见 CURRENT_STATE。
- **LiDAR geometry 是 authoritative geometric verification 和最终 loop acceptance 的依据**。旧视觉召回/PnP路径只保留明确配置下的兼容能力，图像属性不能成为 metric loop constraint。
- BBS → GICP 是当前已验证的 registration 实现，验证范围见 CURRENT_STATE，不是永久固定的算法序列。未来只有在可靠 metric initial pose 经独立验证、且 quality gate 明确时，才可缩小或跳过 BBS；最终 loop constraint 仍必须通过独立于初值来源的 LiDAR geometric consistency / acceptance。
- 视觉不为 odometry 提供残差或修正。图像属性路径只保留运行时选帧用的角点/光流；如显式启用历史视觉几何工具，检测、光流、三角化与描述子必须保持同源，禁止 visual-feature 与 LiDAR/Gaussian 点关联赋深度。
- 图像属性必须保存与像素一致的冻结投影、相机身份、时间和原始观测位姿；相对子图位姿由原始锚点求得，不能用后续优化位姿重解释原始相机观测。图像按子图有界保留并随场景载荷退休，不能作为全局栅格或语义识别结果。
- Runtime 三角化、可用 3D 字段或离线 PnP utility 不等于已持久化 metric scene、在线 metric localization、成功 PnP 回环或 full geometric scene fusion；分别核对生产调用和验收证据。
- 接入两路图像、存在 depth/stereo/scene/3D 字段，均不构成这些能力已工作的证据。
- Metric visual evidence 必须有独立 architecture decision、输入/坐标/质量契约和 verification。持久化视觉三角化点必须保留版本化观测证据与原始相机位姿，并以原始子图锚点生成可复核的三维场景；历史二维观测参与 PnP 必须使用其冻结的校正像素投影参数，不得套用当前配置内参；不得用 optimized map pose 重新解释冻结点坐标。质量摘要不是标定协方差，不得以其替代 PnP 支持检查或独立 LiDAR acceptance。

## Coordinate contract

- 统一明确变换方向：`T_A_B` 将 B 系坐标变到 A 系。每个数据结构说明 frame、单位、锚点及允许的变换；不要凭变量名推断 inverse。
- 任何降维表示都必须说明丢弃了什么信息，以及此后仍可合法应用的 transform。
- 2D grid 丢弃 z 后，不能无数学依据地再按任意 SE(3) 投影。LocalGrid 必须具有显式、可验证的坐标契约；现有问题及后续设计入口见 CURRENT_STATE。
- 不得通过局部删除 roll/pitch 或强设 z=0 掩盖表示与变换的契约不匹配。

## Persistence / lifecycle contract

- 读出某些数据库记录不等于支持系统恢复或续建。区分 persistent state、runtime state、index state、pose graph state，并明确哪一项是事实来源、哪一项可重建。
- `new map` / `resume mapping` / `localization` 必须具有明确、互斥的入口语义；不可静默在失败时退回其他 mode。
- 完整 resume 实现并验证以前，不允许静默向非空历史数据库写入、从 ID 0 覆盖已有 Node。此为应满足的工程约束，**不表示当前代码已实现保护**；实现现状见 CURRENT_STATE。
- 恢复后的 ID、坐标锚定、边方向、索引身份、优化状态和地图输出必须一致。不能以从零重放原始传感器数据冒充持久地图 resume。
- 优化提交、发布及 flush/drain 的成功与失败必须有明确语义；不得把线程退出或没有异常日志等同于所有状态已持久化、已发布。

- Failure invariant: local preparation before mutation remains retryable. Once a non-rollbackable operation may have mutated state, retain Failed, fence work, perform safe cleanup, report failure, and reconstruct before reuse. The boundary must include exceptions from the first potentially mutating operation, not only exceptions after it returns.
- Global invariant, local implementation: final cleanup must not accidentally terminate solely because an arbitrary exception escaped an incomplete catch boundary. Explicit checked finish still reports failure. SQLite complete-close and descriptor-admission lifetime remain mandatory; deliberate fail-stop for an unrecoverable ownership invariant may remain. Fix demonstrated cleanup boundaries locally, without mechanically broadening algorithm/worker catches or introducing lifecycle/error managers.

## Verification contract

| Status | Meaning |
|---|---|
| implemented | 代码与调用路径存在；不暗含验证成功 |
| verified | 在明确输入、配置、环境和验收条件内，有可重复证据 |
| partially verified | 仅部分路径、条件或层级有验证证据 |
| unverified | 证据不足；不自动等于错误 |
| broken | 有可复现反例或明确代码/数学契约违例；说明触发条件 |

- Build success 只证明构建成功；synthetic test success 不等于 real-data validation。
- 性能提升结论必须来自同输入、同参数、同硬件的可重复 A/B benchmark，说明缓存、初始化和测量范围；不能把局部微基准推广为整体加速。
- 任何 loop transform / pose graph factor 修改，必须使用非对称 SE(3) 检查 query/target 顺序和 inverse，并覆盖实际因子/落库边方向。
- 任何 persistence/resume 修改，必须覆盖跨进程 `write → destroy → reopen → continue`，检查历史记录、graph、index、grid一致性；仅同对象load或重启后read不够。
- 把无输入时的最终状态发布、失败可见性和退出行为纳入相应生命周期/发布修改的验收，不依赖下一帧碰巧推动状态。

## Architecture-change procedure

跨模块实现前先完成并记录：

1. Inspect existing Sapphire implementation。
2. Identify existing equivalent abstractions。
3. Define ownership / lifetime / threading。
4. Define coordinate / data contract。
5. Define persistence / lifecycle impact。
6. Define verification criteria、反例与可重复测试入口。

这些内容必须足以让未参与聊天的新 agent 理解修改理由和边界，然后才能实现。尊重 CURRENT_STATE 中的当前 phase 范围；发现跨范围依赖时先说明必要性与最小影响，不把单项 correctness 修复扩大为整个后端重构。
