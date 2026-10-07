# Sapphire：Livox / Airy96 / Hesai 输入

`lidar_mode` 表示输入数量：`single`（默认）或 `dual`；`lidar_type` 表示类型：`livox`、`airy` 或 `hesai`（仅单雷达）。
两者均为启动参数。当前支持 `single + livox`、`single + airy`、`single + hesai`、`dual + airy`；其它双雷达格式启动时明确报错。
`obs_mode` 独立选择观测方式：`lio`（默认，雷达 + IMU）或 `livo`（雷达 + IMU + 视觉，预留）。
当前核心尚无视觉残差参与状态更新，因此选择 `livo` 会在启动时明确报错；`lio` 在 `visual_loop.enabled=true` 时可订阅图像用于后端检索，不将图像作为前端状态残差。
后端图像检索、激光闭环与地面解算见 [后端说明](../../docs/mapping.md)。
同步模块按功能命名为 `LidarSync`，相关参数放在 `lidar` 下。
独立高频前端的代码对照、观测接受策略与实施顺序见 [前端设计](../../docs/frontend.md)；该文档中的后续能力尚未实现。
原来的 Livox 配置、`launch_mid360.py` 和输出话题保持兼容。

所有随附算法配置默认关闭视觉（`visual_loop.enabled=false`），包括 Hilti 和 `mid360_visual` 模板。需要图像属性时显式改为 `true`；关闭时不订阅相机图像、不运行视觉处理。

## ESKF 高斯观测兜底

高斯观测兜底由正式 `frontend/ricp` 组件实现，ESKF 通过算法 TOML 配置选择调用，默认关闭。
在 `algorithm_config` 指向的文件里，将已有配置项改为：

```toml
[odometry.gicp_fallback]
enabled = true
```

保持 `odometry.frontend="eskf"`（默认前端）。开启后，ESKF 点面观测失败才调用 RICP 组件；旧的独立 `frontend="ricp"` 模式已退役。
旧 `[odometry.ricp]` 参数表需删除，改用上面的 `gicp_fallback` 配置；旧配置启动时会明确提示迁移。
启动时读取，修改后需要重启节点。正式 Mid360、双雷达及 Hilti 模板均已列出完整参数：

旧配置节 `odometry.gaussian_fallback` 已改名为 `odometry.gicp_fallback`；继续使用旧名称时启动会提示迁移，避免开关被静默忽略。

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `enabled` | `false` | 开启共享 voxelmap 的高斯观测兜底 |
| `source_resolution` | `0.25` | 当前帧统计体素尺寸，米；只在兜底时计算 |
| `max_distance` | `0.5` | 关联距离上限，米；开启时不超过 `odometry.voxel_size` |
| `variance_floor` | `0.0005` | 几何协方差正则项，平方米；不是已标定的位姿不确定性 |
| `max_iterations` | `20` | small_gicp 默认迭代上限，允许 1–20 |

点面观测正常时沿用原更新；没有匹配或法向支撑不足时，才从同一份 IMU 预测尝试高斯观测。
复用现有地图点簇统计，通过适配器交给 small_gicp 的 GICP 因子、LM 求解器和收敛判断，不建立第二张持久地图。
库只求雷达配准，IMU 预测作为初值；随后将配准结果融合进 ESKF 一次。匹配支撑、修正幅度和数值检查仍由前端负责，库收敛不代表观测必然接受。
正常点面门限、Gal3、去畸变及重置策略保持原样；
无效数据或数值异常不会触发兜底，两种观测均失败时不提交状态或入图。
日志 `[gicp-fallback]` 记录触发时间、接受原因、匹配数量和额外耗时。

Hilti exp09 粗体素试验（1m 根/0.5m 最小叶，4,466帧）从30次重置降为0，库版20次迭代上限下，21次兜底全部接受，平均额外0.56ms。
16个稀疏参考位置 RMSE 为0.421m（此前手写版0.690m）；原0.25m最小叶基线为0.215m，未触发兜底且轨迹与原基线完全一致。
这是单序列验证，不能据此认为粗体素精度优于原配置。
[实现和验证范围](../docs/CURRENT_STATE.md#optional-shared-map-gaussian-eskf-fallback--2026-10-08)。

## 局部滑窗栅格

`/sapphire/local_grid` 是当前 odom 域内的临时 `nav_msgs/OccupancyGrid`，默认开启。
窗口跟随雷达平移；重叠区域保留，离开窗口的单元丢弃，session 重置时清空。
单雷达用命中端点和有预算的射线更新占据/空闲，重复空闲观测可以清除旧障碍；
单元超过 `max_age` 未观测，在下一次扫描更新时变为 unknown。
消息使用观测时间，停传时不会伪造新鲜地图。它不进入数据库、历史地图 revision 或回环求解。

ROS YAML 可设置：

```yaml
local_grid:
  enabled: true
  extent: 20.0              # 正方形边长，m
  resolution: 0.1           # m/cell；每边最多 512 cells
  max_age: 5.0              # s
  sensor_height: 0.8        # 雷达离平地的高度，必须按安装标定
  obstacle_min_height: 0.15 # 相对估计地面的高度，m
  obstacle_max_height: 2.0
```

默认固定 200×200 单元，网格缓存约 1.26 MiB，不随路程增长；复用已有扫描输出线程和有界合并槽。
当前高度分类假设局部平地，尚不支持通用坡地/楼梯地形判别。
双雷达融合点缺少逐点射线起点，因此暂时仅做障碍端点更新和过期，关闭射线空闲清除。
几何数据库模式保留场景几何、图像特征、位姿及约束，不创建 `FlatGrid` / `NaviTrajectory` 表；
在线不再拟合导航样条，保留原始 odom 记录供以后按需构建。

## 编译与启动

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select sapphire_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

ros2 launch sapphire_ros2 launch_sapphire.py lidar_mode:=single lidar_type:=livox obs_mode:=lio

ros2 launch sapphire_ros2 launch_sapphire.py lidar_mode:=dual lidar_type:=airy obs_mode:=lio \
  algorithm_config:=/absolute/path/dual.toml \
  sensor_config:=/absolute/path/dual_ros.yaml \
  lidar_topic:=/front_lidar rear_lidar_topic:=/rear_lidar \
  imu_topic:=/front_lidar/imu rviz:=false
```

先复制 `sapphire/config/dual.toml`、`sapphire_ros2/config/dual_ros.yaml`，填写实际标定。
模板故意没有填入未知的安装外参；未填写时启动会给出参数错误。
Launch 的 topic 参数优先于 YAML；使用非默认话题时按上例显式传入。
Launch 的 `lidar_mode`、`lidar_type`、`obs_mode` 同样优先于 YAML；类型未指定时，single 默认 livox，dual 默认 airy。
单 Airy 可用 `lidar_mode:=single lidar_type:=airy`，复用 Airy 配置模板，仅使用前雷达与 IMU。
也可直接 `ros2 run sapphire_ros2 sapphire --ros-args --params-file /path/dual_ros.yaml -p algorithm_config:=/path/dual.toml`。
直接运行时，未指定 ROS `lidar_type` 则读取 TOML `sensor.lidar_type`；ROS 显式值优先。
TOML 使用 `"livox"` / `"airy"` / `"hesai"`，对应数值 `0` / `1` / `2`。

Hesai 输入必须提供 `timestamp: FLOAT64`（与 IMU 同时钟的绝对秒），XYZ 单位米，
intensity 可选；按字段偏移解析，不假设 PCL 内存布局，支持大小端、乱序和行填充。
扫描起止取全部原始点时间的 min/max，再抽样和过滤；显式终点传给核心同步器。
时间必须有限、非负、≤9e9 秒，扫描跨度必须正且≤`sensor.max_scan_duration`（容忍1µs舍入）。
缺失/错误类型的 timestamp、全相同时间、超长跨度及过滤后空扫描直接拒绝，
不使用 header 或猜测转速来补造逐点时间。数值检查不能证明 IMU 时钟或所有错误单位；
须另行核验原始数据。Livox / Airy 单雷达原有时间语义保持。

使用 launch 选择 `lidar_type:=hesai` 时必须显式给出 `sensor_config` 和 `algorithm_config`，
以及实际话题参数，不默认套用 Mid360 标定。当前只有人工输入测试；Hilti 原始包、
真值和最终标定配置尚待验收。不要把此格式支持当作已完成 Hilti 全包回放。

必需的两个变换（米、行优先 3×3 旋转矩阵）：

| 配置 | 定义 | robot_slam 对应标定项 |
| --- | --- | --- |
| TOML `sensor.extrinsic_rota/tran` | `p_imu = R_imu_front * p_front + t_imu_front` | `lidar_front_to_imu_front` |
| YAML `lidar.extrinsic_rota/tran` | `p_front = R_front_rear * p_rear + t_front_rear` | `lidar_back_to_lidar_front` |

两台雷达均输入原始传感器坐标系的 `sensor_msgs/PointCloud2`，包含 XYZ 和
`timestamp: FLOAT64`（**绝对时间，单位秒**）；intensity 可选。不要输入已经应用安装外参的点云。
支持无序点、大小端、带行填充的 organized cloud。输入帧 ID 必须非空且每路保持不变。
只使用 `topics.imu` 指定的一路 IMU，外参必须对应该 IMU；不会混合两台雷达的 IMU。

两路逐点时间和 IMU 必须在同一时钟域。`front_time_offset` / `rear_time_offset` 定义为
`对齐后时间 = timestamp + offset`（秒）。这是已知固定时间偏移的补偿，不能自动解决时钟漂移。
缺少逐点时间、错误单位或时间倒退会拒收；rosbag 时间回跳后应重启节点。

## 固定时间窗与频率

默认以 **10 Hz、连续的 `[begin, end)` 时间窗**组织两路数据。两个输入对等，输出频率不再由前雷达的驱动分帧决定。
起点取启动时两路最早可用区间的公共起点，此后用整数纳秒推进边界，避免反复累加浮点周期。
窗口末端统一作为 LIO 去畸变参考时刻；原始点的逐点时间不被均匀化或重采样。

在 `dual_ros.yaml` 中设置，例如：

```yaml
lidar:
  lidar_merge_hz: 20.0
  imu_rate_hz: 200.0
  min_points: 1
```

`lidar_merge_hz` 只用于 dual 模式的合并时间窗；single 模式沿用输入帧。
它是启动参数，设为 `0.0` 可以恢复前雷达帧窗口。正值上限为 100 Hz，
窗口时长必须不超过 `sensor.max_scan_duration`，且 `lidar_merge_hz <= imu_rate_hz / 5`。
`imu_rate_hz` 只声明实际 IMU 输入速率，不改变硬件，不生成 IMU 数据。
**核心仍要求每次更新至少 5 个新 IMU 样本**；实际数据不足会跳过更新，并在首次及每累计 100 次时告警。
例如 200 Hz IMU、20 Hz LIO 约为每窗 10 个样本；不要贴着 40 Hz 理论上限配置，要给抖动/掉包留余量。

1. 从所有原始逐点时间确定每个输入区间，之后再抽点；不依赖消息头和数组末点。
2. 等待两路采样时间都达到窗口末端，检查两路的区间覆盖。一个窗可横跨多个驱动消息，一条消息也可拆入多个窗。
3. 两路各有消费游标，边界点进入下一窗；每个原始观测最多使用一次，保留同一时间的不同激光点。
4. 后雷达点应用安装外参，直接写入融合数组。按时间有序的输入不创建索引；无序输入仅建立排序索引，不复制 XYZ。
5. 融合数组 move 进核心，保留显式窗末端，沿用原有 IMU 去畸变、状态更新和建图。不会用重复点填满窗口。
6. 超时、缺失覆盖或任一路有效点不足时丢弃该窗。长时间断流后的空窗按整数步数跳过，避免逐窗空转。

这里的“点不丢不重”仅针对两路都有覆盖的、通过有效性/抽点/盲区检查的完整成功窗口。
共同起点前、主动抽掉、过期、溢出、缺失覆盖以及结尾不足一个完整窗的数据不会补造。
固定时间窗要求每路消息区间单调且不重叠（公共端点允许），帧内点可乱序。
这使最新采样时间可以作为安全的时间水位；旧帧、重复帧、跨消息时间倒退会拒收。

缓存持有 ROS 点云 shared pointer，解码直接写入最终数组，无 PCL 中间转换。
这不代表整个 DDS/LIO 链路零拷贝；索引、融合数组、核心排序和下采样仍有成本。
高频拆窗使用游标递增消费，不为每个子窗反复全量扫描同一输入帧。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `lidar.lidar_merge_hz` | 10 Hz | 目标融合/LIO 时间窗频率；0 恢复前雷达帧模式 |
| `lidar.imu_rate_hz` | 200 Hz | 预期实际 IMU 输入频率，用于启动时的频率约束 |
| `lidar.min_points` | 1 | 每窗每台雷达至少保留的有效点数；可按场景提高，不是可观测性保证 |
| `lidar.max_wait` | 0.3 s | 从涉及该窗的缓存数据到达开始计的最长等待，steady clock、20 ms 定时检查 |
| `lidar.queue_size` | 8 / 路 | 驱动消息缓存上限，满时淘汰最旧消息；小包输入需相应增大 |
| `lidar.max_time_gap` | 0.02 s | 每路帧区间之间/窗起点允许的时间缺口，不是两帧头配对阈值 |
| TOML `sensor.max_scan_duration` | 双 Airy 0.25 s；旧配置 0.11 s | 输入消息与融合窗的合法时长上限 |
| TOML `sensor.point_filter_num` | 1 | 每路按消息内原始点序号抽点；不是时间重采样 |

时间覆盖仅检查帧区间，不能推断帧内 UDP 丢包或空间几何覆盖。
告警中的 invalid/out_of_order/overflow 统计消息，timeout/coverage/empty 在固定模式统计放弃的时间窗
（启动前尚无窗口时，timeout 统计过期消息）。不静默退化成单雷达更新。

### 软件输出频率、驱动交付频率、硬件扫描频率

Airy96 官方规格为 600 RPM / 10 Hz，支持 GPS/PTP/gPTP 时间同步与锁相。
本功能不修改雷达转速，也不把 10 Hz 的一圈扫描变成两个独立的完整扫描。
[Airy 官方手册](https://robosense-robotics.github.io/product-manual/en/Airy/)

比如 20 Hz 软件窗口是把原始观测组织成 50 ms 子帧，每窗空间覆盖通常小于一整圈。
如果驱动仍攒齐 100 ms 后才发消息，软件可能一次产生多个子帧，**不能消除上游积攒延迟**。
获得低延迟应让驱动更早交付块级数据；AIRY 官方 decoder 有 block 分帧入口，但具体 SDK 版本、
回波模式、块组边界及参数需实机核验。本仓库没有修改外部驱动。
[官方 AIRY decoder](https://github.com/RoboSense-LiDAR/rs_driver/blob/main/src/rs_driver/driver/decoder/decoder_RSAIRY.hpp)

先使用统一硬件时钟，再考虑锁相/驱动分块。锁相影响扫描角度的时序，不能代替时间同步。
软件窗口频率、实际到达节奏、有效 LIO 更新频率应分别衡量。
提高频率还会减少每窗点数与视野覆盖，并增加滤波/局部优化次数；非 overlap 安装尤其要检查退化。
现有 `local_submap.win_size` 以帧计数，提高更新频率会缩短它覆盖的实际时间，不能假定只改频率就保持相同优化行为。

## 后续前端路线（尚未实现）

当前 `LidarSync::Scan` 与核心 `push_lidar(begin, points, end)` 分离了时间窗组织和估计器。
进一步可将**时序输入、观测构造、状态更新、地图更新节奏**分开，避免高频前端强制后端也同频优化。

- **Point-LIO 方向**：保留准确逐点时间，逐步评估小批量/逐点观测更新与独立的地图写入节奏。
  Point-LIO 的逐点校正和 IMU 作为观测的随机过程模型涉及估计器与噪声模型，不能只靠缩短窗口获得。
  当前仍使用 Sapphire 的帧级估计器和至少 5 个新 IMU 样本门槛。
  [原作者项目](https://github.com/hku-mars/Point-LIO)
- **恢复 GICP 结果作为观测**：后续可增加配准观测适配器，输出测量时间区间、参考坐标系、相对位姿、
  协方差/信息矩阵和退化指标，再送入状态估计器。相对位姿约束与绝对位姿约束需明确区分。
  必须处理 GICP 初值、去畸变轨迹和当前滤波先验的相关性；不能把 Hessian 的逆直接当作完整可靠的测量协方差。
  同一批点若同时用于原有点面更新与 GICP 更新，需要选择其一或显式处理相关性，防止重复计权。
  这条路径与 Point-LIO 的 IMU 观测模型独立，本轮未恢复 GICP 状态观测。

## overlap 调研与取舍

2026-09-25 检查了本地 `robot_slam` 的 `a715a7f`（`v0.6.0-rc.5-nx`），以及实时拉取的
远程 `main`（`f82ca6eeb4e355a65fdb2e64a78ab791617d50b9`）、`dev.highfreq`、
`dev.zhongzhou1`、`release/v0.5.5`。在所检查的输入/同步/配置代码中，没有发现针对双雷达
**空间视野重叠**的独立配准、在线外参估计或 overlap 权重算法。

主分支依据：

- [sensor_sync.cpp](https://gitlab.jszr.com/jszr/robot_slam/-/blob/f82ca6eeb4e355a65fdb2e64a78ab791617d50b9/src/core/module/sensor_sync.cpp)：按两帧头差小于 20 ms 配对，否则丢较早帧。
- [lidar_process.cpp](https://gitlab.jszr.com/jszr/robot_slam/-/blob/f82ca6eeb4e355a65fdb2e64a78ab791617d50b9/src/core/process/lidar_process.cpp)：后雷达转前雷达坐标系，只保留落在前雷达首末点时间之间的后雷达点。新版本把最大时间的点移到数组末尾，但没有改变配对方式。
- [system_params.h](https://gitlab.jszr.com/jszr/robot_slam/-/blob/f82ca6eeb4e355a65fdb2e64a78ab791617d50b9/include/params/system_params.h)：双雷达配置复用 `use_pair_lidar`，未见空间 overlap 分支。

该实现中 PCL 整帧拼接、多个中间数组、首末点有序假设、后雷达窗外点丢弃，以及并行写
`vector<bool>` 的位存储风险，都没有照搬到这里。也没有沿用针对特定机型的固定安装距离假设。

因此 Sapphire 只增加一种 `dual` 模式。有/无空间 overlap 通过各自标定外参适配，
重叠区点在统一去畸变后进入原有体素下采样与观测模型；没有额外在线标定或特殊去重。
不同安装方式必须使用各自的标定文件，不能仅翻转一个 overlap 开关。

## 验证

```bash
colcon build --packages-select sapphire_ros2 --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/sapphire_ros2 -R 'sapphire_lidar_sync_test|sapphire_core_test' --output-on-failure
# ROS 本机通信测试：独立 DDS 域 215，合成标定，不需要实机。
python3 src/sapphire_ros2/tests/node_smoke_test.py build/sapphire_ros2/sapphire
```

新增合成测试覆盖 5/10/20/30 Hz 固定时间窗、整数边界、逐点数量守恒、不同输入频率与到达顺序、5 个 IMU 样本门槛保持不变，以及跨帧错相位、后缀复用、公共边界、缺帧超时、队列上限、时间偏移、
乱序点、帧乱序、大小端、organized padding、畸形字段、真实扫描结束时间，以及带平移/旋转/
非单位外参的双雷达去畸变。实机精度和时延仍需提供标定与 rosbag 后验证。
本次已在 ROS 2 Humble 完成 Release 编译、上述两项 CTest，以及单 Livox、单 Airy、双 Airy 10 Hz / 20 Hz 的 ROS 节点通信测试。

## B3 版本归属与受检退出

后端 C 表示已知提交版本，Y 表示运行时已就绪版本；ROS 输出线程独立保存 correction
组 Pc 和 navigation 组 Pg。不可用与版本 0 不等价，关闭导航时 Pg 为 N/A。
这些发布状态不会写入地图数据库。标准 TF、Odometry 和 OccupancyGrid 话题保持兼容。

| 默认话题 | 新消息 | 身份与用途 |
| --- | --- | --- |
| `/sapphire/map_correction` | `RevisionedMapCorrection` | U/E/g/R/S + map→odom；reliable/volatile，depth 1 |
| `/map_odom/revisioned` | `RevisionedMapPose` | 同一捕获的已提交锚点及原始时间；reliable/volatile，depth 1 |
| `/map/revisioned` | `RevisionedNavigationGrid` | U/E/R + grid；reliable/transient-local，depth 1 |
| `/sapphire/publication_status` | `MapPublicationStatus` | 有界 best-effort 状态；reliable/transient-local，depth 1 |

U 是数据库 UUID，E 是每次进程启动新建的临时 UUID，g 是运行域 generation，R 是来源
revision，S 是完成的来源 sequence。E 没有时间顺序含义。修改 `topics.map_pose` 或
`topics.navigation` 时，对应 revisioned 话题随其添加 `/revisioned` 后缀；correction
话题可用 `revisioned_correction_topic` 配置。不得从标准话题的 frame/timestamp 推断 revision。

Pc 仅在 revisioned correction、TF、revisioned pose、标准 Odometry 全部本地调用成功后
推进；Pg 仅在两种 grid 调用都成功后推进。成功还要求所拥有的 ROS context 在调用前后
有效，仅表示本地 adapter submission 完成，不保证送达或订阅者接收。部分成功可能产生
重复消息。每组每目标最多初次尝试和一次自动在线重试，间隔不短于后端 update period；
显式 `retry_publication()` 合并为一次尝试，最终 drain 对缺失组再尝试一次。

输出保留四个固定可选槽（odom、scan、完整 trajectory、增量 local-map）。新状态可覆盖旧
状态；增量 local-map 被覆盖会丢失独立增量，计入状态中的 drop counters。required demand
采用合并请求，不保留逐版本队列。可选生产端在复制前限量，输出分配失败不改变 C/Y。

导航 dense export 在分配前限制宽/高各 4000、cells 16,000,000；每条 required grid 编码
上限 16 MiB，元数据预留 64 KiB，frame 名最长 255 bytes。required oversize 会使对应组
失败，不截断地图。权威稀疏 occupancy、dense grid、ROS 消息与序列化缓存分别占内存，
16 MiB 不是进程总内存上限。

`finish:=true`、SIGINT、SIGTERM 请求同一受检退出：停止输入、join 上游、drain 已接受后端
工作、冻结 F/S、完成最终 Pc/Pg、join 输出线程、checked close，最后才销毁 ROS context。
不需要再输入一帧。主进程在映射、required 发布或 close 不完整时返回非零；状态话题本身
不参与成功条件。外部传输调用若不返回，优雅退出仍可能等待；测试 watchdog 不构成生产取消。

ROS 常规启动仍保持既有 new-map 入口；explicit-resume 的 B1 target/seed 授权通过现有 core
接口提供，B3 没有增加隐式 resume 或自动选择历史 target。两条路径的有界验证与保留限制见
[B3 实施审计](/home/user/code/sapphire_git/audit/2026-09-28-b3-implementation/README.md)。
