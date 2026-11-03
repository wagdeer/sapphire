# S3E 有效序列批量运行

这是维护者在本机容器里的批量运行备忘，不是对外说明。对外基准以
`docs/benchmarks/benchmark_table.md` 为准。Tunnel 1 仍待核验，不进入对外均值。
对外回放使用 `use_sim_time` 和 `--clock`；下面的命令不打开仿真时钟。

用户要求执行本文档时，按下面的顺序真正启动并等待结束。一次只跑一对「数据集 + 机器人」。不要只打印命令。

工作空间 `/workspace`，ROS 2 Humble。算法使用消息头时间戳。

三个配置只对应机器人，不对应某一条序列：

| 机器人 | 配置 | 雷达 | IMU | 轨迹文件名 |
|---|---|---|---|---|
| Alpha | `/workspace/src/sapphire/cfg/sapphire_s3e_alpha.toml` | `/Alpha/velodyne_points` | `/Alpha/imu/data` | `alpha.tum` |
| Bob | `/workspace/src/sapphire/cfg/sapphire_s3e_bob.toml` | `/Bob/velodyne_points` | `/Bob/imu/data` | `bob.tum` |
| Carol | `/workspace/src/sapphire/cfg/sapphire_s3e_carol.toml` | `/Carol/velodyne_points` | `/Carol/imu/data` | `carol.tum` |

估计轨迹写到：

```text
/workspace/eval/estimates/sapphire/<数据集>/<机器人>.tum
```

`trajectory_dir` 保持 `/workspace/eval/estimates`。每对开始前只改对应 toml 的 `trajectory_file`，不要改算法参数。

## 待核验：Tunnel 1 的算法结果与比较对象

当前怀疑 `S3E_Tunnel_1` 的算法结果与评估比较对象可能不对应。这个怀疑还没有
被证实，但在核实前，不要把 Tunnel 1 的 ATE 当成 benchmark 结果发布或用于
判断算法优劣。

已知现象：

- 三个估计轨迹的时间范围都落在 Tunnel 1 GT 的时间范围内。
- alpha / bob / carol 分别匹配 2780 / 2799 / 3067 个位姿，匹配覆盖率
  为 100%；问题不像单纯的时间关联失败。
- alpha、bob 的 ATE RMSE 分别为 40.779720 m、47.017840 m，明显异常。
- alpha、bob 的估计路径只有 GT 的 85.02% 和 82.11%，分别短约 78.2 m、
  89.9 m；carol 为 95.00%，短约 25.0 m。
- 三条轨迹都在时间上覆盖完整，但路径明显缩短，说明更可能是轨迹内容、
  agent 对应关系、回放输入、传感器坐标或算法在隧道退化场景中的状态估计
  问题，而不是简单的 coverage 不足。

需要按下面顺序复核：

1. 核对每份 `.tum` 是否确实由对应 agent 的 LiDAR/IMU topic 产生，不能在
   `trajectory_file` 中发生 Alpha/Bob/Carol 串写或旧文件残留。
2. 对齐检查 bag 中对应 topic 的首尾消息时间、估计轨迹首尾时间和 GT 首尾
   时间，确认没有播放其他 sequence 或另一台机器人的 bag。
3. 检查本次运行的 Sapphire launch 日志，确认实际 `lidar_topic`、
   `imu_topic`、`config_file` 和 `trajectory_file` 与 agent 一致。
4. 用 XZ/XY 轨迹图叠加估计和 GT，确认路径是整体缩短、局部漂移，还是
   几何形状对应错误。
5. 分时间段查看 ATE，定位误差从何开始增长，并与路径长度增长对照。
6. 在干净输出目录重新跑 Tunnel 1 的单个 agent，确认结果可重复，排除旧文件
   或中断落盘。

Tunnel 1 复核完成前，ATE 结果状态统一视为 `provisional_suspect`。

## 不要跑

这四个序列的真值不是连续轨迹。每个机器人只有起止 1 到 2 行，时间戳是 `0` 和 `1`，不能做 ATE。`S3E_Laboratory_1/alpha_gt.txt` 还有一行缺空格。

- `/data/S3E_Laboratory_1`
- `/data/S3E_Laboratory_2`
- `/data/S3E_Laboratory_3`
- `/data/S3E_Laboratory_4`

## 要跑

真值都在 `/data/<数据集>/gt/{alpha,bob,carol}_gt.txt`，时间戳严格递增。`S3E_Teaching_Building_1` 中间有大约 10 分钟空洞，但空洞两侧仍有连续位姿，保留。

按这个顺序。包时长是单机器人回放时间，三个机器人串行大约 6 小时。

| 顺序 | 数据集 | 包时长 | 真值行数 alpha / bob / carol |
|---|---|---|---|
| 1 | `S3E_Tunnel_1` | 7.1 分 | 29868 / 26337 / 28955 |
| 2 | `S3E_Library_1` | 7.6 分 | 400 / 379 / 377 |
| 3 | `S3E_Square_1` | 7.7 分 | 384 / 456 / 338 |
| 4 | `S3E_Square_3` | 7.8 分 | 40657 / 26149 / 32969 |
| 5 | `S3E_Library_2` | 8.2 分 | 34555 / 30843 / 30840 |
| 6 | `S3E_Dormitory_1` | 11.2 分 | 508 / 626 / 479 |
| 7 | `S3E_Teaching_Building_1` | 13.3 分 | 139 / 119 / 148 |
| 8 | `S3E_Campus_Road_1` | 14.6 分 | 470 / 858 / 583 |
| 9 | `S3E_Campus_Road_3` | 15.1 分 | 61352 / 54175 / 63771 |
| 10 | `S3E_Campus_Road_2` | 26.6 分 | 111551 / 94504 / 114279 |

每个数据集都跑 Alpha、Bob、Carol。`S3E_Square_1` 的三份轨迹如果已经在输出目录里且超过 100 行，跳过这一对。

## 每一对的步骤

1. 若已有 `sapphire_ros_node` 或 `ros2 bag play`，先对进程组发 `SIGINT`，等它退出。不要 `SIGKILL`，否则轨迹不会落盘。
2. 目标文件已存在且超过 100 行就跳过。
3. 把该机器人 toml 的 `trajectory_file` 改成 `sapphire/<数据集>/<机器人>.tum`。
4. 启动 launch，等到日志出现 `[sapphire_ros2] ready` 再播包。
5. 播包直到 `ros2 bag play` 自己退出。
6. 对 launch 进程组发 `SIGINT`，等到节点退出。
7. 确认目标 `.tum` 存在、超过 100 行，且每行 8 列：`timestamp tx ty tz qx qy qz qw`。失败就记下数据集和机器人，继续下一对。

两个终端都要先：

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
```

Launch：

```bash
ros2 launch sapphire_ros2 sapphire.launch.py \
  config_file:=/workspace/src/sapphire/cfg/<配置>.toml \
  lidar_topic:=<雷达话题> \
  imu_topic:=<IMU话题>
```

播包只放这两路，目录用 `/data/<数据集>`，不要写成里面的 `.db3`：

```bash
ros2 bag play /data/<数据集> \
  --topics <雷达话题> <IMU话题>
```

下面每对给出要写入的 `trajectory_file`。话题和配置按上表，不要换成别的机器人。

## 1. S3E_Tunnel_1

- Alpha：`sapphire/S3E_Tunnel_1/alpha.tum`
- Bob：`sapphire/S3E_Tunnel_1/bob.tum`
- Carol：`sapphire/S3E_Tunnel_1/carol.tum`

## 2. S3E_Library_1

- Alpha：`sapphire/S3E_Library_1/alpha.tum`
- Bob：`sapphire/S3E_Library_1/bob.tum`
- Carol：`sapphire/S3E_Library_1/carol.tum`

## 3. S3E_Square_1

- Alpha：`sapphire/S3E_Square_1/alpha.tum`
- Bob：`sapphire/S3E_Square_1/bob.tum`
- Carol：`sapphire/S3E_Square_1/carol.tum`

这三份如果已有超过 100 行，跳过。

## 4. S3E_Square_3

- Alpha：`sapphire/S3E_Square_3/alpha.tum`
- Bob：`sapphire/S3E_Square_3/bob.tum`
- Carol：`sapphire/S3E_Square_3/carol.tum`

## 5. S3E_Library_2

- Alpha：`sapphire/S3E_Library_2/alpha.tum`
- Bob：`sapphire/S3E_Library_2/bob.tum`
- Carol：`sapphire/S3E_Library_2/carol.tum`

## 6. S3E_Dormitory_1

- Alpha：`sapphire/S3E_Dormitory_1/alpha.tum`
- Bob：`sapphire/S3E_Dormitory_1/bob.tum`
- Carol：`sapphire/S3E_Dormitory_1/carol.tum`

## 7. S3E_Teaching_Building_1

- Alpha：`sapphire/S3E_Teaching_Building_1/alpha.tum`
- Bob：`sapphire/S3E_Teaching_Building_1/bob.tum`
- Carol：`sapphire/S3E_Teaching_Building_1/carol.tum`

## 8. S3E_Campus_Road_1

- Alpha：`sapphire/S3E_Campus_Road_1/alpha.tum`
- Bob：`sapphire/S3E_Campus_Road_1/bob.tum`
- Carol：`sapphire/S3E_Campus_Road_1/carol.tum`

## 9. S3E_Campus_Road_3

- Alpha：`sapphire/S3E_Campus_Road_3/alpha.tum`
- Bob：`sapphire/S3E_Campus_Road_3/bob.tum`
- Carol：`sapphire/S3E_Campus_Road_3/carol.tum`

## 10. S3E_Campus_Road_2

- Alpha：`sapphire/S3E_Campus_Road_2/alpha.tum`
- Bob：`sapphire/S3E_Campus_Road_2/bob.tum`
- Carol：`sapphire/S3E_Campus_Road_2/carol.tum`

## 结束

30 对里跳过的、失败的、写出的轨迹路径各列一份。不要在这里启动 `eval`。

## ATE 结果

以下结果使用 `translation-part ATE`、SE(3) Umeyama alignment、
不做尺度修正；路径长度来自原始估计和 GT 的逐点距离累加。

| 数据集 | Agent | ATE RMSE (m) | Median (m) | P95 (m) | GT 位姿 | 匹配位姿 | 覆盖率 | 时长 (s) | GT 路径 (m) | 估计路径 (m) | 路径差 (m) | 结论 |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| Square 1 | alpha | 1.165867 | 0.800144 | 2.025101 | 384 | 368 | 95.8% | 453.001 | 546.045 | 542.663 | -3.382 | 可用；GT 有长空洞 |
| Square 1 | bob | 0.748235 | 0.550394 | 0.983507 | 456 | 414 | 90.8% | 455.004 | 496.532 | 495.877 | -0.655 | 可用 |
| Square 1 | carol | 0.865892 | 0.623718 | 1.842757 | 338 | 305 | 90.2% | 456.999 | 529.183 | 531.707 | +2.525 | 可用；GT 有长空洞 |
| Tunnel 1 | alpha | 40.779720 | 33.382575 | 61.356885 | 29868 | 2780 | 100% | 420.630 | 521.957 | 443.751 | -78.206 | 待核验，不可发布 |
| Tunnel 1 | bob | 47.017840 | 45.290125 | 71.310256 | 26337 | 2799 | 100% | 424.510 | 502.390 | 412.504 | -89.887 | 待核验，不可发布 |
| Tunnel 1 | carol | 3.309225 | 1.871056 | 6.237737 | 28955 | 3067 | 100% | 421.190 | 501.103 | 476.068 | -25.035 | 待核验，不可发布 |

Square 1 的三 agent 平均 ATE RMSE 为 0.926664 m。Tunnel 1 的三 agent
平均值为 30.368928 m，但该平均值受待核验的 alpha/bob 异常结果支配，
当前不得作为对外指标。

完整结果：

- `eval/artifacts/s3e_square_1_sapphire/summary.md`
- `eval/artifacts/s3e_square1_tunnel1_sapphire/summary.md`
- `eval/artifacts/s3e_square1_tunnel1_sapphire/summary.csv`
- `eval/artifacts/s3e_square1_tunnel1_sapphire/summary.json`
