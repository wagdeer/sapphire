---
name: sapphire-run
description: >-
  Print the ROS 2 launch and rosbag play commands for Sapphire. Use when the
  user asks to run a config, toml, parameter file, or dataset, including S3E,
  GrAco, Mid-360, Alpha, Bob, Carol, or /data bags.
---

# Sapphire 运行命令

用户指定参数文件和数据集时，只给出两条可复制命令，不要自行启动。用户明确要求执行时再跑。用简体中文回复。

工作空间是 `/workspace`，ROS 2 Humble。算法读消息头时间戳，不要加 `--clock` 或 `use_sim_time`，除非用户要求。

## 先确定三个值

1. 配置文件：`/workspace/src/sapphire/cfg/<name>.toml`。用户给短名时按下表解析，然后读该文件开头注释里的 `lidar_topic` 和 `imu_topic`。以文件里的话题为准。
2. 数据集目录：裸名字解析为 `/data/<目录名>`。回放这个目录，不要写成里面的 `.db3`。
3. 话题：配置注释里有就用注释。没有就先 `source /opt/ros/humble/setup.bash`，再 `ros2 bag info <数据集目录>`，从中选一路 `sensor_msgs/msg/PointCloud2` 和一路 `sensor_msgs/msg/Imu`。有多组机器人且用户没指定时先问。

| 用户说法 | 配置文件 | 雷达话题 | IMU 话题 |
|---|---|---|---|
| alpha、s3e_alpha | `sapphire_s3e_alpha.toml` | `/Alpha/velodyne_points` | `/Alpha/imu/data` |
| bob、s3e_bob | `sapphire_s3e_bob.toml` | `/Bob/velodyne_points` | `/Bob/imu/data` |
| carol、s3e_carol | `sapphire_s3e_carol.toml` | `/Carol/velodyne_points` | `/Carol/imu/data` |
| graco | `sapphire_graco.toml` | `/velodyne/points` | `/gnss/imu` |
| mid360 | `sapphire_mid360.toml` | 配置里没有，以 bag 为准 | 配置里没有，以 bag 为准 |

S3E 的配置选机器人，数据集选序列，两者独立。`sapphire_s3e_alpha.toml` 可以跑任意 `S3E_*` 包，话题仍是 Alpha。

`/data` 下的 S3E 序列：`S3E_Campus_Road_1`、`S3E_Campus_Road_2`、`S3E_Campus_Road_3`、`S3E_Dormitory_1`、`S3E_Laboratory_1`、`S3E_Laboratory_2`、`S3E_Laboratory_3`、`S3E_Laboratory_4`、`S3E_Library_1`、`S3E_Library_2`、`S3E_Square_1`、`S3E_Square_3`、`S3E_Teaching_Building_1`、`S3E_Tunnel_1`。

口语对照：宿舍 → `S3E_Dormitory_1`；实验室、图书馆、校园路、广场、教学楼、隧道按编号对应上表。同名有多个且用户没给编号时先问。GrAco 的配置注释写的是 `/data/GrAco/ground-04`；目录不在时先说明，不要改话题。

`config_file` 必须是上面的源码绝对路径。不要用 launch 默认安装副本。

## 命令

把 `<配置>`、`<雷达话题>`、`<IMU话题>`、`<数据集目录>` 换成上面确定的值。两个终端各一段。

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 launch sapphire_ros2 sapphire.launch.py \
  config_file:=/workspace/src/sapphire/cfg/<配置>.toml \
  lidar_topic:=<雷达话题> \
  imu_topic:=<IMU话题>
```

```bash
source /opt/ros/humble/setup.bash
ros2 bag play <数据集目录> \
  --topics <雷达话题> <IMU话题>
```

只回放这两路。不要回放相机、GPS 或其他机器人。`odom_topic`、`world_frame`、`lidar_frame` 仅在用户点名时追加。

Launch 文件是 `src/sapphire_ros2/launch/sapphire.launch.py`。未改话题时它的默认值是 `lidar_topic:=/front_lidar`、`imu_topic:=/front_lidar/imu`；README 里的 Livox 话题不是当前默认值。

`src/sapphire_intern` 有 `COLCON_IGNORE`，不要编译它。

用户要求执行 `s3e-batch.md`，或把有效 S3E 序列全部跑完、准备轨迹时，按 [s3e-batch.md](s3e-batch.md) 实际启动并等待。这时不要停在只打印命令。单次询问一个配置和一条数据集时，仍只给命令。
