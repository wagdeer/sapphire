# Dataset replay

Build the wrapper first. Instructions are in [sapphire_ros1](https://github.com/wagdeer/sapphire_ros1) and [sapphire_ros2](https://github.com/wagdeer/sapphire_ros2). The core must sit next to the wrapper:

```text
ws/src/
├── sapphire/
└── sapphire_ros1/     or sapphire_ros2/
```

Each file in `cfg/` matches one sensor rig. Use the same file for every sequence from that rig. `config_file` is an absolute path.

Pass `trajectory_output`. The TUM file is written when the node exits normally, after you stop the launch with Ctrl-C. `kill -9` and stopping the container do not write it. Some profiles set `output.trajectory_dir` to a path under `/workspace/results`; `trajectory_output` replaces that path.

For a recording, pass `use_sim_time:=true` to the launch file and `--clock` to the bag player.

## Which profile

| Rig | Config | LiDAR | IMU |
|---|---|---|---|
| Livox Mid-360 | `cfg/mid360.toml` | your topic | your topic |
| HILTI 2022 handheld | `cfg/hilti_2022_handheld.toml` | `/hesai/pandar` | `/alphasense/imu` |
| HILTI 2023 handheld | `cfg/hilti_2023_handheld.toml` | `/hesai/pandar` | `/alphasense/imu` |
| HILTI 2023 robot | `cfg/hilti_2023_robot.toml` | `/rslidar_points` | `/imu/data` |
| S3E Alpha, Bob, Carol | `cfg/s3e_alpha.toml`, `s3e_bob.toml`, `s3e_carol.toml` | `/<Robot>/velodyne_points` | `/<Robot>/imu/data` |
| Newer College 2020 | `cfg/newer_college.toml` | `/os1_cloud_node/points` | `/os1_cloud_node/imu` |
| GrAco | `cfg/graco.toml` | `/velodyne/points` | `/gnss/imu` |

S3E has one config per robot. The sequence does not change the file.

## ROS 1

HILTI bags can use a launch file that already sets the topics and frames. `hilti_2023_handheld.launch` also applies the handheld LiDAR time offset. `hilti_2022_handheld.launch` leaves that offset at 0. `hilti_2023_robot.launch` is the robot rig.

```bash
roslaunch sapphire_ros1 hilti_2023_handheld.launch \
  config_file:=/path/to/sapphire/cfg/hilti_2023_handheld.toml \
  bag:=/path/to/site1_handheld_1.bag \
  trajectory_output:=/path/to/trajectory.tum
```

`bag_required:=true` makes the launch exit when the bag ends, which writes the TUM file. `rviz:=true` needs a display. The default RViz config is `sapphire_ros1/launch/display.rviz`.

S3E is published as ROS 2 bags. Convert one robot before replay:

```bash
python3 -m pip install --user "rosbags==0.11.9"

rosbags-convert \
  --src /path/to/S3E_Dormitory_1 \
  --dst /path/to/S3E_Dormitory_1_Alpha.bag \
  --src-typestore ros2_humble \
  --dst-typestore ros1_noetic \
  --include-topic /Alpha/velodyne_points \
  --include-topic /Alpha/imu/data
```

Terminal A:

```bash
roslaunch sapphire_ros1 sapphire.launch \
  config_file:=/path/to/sapphire/cfg/s3e_alpha.toml \
  lidar_topic:=/Alpha/velodyne_points \
  imu_topic:=/Alpha/imu/data \
  use_sim_time:=true \
  trajectory_output:=/path/to/trajectory.tum
```

Terminal B, after the node is up:

```bash
rosbag play --clock --delay 2.0 \
  --topics /Alpha/velodyne_points /Alpha/imu/data \
  /path/to/S3E_Dormitory_1_Alpha.bag
```

For Bob or Carol, change the topic prefix, the config, and the output path together.

Newer College uses `cfg/newer_college.toml` with `/os1_cloud_node/points` and `/os1_cloud_node/imu`, through `sapphire.launch` in the same way.

## ROS 2 Humble

S3E plays directly.

Terminal A:

```bash
ros2 launch sapphire_ros2 sapphire.launch.py \
  config_file:=/path/to/sapphire/cfg/s3e_alpha.toml \
  lidar_topic:=/Alpha/velodyne_points \
  imu_topic:=/Alpha/imu/data \
  use_sim_time:=true \
  trajectory_output:=/path/to/trajectory.tum
```

Terminal B:

```bash
ros2 bag play /path/to/S3E_Dormitory_1 \
  --clock \
  --delay 2.0 \
  --topics /Alpha/velodyne_points /Alpha/imu/data
```

HILTI bags are ROS 1. Convert the two sensor topics, then pass the same topics and frames the ROS 1 launch file uses. ROS 2 has no HILTI launch file.

```bash
rosbags-convert \
  --src /path/to/site1_handheld_1.bag \
  --dst /path/to/site1_handheld_1 \
  --src-typestore ros1_noetic \
  --dst-typestore ros2_humble \
  --include-topic /hesai/pandar \
  --include-topic /alphasense/imu
```

HILTI 2023 handheld:

```bash
ros2 launch sapphire_ros2 sapphire.launch.py \
  config_file:=/path/to/sapphire/cfg/hilti_2023_handheld.toml \
  lidar_topic:=/hesai/pandar \
  imu_topic:=/alphasense/imu \
  lidar_frame:=PandarXT-32 \
  imu_frame:=imu_sensor_frame \
  pose_frame:=imu \
  lidar_time_offset:=0.0015846550361175184 \
  use_sim_time:=true \
  trajectory_output:=/path/to/trajectory.tum
```

```bash
ros2 bag play /path/to/site1_handheld_1 \
  --clock \
  --delay 2.0 \
  --topics /hesai/pandar /alphasense/imu
```

HILTI 2022 handheld uses the same topics and frames, `cfg/hilti_2022_handheld.toml`, and `lidar_time_offset:=0.0`.

HILTI 2023 robot:

```bash
ros2 launch sapphire_ros2 sapphire.launch.py \
  config_file:=/path/to/sapphire/cfg/hilti_2023_robot.toml \
  lidar_topic:=/rslidar_points \
  imu_topic:=/imu/data \
  lidar_frame:=rslidar \
  imu_frame:=xsens_imu_link \
  pose_frame:=imu \
  use_sim_time:=true \
  trajectory_output:=/path/to/trajectory.tum
```

Newer College:

```bash
ros2 launch sapphire_ros2 sapphire.launch.py \
  config_file:=/path/to/sapphire/cfg/newer_college.toml \
  lidar_topic:=/os1_cloud_node/points \
  imu_topic:=/os1_cloud_node/imu \
  use_sim_time:=true \
  trajectory_output:=/path/to/trajectory.tum
```
