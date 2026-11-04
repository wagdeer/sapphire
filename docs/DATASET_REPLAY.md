# Dataset replay

Replay one sequence and write `trajectory.tum` when the node shuts down
normally. Build the wrapper first, using the instructions in
[`sapphire_ros1`](https://github.com/wagdeer/sapphire_ros1) or
[`sapphire_ros2`](https://github.com/wagdeer/sapphire_ros2). Both wrappers
expect this repository as a sibling:

```text
ws/src/
├── sapphire/
└── sapphire_ros1/     or sapphire_ros2/
```

Set the dataset location in the shell you use for the commands below:

```bash
export HILTI_DIR=/path/to/hilti-2023
export S3E_DIR=/path/to/s3e
export OUT=$PWD/results/dataset_replay
```

`HILTI_DIR` should contain `rosbags/<sequence>.bag` for the 2023 handheld
recordings. Each S3E sequence is a ROS 2 bag directory such as
`$S3E_DIR/S3E_Dormitory_1`.

## Clock and the trajectory file

The estimator reads the LiDAR and IMU header stamps. RViz, TF markers, and
bag playback still need one clock. For a recording, pass `use_sim_time:=true`
to the launch file and `--clock` to the bag player.

The TUM file is written once, on a normal shutdown. After the bag ends, stop
the launch process with Ctrl-C. `kill -9` and stopping the container do not
flush the file.

That file is the pose-graph map trajectory, not the frontend odometry. Every
profile in the table at the bottom of this page sets `pgo.enabled = true`.
A profile that sets `output.trajectory_dir` while `pgo.enabled` is false is
rejected at load. `trajectory_output` only replaces the path. It does not
enable PGO, and with PGO disabled shutdown creates no trajectory file.

Give each robot its own output directory. The examples below put the robot
name in the path so Alpha, Bob, and Carol do not overwrite one another.

## ROS 1

Source the workspace, then play HILTI 2023 handheld with the dedicated launch
file. It already selects `/hesai/pandar`, `/alphasense/imu`, the handheld
extrinsic frames, and the LiDAR time offset:

```bash
mkdir -p "$OUT/ros1/hilti_2023/site1_handheld_1"
roslaunch sapphire_ros1 hilti_2023_handheld.launch \
  config_file:=/path/to/sapphire/cfg/sapphire_hilti_2023_handheld.toml \
  bag:="$HILTI_DIR/rosbags/site1_handheld_1.bag" \
  rate:=1.0 \
  rviz:=false \
  bag_required:=true \
  trajectory_output:="$OUT/ros1/hilti_2023/site1_handheld_1/trajectory.tum"
```

`rviz:=true` needs a working display. The default RViz file is
`sapphire_ros1/launch/display.rviz`. See that package's Docker notes for X11.

S3E is published as ROS 2 bags. Convert one robot's topics to a ROS 1 bag
before replay. The image is built from this repository:

```bash
docker build \
  -t sapphire-bag-convert:0.11.9 \
  tools/bag-convert

mkdir -p "$OUT/converted"
docker run --rm \
  -v "$S3E_DIR":/datasets/s3e:ro \
  -v "$OUT/converted":/out \
  sapphire-bag-convert:0.11.9 \
  --src /datasets/s3e/S3E_Dormitory_1 \
  --dst /out/S3E_Dormitory_1_Alpha.bag \
  --src-typestore ros2_humble \
  --dst-typestore ros1_noetic \
  --include-topic /Alpha/velodyne_points \
  --include-topic /Alpha/imu/data
```

Use two terminals.

Terminal A:

```bash
mkdir -p "$OUT/ros1/s3e/S3E_Dormitory_1/Alpha"
roslaunch sapphire_ros1 sapphire.launch \
  config_file:=/path/to/sapphire/cfg/sapphire_s3e_alpha.toml \
  lidar_topic:=/Alpha/velodyne_points \
  imu_topic:=/Alpha/imu/data \
  pose_frame:=lidar \
  use_sim_time:=true \
  rviz:=false \
  trajectory_output:="$OUT/ros1/s3e/S3E_Dormitory_1/Alpha/trajectory.tum"
```

Terminal B, after the node has logged that it is ready:

```bash
rosbag play --clock --delay 2.0 \
  --topics /Alpha/velodyne_points /Alpha/imu/data \
  "$OUT/converted/S3E_Dormitory_1_Alpha.bag"
```

For Bob or Carol, change the topic prefix, the config
(`sapphire_s3e_bob.toml` or `sapphire_s3e_carol.toml`), and the output
directory together. The three configs differ by robot, not by sequence.

### Foxglove

The base Noetic image does not install `foxglove_bridge`. Install
`ros-noetic-foxglove-bridge`, or build `sapphire_ros1/docker/Dockerfile.foxglove`
on top of `sapphire:noetic`. Then:

```bash
roslaunch sapphire_ros1 foxglove.launch
```

Open <https://app.foxglove.dev/>, choose **Open connection**, and use
`ws://localhost:8765`. The bridge port is 8765.

## ROS 2 Humble

S3E can be played directly. Terminal A:

```bash
mkdir -p "$OUT/ros2/s3e/S3E_Dormitory_1/Alpha"
ros2 launch sapphire_ros2 sapphire.launch.py \
  config_file:=/path/to/sapphire/cfg/sapphire_s3e_alpha.toml \
  lidar_topic:=/Alpha/velodyne_points \
  imu_topic:=/Alpha/imu/data \
  pose_frame:=lidar \
  use_sim_time:=true \
  rviz:=false \
  trajectory_output:="$OUT/ros2/s3e/S3E_Dormitory_1/Alpha/trajectory.tum"
```

Terminal B:

```bash
ros2 bag play "$S3E_DIR/S3E_Dormitory_1" \
  --clock \
  --delay 2.0 \
  --topics /Alpha/velodyne_points /Alpha/imu/data
```

HILTI 2023 handheld bags are ROS 1. Convert the two sensor topics first:

```bash
mkdir -p "$OUT/converted"
docker run --rm \
  -v "$HILTI_DIR":/datasets/hilti-2023:ro \
  -v "$OUT/converted":/out \
  sapphire-bag-convert:0.11.9 \
  --src /datasets/hilti-2023/rosbags/site1_handheld_1.bag \
  --dst /out/site1_handheld_1 \
  --src-typestore ros1_noetic \
  --dst-typestore ros2_humble \
  --include-topic /hesai/pandar \
  --include-topic /alphasense/imu
```

Terminal A:

```bash
mkdir -p "$OUT/ros2/hilti_2023/site1_handheld_1"
ros2 launch sapphire_ros2 sapphire.launch.py \
  config_file:=/path/to/sapphire/cfg/sapphire_hilti_2023_handheld.toml \
  lidar_topic:=/hesai/pandar \
  imu_topic:=/alphasense/imu \
  lidar_frame:=PandarXT-32 \
  imu_frame:=imu_sensor_frame \
  pose_frame:=imu \
  lidar_time_offset:=0.0015846550361175184 \
  use_sim_time:=true \
  rviz:=false \
  trajectory_output:="$OUT/ros2/hilti_2023/site1_handheld_1/trajectory.tum"
```

Terminal B:

```bash
ros2 bag play "$OUT/converted/site1_handheld_1" \
  --clock \
  --delay 2.0 \
  --topics /hesai/pandar /alphasense/imu
```

The time offset is the handheld calibration value used by
`sapphire_ros1/launch/hilti_2023_handheld.launch`. ROS 2 has no equivalent
launch file, so the same value is passed on the command line.

## Profiles

| Recording | Config | LiDAR topic | IMU topic |
|---|---|---|---|
| HILTI 2023 handheld | `cfg/sapphire_hilti_2023_handheld.toml` | `/hesai/pandar` | `/alphasense/imu` |
| HILTI 2022 handheld | `cfg/sapphire_hilti_2022_handheld.toml` | `/hesai/pandar` | `/alphasense/imu` |

`sapphire_ros1/launch/hilti_2022_handheld.launch` uses those two topics.
Unlike the 2023 handheld launch, its LiDAR time offset defaults to 0.
| S3E Alpha / Bob / Carol | `cfg/sapphire_s3e_alpha.toml` and the Bob and Carol files | `/<Robot>/velodyne_points` | `/<Robot>/imu/data` |
| Newer College 2020 | `cfg/sapphire_newer_college.toml` | `/os1_cloud_node/points` | `/os1_cloud_node/imu` |

Published ATE for these datasets is defined in
[benchmarks/benchmark_table.md](benchmarks/benchmark_table.md), including which
runs are left out of the headline mean.
