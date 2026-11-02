# Running HILTI and S3E in Docker

This guide treats the work as two independent batches:

1. ROS 1 / `sapphire:noetic`
2. ROS 2 Humble / `sapphire:humble`

The shortest path is:

```text
ROS 1 batch: HILTI direct + convert S3E to ROS 1
ROS 2 batch: S3E direct  + convert HILTI to ROS 2
```

Both containers expose the datasets through the same read-only paths:

```text
HILTI: /datasets/hilti-2023
S3E:   /datasets/s3e
```

Each recording should produce one trajectory and one log under:

```text
results/dataset_replay/<ros-version>/<dataset>/<sequence>/
├── trajectory.tum
└── run.log
```

The S3E examples use `Alpha`. Change `AGENT` to `Bob` or `Carol` if a different
robot should be shown.

## Prepare the shared workspace

Run these commands once from the workspace root:

```bash
cd /home/bindeer/git/sapphire_ws
mkdir -p \
  .cache/sapphire_dataset_replay/ros1_s3e \
  .cache/sapphire_dataset_replay/ros2_hilti

docker build \
  -t sapphire-bag-convert:0.11.9 \
  src/sapphire/tools/bag-convert
```

Only the `velodyne_points` and `imu/data` topics are converted for S3E. This
keeps the ROS 1 conversions small and avoids replaying all three robots and
cameras at once.

---

## Batch 1: ROS 1

### Start and build

The local `CATKIN_IGNORE` marker excludes the ROS 2 package from catkin
discovery:

```bash
cd /home/bindeer/git/sapphire_ws
touch src/sapphire_ros2/CATKIN_IGNORE
docker compose -f docker-compose.ros1.yml build sapphire_ros1
docker compose -f docker-compose.ros1.yml up -d --force-recreate

docker exec sapphire_ros1_dev bash -lc '
  source /opt/ros/noetic/setup.bash
  cd /workspace
  catkin build sapphire_ros1 --no-deps
'
```

### Foxglove web visualization

The ROS 1 image includes `foxglove_bridge` on an overlay image. Port `8765`
is published to Windows, so no Windows client is required.

Start the Sapphire launch in one terminal with RViz disabled:

```bash
docker exec -it sapphire_ros1_dev bash -lc '
  source /opt/ros/noetic/setup.bash
  source /workspace/devel_ros1/setup.bash
  roslaunch sapphire_ros1 hilti_2023_handheld.launch \
    config_file:=/workspace/src/sapphire/cfg/sapphire_hilti_2023_handheld.toml \
    rviz:=false
'
```

Start the bridge in a second terminal:

```bash
docker exec -it sapphire_ros1_dev bash -lc '
  source /opt/ros/noetic/setup.bash
  source /workspace/devel_ros1/setup.bash
  roslaunch sapphire_ros1 foxglove.launch
'
```

Open <https://app.foxglove.dev/>, choose **Open connection**, and use:

```text
ws://localhost:8765
```

The main 3D topics are `/sapphire/pgo/map`, `/sapphire/deskewed`,
`/sapphire/trajectory`, `/sapphire/odometry`, and `/sapphire/pgo/graph`.

### ROS 1 + HILTI

These handheld sequences are currently available:

```text
site1_handheld_1
site1_handheld_2
site1_handheld_3
site1_handheld_4
site1_handheld_5
site2_handheld_4
site2_handheld_5
site2_handheld_6
```

Select one sequence and run:

```bash
cd /home/bindeer/git/sapphire_ws
SEQ=site1_handheld_1
OUT=/workspace/results/dataset_replay/ros1/hilti_2023/$SEQ

docker exec -it sapphire_ros1_dev bash -lc "
  source /opt/ros/noetic/setup.bash
  source /workspace/devel_ros1/setup.bash
  mkdir -p '$OUT'
  roslaunch sapphire_ros1 hilti_2023_handheld.launch \
    config_file:=/workspace/src/sapphire/cfg/sapphire_hilti_2023_handheld.toml \
    bag:=/datasets/hilti-2023/rosbags/$SEQ.bag \
    rate:=1.0 \
    rviz:=true \
    rviz_config:=/workspace/src/sapphire_ros1/launch/display.rviz \
    bag_required:=true \
    trajectory_output:='$OUT/trajectory.tum' \
    2>&1 | tee '$OUT/run.log'
"
```

Start screen recording before this command. The RViz window closes automatically
when the bag ends.

### ROS 1 + S3E

S3E is distributed as ROS 2 bags, so convert one selected S3E sequence to ROS 1
before recording:

```bash
cd /home/bindeer/git/sapphire_ws
SEQ=S3E_Dormitory_1
AGENT=Alpha
BAG=/workspace/ros1_s3e/${SEQ}_${AGENT}.bag

docker run --rm \
  -v /mnt/e/dataset/s3e:/datasets/s3e:ro \
  -v /home/bindeer/git/sapphire_ws/.cache/sapphire_dataset_replay:/workspace \
  sapphire-bag-convert:0.11.9 \
  --src /datasets/s3e/$SEQ \
  --dst "$BAG" \
  --src-typestore ros2_humble \
  --dst-typestore ros1_noetic \
  --include-topic /$AGENT/velodyne_points \
  --include-topic /$AGENT/imu/data
```

Use two terminals.

Terminal A starts Sapphire and RViz:

```bash
cd /home/bindeer/git/sapphire_ws
SEQ=S3E_Dormitory_1
AGENT=Alpha
OUT=/workspace/results/dataset_replay/ros1/s3e/$SEQ

docker exec -it sapphire_ros1_dev bash -lc "
  source /opt/ros/noetic/setup.bash
  source /workspace/devel_ros1/setup.bash
  mkdir -p '$OUT'
  roslaunch sapphire_ros1 sapphire.launch \
    config_file:=/workspace/src/sapphire/cfg/sapphire_s3e_${AGENT,,}.toml \
    lidar_topic:=/$AGENT/velodyne_points \
    imu_topic:=/$AGENT/imu/data \
    pose_frame:=lidar \
    use_sim_time:=true \
    rviz:=true \
    rviz_config:=/workspace/src/sapphire_ros1/launch/display.rviz \
    trajectory_output:='$OUT/trajectory.tum' \
    2>&1 | tee '$OUT/run.log'
"
```

Terminal B replays the converted bag:

```bash
cd /home/bindeer/git/sapphire_ws
SEQ=S3E_Dormitory_1
AGENT=Alpha
BAG=/workspace/.cache/sapphire_dataset_replay/ros1_s3e/${SEQ}_${AGENT}.bag

docker exec -it sapphire_ros1_dev bash -lc "
  source /opt/ros/noetic/setup.bash
  rosbag play --clock --delay 2.0 \
    --topics /$AGENT/velodyne_points /$AGENT/imu/data \
    --bags "$BAG"
"
```

Start recording, then start Terminal B. Press `Ctrl-C` in Terminal A after the
bag ends so the final TUM file is written.

Available S3E sequences with complete `.db3` data:

```text
S3E_Campus_Road_3
S3E_Dormitory_1
S3E_Laboratory_1
S3E_Laboratory_2
S3E_Laboratory_3
S3E_Laboratory_4
S3E_Library_1
S3E_Library_2
S3E_Square_1
S3E_Square_3
S3E_Teaching_Building_1
S3E_Tunnel_1
```

`S3E_Campus_Road_1` and `S3E_Campus_Road_2` currently have only metadata, not a
complete `.db3`.

---

## Batch 2: ROS 2 Humble

### Start and build

ROS 2 must be visible to colcon, so remove the local catkin marker:

```bash
cd /home/bindeer/git/sapphire_ws
rm -f src/sapphire_ros2/CATKIN_IGNORE
docker compose up -d --force-recreate

docker exec sapphire_ros_dev bash -lc '
  source /opt/ros/humble/setup.bash
  cd /workspace
  colcon build --symlink-install --packages-select sapphire_ros2
'
```

### ROS 2 + S3E

Use two terminals.

Terminal A:

```bash
cd /home/bindeer/git/sapphire_ws
SEQ=S3E_Dormitory_1
AGENT=Alpha
OUT=/workspace/results/dataset_replay/ros2/s3e/$SEQ

docker exec -it sapphire_ros_dev bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/install/setup.bash
  mkdir -p '$OUT'
  ros2 launch sapphire_ros2 sapphire.launch.py \
    config_file:=/workspace/src/sapphire/cfg/sapphire_s3e_${AGENT,,}.toml \
    lidar_topic:=/$AGENT/velodyne_points \
    imu_topic:=/$AGENT/imu/data \
    pose_frame:=lidar \
    use_sim_time:=true \
    rviz:=true \
    trajectory_output:='$OUT/trajectory.tum' \
    2>&1 | tee '$OUT/run.log'
"
```

Terminal B:

```bash
cd /home/bindeer/git/sapphire_ws
SEQ=S3E_Dormitory_1
AGENT=Alpha

docker exec -it sapphire_ros_dev bash -lc "
  source /opt/ros/humble/setup.bash
  ros2 bag play /datasets/s3e/$SEQ \
    --clock \
    --delay 2.0 \
    --topics /$AGENT/velodyne_points /$AGENT/imu/data
"
```

Start recording before Terminal B. After playback, press `Ctrl-C` in Terminal A
to save the final TUM.

### ROS 2 + HILTI

Convert the selected ROS 1 HILTI bag to ROS 2 first:

```bash
cd /home/bindeer/git/sapphire_ws
SEQ=site1_handheld_1
BAG_OUT=/workspace/ros2_hilti/$SEQ

docker run --rm \
  -v /mnt/e/dataset/hilti-2023:/datasets/hilti-2023:ro \
  -v /home/bindeer/git/sapphire_ws/.cache/sapphire_dataset_replay:/workspace \
  sapphire-bag-convert:0.11.9 \
  --src /datasets/hilti-2023/rosbags/$SEQ.bag \
  --dst "$BAG_OUT" \
  --src-typestore ros1_noetic \
  --dst-typestore ros2_humble \
  --include-topic /hesai/pandar \
  --include-topic /alphasense/imu
```

If the destination already exists, remove only that exact sequence directory
before converting again.

Use two terminals.

Terminal A:

```bash
cd /home/bindeer/git/sapphire_ws
SEQ=site1_handheld_1
OUT=/workspace/results/dataset_replay/ros2/hilti_2023/$SEQ

docker exec -it sapphire_ros_dev bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/install/setup.bash
  mkdir -p '$OUT'
  ros2 launch sapphire_ros2 sapphire.launch.py \
    config_file:=/workspace/src/sapphire/cfg/sapphire_hilti_2023_handheld.toml \
    lidar_topic:=/hesai/pandar \
    imu_topic:=/alphasense/imu \
    lidar_frame:=PandarXT-32 \
    imu_frame:=imu_sensor_frame \
    pose_frame:=imu \
    lidar_time_offset:=0.0015846550361175184 \
    use_sim_time:=true \
    rviz:=true \
    trajectory_output:='$OUT/trajectory.tum' \
    2>&1 | tee '$OUT/run.log'
"
```

Terminal B:

```bash
cd /home/bindeer/git/sapphire_ws
SEQ=site1_handheld_1

docker exec -it sapphire_ros_dev bash -lc "
  source /opt/ros/humble/setup.bash
  ros2 bag play /workspace/.cache/sapphire_dataset_replay/ros2_hilti/$SEQ \
    --clock \
    --delay 2.0 \
    --topics /hesai/pandar /alphasense/imu
"
```

## Recording checklist

Before each clip:

1. Start the visualizer and wait until `/sapphire/deskewed` is populated.
2. Start the screen recorder.
3. Start bag playback.
4. Stop recording after the trajectory has visibly developed.
5. Press `Ctrl-C` in the visualizer terminal so `trajectory.tum` is flushed.
6. Keep the keyframe in `results/dataset_replay/...` for the later GIF step.

Do not run the ROS 1 and ROS 2 batches at the same time. Both use host
networking and similar topic names.
