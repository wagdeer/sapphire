# Sapphire SLAM

Sapphire uses LiDAR + IMU odometry and a LiDAR mapping backend in one process.
Frozen Gaussian observations transfer memory ownership through a bounded queue.
Internal frontend/backend exchange does not use ROS/DDS or serialization.

```text
core/
  include/        Headers grouped by responsibility
    common/       Frame/point data, geometry helpers and state classes
    sensor/       LiDAR point types and handler
    maplayer/     Voxel map, submap accumulation, nodes and graph
    solver/       LiDAR registration, observation factors and graph optimization
    database/     Node residency, persistence, async writer, codecs and trajectory storage
    module/       Initialization, estimation, synchronization, mapping and relocalization
    pipeline/     Frontend/backend execution and coordination
    tools/        Generic utilities
  src/            Implementations with corresponding names
  thirdparty/     External dependencies
  config/         sensors.toml (installation), params.toml (algorithm)
ros2_warpper/     ROS input/output adaptation and executable entry
scripts/          Offline tools
```

The core project builds the ROS-independent shared library `sapphire_core.so` directly from `core/src` and exposes
`core/include`. There are no frontend/backend source directories, separate build
targets or generated include links. Backend parameter declarations live in
`core/include/parameters.hpp`, alongside the sensor and estimator configuration.
Parameter loading/validation and the SLAM algorithm API belong to core;
`ros2_warpper` links the library and supplies ROS transport and process entry.
Core does not include or compile any app source or depend on ROS libraries.

`database/memory.hpp` manages node identity, resident/cold transfers and persistence.
`Topograph` initializes gravity/odometry constraints and owns the current odometry
anchor; `Mapping` explicitly triggers graph reduction after residency transitions.
Opening, promoting or closing storage does not implicitly reduce the graph.

Installation settings live in one `core/config/sensors.toml`: `[imu]`, `[lidar0]`,
optional `[lidar1]`, `[preprocess]` and `[sync]`. Presence of lidar1 selects dual
input. Both LiDAR extrinsics map into the same IMU. `core/config/params.toml` contains
algorithm tuning; `ros2_warpper/config/ros.yaml` contains ROS output/QoS settings.
The native API reads `load_sensor_parameters(path)`, then
`load_parameters(algorithm_path, sensors.lidars.front())` for the pipeline's
reference LiDAR; the adapter owns input decoding and dual fusion.

`common` holds value structures and state classes. The original IMU state and
SE(3) pose retain their distinct APIs and update rules. Frozen
`LidarFrame` data and Gaussian move ownership retain the existing handoff contract;
no new state conversion layer is introduced.

Camera ingress, visual extraction/matching/retrieval, visual BA, navigation grids
and their assets/dependencies have been removed. OpenCV core remains for generic
matrix storage and graph utilities. LiDAR point cloud display is still available.

New maps use database schema 14. Older schemas are rejected before writing and
require rebuilding; this cleanup does not migrate older schemas. Unused auxiliary user-data storage
has been removed; Gaussian geometry, poses and graph constraints remain.
LiDAR resume requires an explicit historical target node and initial relative pose.

## ROS build

From the surrounding workspace (`/home/user/code/sapphire_git` on this machine):

```sh
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --base-paths src/ros2_warpper \
  --packages-select sapphire_ros2 --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DSAPPHIRE_ENABLE_LTO=OFF
source install/setup.bash
ros2 launch sapphire_ros2 launch_sapphire.py
```

Use the explicit base path: the ROS package lives in `ros2_warpper/`;
the independent core CMake project lives in `core/`. After upgrading from the old directory layout,
use a fresh build directory; CMake caches embed the old source paths.
`PYTHONNOUSERSITE=1` selects this host's compatible system ROS Python packages.

For a core-only build from this repository root:

```sh
cmake -S core -B ../build/core -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF
cmake --build ../build/core -j3
```

Project regression sources stay local and are excluded from Git. If available,
configure with `-DBUILD_TESTING=ON` to build and run them with CTest.

See [ROS input/output usage](../ros2_warpper/README.md).

Configuration cleanup (2026-10-26): the shared algorithm file preserves the
previous Mid360/default values for both single and dual input. Duplicate dual
and Hilti profiles are removed; the former Hilti RICP opt-in (enabled=true,
max_iterations=6) is documented beside the switch. The unreferenced standalone
backend template and offline compression template are removed. Database codec
behavior and pipeline lifetimes are unchanged. Sensor parsing accepts the
installation's numeric model IDs as well as existing model names. Core and ROS
installation/launch paths follow core/config and ros2_warpper.
Verified after this cleanup: unchanged default algorithm values, 17 core tests,
2 ROS tests, fresh installation paths, launch defaults and installed executable
SIGINT/SIGTERM shutdown with temporary databases. Numeric model IDs 0..3 and
invalid numeric/type cases are covered by the configuration regression.
