# Sapphire SLAM

Sapphire LIO frontend and mapping backend, composed in one process.
Frozen Gaussian observations transfer memory ownership directly through a bounded
queue. Internal frontend/backend exchange does not use ROS/DDS or serialization.
Vision is disabled in shipped profiles; the active path is pure LiDAR.

```text
frontend/       LIO frontend
  include/      Public headers: common/, eskf/, initialize/, ricp/
  src/          Implementation, with the same functional grouping
backend/        Mapping backend
  include/      Public headers, without a repeated project-name directory
  src/          Matching implementation directories
    system/action/  Mapping orchestration, keyframe admission and relocalization
    maplayer/       Node content, featmap/, topograph/ and navimap/
    registration/   Detector, descriptor, matcher, retrieval and shared utilities
    solver/         Pose graph optimization and visual BA
    sensor/         camera/ and lidar/ models and utilities
    database/       Persistence, codecs and SQL schema
    common/         Backend data types
    io/             Calibration and runtime resource loading
    params/         Algorithm settings
    tools/          Backend utilities
app/            Pipeline, configuration and output interfaces; ros2/ owns ROS
config/         Algorithm profiles and backend tuning
data/           models/ and masks/ resources
thirdparty/     External dependencies
scripts/        Offline map and trajectory tools
```

The root project builds one `sapphire_core` library. Frontend and backend keep
their own functional directories and CMake source lists; they are not separate
packages. LIO measurements, state and frozen handoff types live in
`frontend/include/common/`. Camera geometry and image utilities live in
`backend/include/sensor/camera/`; each module owns its execution and utility code.
The app owns whole-system configuration and output callbacks. Backend consumers
read frozen frontend observations without accessing the mutable estimator.

Public include names are `frontend/runtime.hpp`, `backend/runtime.hpp`,
`app/parameters.h` and `app/pipeline.hpp`. Build-generated include roots provide
these names without extra project-name directories inside the source tree.

The backend stores database schema 12 and observation archive 10. Earlier map
formats require rebuilding; opening them fails explicitly without migration.

## Model assets

Visual models and the ORB vocabulary in `data/models/` are tracked with Git LFS. Install Git LFS
and run `git lfs pull` after cloning before building/installing the complete
backend. The default runtime still disables vision; RKNN execution requires
`SAPPHIRE_WITH_RKNN=ON` and the device SDK.

## ROS build

From the surrounding workspace (`/home/user/code/sapphire_git` on this machine):

```sh
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --base-paths src/app/ros2 \
  --packages-select sapphire_ros2 --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DSAPPHIRE_ENABLE_LTO=OFF
source install/setup.bash
ros2 launch sapphire_ros2 launch_sapphire.py
```

Use the explicit base path: the repository-root CMake project is the core
aggregate, not the ROS package. After upgrading from the old directory layout,
use a fresh build directory; CMake caches embed the old source paths.
`PYTHONNOUSERSITE=1` selects this host's compatible system ROS Python packages.

For a core-only build from this repository root:

```sh
cmake -S . -B ../build/core -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/opt/ros/humble -DBUILD_TESTING=OFF
cmake --build ../build/core -j3
```

Project regression sources stay local and are excluded from Git. If available,
configure with `-DBUILD_TESTING=ON` to build and run them with CTest.

See [ROS input/output usage](app/ros2/README.md).
