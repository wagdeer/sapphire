from pathlib import Path
import re, sys
root = Path(sys.argv[1]) / 'core'
assert not (root / 'frontend').exists()
assert not (root / 'backend').exists()
for name in ('memory.hpp', 'frontend_runtime.hpp', 'backend_runtime.hpp', 'pipeline.hpp'):
    assert not (root / 'include' / name).exists(), name
for name in ('database/memory.hpp', 'slam.hpp', 'pipeline/backend.hpp', 'pipeline/frontend.hpp'):
    assert (root / 'include' / name).is_file(), name
assert (root / 'src/database/memory.cpp').is_file()
assert not (root / 'src/memory.cpp').exists()
for name in ('maplayer/local_map.hpp', 'maplayer/local_map/voxel.hpp', 'maplayer/local_map/voxel_map.hpp', 'module/initialize.hpp', 'module/imu_estimator.hpp', 'solver/factor/imu_factor.hpp', 'module/ricp.hpp',
             'solver/factor/lidar_factor.hpp', 'module/eskf.hpp', 'module/synchronizer.hpp', 'common/imu_state.hpp',
             'common/se3_state.hpp', 'common/common_type.hpp', 'common/lidar_frame.hpp'):
    assert (root / 'include' / name).is_file(), name
for base in ('include', 'src'):
    for path in (root / base).rglob('*'):
        if path.suffix not in ('.hpp', '.h', '.cpp'): continue
        for inc in re.findall(r'#include\s*[<"]([^>"]+)', path.read_text()):
            assert not inc.startswith(('frontend/', 'backend/', 'app/', 'ros2_warpper/', 'rclcpp/', 'rcl/', 'sensor_msgs/', 'nav_msgs/', 'geometry_msgs/', 'tf2_ros/')), (path, inc)
for name in ('sensor/camera', 'registration', 'maplayer/featmap', 'maplayer/navimap'):
    assert not (root / 'include' / name).exists(), name
print('PASS: flat core layout, shared values and LiDAR-only modules')
