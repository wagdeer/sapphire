from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def validate_hesai_config(context):
    if LaunchConfiguration("lidar_type").perform(context) == "hesai":
        for name in ("sensor_config", "algorithm_config"):
            if not LaunchConfiguration(name).perform(context).strip():
                raise RuntimeError("hesai requires explicit sensor_config and algorithm_config with verified calibration")
    return []


def generate_launch_description():
    package_share = FindPackageShare("sapphire_ros2")
    lidar_mode = LaunchConfiguration("lidar_mode")
    lidar_type = LaunchConfiguration("lidar_type")
    obs_mode = LaunchConfiguration("obs_mode")
    sensor_config = LaunchConfiguration("sensor_config")
    algorithm_config = LaunchConfiguration("algorithm_config")
    lidar_topic = LaunchConfiguration("lidar_topic")
    rear_lidar_topic = LaunchConfiguration("rear_lidar_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    image_topic = LaunchConfiguration("image_topic")
    compressed_image = LaunchConfiguration("compressed_image")
    rviz = LaunchConfiguration("rviz")

    arguments = [
        DeclareLaunchArgument("lidar_mode", default_value="single", choices=["single", "dual"]),
        DeclareLaunchArgument("obs_mode", default_value="lio", choices=["lio", "livo"]),
        DeclareLaunchArgument("lidar_type", default_value=PythonExpression([
            "'airy' if '", lidar_mode, "' == 'dual' else 'livox'"
        ]), choices=["livox", "airy", "hesai"]),
        DeclareLaunchArgument(
            "sensor_config",
            default_value=PythonExpression([
                "'' if '", lidar_type, "' == 'hesai' else '", package_share,
                "/config/' + ('dual_ros.yaml' if '", lidar_type, "' == 'airy' else 'mid360_ros.yaml')"
            ]),
        ),
        DeclareLaunchArgument(
            "algorithm_config",
            default_value=PythonExpression([
                "'' if '", lidar_type, "' == 'hesai' else '", package_share,
                "/config/' + ('dual.toml' if '", lidar_type, "' == 'airy' else 'mid360.toml')"
            ]),
        ),
        DeclareLaunchArgument("lidar_topic", default_value="/front_lidar"),
        DeclareLaunchArgument("rear_lidar_topic", default_value="/rear_lidar"),
        DeclareLaunchArgument(
            "imu_topic", default_value="/front_lidar/imu"
        ),
        DeclareLaunchArgument("image_topic", default_value="/camera/image"),
        DeclareLaunchArgument("compressed_image", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
    ]

    sapphire_node = Node(
        package="sapphire_ros2",
        executable="sapphire",
        name="cmn_sapphire",
        output="screen",
        parameters=[
            sensor_config,
            {
                "lidar_mode": lidar_mode,
                "lidar_type": lidar_type,
                "obs_mode": obs_mode,
                "algorithm_config": algorithm_config,
                "topics.lidar": lidar_topic,
                "topics.rear_lidar": rear_lidar_topic,
                "topics.imu": imu_topic,
                "topics.image": image_topic,
                "image.compressed": ParameterValue(
                    compressed_image, value_type=bool
                ),
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [package_share, "rviz_cfg", "sapphire.rviz"]
            ),
        ],
        condition=IfCondition(rviz),
    )

    shutdown_on_sapphire_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=sapphire_node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(reason="Sapphire processing finished")
                )
            ],
        )
    )

    return LaunchDescription(
        arguments + [OpaqueFunction(function=validate_hesai_config), shutdown_on_sapphire_exit, sapphire_node, rviz_node]
    )
