from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("sapphire_ros2")
    algorithm_config = LaunchConfiguration("algorithm_config")
    lidar_topic = LaunchConfiguration("lidar_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    image_topic = LaunchConfiguration("image_topic")
    compressed_image = LaunchConfiguration("compressed_image")
    rviz = LaunchConfiguration("rviz")

    arguments = [
        DeclareLaunchArgument(
            "algorithm_config",
            default_value=PathJoinSubstitution(
                [package_share, "config", "mid360.toml"]
            ),
        ),
        DeclareLaunchArgument("lidar_topic", default_value="/front_lidar"),
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
            PathJoinSubstitution(
                [package_share, "config", "mid360_ros.yaml"]
            ),
            {
                "algorithm_config": algorithm_config,
                "topics.lidar": lidar_topic,
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
        arguments + [shutdown_on_sapphire_exit, sapphire_node, rviz_node]
    )
