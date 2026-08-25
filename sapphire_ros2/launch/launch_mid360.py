from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("sapphire_ros2")
    algorithm_config = LaunchConfiguration("algorithm_config")
    lidar_topic = LaunchConfiguration("lidar_topic")
    imu_topic = LaunchConfiguration("imu_topic")
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

    return LaunchDescription(arguments + [sapphire_node, rviz_node])
