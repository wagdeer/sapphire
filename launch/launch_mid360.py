from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_share = FindPackageShare("sapphire")
    config_file = PathJoinSubstitution([pkg_share, "config", "mid360.yaml"])
    rviz_config = PathJoinSubstitution([pkg_share, "rviz_cfg", "sapphire.rviz"])
    rviz = LaunchConfiguration("rviz")
    lidar_topic = LaunchConfiguration("lidar_topic")
    imu_topic = LaunchConfiguration("imu_topic")

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("lidar_topic", default_value="/front_lidar"),
        DeclareLaunchArgument("imu_topic", default_value="/front_lidar/imu"),
        Node(
            package="sapphire",
            executable="sapphire",
            name="cmn_sapphire",
            output="screen",
            parameters=[
                config_file,
                {
                    "General.lidar_topic": lidar_topic,
                    "General.imu_topic": imu_topic,
                },
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(rviz),
        ),
    ])
