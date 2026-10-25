from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("sapphire_ros2")
    arguments = [
        DeclareLaunchArgument(name, default_value=PathJoinSubstitution([share, "config", filename]))
        for name, filename in (("sensor_config", "sensors.toml"),
                               ("algorithm_config", "params.toml"), ("ros_config", "ros.yaml"))
    ]
    arguments.append(DeclareLaunchArgument("rviz", default_value="true"))
    node = Node(
        package="sapphire_ros2", executable="sapphire", name="cmn_sapphire", output="screen",
        parameters=[LaunchConfiguration("ros_config"), {
            "sensor_config": LaunchConfiguration("sensor_config"),
            "algorithm_config": LaunchConfiguration("algorithm_config"),
        }],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", PathJoinSubstitution([share, "rviz", "sapphire.rviz"])],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    shutdown = RegisterEventHandler(OnProcessExit(
        target_action=node, on_exit=[EmitEvent(event=Shutdown(reason="Sapphire processing finished"))]))
    return LaunchDescription(arguments + [shutdown, node, rviz])
