"""
camera.launch.py
Launches camera_node (Raspberry Pi Camera Module 3 publisher).

Usage:
  ros2 launch pi5_motor_control camera.launch.py
  ros2 launch pi5_motor_control camera.launch.py params_file:=/path/to/my_params.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("pi5_motor_control")

    default_params = PathJoinSubstitution([pkg_share, "config", "camera_params.yaml"])

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to camera node parameters",
    )

    camera_node = Node(
        package="pi5_motor_control",
        executable="camera_node",
        name="camera_node",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_arg, camera_node])
