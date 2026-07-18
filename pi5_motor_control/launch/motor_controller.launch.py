"""
motor_controller.launch.py
Launches the Pi5 motor controller node with a YAML parameter file.

Usage:
  ros2 launch pi5_motor_control motor_controller.launch.py
  ros2 launch pi5_motor_control motor_controller.launch.py params_file:=/path/to/custom.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("pi5_motor_control")

    default_params = PathJoinSubstitution([pkg_share, "config", "motor_params.yaml"])

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Full path to the ROS2 parameter file",
    )

    motor_node = Node(
        package="pi5_motor_control",
        executable="motor_controller",
        name="motor_controller",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file")],
        remappings=[
            ("/cmd_vel", "/cmd_vel"),       # remap here if needed
        ],
    )

    return LaunchDescription([params_arg, motor_node])
