"""
robot_bringup.launch.py
Launches BOTH nodes together (Raspberry Pi 5 package):
  1. motor_controller  — lgpio driver, subscribes /cmd_vel
  2. robot_action_node — action server, publishes /cmd_vel

Usage:
  ros2 launch pi5_motor_control robot_bringup.launch.py
  ros2 launch pi5_motor_control robot_bringup.launch.py demo_on_start:=true
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
        description="Path to motor controller parameters",
    )
    demo_arg = DeclareLaunchArgument(
        "demo_on_start",
        default_value="false",
        description="Run square demo sequence on startup",
    )

    motor_node = Node(
        package="pi5_motor_control",
        executable="motor_controller",
        name="motor_controller",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file")],
    )

    action_node = Node(
        package="pi5_motor_control",
        executable="robot_action_node",
        name="robot_action_node",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "default_linear_speed":  0.3,
            "default_angular_speed": 1.0,
            "cmd_vel_rate":          10.0,
            "demo_on_start": LaunchConfiguration("demo_on_start"),
        }],
    )

    return LaunchDescription([params_arg, demo_arg, motor_node, action_node])
