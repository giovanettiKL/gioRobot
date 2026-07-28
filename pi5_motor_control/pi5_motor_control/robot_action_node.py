#!/usr/bin/env python3
"""
robot_action_node.py
ROS2 Jazzy — Robot Action Client Node (Raspberry Pi 5 package)

Publishes to /cmd_vel (geometry_msgs/Twist) to drive the motor_controller node.

This node has no direct GPIO dependency — it only talks to motor_controller
over topics, so it is identical in behaviour on Pi 4 and Pi 5. It lives in the
pi5_motor_control package here purely to pair with the Pi 5 motor controller.

Provides three interfaces:
  1. ROS2 Action Server  — /drive_action  (accepts DriveGoal: distance + heading)
  2. ROS2 Service        — /drive_command (simple one-shot command: forward/back/left/right/stop)
  3. Built-in sequence   — runs a demo square pattern on startup (set demo_on_start: true)

Actions allow the caller to:
  - Send a goal
  - Monitor feedback (distance travelled)
  - Cancel mid-motion
  - Receive a result (success / aborted)

Because ROS2 Jazzy ships without a built-in "DriveDistance" action type,
this node defines its own simple action using the standard action_msgs pattern
via rclpy's action server, using only built-in message types.

Usage:
  ros2 run pi5_motor_control robot_action_node

Send a goal from another terminal:
  # Move forward 1.0 m
  ros2 action send_goal /drive_action pi5_motor_control/action/Drive \
    "{linear_speed: 0.3, angular_speed: 0.0, duration: 3.0}"

  # Spin left for 2 seconds
  ros2 action send_goal /drive_action pi5_motor_control/action/Drive \
    "{linear_speed: 0.0, angular_speed: 1.0, duration: 2.0}"

One-shot service call:
  ros2 service call /drive_command pi5_motor_control/srv/DriveCommand \
    "{command: 'forward', speed: 0.3, duration: 2.0}"
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from motor_control_interfaces.msg import MotorCommand

import time
import threading
import math


# ── Inline action / service definitions (no .action/.srv files needed) ────────
# We use the built-in action pattern with a custom goal/result/feedback
# by sending structured dicts via a simple topic-based action pattern.
# For a production package, generate proper .action files with rosidl.

# Since we cannot define custom .action types without a full rosidl build step,
# this node implements the SAME behaviour using:
#   - A goal topic   /drive_goal    (String JSON)
#   - A result topic /drive_result  (String JSON)
#   - A cancel topic /drive_cancel  (String)
#   - A feedback topic /drive_feedback (String JSON)
# This is fully functional and easy to call from any language or the CLI.

# ── Simple command vocabulary ─────────────────────────────────────────────────
COMMANDS = {
    "forward":  ( 1.0,  0.0),   # (linear_sign, angular_sign)
    "backward": (-1.0,  0.0),
    "left":     ( 0.0,  1.0),
    "right":    ( 0.0, -1.0),
    "stop":     ( 0.0,  0.0),
}

# Word → MotorCommand.command, for the /motor_command_text broadcaster below.
MOTOR_COMMAND_WORDS = {
    "forward":   MotorCommand.FORWARD,
    "backward":  MotorCommand.BACKWARD,
    "left":      MotorCommand.LEFT,
    "right":     MotorCommand.RIGHT,
    "set_speed": MotorCommand.SET_SPEED,
    "stop":      MotorCommand.STOP,
}


class RobotActionNode(Node):
    """
    Publishes Twist commands to /cmd_vel in response to:
      - Goals received on /drive_goal
      - Service calls on /drive_command (simple string commands)
      - A built-in demo sequence (optional)
    """

    def __init__(self):
        super().__init__("robot_action_node")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("default_linear_speed",  0.3)   # m/s
        self.declare_parameter("default_angular_speed", 1.0)   # rad/s
        self.declare_parameter("demo_on_start",         False)
        self.declare_parameter("cmd_vel_rate",          10.0)   # Hz publish rate

        self.linear_speed  = self.get_parameter("default_linear_speed").value
        self.angular_speed = self.get_parameter("default_angular_speed").value
        self.demo_on_start = self.get_parameter("demo_on_start").value
        self.publish_rate  = self.get_parameter("cmd_vel_rate").value

        # ── Publishers & Subscribers ──────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.feedback_pub = self.create_publisher(String, "/drive_feedback", 10)
        self.result_pub   = self.create_publisher(String, "/drive_result",   10)

        # Goal subscriber — accepts JSON string goals
        self.goal_sub  = self.create_subscription(
            String, "/drive_goal",   self._goal_callback,   10
        )
        self.cancel_sub = self.create_subscription(
            String, "/drive_cancel", self._cancel_callback, 10
        )
        # Simple one-shot command subscriber
        self.cmd_sub = self.create_subscription(
            String, "/drive_command", self._command_callback, 10
        )

        # Discrete motor-command broadcaster: translates plain-text words into
        # MotorCommand messages on /motor_command, which motor_controller_node
        # listens to directly (no /cmd_vel Twist mixing involved).
        self.motor_cmd_pub = self.create_publisher(MotorCommand, "/motor_command", 10)
        self.motor_cmd_text_sub = self.create_subscription(
            String, "/motor_command_text", self._motor_command_text_callback, 10
        )

        # ── Internal state ────────────────────────────────────────────────
        self._active_goal   = None
        self._cancel_flag   = False
        self._goal_lock     = threading.Lock()

        self.get_logger().info("RobotActionNode ready")
        self.get_logger().info("  Publish goal to : /drive_goal")
        self.get_logger().info("  Simple command  : /drive_command")
        self.get_logger().info("  Cancel          : /drive_cancel")
        self.get_logger().info("  Feedback        : /drive_feedback  (subscribe)")
        self.get_logger().info("  Result          : /drive_result    (subscribe)")
        self.get_logger().info("  Motor command   : /motor_command_text  (forward/backward/left/right/set_speed <0-1>/stop)")

        # ── Optional demo sequence ────────────────────────────────────────
        if self.demo_on_start:
            threading.Thread(target=self._run_demo, daemon=True).start()

    # ── Internal motion primitive ─────────────────────────────────────────────

    def _publish_twist(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _stop(self):
        self._publish_twist(0.0, 0.0)

    def _drive_for(self, linear: float, angular: float, duration: float,
                   goal_id: str = "") -> bool:
        """
        Drive at the given velocities for `duration` seconds.
        Publishes feedback every second.
        Returns True on completion, False if cancelled.
        """
        rate_sleep = 1.0 / self.publish_rate
        elapsed    = 0.0
        start_time = time.time()

        self.get_logger().info(
            f"[{goal_id}] Driving  linear={linear:.2f}  angular={angular:.2f}"
            f"  for {duration:.1f}s"
        )

        while elapsed < duration:
            if self._cancel_flag:
                self.get_logger().info(f"[{goal_id}] Cancelled at {elapsed:.1f}s")
                self._stop()
                return False

            self._publish_twist(linear, angular)
            time.sleep(rate_sleep)
            elapsed = time.time() - start_time

            # Feedback every ~1 second
            if int(elapsed) != int(elapsed - rate_sleep):
                fb = String()
                fb.data = (
                    f'{{"goal_id":"{goal_id}",'
                    f'"elapsed":{elapsed:.1f},'
                    f'"remaining":{max(0.0, duration-elapsed):.1f}}}'
                )
                self.feedback_pub.publish(fb)
                self.get_logger().debug(
                    f"[{goal_id}] elapsed={elapsed:.1f}s remaining={duration-elapsed:.1f}s"
                )

        self._stop()
        return True

    # ── Goal callback ─────────────────────────────────────────────────────────

    def _goal_callback(self, msg: String):
        """
        Accepts a JSON goal string, e.g.:
          {"goal_id": "g1", "linear": 0.3, "angular": 0.0, "duration": 3.0}
        """
        import json

        try:
            goal = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Bad goal JSON: {e}")
            return

        goal_id  = goal.get("goal_id", "goal")
        linear   = float(goal.get("linear",   self.linear_speed))
        angular  = float(goal.get("angular",  0.0))
        duration = float(goal.get("duration", 2.0))

        with self._goal_lock:
            if self._active_goal is not None:
                self.get_logger().warn(
                    f"Goal '{self._active_goal}' already running — rejecting '{goal_id}'"
                )
                result = String()
                result.data = (
                    f'{{"goal_id":"{goal_id}","status":"rejected",'
                    f'"reason":"another goal is active"}}'
                )
                self.result_pub.publish(result)
                return

            self._active_goal = goal_id
            self._cancel_flag = False

        # Run in a background thread so we don't block the executor
        def _run():
            success = self._drive_for(linear, angular, duration, goal_id)
            result  = String()
            result.data = (
                f'{{"goal_id":"{goal_id}",'
                f'"status":"{"succeeded" if success else "cancelled"}"}}'
            )
            self.result_pub.publish(result)
            self.get_logger().info(
                f"[{goal_id}] {'Succeeded' if success else 'Cancelled'}"
            )
            with self._goal_lock:
                self._active_goal = None
                self._cancel_flag = False

        threading.Thread(target=_run, daemon=True).start()

    def _cancel_callback(self, msg: String):
        self.get_logger().info("Cancel requested")
        self._cancel_flag = True

    # ── Simple command callback ───────────────────────────────────────────────

    def _command_callback(self, msg: String):
        """
        Accepts simple string commands with optional args:
          "forward"              — drive forward at default speed for default duration
          "forward 0.5 3.0"     — forward at 0.5 m/s for 3 seconds
          "left 1.0 2.0"        — spin left at 1.0 rad/s for 2 seconds
          "stop"                — stop immediately
        """
        parts = msg.data.strip().split()
        if not parts:
            return

        command  = parts[0].lower()
        speed    = float(parts[1]) if len(parts) > 1 else self.linear_speed
        duration = float(parts[2]) if len(parts) > 2 else 2.0

        if command == "stop":
            self._cancel_flag = True
            self._stop()
            self.get_logger().info("STOP command received")
            return

        if command not in COMMANDS:
            self.get_logger().error(
                f"Unknown command '{command}'. "
                f"Valid: {list(COMMANDS.keys())}"
            )
            return

        lin_sign, ang_sign = COMMANDS[command]
        linear  = lin_sign * speed if lin_sign != 0 else 0.0
        angular = ang_sign * speed if ang_sign != 0 else 0.0

        self.get_logger().info(
            f"Command: {command}  speed={speed}  duration={duration}s"
        )

        threading.Thread(
            target=self._drive_for,
            args=(linear, angular, duration, command),
            daemon=True
        ).start()

    # ── Motor command broadcaster ─────────────────────────────────────────────

    def _motor_command_text_callback(self, msg: String):
        """
        Accepts plain-text words and republishes them as MotorCommand on
        /motor_command:
          "forward"          — drive forward at the current default speed
          "backward"         — drive backward at the current default speed
          "left"             — turn left (pivot) at the current default speed
          "right"            — turn right (pivot) at the current default speed
          "set_speed 0.7"    — set the default speed to 70% of max
          "stop"             — stop immediately
        """
        parts = msg.data.strip().split()
        if not parts:
            return

        word = parts[0].lower()
        if word not in MOTOR_COMMAND_WORDS:
            self.get_logger().error(
                f"Unknown motor command '{word}'. Valid: {list(MOTOR_COMMAND_WORDS.keys())}"
            )
            return

        cmd = MotorCommand()
        cmd.command = MOTOR_COMMAND_WORDS[word]
        if len(parts) > 1:
            cmd.speed = float(parts[1])

        self.motor_cmd_pub.publish(cmd)
        self.get_logger().info(f"motor_command → {word} {cmd.speed if len(parts) > 1 else ''}".strip())

    # ── Demo sequence ─────────────────────────────────────────────────────────

    def _run_demo(self):
        """Drive a square: forward, left turn × 4."""
        time.sleep(2.0)   # wait for motor_controller to be ready
        self.get_logger().info("=== Starting demo square sequence ===")

        side_duration = 3.0    # seconds per side
        turn_duration = 1.57   # seconds for ~90° turn at angular_speed=1.0

        for i in range(4):
            self.get_logger().info(f"Square side {i+1}/4")
            ok = self._drive_for(self.linear_speed, 0.0,  side_duration, f"side_{i+1}")
            if not ok:
                break
            time.sleep(0.2)
            ok = self._drive_for(0.0, self.angular_speed, turn_duration, f"turn_{i+1}")
            if not ok:
                break
            time.sleep(0.2)

        self.get_logger().info("=== Demo sequence complete ===")


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = RobotActionNode()

    # MultiThreadedExecutor allows goal callbacks to run alongside spin
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
