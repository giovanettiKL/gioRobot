#!/usr/bin/env python3
"""
motor_controller_node.py
ROS2 Jazzy — Raspberry Pi 5 Differential Drive Motor Controller

Subscribes to /cmd_vel (geometry_msgs/Twist) and drives two motors
via a PWM motor driver (e.g. L298N, DRV8833, TB6612FNG) using
lgpio (the RP1-compatible GPIO library; RPi.GPIO does not support Pi 5).

Wiring (L298N example):
  ENA  → GPIO 12  (PWM, left motor speed)
  IN1  → GPIO 20  (left motor direction)
  IN2  → GPIO 21  (left motor direction)
  ENB  → GPIO 13  (PWM, right motor speed)
  IN3  → GPIO 19  (right motor direction)
  IN4  → GPIO 26  (right motor direction)

Install deps:
  sudo apt install python3-lgpio
  pip3 install lgpio   # if not available via apt
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from motor_control_interfaces.msg import MotorCommand
import math

# Try to import lgpio; fall back to a mock for development on non-Pi hardware.
# RPi.GPIO is intentionally NOT used here — it does not support the Pi 5's
# RP1 I/O controller and will fail (or silently misbehave) on this board.
try:
    import lgpio
    SIMULATION = False
except (ImportError, RuntimeError):
    SIMULATION = True
    print("[WARN] lgpio not found — running in SIMULATION mode")


# ── GPIO pin defaults (override via ROS2 parameters) ─────────────────────────
DEFAULT_PINS = {
    "ena": 12,   # Left motor PWM enable
    "in1": 20,   # Left motor dir A
    "in2": 21,   # Left motor dir B
    "enb": 13,   # Right motor PWM enable
    "in3": 19,   # Right motor dir A
    "in4": 26,   # Right motor dir B
}

PWM_FREQ = 1000          # Hz — suitable for most DC motor drivers
CMD_VEL_TIMEOUT = 0.5    # seconds; stop motors if no command received
DEFAULT_GPIOCHIP = 4     # Pi 5: the 40-pin header is exposed on gpiochip4 (RP1).
                         # Older Bookworm images may need 0 — override via the
                         # 'gpiochip' parameter if gpiochip_open(4) fails.


class MotorControllerNode(Node):
    """Differential drive motor controller for Raspberry Pi 5."""

    def __init__(self):
        super().__init__("motor_controller")

        # ── Declare & read parameters ─────────────────────────────────────
        self.declare_parameter("wheel_base", 0.20)        # metres, distance between wheels
        self.declare_parameter("max_linear_speed", 0.5)   # m/s
        self.declare_parameter("max_angular_speed", 2.0)  # rad/s
        self.declare_parameter("pwm_frequency", PWM_FREQ)
        self.declare_parameter("gpiochip", DEFAULT_GPIOCHIP)
        self.declare_parameter("default_command_speed", 0.5)  # fraction of max, for /motor_command

        for pin_name, default in DEFAULT_PINS.items():
            self.declare_parameter(f"pin_{pin_name}", default)

        self.wheel_base       = self.get_parameter("wheel_base").value
        self.max_linear       = self.get_parameter("max_linear_speed").value
        self.max_angular      = self.get_parameter("max_angular_speed").value
        self.pwm_freq         = self.get_parameter("pwm_frequency").value
        self.gpiochip         = self.get_parameter("gpiochip").value
        self._command_speed   = self.get_parameter("default_command_speed").value

        self.pins = {k: self.get_parameter(f"pin_{k}").value for k in DEFAULT_PINS}

        # ── GPIO setup ───────────────────────────────────────────────────
        self._setup_gpio()

        # ── ROS interfaces ────────────────────────────────────────────────
        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_callback, 10
        )

        # Discrete command interface: forward / backward / left / right /
        # set_speed / stop. See motor_control_interfaces/msg/MotorCommand.msg.
        self.motor_cmd_sub = self.create_subscription(
            MotorCommand, "/motor_command", self._motor_command_callback, 10
        )

        # Publishes current duty cycles for monitoring / debugging
        self.duty_pub = self.create_publisher(Float32MultiArray, "/motor_duty", 10)

        # Watchdog: stop motors if no command arrives within timeout
        self._last_cmd_time = self.get_clock().now()
        self.create_timer(0.1, self._watchdog_callback)

        self.get_logger().info(
            f"MotorController ready  |  wheel_base={self.wheel_base} m  "
            f"|  sim={'YES' if SIMULATION else 'NO'}"
        )

    # ── GPIO helpers ──────────────────────────────────────────────────────────

    def _setup_gpio(self):
        if SIMULATION:
            self.h = None
            return

        self.h = lgpio.gpiochip_open(self.gpiochip)

        # Direction pins are plain digital outputs.
        for name in ("in1", "in2", "in3", "in4"):
            lgpio.gpio_claim_output(self.h, self.pins[name], 0)

        # PWM pins are driven via lgpio's software-timed tx_pwm — do NOT
        # claim them as plain outputs first, tx_pwm claims them itself.
        lgpio.tx_pwm(self.h, self.pins["ena"], self.pwm_freq, 0)
        lgpio.tx_pwm(self.h, self.pins["enb"], self.pwm_freq, 0)

    def _set_motor(self, pwm_pin: int, in_a_pin: int, in_b_pin: int, duty: float):
        """
        Drive one motor.
        duty: -100.0 … +100.0  (negative = reverse)
        """
        duty = max(-100.0, min(100.0, duty))
        forward = duty >= 0
        speed   = abs(duty)

        if SIMULATION:
            direction = "FWD" if forward else "REV"
            # Logged at debug level to avoid console spam
            self.get_logger().debug(
                f"[SIM] pin({in_a_pin},{in_b_pin}) {direction} {speed:.1f}%"
            )
            return

        lgpio.gpio_write(self.h, in_a_pin, 1 if forward else 0)
        lgpio.gpio_write(self.h, in_b_pin, 0 if forward else 1)
        lgpio.tx_pwm(self.h, pwm_pin, self.pwm_freq, speed)

    def _stop_all(self):
        self._set_motor(self.pins["ena"], self.pins["in1"], self.pins["in2"], 0.0)
        self._set_motor(self.pins["enb"], self.pins["in3"], self.pins["in4"], 0.0)

    # ── Kinematic conversion ──────────────────────────────────────────────────

    def _twist_to_duty(self, linear_x: float, angular_z: float):
        """
        Convert Twist → left/right duty cycle (%).

        Arcade-style mixing: linear_x and angular_z are each normalised
        against their own max (max_linear_speed / max_angular_speed) and
        combined directly in duty-percent space, instead of converting to
        physical per-wheel m/s (using wheel_base) and normalising by
        max_linear_speed. The physical-kinematics approach starves
        rotate-in-place commands of torque on robots with a narrow
        wheel_base: e.g. wheel_base=0.20m at angular_z=1.0 rad/s only
        produces a ±0.1 m/s wheel-speed delta, which normalises down to
        ~20% PWM duty — too weak to overcome motor/gearbox static
        friction. Arcade mixing guarantees a full-speed pivot (angular_z
        == max_angular_speed, linear_x == 0) reaches ±100% duty.
        wheel_base is kept as a declared parameter for any future
        odometry/kinematics use, but no longer feeds this duty mix.
        """
        # Clamp inputs
        linear_x  = max(-self.max_linear,  min(self.max_linear,  linear_x))
        angular_z = max(-self.max_angular, min(self.max_angular, angular_z))

        linear_frac  = (linear_x  / self.max_linear)  if self.max_linear  else 0.0
        angular_frac = (angular_z / self.max_angular) if self.max_angular else 0.0

        left_duty  = (linear_frac - angular_frac) * 100.0
        right_duty = (linear_frac + angular_frac) * 100.0

        left_duty  = max(-100.0, min(100.0, left_duty))
        right_duty = max(-100.0, min(100.0, right_duty))

        return left_duty, right_duty

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cmd_vel_callback(self, msg: Twist):
        self._last_cmd_time = self.get_clock().now()

        left_duty, right_duty = self._twist_to_duty(msg.linear.x, msg.angular.z)

        self._set_motor(self.pins["ena"], self.pins["in1"], self.pins["in2"], left_duty)
        self._set_motor(self.pins["enb"], self.pins["in3"], self.pins["in4"], right_duty)

        self.get_logger().debug(
            f"cmd_vel → L:{left_duty:+.1f}%  R:{right_duty:+.1f}%"
        )

        # Publish duty cycles for external monitoring
        duty_msg = Float32MultiArray()
        duty_msg.data = [float(left_duty), float(right_duty)]
        self.duty_pub.publish(duty_msg)

    def _motor_command_callback(self, msg: MotorCommand):
        self._last_cmd_time = self.get_clock().now()

        if msg.command == MotorCommand.STOP:
            self._stop_all()
            self.get_logger().debug("motor_command → STOP")
            return

        if msg.command == MotorCommand.SET_SPEED:
            self._command_speed = max(0.0, min(1.0, msg.speed))
            self.get_logger().debug(f"motor_command → SET_SPEED {self._command_speed:.2f}")
            return

        if msg.speed > 0.0:
            self._command_speed = max(0.0, min(1.0, msg.speed))
        speed = self._command_speed

        if msg.command == MotorCommand.FORWARD:
            left_duty, right_duty = speed * 100.0, speed * 100.0
        elif msg.command == MotorCommand.BACKWARD:
            left_duty, right_duty = -speed * 100.0, -speed * 100.0
        elif msg.command == MotorCommand.LEFT:
            left_duty, right_duty = -speed * 100.0, speed * 100.0
        elif msg.command == MotorCommand.RIGHT:
            left_duty, right_duty = speed * 100.0, -speed * 100.0
        else:
            self.get_logger().error(f"Unknown MotorCommand.command={msg.command}")
            return

        self._set_motor(self.pins["ena"], self.pins["in1"], self.pins["in2"], left_duty)
        self._set_motor(self.pins["enb"], self.pins["in3"], self.pins["in4"], right_duty)

        duty_msg = Float32MultiArray()
        duty_msg.data = [float(left_duty), float(right_duty)]
        self.duty_pub.publish(duty_msg)

    def _watchdog_callback(self):
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if elapsed > CMD_VEL_TIMEOUT:
            self._stop_all()

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._stop_all()
        if not SIMULATION:
            lgpio.tx_pwm(self.h, self.pins["ena"], 0, 0)
            lgpio.tx_pwm(self.h, self.pins["enb"], 0, 0)
            lgpio.gpiochip_close(self.h)
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
