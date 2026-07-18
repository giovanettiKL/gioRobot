#!/usr/bin/env python3
""""""""""""""""""""""""""""""
motor_controller_node.py
ROS2 Humble — Raspberry Pi 4 Differential Drive Motor Controller

Subscribes to /cmd_vel (geometry_msgs/Twist) and drives two motors
via a PWM motor driver (e.g. L298N, DRV8833, TB6612FNG) using
RPi.GPIO or pigpio.

Wiring (L298N example):
  ENA  → GPIO 12  (PWM, left motor speed)
  IN1  → GPIO 20  (left motor direction)
  IN2  → GPIO 21  (left motor direction)
  ENB  → GPIO 13  (PWM, right motor speed)
  IN3  → GPIO 19  (right motor direction)
  IN4  → GPIO 26  (right motor direction)

Install deps:
  sudo apt install python3-rpi.gpio
  pip3 install RPi.GPIO   # if not available via apt
"""""""""""""""""""""""""""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math

# Try to import RPi.GPIO; fall back to a mock for development on non-Pi hardware
try:
    import RPi.GPIO as GPIO
    SIMULATION = False
except (ImportError, RuntimeError):
    SIMULATION = True
    print("[WARN] RPi.GPIO not found — running in SIMULATION mode")


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


class MotorControllerNode(Node):
    """Differential drive motor controller for Raspberry Pi 4."""

    def __init__(self):
        super().__init__("motor_controller")

        # ── Declare & read parameters ─────────────────────────────────────
        self.declare_parameter("wheel_base", 0.20)        # metres, distance between wheels
        self.declare_parameter("max_linear_speed", 0.5)   # m/s
        self.declare_parameter("max_angular_speed", 2.0)  # rad/s
        self.declare_parameter("pwm_frequency", PWM_FREQ)

        for pin_name, default in DEFAULT_PINS.items():
            self.declare_parameter(f"pin_{pin_name}", default)

        self.wheel_base       = self.get_parameter("wheel_base").value
        self.max_linear       = self.get_parameter("max_linear_speed").value
        self.max_angular      = self.get_parameter("max_angular_speed").value
        self.pwm_freq         = self.get_parameter("pwm_frequency").value

        self.pins = {k: self.get_parameter(f"pin_{k}").value for k in DEFAULT_PINS}

        # ── GPIO setup ───────────────────────────────────────────────────
        self._setup_gpio()

        # ── ROS interfaces ────────────────────────────────────────────────
        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_callback, 10
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
            self.left_pwm = self.right_pwm = None
            return

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        output_pins = list(self.pins.values())
        GPIO.setup(output_pins, GPIO.OUT, initial=GPIO.LOW)

        self.left_pwm  = GPIO.PWM(self.pins["ena"], self.pwm_freq)
        self.right_pwm = GPIO.PWM(self.pins["enb"], self.pwm_freq)
        self.left_pwm.start(0)
        self.right_pwm.start(0)

    def _set_motor(self, pwm, in_a_pin, in_b_pin, duty: float):
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

        GPIO.output(in_a_pin,  GPIO.HIGH if forward else GPIO.LOW)
        GPIO.output(in_b_pin,  GPIO.LOW  if forward else GPIO.HIGH)
        pwm.ChangeDutyCycle(speed)

    def _stop_all(self):
        self._set_motor(self.left_pwm,  self.pins["in1"], self.pins["in2"], 0.0)
        self._set_motor(self.right_pwm, self.pins["in3"], self.pins["in4"], 0.0)

    # ── Kinematic conversion ──────────────────────────────────────────────────

    def _twist_to_duty(self, linear_x: float, angular_z: float):
        """
        Convert Twist → left/right duty cycle (%).
        Uses simple differential-drive kinematics.
        """
        # Clamp inputs
        linear_x  = max(-self.max_linear,  min(self.max_linear,  linear_x))
        angular_z = max(-self.max_angular, min(self.max_angular, angular_z))

        # Wheel velocities (m/s)
        v_left  = linear_x - (angular_z * self.wheel_base / 2.0)
        v_right = linear_x + (angular_z * self.wheel_base / 2.0)

        # Normalise to duty % using max_linear as scale
        max_v = max(self.max_linear, abs(v_left), abs(v_right))  # avoid div/0
        left_duty  = (v_left  / max_v) * 100.0
        right_duty = (v_right / max_v) * 100.0

        return left_duty, right_duty

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cmd_vel_callback(self, msg: Twist):
        self._last_cmd_time = self.get_clock().now()

        left_duty, right_duty = self._twist_to_duty(msg.linear.x, msg.angular.z)

        self._set_motor(self.left_pwm,  self.pins["in1"], self.pins["in2"], left_duty)
        self._set_motor(self.right_pwm, self.pins["in3"], self.pins["in4"], right_duty)

        self.get_logger().debug(
            f"cmd_vel → L:{left_duty:+.1f}%  R:{right_duty:+.1f}%"
        )

        # Publish duty cycles for external monitoring
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
            if self.left_pwm:  self.left_pwm.stop()
            if self.right_pwm: self.right_pwm.stop()
            GPIO.cleanup()
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
