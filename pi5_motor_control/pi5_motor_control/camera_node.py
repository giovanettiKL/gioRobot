#!/usr/bin/env python3
"""
camera_node.py
ROS2 Jazzy — Raspberry Pi Camera Module 3 publisher (Raspberry Pi 5 package)

Captures frames via picamera2/libcamera (the CSI camera stack — the legacy
`raspistill`/`picamera` v1 API does not support the IMX708 sensor used in
Camera Module 3) and publishes them as sensor_msgs/Image on /camera/image_raw,
plus a matching sensor_msgs/CameraInfo on /camera/camera_info.

Install deps:
  sudo apt install python3-picamera2 python3-libcamera ros-jazzy-cv-bridge

Camera Module 3 autofocus:
  The IMX708 sensor supports three AfMode settings, selected via the
  'autofocus_mode' parameter:
    continuous — camera continuously refocuses (default)
    auto       — single autofocus sweep triggered once at startup
    manual     — fixed focus at 'lens_position' dioptres (0.0 = infinity)

Other cameras (e.g. HQ Camera + manual-focus lens):
  Set 'has_autofocus' to False to skip AF control calls entirely — required
  for sensors/lenses with no AF motor, where focus (and iris, on lenses that
  have one) is set by hand on the lens barrel. See camera_params_hq.yaml.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

# Try to import picamera2; fall back to a synthetic test-pattern generator
# for development on non-Pi hardware (same pattern as motor_controller_node's
# lgpio fallback).
try:
    from picamera2 import Picamera2
    from libcamera import controls
    SIMULATION = False
except (ImportError, RuntimeError):
    SIMULATION = True
    print("[WARN] picamera2 not found — running in SIMULATION mode")


AF_MODE_MAP = {
    "continuous": "Continuous",
    "auto":       "Auto",
    "manual":     "Manual",
}


class CameraNode(Node):
    """Publishes frames from the Raspberry Pi Camera Module 3."""

    def __init__(self):
        super().__init__("camera_node")

        # ── Declare & read parameters ─────────────────────────────────────
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("framerate", 30.0)
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("camera_num", 0)                 # which CSI port/libcamera index to open: 0 or 1 (Pi 5 has two)
        self.declare_parameter("has_autofocus", True)           # False for lenses/sensors with no AF motor (e.g. HQ Camera + manual-focus lens)
        self.declare_parameter("autofocus_mode", "continuous")  # continuous|auto|manual (ignored if has_autofocus is False)
        self.declare_parameter("lens_position", 0.0)            # dioptres, manual AF mode only

        self.width          = self.get_parameter("width").value
        self.height         = self.get_parameter("height").value
        self.framerate      = self.get_parameter("framerate").value
        self.frame_id       = self.get_parameter("frame_id").value
        self.camera_num     = self.get_parameter("camera_num").value
        self.has_autofocus  = self.get_parameter("has_autofocus").value
        self.autofocus_mode = self.get_parameter("autofocus_mode").value
        self.lens_position  = self.get_parameter("lens_position").value

        if self.autofocus_mode not in AF_MODE_MAP:
            self.get_logger().warn(
                f"Unknown autofocus_mode '{self.autofocus_mode}', defaulting to 'continuous'"
            )
            self.autofocus_mode = "continuous"

        self.bridge = CvBridge()

        # ── Camera setup ─────────────────────────────────────────────────
        self._setup_camera()

        # ── ROS interfaces ────────────────────────────────────────────────
        self.image_pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self.info_pub  = self.create_publisher(CameraInfo, "/camera/camera_info", 10)

        self.create_timer(1.0 / self.framerate, self._capture_callback)

        self.get_logger().info(
            f"CameraNode ready  |  cam{self.camera_num}  |  {self.width}x{self.height}@{self.framerate:.0f}fps  "
            f"|  af={self.autofocus_mode if self.has_autofocus else 'manual'}  |  sim={'YES' if SIMULATION else 'NO'}"
        )

    # ── Camera helpers ────────────────────────────────────────────────────────

    def _setup_camera(self):
        if SIMULATION:
            self.picam2 = None
            self._sim_frame_count = 0
            return

        self.picam2 = Picamera2(camera_num=self.camera_num)
        config = self.picam2.create_video_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"}
        )
        self.picam2.configure(config)
        self.picam2.start()

        if not self.has_autofocus:
            # Manual-focus lens/sensor (e.g. HQ Camera + RPi 6mm CS-mount lens) —
            # no AfMode control exists on this hardware; focus and iris are set
            # by hand on the lens barrel. Skip the AF control calls entirely.
            self.get_logger().info(
                "has_autofocus=False — skipping AF controls (manual focus/iris lens)"
            )
            return

        af_mode = getattr(controls.AfModeEnum, AF_MODE_MAP[self.autofocus_mode])
        if self.autofocus_mode == "manual":
            self.picam2.set_controls({
                "AfMode": af_mode,
                "LensPosition": self.lens_position,
            })
        else:
            self.picam2.set_controls({"AfMode": af_mode})
            if self.autofocus_mode == "auto":
                self.picam2.set_controls({"AfTrigger": controls.AfTriggerEnum.Start})

    def _capture_frame(self) -> np.ndarray:
        """Returns an HxWx3 array. RGB888 format from picamera2 is ordered BGR
        (a long-standing libcamera quirk kept for OpenCV compatibility) — this
        is why the published encoding below is 'bgr8', not 'rgb8'."""
        if SIMULATION:
            # Synthetic moving gradient so downstream consumers have something
            # to look at without real hardware.
            self._sim_frame_count += 1
            x = np.linspace(0, 255, self.width, dtype=np.uint8)
            frame = np.tile(x, (self.height, 1))
            frame = np.stack([
                np.roll(frame, self._sim_frame_count, axis=1),
                frame,
                np.roll(frame, -self._sim_frame_count, axis=1),
            ], axis=-1)
            return frame

        return self.picam2.capture_array()

    # ── Callback ──────────────────────────────────────────────────────────────

    def _capture_callback(self):
        frame = self._capture_frame()

        now = self.get_clock().now().to_msg()

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.frame_id
        self.image_pub.publish(img_msg)

        info_msg = CameraInfo()
        info_msg.header.stamp = now
        info_msg.header.frame_id = self.frame_id
        info_msg.width  = self.width
        info_msg.height = self.height
        self.info_pub.publish(info_msg)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        if not SIMULATION and self.picam2 is not None:
            self.picam2.stop()
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
