#!/usr/bin/env python3
"""
watch_focus.py
Continuously overwrites a single JPEG file with the latest frame from
/camera/image_raw, at a throttled rate (default: every 2 seconds).

This is a "poor man's live view" for headless setups where web_video_server
isn't available (e.g. archive dependency issues) — repeatedly re-fetching
this one file (via scp, or serving it with `python3 -m http.server`) gives
a manually-refreshed near-live feed, handy for turning a manual focus ring
while checking sharpness.

Requires the camera node to already be running and publishing on
/camera/image_raw in another terminal.

Usage:
  python3 watch_focus.py [output_path] [interval_seconds]

Defaults: ~/frame.jpg, every 2 seconds. Ctrl+C to stop.
"""
import sys
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class FocusWatcher(Node):
    def __init__(self, output_path: str, interval: float):
        super().__init__("focus_watcher")
        self.output_path = output_path
        self.interval = interval
        self.bridge = CvBridge()
        self.last_save = 0.0
        self.save_count = 0
        self.sub = self.create_subscription(
            Image, "/camera/image_raw", self.callback, 10
        )
        self.get_logger().info(
            f"Overwriting {output_path} every {interval}s. Ctrl+C to stop."
        )

    def callback(self, msg: Image):
        now = time.time()
        if now - self.last_save < self.interval:
            return
        self.last_save = now
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(self.output_path, frame)
        self.save_count += 1
        self.get_logger().info(f"[{self.save_count}] saved {self.output_path}")


def main():
    output_path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/frame.jpg")
    interval = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

    rclpy.init()
    node = FocusWatcher(output_path, interval)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
