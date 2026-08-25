#!/usr/bin/env python3
"""
grab_frame.py
Subscribes to /camera/image_raw, saves the first frame received to a JPEG,
then exits. Useful for a quick visual sanity check on a headless Pi (no
display/X server) — run this, then scp the saved file to another machine
to actually look at it.

Requires the camera node (camera_node.py) to already be running and
publishing on /camera/image_raw in another terminal.

Usage:
  python3 grab_frame.py [output_path]

Defaults to saving as ~/frame.jpg if no path is given.
"""
import sys
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class FrameGrabber(Node):
    def __init__(self, output_path: str):
        super().__init__("frame_grabber")
        self.output_path = output_path
        self.bridge = CvBridge()
        self.got_it = False
        self.sub = self.create_subscription(
            Image, "/camera/image_raw", self.callback, 10
        )
        self.get_logger().info(f"Waiting for a frame on /camera/image_raw ...")

    def callback(self, msg: Image):
        if self.got_it:
            return
        self.got_it = True
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(self.output_path, frame)
        self.get_logger().info(
            f"Saved frame ({msg.width}x{msg.height}) to {self.output_path}"
        )


def main():
    output_path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/frame.jpg")

    rclpy.init()
    node = FrameGrabber(output_path)
    try:
        while rclpy.ok() and not node.got_it:
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
