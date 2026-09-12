#!/usr/bin/env python3
"""
live_view_server.py
Serves /camera/image_raw as an MJPEG stream over plain HTTP, viewable from
any browser on the same network — no file copying, no display/X server on
the Pi, and no web_video_server (which hit archive dependency issues on
this Pi 5 / Ubuntu 24.04 build — see git history around the camera build
scripts). This is the real "live view" that watch_focus.py was only ever a
stopgap for (its own docstring calls it a "poor man's live view").

Requires the camera node (camera_node.py) to already be running and
publishing on /camera/image_raw in another terminal.

Usage:
  python3 live_view_server.py [port]

Defaults to port 8080. Then, from any browser on the same network as the
Pi (e.g. your Mac, if both are on the same Wi-Fi/LAN):
  http://<pi5-ip-address>:8080/

Find the Pi's IP address with:
  hostname -I

Ctrl+C to stop.
"""
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

BOUNDARY = "frame"

# Shared state between the ROS subscriber (runs in the main thread via
# rclpy.spin) and the HTTP server (runs in a background thread) — guarded
# by _lock since both sides touch _latest_jpeg concurrently.
_lock = threading.Lock()
_latest_jpeg = None


class ImageSubscriber(Node):
    """Keeps _latest_jpeg updated with the newest frame, JPEG-encoded."""

    def __init__(self):
        super().__init__("live_view_server")
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, "/camera/image_raw", self._callback, 10
        )
        self.get_logger().info("Subscribed to /camera/image_raw")

    def _callback(self, msg: Image):
        global _latest_jpeg
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        with _lock:
            _latest_jpeg = buf.tobytes()


class StreamHandler(BaseHTTPRequestHandler):
    """Minimal MJPEG server: one HTML page with an <img>, one multipart
    JPEG stream endpoint. No external HTTP framework needed."""

    def log_message(self, fmt, *args):
        pass  # silence per-request access-log spam in the terminal

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._serve_index()
        elif self.path == "/stream":
            self._serve_stream()
        else:
            self.send_error(404)

    def _serve_index(self):
        html = (
            b"<html><head><title>Pi5 Live View</title></head>"
            b"<body style='margin:0;background:#111'>"
            b"<img src='/stream' style='width:100%;height:auto;display:block'>"
            b"</body></html>"
        )
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.send_header("Content-Length", str(len(html)))
        self.end_headers()
        self.wfile.write(html)

    def _serve_stream(self):
        self.send_response(200)
        self.send_header(
            "Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}"
        )
        self.end_headers()
        try:
            while True:
                with _lock:
                    frame = _latest_jpeg
                if frame is not None:
                    self.wfile.write(f"--{BOUNDARY}\r\n".encode())
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                time.sleep(0.1)  # ~10fps ceiling on the stream side
        except (BrokenPipeError, ConnectionResetError):
            pass  # client closed the browser tab / navigated away


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080

    rclpy.init()
    node = ImageSubscriber()

    server = ThreadingHTTPServer(("0.0.0.0", port), StreamHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    node.get_logger().info(f"Live view server running on port {port}")
    node.get_logger().info("Find this Pi's IP with `hostname -I`, then open http://<that-ip>:%d/ in a browser" % port)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
