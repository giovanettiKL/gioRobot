#!/usr/bin/env bash
# 02_camera_build.sh
# Step 2 of 3: build Raspberry Pi's libcamera fork (with rpi/vc4 + rpi/pisp
# pipeline handlers and Python bindings) and rpicam-apps, from source.
#
# Run this AFTER 01_camera_prep.sh, AFTER editing /boot/firmware/config.txt,
# and AFTER rebooting. Takes roughly 15-20 minutes on a Pi 5.
#
# Run with: ./02_camera_build.sh

set -euo pipefail

WORKDIR="$HOME/camera_build"
mkdir -p "$WORKDIR"
cd "$WORKDIR"

echo "==> Cloning Raspberry Pi's libcamera fork"
if [ -d libcamera ]; then
  echo "    libcamera/ already exists, pulling latest instead"
  (cd libcamera && git pull)
else
  git clone https://github.com/raspberrypi/libcamera.git
fi

cd libcamera
echo "==> Configuring libcamera build (rpi/vc4 + rpi/pisp pipelines, Python bindings)"
meson setup build --buildtype=release \
  -Dpipelines=rpi/vc4,rpi/pisp \
  -Dipas=rpi/vc4,rpi/pisp \
  -Dv4l2=enabled -Dgstreamer=enabled \
  -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled \
  -Ddocumentation=disabled -Dpycamera=enabled \
  --reconfigure 2>/dev/null || \
meson setup build --buildtype=release \
  -Dpipelines=rpi/vc4,rpi/pisp \
  -Dipas=rpi/vc4,rpi/pisp \
  -Dv4l2=enabled -Dgstreamer=enabled \
  -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled \
  -Ddocumentation=disabled -Dpycamera=enabled

echo "==> Building libcamera (this is the slow part)"
ninja -C build

echo "==> Installing libcamera"
sudo ninja -C build install
sudo ldconfig
cd "$WORKDIR"

echo "==> Cloning rpicam-apps"
if [ -d rpicam-apps ]; then
  echo "    rpicam-apps/ already exists, pulling latest instead"
  (cd rpicam-apps && git pull)
else
  git clone https://github.com/raspberrypi/rpicam-apps.git
fi

cd rpicam-apps
echo "==> Configuring and building rpicam-apps"
meson setup build --buildtype=release --reconfigure 2>/dev/null || \
meson setup build --buildtype=release
ninja -C build

echo "==> Installing rpicam-apps"
sudo ninja -C build install
sudo ldconfig
cd "$WORKDIR"

cat <<'EOF'

============================================================
Build complete. Verifying camera detection now:
============================================================
EOF

rpicam-hello --list-cameras || true

cat <<'EOF'

============================================================
If both cameras (imx708 and imx477) are listed above, you're done.

If not, check:
  sudo i2cdetect -y 10   # cam0 - expect a device address
  sudo i2cdetect -y 11   # cam1 - expect a device address
  sudo dmesg | grep -iE "imx708|imx477"
  groups                 # confirm 'video' is listed (log out/in if not)

Once cameras are confirmed working with rpicam-hello, retest with ROS2:
  python3 -c "from picamera2 import Picamera2; import pprint; pprint.pprint(Picamera2.global_camera_info())"
============================================================
EOF
