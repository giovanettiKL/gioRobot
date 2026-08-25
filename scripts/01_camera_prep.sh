#!/usr/bin/env bash
# 01_camera_prep.sh
# Step 1 of 3: install build dependencies + video group, on the Pi itself.
# Run with: ./01_camera_prep.sh
#
# After this finishes, you MUST manually edit /boot/firmware/config.txt
# (see the printed instructions at the end) and reboot before running
# 02_camera_build.sh.

set -euo pipefail

echo "==> Installing build tools"
sudo apt update
sudo apt install -y git pkg-config meson ninja-build cmake python3-dev pybind11-dev

echo "==> Installing libcamera build dependencies"
# Note: deliberately not installing libgnutls28-dev — it pulls in a chain of
# -dev packages (libidn2-dev, libp11-kit-dev, libpcre2-dev, libselinux1-dev,
# nettle-dev) that can hit archive version-skew errors. libcamera's meson
# build falls back to OpenSSL's libcrypto for IPA module signing when GnuTLS
# isn't present, and libssl-dev/openssl below cover that fine.
sudo apt install -y libboost-dev libssl-dev openssl libtiff-dev \
    libglib2.0-dev libgstreamer-plugins-base1.0-dev

echo "==> Installing Python modules needed by libcamera's build system"
sudo apt install -y python3-ply python3-yaml

echo "==> Installing rpicam-apps dependencies"
sudo apt install -y libboost-program-options-dev libexif-dev libavcodec-dev \
    libdrm-dev libjpeg-dev libpng-dev

echo "==> Installing i2c-tools (for hardware-level camera checks)"
sudo apt install -y i2c-tools

echo "==> Adding $USER to the video group"
sudo usermod -aG video "$USER"

cat <<'EOF'

============================================================
Step 1 complete. Two manual steps before running 02_camera_build.sh:

1) Edit the boot config:
     sudo nano /boot/firmware/config.txt

   Find:
     camera_auto_detect=1

   Replace with:
     camera_auto_detect=0
     dtoverlay=imx708,cam0
     dtoverlay=imx477,cam1

   Save (Ctrl+O, Enter) and exit (Ctrl+X).

2) Reboot:
     sudo reboot

After it comes back up, reconnect over SSH and run:
     ./02_camera_build.sh
============================================================
EOF
