#!/bin/bash

echo "=============================="
echo " AMR Camera Healthcheck"
echo "=============================="
echo

echo "[1] System Info"
echo "Hostname: $(hostname)"
echo "User: $(whoami)"
echo "Date: $(date)"
echo

echo "[2] USB Devices"
lsusb
echo

echo "[3] USB Tree"
lsusb -t
echo

echo "[4] Video Devices"
if ls /dev/video* >/dev/null 2>&1; then
    ls -l /dev/video*
else
    echo "FAIL: No /dev/video* devices found"
fi
echo

echo "[5] V4L2 Devices"
if command -v v4l2-ctl >/dev/null 2>&1; then
    v4l2-ctl --list-devices
else
    echo "WARN: v4l2-ctl not installed"
    echo "Install with: sudo apt install v4l-utils -y"
fi
echo

echo "[6] RealSense SDK Check"
if command -v rs-enumerate-devices >/dev/null 2>&1; then
    rs-enumerate-devices
else
    echo "WARN: rs-enumerate-devices not found"
    echo "Install librealsense2-utils first"
fi
echo

echo "[7] Recent Kernel USB Logs"
sudo dmesg | tail -n 80 | grep -Ei "usb|video|uvc|realsense|intel|disconnect|reset|error|fail" || true
echo

echo "=============================="
echo " Healthcheck Done"
echo "=============================="