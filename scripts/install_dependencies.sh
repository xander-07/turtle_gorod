#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"

sudo apt update
sudo apt install -y \
  git cmake build-essential python3-pip python3-serial \
  python3-colcon-common-extensions python3-rosdep \
  "ros-${ROS_DISTRO}-navigation2" \
  "ros-${ROS_DISTRO}-nav2-bringup" \
  "ros-${ROS_DISTRO}-slam-toolbox" \
  "ros-${ROS_DISTRO}-teleop-twist-keyboard"

if ! groups "$USER" | grep -qw dialout; then
  sudo usermod -aG dialout "$USER"
  echo "Added $USER to dialout. Log out/in before using serial devices."
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/ros2_ws"

# Install YDLidar SDK if it is not present.
if ! ldconfig -p 2>/dev/null | grep -q ydlidar_sdk; then
  SDK_DIR="$HOME/YDLidar-SDK"
  if [ ! -d "$SDK_DIR/.git" ]; then
    git clone https://github.com/YDLIDAR/YDLidar-SDK.git "$SDK_DIR"
  else
    git -C "$SDK_DIR" pull --ff-only
  fi
  cmake -S "$SDK_DIR" -B "$SDK_DIR/build"
  cmake --build "$SDK_DIR/build" -j"$(nproc)"
  sudo cmake --install "$SDK_DIR/build"
  sudo ldconfig
fi

# The upstream driver has a dedicated Humble branch. Use it on Ubuntu 22.04/ROS 2 Humble.
DRIVER_DIR="$WS/src/ydlidar_ros2_driver"
if [ ! -d "$DRIVER_DIR/.git" ]; then
  git clone --branch humble --single-branch \
    https://github.com/YDLIDAR/ydlidar_ros2_driver.git "$DRIVER_DIR"
fi

echo
echo "Dependencies installed."
echo "Next: bash scripts/build.sh"
