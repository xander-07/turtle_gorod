#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/ros2_ws"

# ROS 2 Humble setup scripts may reference variables that are not defined yet.
# Keep nounset disabled while sourcing ROS, then enable it for our own script.
source /opt/ros/humble/setup.bash
set -u

cd "$WS"

if command -v rosdep >/dev/null 2>&1; then
  rosdep install --from-paths src --ignore-src -r -y || true
fi

colcon build --symlink-install

echo
echo "Build complete."
echo "Run: source $WS/install/setup.bash"
