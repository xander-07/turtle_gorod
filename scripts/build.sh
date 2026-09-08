#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/ros2_ws"

source /opt/ros/humble/setup.bash

cd "$WS"

if command -v rosdep >/dev/null 2>&1; then
  rosdep install --from-paths src --ignore-src -r -y || true
fi

colcon build --symlink-install

echo
echo "Build complete."
echo "Run: source $WS/install/setup.bash"
