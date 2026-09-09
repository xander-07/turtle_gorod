#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEFAULT_SKETCH="$ROOT/firmware/Low_level.ino"
FQBN="${ARDUINO_FQBN:-arduino:avr:uno}"
SKETCH_INPUT="${1:-$DEFAULT_SKETCH}"
PORT_INPUT="${2:-${ARDUINO_PORT:-}}"

if command -v arduino-cli >/dev/null 2>&1; then
  CLI="$(command -v arduino-cli)"
elif [ -x "$HOME/.local/bin/arduino-cli" ]; then
  CLI="$HOME/.local/bin/arduino-cli"
else
  echo "arduino-cli is not installed." >&2
  echo "Run first: cd ~/turtle_gorod && bash scripts/install_arduino_cli.sh" >&2
  exit 1
fi

if [ ! -e "$SKETCH_INPUT" ]; then
  echo "Sketch not found: $SKETCH_INPUT" >&2
  exit 1
fi

if [ -z "$PORT_INPUT" ]; then
  shopt -s nullglob
  CANDIDATES=(/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_*)
  shopt -u nullglob

  if [ "${#CANDIDATES[@]}" -eq 1 ]; then
    PORT_INPUT="${CANDIDATES[0]}"
  elif [ "${#CANDIDATES[@]}" -gt 1 ]; then
    echo "More than one Arduino Uno was found. Specify the port as the second argument:" >&2
    printf '  %s\n' "${CANDIDATES[@]}" >&2
    exit 1
  elif [ -e /dev/ttyACM0 ]; then
    PORT_INPUT="/dev/ttyACM0"
    echo "WARNING: persistent /dev/serial/by-id Arduino name was not found; using /dev/ttyACM0." >&2
  else
    echo "Arduino Uno was not found." >&2
    echo "Check: ls -l /dev/serial/by-id/" >&2
    exit 1
  fi
fi

if [ ! -e "$PORT_INPUT" ]; then
  echo "Serial port does not exist: $PORT_INPUT" >&2
  exit 1
fi

PORT_REAL="$(readlink -f "$PORT_INPUT" 2>/dev/null || printf '%s' "$PORT_INPUT")"

# Do not fight with serial_bridge/another serial monitor for the Uno port.
if command -v fuser >/dev/null 2>&1; then
  BUSY_PIDS="$(fuser "$PORT_REAL" 2>/dev/null || true)"
  if [ -n "$BUSY_PIDS" ]; then
    echo "Arduino serial port is busy: $PORT_INPUT ($PORT_REAL)" >&2
    echo "Processes using it:$BUSY_PIDS" >&2
    echo "Stop 'ros2 launch turtle_gorod base.launch.py' or any serial monitor, then retry." >&2
    exit 2
  fi
fi

if ! groups "$USER" | tr ' ' '\n' | grep -qx dialout; then
  echo "WARNING: $USER is not in the dialout group. Upload may fail with Permission denied." >&2
  echo "Fix once with: sudo usermod -aG dialout $USER && sudo reboot" >&2
fi

if ! "$CLI" core list | awk 'NR > 1 {print $1}' | grep -qx 'arduino:avr'; then
  echo "Arduino AVR core is missing. Installing it..."
  "$CLI" core update-index
  "$CLI" core install arduino:avr
fi

TMP_DIR=""
cleanup() {
  if [ -n "$TMP_DIR" ] && [ -d "$TMP_DIR" ]; then
    rm -rf "$TMP_DIR"
  fi
}
trap cleanup EXIT

if [ -d "$SKETCH_INPUT" ]; then
  SKETCH_DIR="$(cd "$SKETCH_INPUT" && pwd)"
else
  SKETCH_FILE="$(cd "$(dirname "$SKETCH_INPUT")" && pwd)/$(basename "$SKETCH_INPUT")"
  case "$SKETCH_FILE" in
    *.ino) ;;
    *)
      echo "Expected an .ino file or a sketch directory: $SKETCH_FILE" >&2
      exit 1
      ;;
  esac

  SKETCH_NAME="$(basename "$SKETCH_FILE" .ino)"
  TMP_DIR="$(mktemp -d)"
  SKETCH_DIR="$TMP_DIR/$SKETCH_NAME"
  mkdir -p "$SKETCH_DIR"
  cp "$SKETCH_FILE" "$SKETCH_DIR/$SKETCH_NAME.ino"

  # Copy conventional companion source/header files located next to the .ino file.
  SRC_DIR="$(dirname "$SKETCH_FILE")"
  shopt -s nullglob
  for companion in "$SRC_DIR"/*.h "$SRC_DIR"/*.hpp "$SRC_DIR"/*.c "$SRC_DIR"/*.cpp; do
    cp "$companion" "$SKETCH_DIR/"
  done
  shopt -u nullglob
fi

echo "Arduino CLI: $CLI"
echo "Board:       $FQBN"
echo "Port:        $PORT_INPUT -> $PORT_REAL"
echo "Sketch:      $SKETCH_INPUT"
echo
echo "Compiling and uploading..."

"$CLI" compile \
  --fqbn "$FQBN" \
  --port "$PORT_INPUT" \
  --upload \
  --verify \
  "$SKETCH_DIR"

echo
echo "Firmware upload completed successfully."
echo "For the robot lower level, start ROS again with:"
echo "  source ~/turtle_gorod/ros2_ws/install/setup.bash"
echo "  ros2 launch turtle_gorod base.launch.py"
