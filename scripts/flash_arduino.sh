#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEFAULT_SKETCH="$ROOT/firmware/Low_level.ino"
SKETCH_INPUT="${1:-$DEFAULT_SKETCH}"
PORT_INPUT="${2:-${ARDUINO_PORT:-}}"

if [ ! -f /usr/share/arduino/Arduino.mk ]; then
  echo "Arduino terminal toolchain is not installed." >&2
  echo "Run first: cd ~/turtle_gorod && bash scripts/install_arduino_cli.sh" >&2
  exit 1
fi

if ! command -v make >/dev/null 2>&1 || ! command -v avrdude >/dev/null 2>&1; then
  echo "Arduino build/upload tools are incomplete." >&2
  echo "Run: bash scripts/install_arduino_cli.sh" >&2
  exit 1
fi

AVRDUDE_BIN="$(command -v avrdude)"

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
    echo "WARNING: persistent Arduino name was not found; using /dev/ttyACM0." >&2
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

# Do not fight with serial_bridge/serial monitor for the Uno port.
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
  echo "WARNING: $USER is not in the dialout group. Upload may fail." >&2
  echo "Fix once with: sudo usermod -aG dialout $USER && sudo reboot" >&2
fi

TMP_DIR="$(mktemp -d)"
cleanup() {
  rm -rf "$TMP_DIR"
}
trap cleanup EXIT

if [ -d "$SKETCH_INPUT" ]; then
  SOURCE_DIR="$(cd "$SKETCH_INPUT" && pwd)"
  shopt -s nullglob
  INO_FILES=("$SOURCE_DIR"/*.ino)
  shopt -u nullglob
  if [ "${#INO_FILES[@]}" -ne 1 ]; then
    echo "Sketch directory must contain exactly one .ino file: $SOURCE_DIR" >&2
    exit 1
  fi
  SKETCH_NAME="$(basename "${INO_FILES[0]}" .ino)"
else
  SKETCH_FILE="$(cd "$(dirname "$SKETCH_INPUT")" && pwd)/$(basename "$SKETCH_INPUT")"
  case "$SKETCH_FILE" in
    *.ino) ;;
    *)
      echo "Expected an .ino file or a sketch directory: $SKETCH_FILE" >&2
      exit 1
      ;;
  esac
  SOURCE_DIR="$(dirname "$SKETCH_FILE")"
  SKETCH_NAME="$(basename "$SKETCH_FILE" .ino)"
fi

SKETCH_DIR="$TMP_DIR/$SKETCH_NAME"
mkdir -p "$SKETCH_DIR"

if [ -d "$SKETCH_INPUT" ]; then
  cp "${INO_FILES[0]}" "$SKETCH_DIR/$SKETCH_NAME.ino"
else
  cp "$SKETCH_FILE" "$SKETCH_DIR/$SKETCH_NAME.ino"
fi

# Copy conventional companion source/header files located next to the sketch.
shopt -s nullglob
for companion in "$SOURCE_DIR"/*.h "$SOURCE_DIR"/*.hpp "$SOURCE_DIR"/*.c "$SOURCE_DIR"/*.cpp; do
  cp "$companion" "$SKETCH_DIR/"
done
shopt -u nullglob

cat > "$SKETCH_DIR/Makefile" <<EOF
ARDUINO_DIR = /usr/share/arduino
ARDMK_DIR = /usr/share/arduino
BOARD_TAG = uno
MONITOR_PORT = $PORT_REAL
include /usr/share/arduino/Arduino.mk
EOF

echo "Backend:     Arduino-Makefile + system avrdude"
echo "Board:       Arduino Uno"
echo "Port:        $PORT_INPUT -> $PORT_REAL"
echo "Sketch:      $SKETCH_INPUT"
echo "avrdude:     $AVRDUDE_BIN"
echo
echo "Compiling..."

# Only build with Arduino-Makefile. Its Ubuntu 22.04 upload rule may point to a
# non-existent bundled avrdude.conf, so uploading is performed separately with
# the distro's /usr/bin/avrdude, which uses the valid system configuration.
make -C "$SKETCH_DIR"

HEX_FILE="$(find "$SKETCH_DIR" -type f -path '*/build-*/*.hex' ! -name '*.with_bootloader.hex' | head -n 1)"
if [ -z "$HEX_FILE" ] || [ ! -f "$HEX_FILE" ]; then
  echo "Compiled .hex file was not found." >&2
  find "$SKETCH_DIR" -maxdepth 3 -type f -name '*.hex' -print >&2 || true
  exit 1
fi

echo
echo "Uploading: $HEX_FILE"

# Arduino Uno bootloader uses the STK500/Arduino protocol at 115200 baud.
# Do not pass -C here: the system avrdude automatically uses its installed
# configuration (normally /etc/avrdude.conf). Verification remains enabled.
"$AVRDUDE_BIN" \
  -p atmega328p \
  -c arduino \
  -P "$PORT_REAL" \
  -b 115200 \
  -D \
  -U "flash:w:$HEX_FILE:i"

echo
echo "Firmware upload completed successfully."
echo "Start ROS again with:"
echo "  source ~/turtle_gorod/ros2_ws/install/setup.bash"
echo "  ros2 launch turtle_gorod base.launch.py"
