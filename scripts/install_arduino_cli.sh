#!/usr/bin/env bash
set -euo pipefail

BIN_DIR="${HOME}/.local/bin"
mkdir -p "$BIN_DIR"

if ! command -v curl >/dev/null 2>&1; then
  echo "curl not found; installing it..."
  sudo apt update
  sudo apt install -y curl ca-certificates
fi

if command -v arduino-cli >/dev/null 2>&1; then
  CLI="$(command -v arduino-cli)"
elif [ -x "$BIN_DIR/arduino-cli" ]; then
  CLI="$BIN_DIR/arduino-cli"
else
  ARCH="$(uname -m)"
  case "$ARCH" in
    aarch64|arm64)
      DIST="Linux_ARM64"
      ;;
    armv7l|armv7*)
      DIST="Linux_ARMv7"
      ;;
    x86_64|amd64)
      DIST="Linux_64bit"
      ;;
    i386|i686)
      DIST="Linux_32bit"
      ;;
    *)
      echo "Unsupported architecture for automatic Arduino CLI install: $ARCH" >&2
      exit 1
      ;;
  esac

  URL="https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_${DIST}.tar.gz"
  TMP_DIR="$(mktemp -d)"
  trap 'rm -rf "$TMP_DIR"' EXIT

  echo "Downloading Arduino CLI for $ARCH..."
  curl -fL --retry 3 --connect-timeout 15 "$URL" -o "$TMP_DIR/arduino-cli.tar.gz"
  tar -xzf "$TMP_DIR/arduino-cli.tar.gz" -C "$TMP_DIR"

  if [ ! -f "$TMP_DIR/arduino-cli" ]; then
    echo "arduino-cli binary was not found in the downloaded archive." >&2
    exit 1
  fi

  install -m 0755 "$TMP_DIR/arduino-cli" "$BIN_DIR/arduino-cli"
  CLI="$BIN_DIR/arduino-cli"
fi

# Make ~/.local/bin available in future interactive shells.
PATH_LINE='export PATH="$HOME/.local/bin:$PATH"'
if ! grep -Fqx "$PATH_LINE" "$HOME/.bashrc" 2>/dev/null; then
  printf '\n%s\n' "$PATH_LINE" >> "$HOME/.bashrc"
fi
export PATH="$BIN_DIR:$PATH"

echo
"$CLI" version

echo
echo "Updating Arduino package index..."
"$CLI" core update-index

if "$CLI" core list | awk 'NR > 1 {print $1}' | grep -qx 'arduino:avr'; then
  echo "Arduino AVR core is already installed."
else
  echo "Installing Arduino AVR core..."
  "$CLI" core install arduino:avr
fi

echo
echo "Arduino CLI is ready."
echo "Flash the robot lower level with:"
echo "  cd ~/turtle_gorod && bash scripts/flash_arduino.sh"
