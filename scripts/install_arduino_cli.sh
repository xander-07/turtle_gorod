#!/usr/bin/env bash
set -euo pipefail

# Ubuntu 22.04/Jammy does not ship arduino-cli, and the MIREA network
# intercepts external HTTPS with a self-signed certificate. To keep firmware
# flashing reliable from SSH, use Ubuntu-packaged AVR tools only.
#
# Installed backend:
#   arduino-mk + arduino-core-avr + avrdude + gcc-avr

if [ "${EUID:-$(id -u)}" -eq 0 ]; then
  echo "Run this script as the normal user, not with sudo." >&2
  exit 1
fi

echo "Installing Arduino terminal toolchain from Ubuntu repositories..."
sudo apt update
sudo apt install -y \
  arduino-mk \
  arduino-core-avr \
  avrdude \
  avr-libc \
  gcc-avr \
  make

if [ ! -f /usr/share/arduino/Arduino.mk ]; then
  echo "Arduino.mk was not installed at /usr/share/arduino/Arduino.mk" >&2
  exit 1
fi

if ! command -v avrdude >/dev/null 2>&1; then
  echo "avrdude was not installed correctly." >&2
  exit 1
fi

if ! command -v avr-g++ >/dev/null 2>&1; then
  echo "avr-g++ was not installed correctly." >&2
  exit 1
fi

if ! groups "$USER" | tr ' ' '\n' | grep -qx dialout; then
  sudo usermod -aG dialout "$USER"
  echo
  echo "Added $USER to dialout. Reboot once before flashing: sudo reboot"
fi

echo
echo "Arduino terminal toolchain is ready."
echo "Flash the robot firmware with:"
echo "  cd ~/turtle_gorod"
echo "  bash scripts/flash_arduino.sh"
