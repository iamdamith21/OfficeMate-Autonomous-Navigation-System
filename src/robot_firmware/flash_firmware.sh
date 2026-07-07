#!/usr/bin/env bash
# flash_firmware.sh — compile & upload robot_firmware to the Mega.
#
# Run on whichever machine the Mega is plugged into (normally the Pi):
#   ./flash_firmware.sh [port]        default port: /dev/arduino
#
# First run bootstraps arduino-cli + AVR core + libraries into ~/.local.
# IMPORTANT: stop the arduino_bridge node first — the upload needs
# exclusive access to the serial port.
set -euo pipefail

PORT="${1:-/dev/arduino}"
SKETCH_DIR="$(cd "$(dirname "$0")/arduino/robot_firmware" && pwd)"
CLI="$HOME/.local/bin/arduino-cli"

if [ ! -x "$CLI" ]; then
    echo ">> Installing arduino-cli to ~/.local/bin ..."
    mkdir -p "$HOME/.local/bin"
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
        | BINDIR="$HOME/.local/bin" sh
fi

if ! "$CLI" core list | grep -q arduino:avr; then
    echo ">> Installing AVR core ..."
    "$CLI" core update-index
    "$CLI" core install arduino:avr
fi

for lib in Servo "LiquidCrystal I2C" MFRC522; do
    "$CLI" lib list | grep -qi "^${lib}" || "$CLI" lib install "$lib"
done

echo ">> Compiling ..."
"$CLI" compile --fqbn arduino:avr:mega "$SKETCH_DIR"

echo ">> Uploading to $PORT ..."
"$CLI" upload --fqbn arduino:avr:mega -p "$PORT" "$SKETCH_DIR"

echo ">> Done. Watch the boot banner with:"
echo "   python3 -c \"import serial;s=serial.Serial('$PORT',115200,timeout=5);[print(s.readline().decode(errors='replace'),end='') for _ in range(20)]\""
