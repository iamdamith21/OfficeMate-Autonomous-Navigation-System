#!/usr/bin/env bash
set -euo pipefail
CLI="$HOME/.local/bin/arduino-cli"
if [ ! -x "$CLI" ]; then
  echo ">> installing arduino-cli"
  mkdir -p "$HOME/.local/bin"
  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR="$HOME/.local/bin" sh
fi
"$CLI" version
if ! "$CLI" core list 2>/dev/null | grep -q "arduino:avr"; then
  echo ">> installing AVR core"
  "$CLI" core update-index
  "$CLI" core install arduino:avr
fi
"$CLI" core list
echo ">> TOOLCHAIN OK"
