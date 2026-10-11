#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARDUINO_CLI="$HOME/.local/bin/arduino-cli"
FQBN="arduino:avr:uno"

# Auto-detect port if not provided
if [ -n "$1" ]; then
  PORT="$1"
else
  PORT=$("$ARDUINO_CLI" board list 2>/dev/null | grep -i "uno" | awk '{print $1}' | head -n1)
  if [ -z "$PORT" ] || [ ! -e "$PORT" ]; then
    for candidate in /dev/ttyUSB0 /dev/ttyACM0 /dev/ttyUSB* /dev/ttyACM*; do
      if [ -e "$candidate" ]; then
        PORT="$candidate"
        break
      fi
    done
  fi
  if [ -z "$PORT" ]; then
    PORT="/dev/ttyUSB0"
  fi
fi

echo "=== Uploading Robotic Arm Firmware to Arduino Uno ==="
echo "Port: $PORT"
echo "Board: $FQBN"
echo "Firmware path: $DIR/firmware/robotic_arm"

if [ ! -f "$ARDUINO_CLI" ]; then
  ARDUINO_CLI="arduino-cli"
fi

# If dashboard server is running on port 8000, ask it to disconnect before uploading
DASHBOARD_WAS_ONLINE=false
if curl -s -f http://127.0.0.1:8000/api/status >/dev/null 2>&1; then
  echo "Detected running dashboard server; requesting serial port release..."
  curl -s -X POST http://127.0.0.1:8000/api/disconnect >/dev/null 2>&1 || true
  sleep 1.5
  DASHBOARD_WAS_ONLINE=true
fi

"$ARDUINO_CLI" compile --fqbn "$FQBN" "$DIR/firmware/robotic_arm"
"$ARDUINO_CLI" upload -p "$PORT" --fqbn "$FQBN" "$DIR/firmware/robotic_arm"

if [ "$DASHBOARD_WAS_ONLINE" = true ]; then
  echo "Reconnecting dashboard server to $PORT..."
  sleep 2.0
  curl -s -X POST http://127.0.0.1:8000/api/connect -H "Content-Type: application/json" -d "{\"port\":\"$PORT\"}" >/dev/null 2>&1 || true
fi

echo "=== Firmware successfully flashed! ==="

