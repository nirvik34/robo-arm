#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_PYTHON="$DIR/venv/bin/python3"
PORT=8000

if [ ! -f "$VENV_PYTHON" ]; then
  echo "Virtualenv not found, using system python3..."
  VENV_PYTHON="python3"
fi

echo "=== Starting Robotic Arm Controller Dashboard ==="
echo "Access Dashboard at: http://localhost:$PORT"
echo "Press Ctrl+C to terminate."

cd "$DIR/server"
exec "$VENV_PYTHON" -m uvicorn app:app --host 0.0.0.0 --port $PORT --reload
