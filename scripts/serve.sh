#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
port="${1:-8765}"
echo "Open http://127.0.0.1:$port/web/"
exec .venv/bin/python -m http.server "$port" --bind 127.0.0.1
