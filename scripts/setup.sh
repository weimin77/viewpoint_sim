#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if ! command -v uv >/dev/null 2>&1; then
    echo 'Please install uv first: https://docs.astral.sh/uv/getting-started/installation/'
    exit 1
fi
uv python install 3.11
if [ ! -x .venv/bin/python ]; then
    uv venv --python 3.11 .venv
fi
uv pip install --python .venv/bin/python -r requirements-lock.txt
.venv/bin/python -m viewpoint_sim check
echo 'Ready. Run: .venv/bin/python -m viewpoint_sim run --headless --output outputs/my_run'
