#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if ! command -v uv >/dev/null 2>&1; then
    echo 'Install uv first: https://docs.astral.sh/uv/getting-started/installation/'
    exit 1
fi
uv python install 3.12
if [ ! -x .venv/bin/python ]; then
    uv venv --python 3.12 .venv
fi
export UV_HTTP_TIMEOUT="${UV_HTTP_TIMEOUT:-120}"
task_package_index="${UV_DEFAULT_INDEX:-https://pypi.tuna.tsinghua.edu.cn/simple}"
uv pip install --python .venv/bin/python --default-index "$task_package_index" -r requirements-lock.txt
.venv/bin/python -m viewpoint_sim check
echo 'Ready. Replay: .venv/bin/python -m viewpoint_sim replay --result outputs/demo'
