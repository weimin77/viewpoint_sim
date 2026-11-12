$ErrorActionPreference = "Stop"
Set-Location (Join-Path $PSScriptRoot "..")
if (-not (Get-Command uv -ErrorAction SilentlyContinue)) { throw "Install uv: https://docs.astral.sh/uv/getting-started/installation/" }
uv python install 3.12
if ($LASTEXITCODE -ne 0) { throw "Python installation failed" }
if (-not (Test-Path .venv/Scripts/python.exe)) {
    uv venv --python 3.12 .venv
    if ($LASTEXITCODE -ne 0) { throw "Virtual environment creation failed" }
}
if (-not $env:UV_HTTP_TIMEOUT) { $env:UV_HTTP_TIMEOUT = "120" }
$taskPackageIndex = if ($env:UV_DEFAULT_INDEX) { $env:UV_DEFAULT_INDEX } else { "https://pypi.tuna.tsinghua.edu.cn/simple" }
uv pip install --python .venv/Scripts/python.exe --default-index $taskPackageIndex -r requirements-lock.txt
if ($LASTEXITCODE -ne 0) { throw "Dependency installation failed" }
.venv/Scripts/python.exe -m viewpoint_sim check
if ($LASTEXITCODE -ne 0) { throw "Model check failed" }
Write-Output 'Ready. Replay: .venv/Scripts/python.exe -m viewpoint_sim replay --result outputs/demo'
