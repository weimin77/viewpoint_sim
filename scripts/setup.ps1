$ErrorActionPreference = "Stop"
Set-Location (Join-Path $PSScriptRoot "..")
if (-not (Get-Command uv -ErrorAction SilentlyContinue)) {
    throw "Install uv first: https://docs.astral.sh/uv/getting-started/installation/"
}
uv python install 3.11
if ($LASTEXITCODE -ne 0) { throw "Python install failed" }
if (-not (Test-Path .venv/Scripts/python.exe)) {
    uv venv --python 3.11 .venv
    if ($LASTEXITCODE -ne 0) { throw "Environment creation failed" }
}
uv pip install --python .venv/Scripts/python.exe -r requirements-lock.txt
if ($LASTEXITCODE -ne 0) { throw "Dependency install failed" }
.venv/Scripts/python.exe -m viewpoint_sim check
if ($LASTEXITCODE -ne 0) { throw "Initial configuration check failed" }
Write-Output 'Ready. Run: .venv/Scripts/python.exe -m viewpoint_sim run --gui --output outputs/my_run'
