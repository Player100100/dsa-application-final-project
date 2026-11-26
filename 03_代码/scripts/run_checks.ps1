# 一键回归：单元测试 + 基准 + mock 导航冒烟
# 用法：powershell -ExecutionPolicy Bypass -File 03_代码\scripts\run_checks.ps1
#      -SkipNav  只跑测试与基准

param(
    [switch]$SkipNav
)

$ErrorActionPreference = "Stop"
$CodeRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$ProjectRoot = Resolve-Path (Join-Path $CodeRoot "..")
$Python = Join-Path $ProjectRoot ".venv\Scripts\python.exe"

if (-not (Test-Path $Python)) {
    throw "找不到虚拟环境，请先运行 scripts\setup_env.ps1"
}

Push-Location $CodeRoot
try {
    Write-Host "== 1. 单元测试 =="
    & $Python -m unittest discover -s tests -t .
    if ($LASTEXITCODE -ne 0) { throw "单元测试失败" }

    Write-Host "== 2. 性能基准 =="
    & $Python demo\bench.py --save

    if (-not $SkipNav) {
        Write-Host "== 3. mock 导航冒烟（目标 2.0 2.0）=="
        & $Python demo\nav_demo.py --ip mock --goal 2.0 2.0 --duration 40
        if ($LASTEXITCODE -ne 0) { throw "mock 导航未在时限内到达" }
    }
    Write-Host "全部通过。"
}
finally {
    Pop-Location
}
