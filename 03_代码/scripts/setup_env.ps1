# 建立项目虚拟环境并安装厂商 SDK（Windows PowerShell）
# 用法：在项目根目录执行
#   powershell -ExecutionPolicy Bypass -File 03_代码\scripts\setup_env.ps1

$ErrorActionPreference = "Stop"

$ProjectRoot = Resolve-Path (Join-Path $PSScriptRoot "..\..")
$WorkspaceRoot = Resolve-Path (Join-Path $ProjectRoot "..\..")
$Wheel = Join-Path $WorkspaceRoot "用户传入\数据结构与算法应用\客户端v0.3\rosmaster_client-0.3.0-py3-none-any.whl"

if (-not (Test-Path $Wheel)) {
    throw "找不到 SDK wheel：$Wheel"
}

$Venv = Join-Path $ProjectRoot ".venv"
if (-not (Test-Path $Venv)) {
    Write-Host "[1/3] 创建虚拟环境 $Venv"
    python -m venv $Venv
} else {
    Write-Host "[1/3] 复用已有虚拟环境 $Venv"
}

$Python = Join-Path $Venv "Scripts\python.exe"
Write-Host "[2/3] 安装 SDK 与地图可视化依赖"
& $Python -m pip install --quiet --disable-pip-version-check $Wheel
& $Python -m pip install --quiet --disable-pip-version-check PyYAML matplotlib

Write-Host "[3/3] 自检"
& $Python -c "import roslibpy, yaml, matplotlib, rosmaster_client; print('依赖 OK：roslibpy', roslibpy.__version__)"

Write-Host ""
Write-Host "完成。激活环境："
Write-Host "  $Venv\Scripts\Activate.ps1"
