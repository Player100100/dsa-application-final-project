# 启动仿真虚拟机并查 IP（Windows PowerShell）
# 用法：
#   powershell -ExecutionPolicy Bypass -File scripts\start_sim_vm.ps1
#   ... -Stop        正常关机
#   ... -Gui         有界面启动（默认 nogui）
#   ... -Vmx "D:\...\xxx.vmx"

param(
    [string]$Vmx = "F:\数据结构与算法应用\仿真器等2项文件\仿真器\VM\ros2-masterX3\ros2-masterX3.vmx",
    [switch]$Stop,
    [switch]$Gui,
    [int]$TimeoutSec = 150
)

$ErrorActionPreference = "Stop"
$Vmrun = "C:\Program Files (x86)\VMware\VMware Workstation\vmrun.exe"
$CodeRoot = Resolve-Path (Join-Path $PSScriptRoot "..")

if (-not (Test-Path $Vmrun)) { throw "找不到 vmrun：$Vmrun" }
if (-not (Test-Path $Vmx)) { throw "找不到虚拟机：$Vmx" }

if ($Stop) {
    Write-Host "正在正常关机 ..."
    & $Vmrun -T ws stop $Vmx soft
    exit $LASTEXITCODE
}

$running = (& $Vmrun list) -match [regex]::Escape($Vmx)
if (-not $running) {
    $mode = if ($Gui) { "gui" } else { "nogui" }
    Write-Host "启动虚拟机（$mode）..."
    & $Vmrun -T ws start $Vmx $mode
} else {
    Write-Host "虚拟机已在运行。"
}

Write-Host "等待 DHCP 租约（最多 $TimeoutSec 秒）..."
$deadline = (Get-Date).AddSeconds($TimeoutSec)
$lastOutput = ""
do {
    $lastOutput = & python (Join-Path $CodeRoot "scripts\find_vm_ip.py") --vmx $Vmx 2>&1
    if ($LASTEXITCODE -eq 0) { break }
    Start-Sleep -Seconds 5
} while ((Get-Date) -lt $deadline)

$lastOutput
if ($LASTEXITCODE -ne 0) {
    Write-Host "还没拿到 IP。虚拟机可能仍在开机，或网卡未接 NAT。" -ForegroundColor Yellow
    exit 1
}

Write-Host ""
Write-Host "下一步：" -ForegroundColor Green
Write-Host "  python scripts\check_sim.py --ip <上面的 IP>"
