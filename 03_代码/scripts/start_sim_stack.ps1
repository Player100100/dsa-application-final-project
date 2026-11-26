# 一键启动/停止仿真栈：VMware 虚拟机 + Gazebo + rosbridge
# 用法（在 03_代码 下）：
#   powershell -ExecutionPolicy Bypass -File scripts\start_sim_stack.ps1
#   ... -Ideal        用 ideal 运动/传感器模型
#   ... -Empty        用 empty.world（验证窄口是否 maze 特有）
#   ... -Stop         停仿真并正常关机
#
# 前提：虚拟机已导入并配置好 ssh 别名 rmsim（见 00_任务与输入/仿真环境接入.md）

param(
    [string]$Vmx = "F:\数据结构与算法应用\仿真器等2项文件\仿真器\VM\ros2-masterX3\ros2-masterX3.vmx",
    [switch]$Ideal,
    [switch]$Empty,
    [switch]$Maze1,
    [switch]$Maze1Stress,
    [switch]$Stop,
    [int]$TimeoutSec = 180
)

$ErrorActionPreference = "Stop"
$Vmrun = "C:\Program Files (x86)\VMware\VMware Workstation\vmrun.exe"
$CodeRoot = Resolve-Path (Join-Path $PSScriptRoot "..")

if (-not (Test-Path $Vmrun)) { throw "找不到 vmrun：$Vmrun" }
if (-not (Test-Path $Vmx)) { throw "找不到虚拟机：$Vmx" }

if ($Stop) {
    Write-Host "停止仿真进程并关机 ..."
    ssh rmsim "~/vm/clean_all.sh" 2>$null | Out-Null
    & $Vmrun -T ws stop $Vmx soft
    exit $LASTEXITCODE
}

$running = (& $Vmrun list) -match [regex]::Escape($Vmx)
if (-not $running) {
    Write-Host "启动虚拟机（nogui）..."
    & $Vmrun -T ws start $Vmx nogui
} else {
    Write-Host "虚拟机已在运行。"
}

Write-Host "等待虚拟机 IP ..."
$ip = $null
$deadline = (Get-Date).AddSeconds($TimeoutSec)
while ((Get-Date) -lt $deadline) {
    $out = & python (Join-Path $CodeRoot "scripts\find_vm_ip.py") --vmx $Vmx 2>&1
    if ($LASTEXITCODE -eq 0) {
        $m = [regex]::Match(($out -join "`n"), "\b(\d{1,3}\.){3}\d{1,3}\b")
        if ($m.Success) { $ip = $m.Value; break }
    }
    Start-Sleep -Seconds 5
}
if (-not $ip) { throw "没等到 DHCP 租约" }
Write-Host "虚拟机 IP：$ip"

Write-Host "等待 SSH ..."
$sshOk = $false
$deadline = (Get-Date).AddSeconds($TimeoutSec)
$oldEap = $ErrorActionPreference
$ErrorActionPreference = "SilentlyContinue"
while ((Get-Date) -lt $deadline) {
    ssh -o ConnectTimeout=5 -o BatchMode=yes rmsim "true" 2>$null
    if ($LASTEXITCODE -eq 0) { $sshOk = $true; break }
    Start-Sleep -Seconds 5
}
$ErrorActionPreference = $oldEap
if (-not $sshOk) { throw "SSH 连不上 rmsim（检查 ~/.ssh/config 与虚拟机 ssh 服务）" }

$simScript = if ($Ideal) { "sim_ideal.sh" } elseif ($Empty) { "sim_empty.sh" } elseif ($Maze1) { "sim_maze1.sh" } elseif ($Maze1Stress) { "sim_maze1_stress.sh" } else { "sim_start.sh" }
Write-Host "配置：Ideal=$Ideal Empty=$Empty Maze1=$Maze1 Maze1Stress=$Maze1Stress -> $simScript"
Write-Host "启动仿真（$simScript）与 rosbridge ..."
ssh rmsim "~/vm/run_stack.sh $simScript"

Write-Host "等待 rosbridge 9090 ..."
$portOk = $false
$deadline = (Get-Date).AddSeconds($TimeoutSec)
while ((Get-Date) -lt $deadline) {
    if (Test-NetConnection $ip -Port 9090 -InformationLevel Quiet -WarningAction SilentlyContinue) {
        $portOk = $true
        break
    }
    Start-Sleep -Seconds 5
}
if (-not $portOk) {
    Write-Host "9090 还没起来，查看日志：" -ForegroundColor Yellow
    Write-Host "  ssh rmsim 'tail -50 ~/sim.log; tail -20 ~/bridge.log'"
    exit 1
}

Write-Host "等待 /scan 与 /odom 真正有数据（最多 120 s）..."
$ready = $false
$deadline = (Get-Date).AddSeconds(120)
$oldEap = $ErrorActionPreference
$ErrorActionPreference = "SilentlyContinue"
while ((Get-Date) -lt $deadline) {
    ssh rmsim "source /opt/ros/humble/setup.bash; source ~/Desktop/virtual_car/install/setup.bash; timeout 4 ros2 topic echo --once /scan --field ranges >/dev/null 2>&1 && timeout 4 ros2 topic echo --once /odom --field pose.pose.position >/dev/null 2>&1" 2>$null
    if ($LASTEXITCODE -eq 0) { $ready = $true; break }
    Start-Sleep -Seconds 5
}
$ErrorActionPreference = $oldEap
if (-not $ready) {
    Write-Host "/scan 或 /odom 在 120 s 内没有数据，检查 ~/sim.log（常见：Gazebo 渲染未初始化）" -ForegroundColor Yellow
    exit 1
}

Write-Host ""
Write-Host "就绪。建议先预检：" -ForegroundColor Green
Write-Host "  ..\.venv\Scripts\python scripts\check_sim.py --ip $ip"
Write-Host "然后导航："
Write-Host "  ..\.venv\Scripts\python demo\nav_demo.py --ip $ip --goal 1.5 0 --speed 0.15 --inflate 0.15 --stop-distance 0.18 --cluster-slow-distance 0.30 --lookahead 0.30"
