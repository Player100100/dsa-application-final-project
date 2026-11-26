# 虚拟机侧脚本（canonical）

这些脚本要放在虚拟机的 `/home/ros/vm/`，`start_sim_stack.ps1` 会调用它们。
换机器/重装时用下面的命令上传：

```powershell
scp -r 03_代码\scripts\vm rmsim:~/ 
ssh rmsim "sed -i 's/\r$//' ~/vm/*.sh ~/vm/*.xml; chmod +x ~/vm/*.sh"
```

> `.sh` 必须是 LF；Windows 检出有时带 CRLF，scp 后会报
> `/bin/bash^M: bad interpreter`。仓库已加 `.gitattributes`。

| 文件 | 作用 |
|---|---|
| `sim_common.sh` | 公共前缀：DISPLAY/XAUTHORITY、ROS 环境、UDP-only FastDDS |
| `sim_start.sh` | stress 运动模型 + physical 传感器，`maze_3_6x6` |
| `sim_ideal.sh` | ideal 运动/传感器模型，`maze_3_6x6` |
| `sim_empty.sh` | ideal，`empty.world` |
| `sim_maze1.sh` | ideal，`maze_1_6x5`（基线世界） |
| `sim_maze1_stress.sh` | stress，`maze_1_6x5` |
| `sim_stress_gt.sh` | stress + ground truth 里程计，用于诊断 |
| `bridge_start.sh` | 启动 rosbridge（**必须 source 工作空间**） |
| `restart_bridge.sh` | 杀掉旧 rosbridge 再启动 |
| `stop_sim.sh` | 杀仿真/桥接/残留进程并清 FastDDS 共享内存 |
| `run_stack.sh` | 原子重启：stop → 确认无残留 → start → 重启 bridge |
| `dyn_box.sdf` | 动态障碍演示用 0.2×0.2×0.4 m 红色箱子 |
| `move_box.sh` / `move_box_empty.sh` | 周期性调用 set_pose 让箱子横穿走廊 |
| `stop_mover.sh` | 停掉搬箱子脚本 |

## 为什么有 fastdds_udp.xml

FastDDS 默认用共享内存；反复热重启会留下端口/文件锁：

```text
RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7411: open_and_lock_file failed
```

表现为 guest 里 `/scan` 正常、rosbridge 订阅后收不到任何数据。
`fastdds_udp.xml` 强制 UDPv4，`sim_common.sh` 与 `bridge_start.sh` 都设了
`FASTRTPS_DEFAULT_PROFILES_FILE`，可绕开这个问题。

## 启动方式（Windows 侧）

```powershell
powershell -ExecutionPolicy Bypass -File 03_代码\scripts\start_sim_stack.ps1 -Maze1
powershell -ExecutionPolicy Bypass -File 03_代码\scripts\start_sim_stack.ps1 -Maze1Stress
powershell -ExecutionPolicy Bypass -File 03_代码\scripts\start_sim_stack.ps1 -Stop
```
