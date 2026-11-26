# AGENTS.md — AI 工作台规则

本文件是给 AI（以及未来的你）看的操作手册。**每次会话先读完这四份再动手：**

1. 本文件
2. [README.md](README.md) — 现状与命令
3. [06_进度/看板.md](06_进度/看板.md) — 哪些在做、哪些卡住
4. [06_进度/下一步.md](06_进度/下一步.md) — 当前优先级最高的动作

如果任务涉及具体模块，再读 `02_设计/模块规格_*.md`。

---

## 1. 项目边界（不可越线）

| 能做 | 不能做 |
|---|---|
| 改 `03_代码/` 下的所有文件 | 改 `用户传入/` 里的厂商 SDK 或任务书 |
| 追加实验记录、更新看板 | 覆盖历史实验记录、改写已验收结论 |
| 为联调写新的适配层 | 把 numpy/scipy 引入核心算法 |
| 在 mock 上随意跑 | 未经确认向真车发运动指令 |

- `03_代码/ds/` 是**交付物本体**，只用 Python 标准库。
- `03_代码/robot_io.py` 是唯一允许 import `rosmaster_client` 的地方。
- `04_实验/` 只增不改；文件名格式 `YYYY-MM-DD_主题.md`。
- 地图交付物只能由 `GridMap.save()` 生成。

## 2. 每次代码改动的最低验收

```powershell
cd 03_代码
..\.venv\Scripts\python -m unittest discover -s tests -t .
..\.venv\Scripts\python demo\nav_demo.py --ip mock --goal 2.0 2.0 --duration 60
```

- 40 个单测必须全过；
- 导航必须到达（退出码 0）；
- 新增算法/数据结构必须同时补单测；
- 性能相关改动要在 `04_实验/` 追加一行对比数据，不能只写“更快了”。

## 3. 会话结束的交接动作（强制）

按顺序更新三份文件，缺一不可：

1. `06_进度/工作日志.md`：**追加**一条，写清做了什么、为什么、验证结果、遗留问题；
2. `06_进度/看板.md`：移动卡片状态；
3. `06_进度/下一步.md`：重写（不是追加）成下一次会话可直接执行的 3–7 条。

交接的标准：一个不了解本次对话的人，只看这三份文件就能继续干。

## 4. 工作习惯

- **先证据后结论**：说“更快/更稳”必须贴命令输出或实验文件名。
- **区分三类数据**：合成基准、mock、真车/仿真，报告里分表，不许混。
- **保持可回退**：大的算法替换先在 `demo/` 里做对照，别直接改 `ds/`。
- **安全优先**：真车脚本必须有 `try/finally: stop()`；新动作先在 mock 验证。
- **中文文档、中文注释**；代码标识符用英文；类型注解写全。

## 5. 常用命令

| 目的 | 命令（在 `03_代码/` 下） |
|---|---|
| 单测 | `..\.venv\Scripts\python -m unittest discover -s tests -t . -v` |
| 基准（合成） | `..\.venv\Scripts\python demo\bench.py --save` |
| 基准（真实地图） | `..\.venv\Scripts\python demo\bench_real_map.py --save` |
| 聚类参数（真实点云） | `..\.venv\Scripts\python demo\bench_cluster_real.py --ip <IP> --save` |
| mock 导航 | `..\.venv\Scripts\python demo\nav_demo.py --ip mock --goal 2 2` |
| 链路预检（VM/真车） | `..\.venv\Scripts\python scripts\check_sim.py --ip <IP>` |
| 查 NAT 虚拟机 IP | `python scripts\find_vm_ip.py --vmx <VMX路径>` |
| 一键启动仿真栈 | `powershell -ExecutionPolicy Bypass -File scripts\start_sim_stack.ps1`（`-Ideal` / `-Empty` / `-Stop`） |
| 仿真导航（窄道参数） | `..\.venv\Scripts\python demo\nav_demo.py --ip 192.168.150.131 --goal 1.5 0 --speed 0.15 --inflate 0.15 --stop-distance 0.18 --cluster-slow-distance 0.30 --lookahead 0.30` |
| 真车自检 | `..\.venv\Scripts\python ..\..\..\用户传入\...\examples\00_自检.py --ip <IP>` |
| 一键回归 | `powershell -ExecutionPolicy Bypass -File scripts\run_checks.ps1` |

## 6. 已知坑（踩过，别再踩）

- 雷达倒装 180°，方位角必须叠加 `get_transform` 查到的偏航；
- 车体贴墙时可能落进规划膨胀带 → A* 无解。`nav_demo` 会逐级降低膨胀半径重试；
- 聚类阈值过紧会把“墙”判成障碍簇并让车原地打转；簇只用于减速，硬停交给
  `front_range` 阈值 + 转向保持；
- mock 房间 6×6 m、墙在 ±3 m；目标点别超出这张图；
- `get_scan()` 只在有新帧时返回，循环里 `None` 是正常现象，不要当断连；
- 仿真 VM 的 `virtualhw.version` 必须是 `21`（ovftool 会写成 99，Workstation 25 拒绝启动）；
- SSH 启动 Gazebo 必须设 `DISPLAY=:0` 和 Xwayland 的 `XAUTHORITY`，否则 `/scan` 全是 NaN；
- rosbridge 启动前必须 `source ~/Desktop/virtual_car/install/setup.bash`，否则 TF 服务导入失败；
- 仿真迷宫是窄道（离墙约 0.15–0.3 m）：默认 0.35 m 硬停会原地摆头，
  用 `--stop-distance 0.18 --cluster-slow-distance 0.30 --lookahead 0.30`；
- **目标点只从 live map 选**：`maps/maps/*.yaml` 与 Gazebo world 在局部不一致
  （实测 (2,2) 静态图空闲、实际是墙）；已验证可达点 `(1.5,0)`、`(1,2)`、`(0,2)`；
- `/scan` 中继跑久会停（`/internal/scan` 仍发布）；每批实验前重启
  `start_sim_stack.ps1`，不要一次仿真连跑很久；
- SDK 的 `RobotClient.is_connected` 是 **property**，别加括号；
- Gazebo Fortress 的服务类型是 **`ignition.msgs.*`**（不是 `gz.msgs.*`）；
  动态箱子用 `ros_gz_sim create -world <实际 world 名>` 生成，world 名可从
  `ign service -l | grep set_pose` 查（maze_1 是 `maze_6x5_v1`，空世界是 `empty_world`）；
- 在 Git Bash 里给 `--odom-topic /ground_truth/odom` 这类参数要加
  `MSYS_NO_PATHCONV=1`，否则路径会被转成 `D:/Git/...`（PowerShell 无此问题）；
- `maze_3_6x6` 的第一个窄口（约 (0.4,0.34)）连 ideal 都过不去，换 ground truth
  里程计也一样 → 车体/窄口几何问题；参数扫描无效。**基线改用 `maze_1_6x5`**
  （`start_sim_stack.ps1 -Maze1/-Maze1Stress`），maze_3 只作失败对照；
- VM 内存 **4 GB 会 OOM**（Gazebo exit -9、swap 满、rosbridge 收不到数据）；
  已调到 6 GB；批量实验前务必确认 `free -m` 还有余量；
- 热重启会留 `parameter_bridge` 等残留进程和 FastDDS SHM 锁；必须用
  `~/vm/run_stack.sh`（原子 stop→确认→start）和 UDP-only profile；
  `.sh` 用 LF（scp 后 `sed -i 's/
$//'`）；
- 停止批处理时要连 `nav_demo.py` 子进程一起杀，否则它会继续开车，
  下一次仿真一启动位姿就不是原点。
