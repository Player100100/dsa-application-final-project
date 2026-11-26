# 数据结构与算法应用 · 机器人算法冲刺项目

> 课程：数据结构与算法应用（PBL）｜冲刺路线：**从四个可拼接模块做成“动态避障机器人”**
> 输入材料：[任务书 PDF](../../用户传入/数据结构与算法应用/a451eaf8e9a0c3753c65a7e4f450c884.pdf)、
> [厂商 SDK v0.3](../../用户传入/数据结构与算法应用/客户端v0.3/使用手册.md)

这是一个 **AI 工作台**：人负责定方向、验收、上真车；AI 按本目录的规则读写代码、
跑实验、维护进度。每次新会话先读 [AGENTS.md](AGENTS.md)。

---

## 当前状态（2026-11-25）

| 项 | 状态 | 证据 |
|---|---|---|
| 工作台结构与规则 | ✅ 完成 | 本文件、[AGENTS.md](AGENTS.md) |
| 四个核心模块（线性表/数组/树/图） | ✅ 可运行 | [03_代码/ds/](03_代码/ds/) |
| 单元测试 | ✅ 40/40 通过 | [04_实验/2026-11-25_环境与mock导航.md](04_实验/2026-11-25_环境与mock导航.md) |
| mock 端到端导航 | ✅ 3 个目标均到达 | 同上 |
| 性能基准（合成数据） | ✅ 有数据 | [04_实验/基准记录_2026-11-25.md](04_实验/基准记录_2026-11-25.md) |
| Gazebo 仿真接通 | ✅ 2026-11-25 | [04_实验/2026-11-25_仿真联调.md](04_实验/2026-11-25_仿真联调.md) |
| 仿真端到端导航 | ✅ maze_1 3 目标 × 5 次 × ideal/stress，**30/30 到达** | [Sim2Real 对照](04_实验/2026-11-26_Sim2Real对照.md) |
| Sim2Real 差异 | ✅ stress 比 ideal 慢 1–6%，到达率相同 | 同上 |
| 动态障碍（M3） | ✅ Gazebo 真箱子 3/3：12.1→21.5 s，重规划 14→30 | [M3 记录](04_实验/2026-11-26_M3动态障碍.md) |
| 真车联调 | ⏳ 等车 | [06_进度/下一步.md](06_进度/下一步.md) |
| 报告初稿 / 程序清单 | ✅ 已完成草稿与 zip | [报告总稿草稿](05_交付/报告_总稿_草稿.md) |
| 视频 / 答辩 | ⏳ 未开始 | [视频脚本](05_交付/测试视频脚本.md) |

**一句话现状**：算法主线已在 mock 上闭环，下一步是把 mock 换成课程方的仿真环境，
再补真车数据与 Sim2Real 对比。

---

## 目录导航

| 目录 | 放什么 | 谁维护 |
|---|---|---|
| [00_任务与输入/](00_任务与输入/) | 任务书摘要、SDK 接口速查 | 人改，AI 只读 |
| [01_规划/](01_规划/) | 冲刺路线、里程碑验收、风险止损 | 人定，AI 提建议 |
| [02_设计/](02_设计/) | 总体架构 + 四个模块规格 | AI 写，人审 |
| [03_代码/](03_代码/) | 可运行代码、测试、脚本 | AI 写，人验收 |
| [04_实验/](04_实验/) | 实验记录、基准数据、地图产物 | AI 追加，不覆盖 |
| [05_交付/](05_交付/) | 报告大纲、视频脚本、程序清单 | 人主导 |
| [06_进度/](06_进度/) | 看板、工作日志、下一步 | AI 每次会话更新 |

---

## 快速开始

### 第一次：建环境（约 1 分钟）

```powershell
cd 项目文档\数据结构与算法应用_冲刺项目
powershell -ExecutionPolicy Bypass -File 03_代码\scripts\setup_env.ps1
```

### 日常：测 / 跑 / 验

```powershell
cd 03_代码
..\.venv\Scripts\python -m unittest discover -s tests -t .   # 40 个单测
..\.venv\Scripts\python demo\bench.py --save                  # 性能基准
..\.venv\Scripts\python demo\nav_demo.py --ip mock --goal 2 2 # mock 导航
..\.venv\Scripts\python scripts\check_sim.py --ip <VM_IP>     # 虚拟机/真车链路预检
```

真车/仿真：

```powershell
# 一键起仿真栈（VM + Gazebo headless + rosbridge），约 1 分钟
powershell -ExecutionPolicy Bypass -File scripts\start_sim_stack.ps1
# 用 ideal 运动/传感器模型、empty.world，或关机：
#   ... start_sim_stack.ps1 -Ideal
#   ... start_sim_stack.ps1 -Empty
#   ... start_sim_stack.ps1 -Stop

# 本机 Gazebo 仿真（VM 已起、仿真已在跑时）
..\.venv\Scripts\python demo\nav_demo.py --ip 192.168.150.131 --goal 1.5 0 `
  --speed 0.15 --inflate 0.15 --stop-distance 0.18 `
  --cluster-slow-distance 0.30 --lookahead 0.30

# 真车/其他仿真地址
..\.venv\Scripts\python demo\nav_demo.py --ip <IP> --goal 1.5 0.0
```

虚拟机怎么组网、怎么开 SSH 让 AI 能控制仿真，见
[00_任务与输入/仿真环境接入.md](00_任务与输入/仿真环境接入.md)。

> ⚠️ 非 mock 会让车真的动。先确认场地空旷、能随时急停；第一次先跑
> `python 用户传入\...\examples\00_自检.py --ip <IP>` 确认链路。

已有虚拟环境可直接用；不想装 wheel 也可把 SDK 源码根目录设进
`ROSMASTER_SDK_ROOT`，[robot_io.py](03_代码/robot_io.py) 会自动找。

---

## 四条硬约定

1. **不改厂商 SDK**（`用户传入/` 只读）。要加功能就写适配层。
2. **核心数据结构只用标准库**，不引入 numpy/scipy——验收机不一定装得齐。
3. **控制环 10 Hz**：远端看门狗 0.5 s 自动停车，任何“走一段”都必须持续发速度。
4. **地图交付只走 `GridMap.save()`**：落盘固定三值（-1/0/100），中间概率不出磁盘。

---

## 冲刺路线一句话

线性表做输入缓冲，二维数组做地图，KD-Tree 做障碍聚类，图搜索做路径规划，
四条线在 [demo/nav_demo.py](03_代码/demo/nav_demo.py) 汇合成“定点导航”；
期末再叠加动态障碍与重规划，冲“动态避障机器人”。

详细拆解见 [01_规划/冲刺路线.md](01_规划/冲刺路线.md)，
当前该做什么见 [06_进度/下一步.md](06_进度/下一步.md)。

---

## 仓库

代码与实验产物：<https://github.com/Player100100/dsa-application-final-project>

## AI 使用披露

本项目的代码、文档与实验整理使用了 AI 编程助手（Claude Code）辅助；选题、
实验方案、验收标准与最终结论由小组成员负责。文中所有性能数字均来自本仓库
`04_实验/` 记录的 mock / Gazebo 仿真运行，未编造数据。
