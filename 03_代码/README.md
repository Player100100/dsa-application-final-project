# 代码目录

```text
03_代码/
├── robot_io.py           # 唯一接触厂商 SDK 的适配层；支持 mock / 真车
├── ds/                   # 交付本体：纯标准库数据结构与算法
│   ├── ring_buffer.py    # 线性表：定长环形缓冲
│   ├── grid_map.py       # 数组：占据栅格 + YAML/PGM 读写
│   ├── mapping.py        # 激光 → 栅格（Bresenham 射线）
│   ├── kdtree.py         # 树：KNN / 半径查询 / 欧氏聚类
│   ├── astar.py          # 图：Dijkstra / A*
│   └── cluster_tracker.py # 树/动态：跨帧簇跟踪与动态障碍判定
├── demo/
│   ├── nav_demo.py       # 端到端：建图 → A* → 10 Hz 跟随
│   ├── bench.py          # 性能基准（合成数据，可复现）
│   ├── bench_real_map.py # 真实 live map 上的 A* vs Dijkstra
│   └── bench_cluster_real.py # 真实点云上的聚类参数扫描
├── tests/                # 31 个 unittest 用例
└── scripts/
    ├── setup_env.ps1     # 建 .venv 并安装 SDK
    ├── run_checks.ps1    # 单测 + 基准 + mock 导航冒烟
    ├── check_sim.py      # 仿真/真车链路预检：TCP → WebSocket → SDK → 首帧
    ├── find_vm_ip.py     # 从 VMware DHCP 租约查 NAT 虚拟机的 IP
    ├── start_sim_vm.ps1  # 启动仿真 VM 并等租约出 IP
    ├── start_sim_stack.ps1 # 一键启动 VM + Gazebo + rosbridge（-Ideal/-Empty/-Maze1/-Maze1Stress/-Stop）
    └── vm/               # 虚拟机侧脚本（sim_*.sh、bridge、stop、run_stack）
```

## 运行

在 `03_代码/` 下：

```powershell
..\.venv\Scripts\python -m unittest discover -s tests -t . -v
..\.venv\Scripts\python demo\bench.py --save
..\.venv\Scripts\python demo\nav_demo.py --ip mock --goal 2.0 2.0 --duration 40
```

没有 `.venv` 时：`python` 也能跑单测与基准（零依赖）；`nav_demo` 需要
`roslibpy`（mock 也要）。

## 编码约定

- 核心算法只用标准库；类型注解写全；docstring 用中文并注明复杂度；
- 一个文件一个主题；`ds/` 不 import `robot_io`；
- 新算法先补单测，再接到 demo；
- 失败要显式：A* 返回 `PathResult(reason=...)`，不要静默返回空路径。

## 当前验证基线（2026-11-26）

| 项 | 结果 |
|---|---|
| 单测 | 40/40 通过 |
| A* vs Dijkstra（合成，160²，18% 障碍） | 34.0 ms / 86.4 ms；展开节点 31.9% |
| A* vs Dijkstra（真实 live map，10 张） | 41 / 566 节点；0.3 / 2.3 ms；代价一致 |
| KNN（5000 点，200 查询） | 8.8 ms vs 973.3 ms（111.0×） |
| 聚类（1200 点） | 84.5 ms vs 122.7 ms（1.5×，见规格说明） |
| mock 导航 | (2,2) 16.1 s；(2.7,0) 13.4 s；(−2,−2) 15.8 s，均到达 |
| 仿真链路 | `check_sim --ip 192.168.150.131` 全通（TCP→101→/scan→tf） |
| 仿真基线（maze_1, ideal） | 3 目标 × 5 次 **15/15**：2.08 / 5.76 / 12.30 s |
| 仿真基线（maze_1, stress） | 3 目标 × 5 次 **15/15**：2.10 / 5.86 / 12.98 s |
| maze_3 窄口 | ideal/GT 都卡在 (0.4,0.34)；几何不可通行，作对照 |
| 目标集 | 用 (1.5,0)/(1,2)/(0,2)；**(2,2) 实际是墙** |
| 仿真话题 | `/scan` 7.18 Hz（physical）、`/odom` 9.77 Hz、`/imu` ~9.9 Hz |
| 主循环耗时 | 建图 4.3 ms + 聚类 ~15 ms，10 Hz 有余量 |

详细日志：[2026-11-25 mock](../04_实验/2026-11-25_环境与mock导航.md)、[2026-11-26 M1 修复](../04_实验/2026-11-26_M1控制修复与复测.md)
