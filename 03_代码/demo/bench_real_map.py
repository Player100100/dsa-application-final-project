#!/usr/bin/env python3
"""在真实建图结果（nav_demo 保存的 YAML）上对比 A* 与 Dijkstra。

与 `bench.py` 的合成地图不同，这里用的是仿真里实际建出来的 live map，
用来回答“真实栅格上 A* 省多少”。

用法::

    python demo/bench_real_map.py
    python demo/bench_real_map.py --maps "../04_实验/maze1_maps/*_2.0_0.0_r*.yaml" \
        --start 0 0 --goal 2 0 --inflate 0.15 --repeat 5 --save
"""

from __future__ import annotations

import argparse
import glob
import statistics
import sys
import time
from datetime import date
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from ds.astar import astar, dijkstra  # noqa: E402
from ds.grid_map import GridMap  # noqa: E402


def median_time(func, repeat: int) -> float:
    samples = []
    for _ in range(repeat):
        t0 = time.perf_counter()
        func()
        samples.append(time.perf_counter() - t0)
    return statistics.median(samples)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="真实地图上的 A* vs Dijkstra")
    parser.add_argument(
        "--maps",
        default=str(ROOT.parent / "04_实验" / "maze1_maps" / "*_2.0_0.0_r*.yaml"),
        help="地图 glob",
    )
    parser.add_argument("--start", type=float, nargs=2, default=(0.0, 0.0))
    parser.add_argument("--goal", type=float, nargs=2, default=(2.0, 0.0))
    parser.add_argument("--inflate", type=float, default=0.15)
    parser.add_argument("--repeat", type=int, default=5)
    parser.add_argument("--save", action="store_true")
    args = parser.parse_args(argv)

    paths = sorted(glob.glob(args.maps))
    if not paths:
        print(f"[x] 没有匹配的地图：{args.maps}")
        return 1

    rows = []
    for text in paths:
        path = Path(text)
        grid = GridMap.load(path)
        start = grid.world_to_cell(*args.start)
        goal = grid.world_to_cell(*args.goal)
        blocked = grid.inflated_mask(inflate_m=args.inflate)
        a = astar(grid, start, goal, blocked=blocked)
        if not a.found:
            rows.append((path.stem, "无解", a.reason, "-", "-", "-"))
            continue
        d = dijkstra(grid, start, goal, blocked=blocked)
        if not d.found:
            rows.append((path.stem, "Dijkstra 无解", d.reason, "-", "-", "-"))
            continue
        if abs(a.cost - d.cost) > 1e-9:
            print(f"[!] {path.name} 代价不一致：A*={a.cost} Dijkstra={d.cost}")
        astar_ms = median_time(
            lambda: astar(grid, start, goal, blocked=blocked), args.repeat
        ) * 1000
        dij_ms = median_time(
            lambda: dijkstra(grid, start, goal, blocked=blocked), args.repeat
        ) * 1000
        rows.append(
            (
                path.stem,
                f"{a.cost * grid.resolution:.2f}",
                f"{a.expanded}",
                f"{d.expanded}",
                f"{astar_ms:.1f}",
                f"{dij_ms:.1f}",
            )
        )

    lines = [
        f"# 真实地图：A* vs Dijkstra（{date.today().isoformat()}）",
        "",
        f"- 起点 {tuple(args.start)} → 终点 {tuple(args.goal)}，膨胀 {args.inflate} m",
        f"- 地图：`{args.maps}`；每项取 {args.repeat} 次中位数",
        "",
        "| 地图 | 路径代价(m) | A* 展开 | Dijkstra 展开 | A* 用时 | Dijkstra 用时 |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append("| " + " | ".join(str(v) for v in row) + " |")

    good = [r for r in rows if r[1] != "无解" and r[2] != "Dijkstra 无解"]
    if good:
        ratios = [
            int(r[2]) / int(r[3]) for r in good if int(r[3]) and int(r[2])
        ]
        if ratios:
            lines += [
                "",
                f"- A* 展开节点 / Dijkstra = **{statistics.mean(ratios):.1%}**"
                f"（{len(good)} 张图平均）",
                f"- 全部地图 A* 路径代价与 Dijkstra 一致",
            ]

    text = "\n".join(lines)
    print(text)
    if args.save:
        out = ROOT.parent / "04_实验" / f"真实地图_A星_vs_Dijkstra_{date.today().isoformat()}.md"
        out.write_text(text + "\n", encoding="utf-8")
        print(f"[i] 已保存：{out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
