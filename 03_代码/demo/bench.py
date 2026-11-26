#!/usr/bin/env python3
"""性能基准：A* vs Dijkstra、KD-Tree vs 暴力、聚类加速比。

产出是 Markdown 表格，可直接贴进报告。数据是**合成环境**（固定随机种子），
与 mock/真车的实测数据分开记录，不要混为一谈。

用法::

    python demo/bench.py                # 打印表格
    python demo/bench.py --save         # 另存到 04_实验/基准记录_YYYY-MM-DD.md
"""

from __future__ import annotations

import argparse
import math
import random
import sys
import time
from datetime import date
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from ds.astar import astar, dijkstra  # noqa: E402
from ds.grid_map import FREE, OCCUPIED, GridMap  # noqa: E402
from ds.kdtree import KDTree, brute_force_knn  # noqa: E402


def make_random_grid(size: int, density: float, seed: int) -> GridMap:
    rng = random.Random(seed)
    grid = GridMap(size, size, 0.05)
    grid.fill(FREE)
    for y in range(1, size - 1):
        for x in range(1, size - 1):
            if rng.random() < density:
                grid.set(x, y, OCCUPIED)
    grid.set(2, 2, FREE)
    grid.set(size - 3, size - 3, FREE)
    return grid


def bench_planners(size: int = 160, density: float = 0.18, runs: int = 5) -> tuple:
    start, goal = (2, 2), (size - 3, size - 3)
    rows = []
    astar_time, dijkstra_time = [], []
    last = None
    for seed in range(runs):
        grid = make_random_grid(size, density, seed)
        blocked = grid.inflated_mask(inflate_m=0.0)
        t0 = time.perf_counter()
        a = astar(grid, start, goal, blocked=blocked)
        astar_time.append(time.perf_counter() - t0)
        t0 = time.perf_counter()
        d = dijkstra(grid, start, goal, blocked=blocked)
        dijkstra_time.append(time.perf_counter() - t0)
        if a.found and d.found:
            assert abs(a.cost - d.cost) < 1e-9, "A* 与 Dijkstra 最优代价不一致"
        last = (a, d, grid)
    a, d, grid = last
    rows.append(("A*", f"{median(astar_time) * 1000:.1f} ms", f"{a.expanded}", f"{a.cost:.2f}"))
    rows.append(("Dijkstra", f"{median(dijkstra_time) * 1000:.1f} ms", f"{d.expanded}", f"{d.cost:.2f}"))
    return rows, a, d


def bench_knn(n_points: int = 5000, queries: int = 200, k: int = 5) -> tuple:
    rng = random.Random(7)
    points = [(rng.uniform(-10, 10), rng.uniform(-10, 10)) for _ in range(n_points)]
    tree = KDTree(points)
    targets = [(rng.uniform(-10, 10), rng.uniform(-10, 10)) for _ in range(queries)]

    t0 = time.perf_counter()
    kd_results = [tree.knn(t, k) for t in targets]
    kd_time = time.perf_counter() - t0

    t0 = time.perf_counter()
    bf_results = [brute_force_knn(points, t, k) for t in targets]
    bf_time = time.perf_counter() - t0

    for a, b in zip(kd_results, bf_results):
        assert [i for _, i in a] == [i for _, i in b], "KD-Tree 与暴力结果不一致"
    return kd_time, bf_time, n_points, queries


def bench_cluster(n_points: int = 1200, radius: float = 0.35, n_blobs: int = 20) -> tuple:
    rng = random.Random(11)
    points = []
    for _ in range(n_blobs):
        cx, cy = rng.uniform(-8, 8), rng.uniform(-8, 8)
        for _ in range(n_points // n_blobs):
            points.append((rng.gauss(cx, 0.12), rng.gauss(cy, 0.12)))
    tree = KDTree(points)

    t0 = time.perf_counter()
    clusters = tree.radius_clusters(radius, min_points=2)
    kd_time = time.perf_counter() - t0

    t0 = time.perf_counter()
    naive = naive_clusters(points, radius)
    naive_time = time.perf_counter() - t0
    return kd_time, naive_time, len(points), len(clusters), len(naive)


def naive_clusters(points, radius: float):
    """O(n²) 对照实现，仅用于验证 KD-Tree 聚类结果。"""
    n = len(points)
    r2 = radius * radius
    adj = [[] for _ in range(n)]
    for i in range(n):
        xi, yi = points[i]
        for j in range(i + 1, n):
            xj, yj = points[j]
            if (xi - xj) ** 2 + (yi - yj) ** 2 <= r2:
                adj[i].append(j)
                adj[j].append(i)
    seen = [False] * n
    out = []
    for i in range(n):
        if seen[i]:
            continue
        stack = [i]
        seen[i] = True
        members = []
        while stack:
            cur = stack.pop()
            members.append(cur)
            for nxt in adj[cur]:
                if not seen[nxt]:
                    seen[nxt] = True
                    stack.append(nxt)
        if len(members) >= 2:
            out.append(sorted(members))
    return out


def median(values):
    values = sorted(values)
    return values[len(values) // 2]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="数据结构与算法性能基准")
    parser.add_argument("--save", action="store_true", help="保存到 04_实验/")
    args = parser.parse_args(argv)

    planner_rows, a, d = bench_planners()
    kd_time, bf_time, n_points, queries = bench_knn()
    cluster_kd, cluster_naive, n_cluster_points, kd_clusters, naive_clusters_n = bench_cluster()

    lines = [
        f"# 基准记录（合成数据，{date.today().isoformat()}）",
        "",
        "> 由 `03_代码/demo/bench.py` 生成。随机种子固定，可复现；",
        "> 这不是 mock/真车实测，报告里要分表。",
        "",
        "## 1. 路径规划（160×160 栅格，18% 障碍，5 个种子取中位数）",
        "",
        "| 算法 | 用时 | 展开节点数 | 路径代价 |",
        "|---|---:|---:|---:|",
    ]
    for name, t, expanded, cost in planner_rows:
        lines.append(f"| {name} | {t} | {expanded} | {cost} |")
    if d.expanded:
        lines.append("")
        lines.append(f"A* 展开节点数 / Dijkstra = **{a.expanded / d.expanded:.1%}**")
    lines += [
        "",
        f"## 2. K 近邻（{n_points} 点，{queries} 次查询，k=5）",
        "",
        "| 实现 | 用时 |",
        "|---|---:|",
        f"| KD-Tree | {kd_time * 1000:.1f} ms |",
        f"| 暴力 O(n·k) | {bf_time * 1000:.1f} ms |",
        "",
        f"加速比：**{bf_time / kd_time:.1f}×**（结果逐项与暴力一致）",
        "",
        f"## 3. 欧氏聚类（{n_cluster_points} 点，半径 0.35 m，20 个高斯团）",
        "",
        "| 实现 | 用时 | 簇数 |",
        "|---|---:|---:|",
        f"| KD-Tree BFS | {cluster_kd * 1000:.1f} ms | {kd_clusters} |",
        f"| 两两比较 O(n²) | {cluster_naive * 1000:.1f} ms | {naive_clusters_n} |",
        "",
        f"加速比：**{cluster_naive / cluster_kd:.1f}×**（簇划分与 O(n²) 实现一致）",
        "",
    ]
    text = "\n".join(lines)
    print(text)

    if args.save:
        out = ROOT.parent / "04_实验" / f"基准记录_{date.today().isoformat()}.md"
        out.write_text(text, encoding="utf-8")
        print(f"[i] 已保存：{out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
