#!/usr/bin/env python3
"""用真实 `/scan` 一帧点云扫 KD-Tree 聚类参数。

指标：
- 簇数：越少说明墙越完整，越不容易把一面墙拆成多个“动态目标”；
- 最大簇占比：越大说明主结构（墙/大障碍）被完整聚在一起；
- 噪点数：未进入任何簇（小于 min_points）的点；
- 用时：建树 + 聚类。

用法::

    python demo/bench_cluster_real.py --ip 192.168.150.131 --save
    python demo/bench_cluster_real.py --ip mock
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from datetime import date
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from ds.kdtree import KDTree  # noqa: E402
from robot_io import RobotIO  # noqa: E402


def scan_points_body(frame, yaw_offset: float, mount, stride: int = 2):
    mx, my = mount
    points = []
    for i in range(0, frame.count, stride):
        r = frame.ranges[i]
        if not (math.isfinite(r) and frame.range_min <= r <= frame.range_max):
            continue
        bearing = frame.angle_at(i) + yaw_offset
        points.append((mx + r * math.cos(bearing), my + r * math.sin(bearing)))
    return points


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="真实点云聚类参数扫描")
    parser.add_argument("--ip", default="mock")
    parser.add_argument("--stride", type=int, default=2, help="每 N 束取 1，默认 2")
    parser.add_argument("--radii", default="0.15,0.20,0.30")
    parser.add_argument("--min-points", default="2,3,5")
    parser.add_argument("--repeat", type=int, default=3)
    parser.add_argument("--save", action="store_true")
    args = parser.parse_args(argv)

    radii = [float(v) for v in args.radii.split(",")]
    min_points = [int(v) for v in args.min_points.split(",")]

    robot = RobotIO(args.ip)
    try:
        frame = robot.wait_scan(5.0)
        if frame is None:
            print("[x] 5 秒内没有 /scan")
            return 1
        offset, mount = robot.laser_mount()
        points = scan_points_body(frame, offset, mount, args.stride)
    finally:
        robot.close()

    print(f"[i] 有效点 {len(points)}（stride={args.stride}）")
    rows = []
    for radius in radii:
        for mp in min_points:
            times = []
            clusters = []
            for _ in range(args.repeat):
                t0 = time.perf_counter()
                tree = KDTree(points)
                clusters = tree.radius_clusters(radius, min_points=mp)
                times.append(time.perf_counter() - t0)
            sizes = sorted((len(c) for c in clusters), reverse=True)
            largest = sizes[0] / len(points) if sizes and points else 0.0
            clustered = sum(sizes)
            noise = len(points) - clustered
            rows.append(
                (
                    f"{radius:.2f}",
                    str(mp),
                    str(len(clusters)),
                    f"{largest:.1%}",
                    str(noise),
                    f"{statistics.median(times) * 1000:.1f}",
                )
            )

    lines = [
        f"# 真实点云聚类参数扫描（{date.today().isoformat()}）",
        "",
        f"- 点云：{len(points)} 点（/scan，stride={args.stride}）",
        "- KD-Tree 欧氏半径聚类；每项取 3 次中位数",
        "",
        "| 半径 | min_points | 簇数 | 最大簇占比 | 噪点 | 用时(ms) |",
        "|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append("| " + " | ".join(row) + " |")
    text = "\n".join(lines)
    print(text)
    if args.save:
        out = ROOT.parent / "04_实验" / f"聚类参数_真实点云_{date.today().isoformat()}.md"
        out.write_text(text + "\n", encoding="utf-8")
        print(f"[i] 已保存：{out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
