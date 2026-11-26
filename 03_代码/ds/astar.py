"""网格图上的 Dijkstra 与 A* 路径规划。

对应任务书「图结构 → 路径规划与拓扑建模 → 基础级 + 进阶级」。

- ``astar``：八邻域、octile 启发式，禁止斜穿障碍物拐角。
- ``dijkstra``：同一套搜索框架、启发式恒为 0；用于和 A* 对拍/对比性能。
- 两者共用 ``blocked`` 布尔表：``True`` = 不可通行。默认由 ``GridMap`` 生成，
  可指定膨胀半径与“未知区域是否可走”。
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from .grid_map import GridMap

Cell = Tuple[int, int]

_SQRT2 = math.sqrt(2.0)


@dataclass
class PathResult:
    """一次搜索的结果。"""

    path: List[Cell] = field(default_factory=list)
    cost: float = math.inf
    expanded: int = 0        # 弹出并展开的节点数（性能对比用）
    found: bool = False
    reason: str = ""

    @property
    def length_cells(self) -> int:
        return len(self.path)


def astar(
    grid: GridMap,
    start: Cell,
    goal: Cell,
    blocked: Optional[Sequence[bool]] = None,
    diagonal: bool = True,
    inflate_m: float = 0.0,
    unknown_blocked: bool = False,
) -> PathResult:
    """A* 搜索；``blocked`` 省略时按占据栅格自动生成。"""
    return _search(
        grid,
        start,
        goal,
        blocked=blocked,
        diagonal=diagonal,
        inflate_m=inflate_m,
        unknown_blocked=unknown_blocked,
        heuristic="octile",
    )


def dijkstra(
    grid: GridMap,
    start: Cell,
    goal: Cell,
    blocked: Optional[Sequence[bool]] = None,
    diagonal: bool = True,
    inflate_m: float = 0.0,
    unknown_blocked: bool = False,
) -> PathResult:
    """Dijkstra 搜索（A* 的特例：启发式为 0）。"""
    return _search(
        grid,
        start,
        goal,
        blocked=blocked,
        diagonal=diagonal,
        inflate_m=inflate_m,
        unknown_blocked=unknown_blocked,
        heuristic="zero",
    )


# ---------------------------------------------------------------- 实现


def _search(
    grid: GridMap,
    start: Cell,
    goal: Cell,
    *,
    blocked: Optional[Sequence[bool]],
    diagonal: bool,
    inflate_m: float,
    unknown_blocked: bool,
    heuristic: str,
) -> PathResult:
    width, height = grid.width, grid.height

    if not grid.in_bounds(*start) or not grid.in_bounds(*goal):
        return PathResult(reason="起点或终点越界")

    if blocked is None:
        blocked = grid.inflated_mask(
            inflate_m=inflate_m, unknown_blocked=unknown_blocked
        )
    # 允许起点本身在膨胀带内（车贴墙时也应当能开出去）。
    blocked_list = list(blocked)
    blocked_list[start[1] * width + start[0]] = False

    def is_blocked(cell: Cell) -> bool:
        return blocked_list[cell[1] * width + cell[0]]

    if is_blocked(goal):
        return PathResult(reason="终点不可达（被障碍物或未知区域占据）")

    moves: List[Tuple[int, int, float]] = [
        (1, 0, 1.0),
        (-1, 0, 1.0),
        (0, 1, 1.0),
        (0, -1, 1.0),
    ]
    if diagonal:
        moves += [(1, 1, _SQRT2), (1, -1, _SQRT2), (-1, 1, _SQRT2), (-1, -1, _SQRT2)]

    def h(cell: Cell) -> float:
        if heuristic == "zero":
            return 0.0
        dx = abs(cell[0] - goal[0])
        dy = abs(cell[1] - goal[1])
        return max(dx, dy) + (_SQRT2 - 1.0) * min(dx, dy)

    open_heap: List[Tuple[float, float, Cell]] = [(h(start), 0.0, start)]
    g_score: Dict[Cell, float] = {start: 0.0}
    parent: Dict[Cell, Cell] = {}
    closed: set[Cell] = set()
    expanded = 0

    while open_heap:
        _f, g, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if g > g_score.get(current, math.inf):
            continue
        closed.add(current)
        expanded += 1
        if current == goal:
            path: List[Cell] = [current]
            while current in parent:
                current = parent[current]
                path.append(current)
            path.reverse()
            return PathResult(path=path, cost=g, expanded=expanded, found=True)

        cx, cy = current
        for dx, dy, step in moves:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < width and 0 <= ny < height):
                continue
            neighbour = (nx, ny)
            if is_blocked(neighbour):
                continue
            # 禁止斜穿障碍物拐角：两个正交邻格都必须可走。
            if dx != 0 and dy != 0:
                if is_blocked((cx + dx, cy)) or is_blocked((cx, cy + dy)):
                    continue
            tentative = g + step
            if tentative < g_score.get(neighbour, math.inf):
                g_score[neighbour] = tentative
                parent[neighbour] = current
                heapq.heappush(open_heap, (tentative + h(neighbour), tentative, neighbour))

    return PathResult(reason="没有可行路径", expanded=expanded)


def clear_around(
    blocked: Sequence[bool],
    width: int,
    height: int,
    cell: Cell,
    radius: int = 1,
) -> List[bool]:
    """复制阻挡表，并把 ``cell`` 周围 ``radius`` 格（切比雪夫）解阻。

    用途：车贴墙时起点会落进膨胀带，A* 连第一步都迈不出去。把起点邻域
    短暂视为可走，让规划器先给出“离开墙边”的路径，是比直接放弃更稳的恢复策略。
    """
    out = list(blocked)
    cx, cy = cell
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            x, y = cx + dx, cy + dy
            if 0 <= x < width and 0 <= y < height:
                out[y * width + x] = False
    return out


def path_to_world(grid: GridMap, path: Sequence[Cell]) -> List[Tuple[float, float]]:
    return [grid.cell_to_world(x, y) for x, y in path]


def smooth_path(
    points: Sequence[Tuple[float, float]], tolerance: float = 0.05
) -> List[Tuple[float, float]]:
    """去掉共线点，缩短跟随路径（简单折线简化）。"""
    if len(points) <= 2:
        return list(points)
    out: List[Tuple[float, float]] = [points[0]]
    for i in range(1, len(points) - 1):
        ax, ay = out[-1]
        bx, by = points[i]
        cx, cy = points[i + 1]
        cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        if abs(cross) > tolerance:
            out.append((bx, by))
    out.append(points[-1])
    return out
