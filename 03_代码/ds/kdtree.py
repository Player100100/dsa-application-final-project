"""KD-Tree：激光点云空间索引 + 欧氏距离聚类。

对应任务书「树与二叉树 → 决策与空间索引 → 进阶级」。

实现要点
--------
- 递归按方差最大（或轮换）轴取中位数切分，建树 O(n log n)。
- 查询支持最近邻、K 近邻（最大堆）与半径查询；平均 O(log n)。
- ``cluster`` 用半径查询做 BFS 连通分量（欧氏聚类），复杂度远低于两两比较。
- 维度自适应：点可以是 ``(x, y)`` 或更高维元组，但本项目只用 2D。
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, ...]


def _dist2(a: Sequence[float], b: Sequence[float]) -> float:
    return sum((ai - bi) ** 2 for ai, bi in zip(a, b))


@dataclass
class _Node:
    point: Point
    index: int
    axis: int
    left: Optional["_Node"] = None
    right: Optional["_Node"] = None


class KDTree:
    """静态 KD-Tree（建好后不再插入/删除）。"""

    def __init__(self, points: Sequence[Point]) -> None:
        self.points: List[Point] = [tuple(float(v) for v in p) for p in points]
        self._indices = list(range(len(self.points)))
        self._dim = len(self.points[0]) if self.points else 2
        self.root = self._build(self._indices, 0)

    # ------------------------------------------------------------ 建树
    def _build(self, indices: List[int], depth: int) -> Optional[_Node]:
        if not indices:
            return None
        axis = depth % self._dim
        indices.sort(key=lambda i: self.points[i][axis])
        mid = len(indices) // 2
        return _Node(
            point=self.points[indices[mid]],
            index=indices[mid],
            axis=axis,
            left=self._build(indices[:mid], depth + 1),
            right=self._build(indices[mid + 1 :], depth + 1),
        )

    # ------------------------------------------------------------ 查询
    def nearest(self, target: Sequence[float]) -> Optional[int]:
        """返回最近点的下标；空树返回 ``None``。"""
        result = self.knn(target, 1)
        return result[0][1] if result else None

    def knn(self, target: Sequence[float], k: int = 1) -> List[Tuple[float, int]]:
        """返回 ``[(距离, 下标), ...]``，按距离升序，最多 k 个。"""
        if k <= 0 or self.root is None:
            return []
        best: List[Tuple[float, int]] = []  # 最大堆（存负距离）
        t = tuple(float(v) for v in target)

        def visit(node: Optional[_Node]) -> None:
            if node is None:
                return
            d2 = _dist2(t, node.point)
            if len(best) < k:
                heapq.heappush(best, (-d2, node.index))
            elif d2 < -best[0][0]:
                heapq.heapreplace(best, (-d2, node.index))

            axis = node.axis
            delta = t[axis] - node.point[axis]
            near, far = (node.left, node.right) if delta < 0 else (node.right, node.left)
            visit(near)
            if len(best) < k or delta * delta < -best[0][0]:
                visit(far)

        visit(self.root)
        out = sorted((-neg_d2, idx) for neg_d2, idx in best)
        return [(math.sqrt(d2), idx) for d2, idx in out]

    def range_query(self, target: Sequence[float], radius: float) -> List[int]:
        """返回与目标距离 <= ``radius`` 的所有点下标。"""
        if radius < 0 or self.root is None:
            return []
        t = tuple(float(v) for v in target)
        r2 = radius * radius
        found: List[int] = []

        def visit(node: Optional[_Node]) -> None:
            if node is None:
                return
            if _dist2(t, node.point) <= r2:
                found.append(node.index)
            axis = node.axis
            delta = t[axis] - node.point[axis]
            near, far = (node.left, node.right) if delta < 0 else (node.right, node.left)
            visit(near)
            if delta * delta <= r2:
                visit(far)

        visit(self.root)
        return found

    def radius_clusters(
        self, radius: float, min_points: int = 1
    ) -> List[List[int]]:
        """按欧氏距离把点连成簇（BFS 连通分量），返回下标列表的列表。

        复杂度约 O(n log n + 邻域边数)，比两两比较 O(n²) 快。
        """
        n = len(self.points)
        visited = [False] * n
        clusters: List[List[int]] = []
        for seed in range(n):
            if visited[seed]:
                continue
            visited[seed] = True
            queue = [seed]
            members: List[int] = []
            while queue:
                current = queue.pop()
                members.append(current)
                for neighbour in self.range_query(self.points[current], radius):
                    if not visited[neighbour]:
                        visited[neighbour] = True
                        queue.append(neighbour)
            if len(members) >= min_points:
                clusters.append(sorted(members))
        return clusters

    def __len__(self) -> int:
        return len(self.points)

    def __bool__(self) -> bool:
        return self.root is not None


def brute_force_knn(
    points: Sequence[Point], target: Sequence[float], k: int = 1
) -> List[Tuple[float, int]]:
    """暴力 K 近邻，用于单测对拍与性能基准。"""
    dists = [(_dist2(target, p), i) for i, p in enumerate(points)]
    dists.sort()
    return [(math.sqrt(d2), i) for d2, i in dists[:k]]
