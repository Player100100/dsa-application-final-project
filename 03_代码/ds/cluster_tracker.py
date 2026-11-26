"""跨帧簇跟踪：把每帧的障碍簇关联成轨迹，并标记“动态障碍”。

对应任务书「树与二叉树 → 动态场景理解」的挑战级方向（简化版）。

输入是车体系下的点与簇（`KDTree.radius_clusters` 的输出），输出是带
速度估计的轨迹。关联用最近质心 + 门限，速度用前后两帧质心差 / 时间差。

不依赖 numpy / ROS；可直接用于 mock、仿真或真车。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]
Centroid = Tuple[float, float]


@dataclass
class Track:
    """一条障碍簇轨迹。"""

    track_id: int
    centroid: Centroid
    size: int
    last_seen: float
    velocity: Centroid = (0.0, 0.0)
    hits: int = 1
    dynamic: bool = False

    @property
    def speed(self) -> float:
        return (self.velocity[0] ** 2 + self.velocity[1] ** 2) ** 0.5


class ClusterTracker:
    """把每帧的簇关联成轨迹。

    参数
    ----
    gate:
        关联门限（米）；新质心与旧轨迹质心距离超过它就视为新目标。
    dynamic_speed:
        判定为动态的速度阈值（米/秒）。
    max_age:
        轨迹多久没被看到就删除（秒）。
    smoothing:
        速度低通系数（0–1，越大越相信新观测）。
    min_hits:
        至少被看到几帧才允许标记为动态，避免单帧噪声。
    """

    def __init__(
        self,
        gate: float = 0.60,
        dynamic_speed: float = 0.15,
        max_age: float = 1.0,
        smoothing: float = 0.5,
        min_hits: int = 2,
    ) -> None:
        self.gate = float(gate)
        self.dynamic_speed = float(dynamic_speed)
        self.max_age = float(max_age)
        self.smoothing = float(smoothing)
        self.min_hits = max(1, int(min_hits))
        self.tracks: List[Track] = []
        self._next_id = 1

    @staticmethod
    def centroid(points: Sequence[Point], members: Sequence[int]) -> Centroid:
        if not members:
            return (0.0, 0.0)
        sx = sum(points[i][0] for i in members) / len(members)
        sy = sum(points[i][1] for i in members) / len(members)
        return (sx, sy)

    def update(
        self,
        points: Sequence[Point],
        clusters: Sequence[Sequence[int]],
        now: float,
    ) -> List[Track]:
        """用当前帧的簇更新轨迹，返回当前仍存在的轨迹列表。"""
        # 先清掉过期轨迹
        self.tracks = [t for t in self.tracks if now - t.last_seen <= self.max_age]

        centroids = [self.centroid(points, members) for members in clusters]
        used_tracks = set()
        matched_tracks = set()

        for members, centroid in zip(clusters, centroids):
            best: Optional[Track] = None
            best_dist = self.gate
            for track in self.tracks:
                if track.track_id in used_tracks:
                    continue
                dx = centroid[0] - track.centroid[0]
                dy = centroid[1] - track.centroid[1]
                dist = (dx * dx + dy * dy) ** 0.5
                if dist < best_dist:
                    best, best_dist = track, dist

            if best is None:
                track = Track(
                    track_id=self._next_id,
                    centroid=centroid,
                    size=len(members),
                    last_seen=now,
                )
                self._next_id += 1
                self.tracks.append(track)
                matched_tracks.add(track.track_id)
                continue

            used_tracks.add(best.track_id)
            dt = max(1e-3, now - best.last_seen)
            vx = (centroid[0] - best.centroid[0]) / dt
            vy = (centroid[1] - best.centroid[1]) / dt
            # 低通，抑制单帧质心跳变
            best.velocity = (
                self.smoothing * vx + (1 - self.smoothing) * best.velocity[0],
                self.smoothing * vy + (1 - self.smoothing) * best.velocity[1],
            )
            best.centroid = centroid
            best.size = len(members)
            best.last_seen = now
            best.hits += 1
            best.dynamic = (
                best.hits >= self.min_hits and best.speed >= self.dynamic_speed
            )
            matched_tracks.add(best.track_id)

        # 本帧没匹配上的轨迹：保留（可能会再次出现），但不算当前帧命中
        return [t for t in self.tracks if t.last_seen == now]

    def dynamic_near(
        self,
        origin: Centroid,
        heading: float,
        now: Optional[float] = None,
        max_distance: float = 0.8,
        half_angle: float = 0.6,
    ) -> Optional[Track]:
        """返回相对 ``origin``/``heading`` 的车头扇区内最近的动态轨迹。

        轨迹坐标必须与 ``origin`` 同坐标系（本项目用世界系，避免自车运动
        把静止墙误判成动态）。
        """
        best: Optional[Track] = None
        best_dist = math.inf
        for track in self.tracks:
            if not track.dynamic:
                continue
            if now is not None and now - track.last_seen > 0.3:
                continue
            dx = track.centroid[0] - origin[0]
            dy = track.centroid[1] - origin[1]
            dist = math.hypot(dx, dy)
            if dist > max_distance or dist >= best_dist:
                continue
            bearing = (math.atan2(dy, dx) - heading + math.pi) % (2 * math.pi) - math.pi
            if abs(bearing) > half_angle:
                continue
            best, best_dist = track, dist
        return best

    def dynamic_in_front(
        self,
        now: Optional[float] = None,
        max_distance: float = 0.8,
        half_angle: float = 0.6,
    ) -> Optional[Track]:
        """车体坐标系下的便捷版本（原点在车、车头为 +x）。"""
        return self.dynamic_near(
            (0.0, 0.0), 0.0, now=now,
            max_distance=max_distance, half_angle=half_angle,
        )
