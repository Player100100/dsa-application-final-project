"""激光 → 栅格地图：把 /scan 点按里程计位姿写进二维占据栅格。

用途：给 A* 提供“当前已知环境”。这里刻意只做 **栅格建图**，不做 SLAM：
位姿来自轮式里程计（会漂），所以本模块的产物是“带误差的局部地图”，
报告里的 Sim2Real 对比要如实写明这一点。
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

from .grid_map import FREE, OCCUPIED, UNKNOWN, GridMap

Cell = Tuple[int, int]


def bresenham(x0: int, y0: int, x1: int, y1: int) -> List[Cell]:
    """整数栅格直线（含两端点）。"""
    points: List[Cell] = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        points.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
    return points


class ScanMapper:
    """在线占据栅格建图器（命中/未命中计数版）。

    参数
    ----
    grid:
        目标地图容器；若为 ``None`` 则按 ``width_m/height_m/resolution`` 新建。
    laser_offset_yaw:
        雷达坐标系相对车体的偏航（``get_transform("base_link", "laser_link")``）。
    mount_xy:
        雷达安装位置（米，车体系）。
    stride:
        每帧参与建图的激光束步长；1080 束全用时 CPU 压力大，默认每 4 束取 1。
    occ_hits / free_misses:
        连续命中/穿过多少次才把格子定为占据/空闲。
    """

    def __init__(
        self,
        grid: Optional[GridMap] = None,
        *,
        laser_offset_yaw: float = 0.0,
        mount_xy: Tuple[float, float] = (0.0, 0.0),
        stride: int = 4,
        occ_hits: int = 2,
        free_misses: int = 2,
        width_m: float = 8.0,
        height_m: float = 8.0,
        resolution: float = 0.05,
        center: Tuple[float, float] = (0.0, 0.0),
    ) -> None:
        if grid is None:
            width = int(round(width_m / resolution))
            height = int(round(height_m / resolution))
            origin = (center[0] - width * resolution / 2.0,
                      center[1] - height * resolution / 2.0)
            grid = GridMap(width, height, resolution, origin=origin)
        self.grid = grid
        self.laser_offset_yaw = float(laser_offset_yaw)
        self.mount_xy = (float(mount_xy[0]), float(mount_xy[1]))
        self.stride = max(1, int(stride))
        self.occ_hits = max(1, int(occ_hits))
        self.free_misses = max(1, int(free_misses))
        n = self.grid.width * self.grid.height
        self.hits: List[int] = [0] * n
        self.misses: List[int] = [0] * n
        self.frames = 0
        self.beams_used = 0

    # ------------------------------------------------------------ 主入口
    def update(self, frame, pose) -> int:
        """用一帧 ``/scan`` 与对应位姿更新地图，返回参与建图的束数。"""
        x, y, yaw = _pose_xyz(pose)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        mx, my = self.mount_xy
        # 雷达安装点在车体系 → 世界系
        sx = x + mx * cos_yaw - my * sin_yaw
        sy = y + mx * sin_yaw + my * cos_yaw
        start_cell = self.grid.world_to_cell(sx, sy)

        width, height = self.grid.width, self.grid.height
        offset = self.laser_offset_yaw
        used = 0
        for i in range(0, frame.count, self.stride):
            r = frame.ranges[i]
            if not (math.isfinite(r) and frame.range_min <= r <= frame.range_max):
                continue
            bearing = yaw + offset + frame.angle_at(i)
            ex = sx + r * math.cos(bearing)
            ey = sy + r * math.sin(bearing)
            end_cell = self.grid.world_to_cell(ex, ey)
            ray = bresenham(start_cell[0], start_cell[1], end_cell[0], end_cell[1])
            # 起点那一格不记（车体自身所在位置）
            for cx, cy in ray[1:-1]:
                if 0 <= cx < width and 0 <= cy < height:
                    self.misses[cy * width + cx] += 1
            hx, hy = end_cell
            if 0 <= hx < width and 0 <= hy < height:
                self.hits[hy * width + hx] += 1
            used += 1

        self.frames += 1
        self.beams_used += used
        return used

    # ------------------------------------------------------------ 输出
    def build_grid(self) -> GridMap:
        """按计数阈值生成三值地图（``-1/0/100``），不修改内部计数。"""
        out = GridMap(
            self.grid.width,
            self.grid.height,
            self.grid.resolution,
            origin=(self.grid.origin_x, self.grid.origin_y),
        )
        for idx in range(len(self.hits)):
            hits = self.hits[idx]
            misses = self.misses[idx]
            if hits >= self.occ_hits and hits >= misses:
                out.data[idx] = OCCUPIED
            elif misses >= self.free_misses:
                out.data[idx] = FREE
            else:
                out.data[idx] = UNKNOWN
        return out

    def occupied_ratio(self) -> float:
        total = len(self.hits)
        occupied = sum(1 for h, m in zip(self.hits, self.misses)
                       if h >= self.occ_hits and h >= m)
        return occupied / total if total else 0.0

    def stats(self) -> str:
        return (
            f"建图帧 {self.frames}，共写入 {self.beams_used} 束；"
            f"已知占据格占比 {self.occupied_ratio():.2%}"
        )


def _pose_xyz(pose) -> Tuple[float, float, float]:
    """兼容 ``OdomPose`` 与 ``(x, y, yaw)`` 元组。"""
    if hasattr(pose, "x") and hasattr(pose, "yaw"):
        return float(pose.x), float(pose.y), float(pose.yaw)
    x, y, yaw = pose
    return float(x), float(y), float(yaw)
