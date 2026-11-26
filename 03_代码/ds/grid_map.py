"""二维占据栅格地图（二维数组实现）+ ROS 2 标准 YAML/PGM 读写。

对应任务书「数组和广义表 → 环境表示与地图建模 → 基础级/进阶级」。

约定（与 rosmaster_gridmap / ROS 2 一致）
----------------------------------------
- 栅格坐标 ``(x, y)``：``x`` 是列、``y`` 是行，``(0, 0)`` 在**左下角**；
  内部 ``data`` 按行主序存储，``index = y * width + x``。
- 世界坐标：``world = origin + (cell + 0.5) * resolution``（格心）。
- 栅格值：``-1`` 未知，``0`` 空闲，``1..100`` 占用程度（建图时可保留连续值）。

落盘契约
--------
``save()`` 把内部连续值压成三值（``-1/0/100``）并写 ``mode: trinary``，
阈值固定为 ``occupied_thresh=0.65``、``free_thresh=0.196``。中间值只活在内存里。
"""

from __future__ import annotations

import math
import re
import warnings
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

UNKNOWN = -1
FREE = 0
OCCUPIED = 100

OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.196

Cell = Tuple[int, int]


class GridMap:
    """定分辨率二维占据栅格。"""

    __slots__ = ("width", "height", "resolution", "origin_x", "origin_y", "data")

    def __init__(
        self,
        width: int,
        height: int,
        resolution: float = 0.05,
        origin: Tuple[float, float] = (0.0, 0.0),
        fill: int = UNKNOWN,
    ) -> None:
        if width <= 0 or height <= 0:
            raise ValueError("width/height 必须为正")
        if resolution <= 0:
            raise ValueError("resolution 必须为正")
        if not (fill == UNKNOWN or 0 <= fill <= 100):
            raise ValueError("fill 必须是 -1 或 0..100")
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x, self.origin_y = float(origin[0]), float(origin[1])
        self.data: List[int] = [int(fill)] * (self.width * self.height)

    # ------------------------------------------------------------ 基础访问
    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def get(self, x: int, y: int) -> int:
        if not self.in_bounds(x, y):
            raise IndexError(f"栅格 ({x}, {y}) 越界：地图 {self.width}x{self.height}")
        return self.data[y * self.width + x]

    def set(self, x: int, y: int, value: int) -> None:
        if not (value == UNKNOWN or 0 <= value <= 100):
            raise ValueError("value 必须是 -1 或 0..100")
        if not self.in_bounds(x, y):
            raise IndexError(f"栅格 ({x}, {y}) 越界：地图 {self.width}x{self.height}")
        self.data[y * self.width + x] = int(value)

    def fill(self, value: int) -> None:
        if not (value == UNKNOWN or 0 <= value <= 100):
            raise ValueError("value 必须是 -1 或 0..100")
        self.data = [int(value)] * (self.width * self.height)

    # ------------------------------------------------------------ 坐标换算
    def world_to_cell(self, wx: float, wy: float) -> Cell:
        """世界坐标 → 栅格坐标（可能越界，调用方自行判断）。"""
        return (
            math.floor((wx - self.origin_x) / self.resolution),
            math.floor((wy - self.origin_y) / self.resolution),
        )

    def cell_to_world(self, x: int, y: int) -> Tuple[float, float]:
        """栅格坐标 → 格心世界坐标。"""
        return (
            self.origin_x + (x + 0.5) * self.resolution,
            self.origin_y + (y + 0.5) * self.resolution,
        )

    def clamp_cell(self, cell: Cell) -> Cell:
        return (
            min(max(cell[0], 0), self.width - 1),
            min(max(cell[1], 0), self.height - 1),
        )

    # ------------------------------------------------------------ 几何操作
    def inflated_mask(
        self,
        inflate_m: float = 0.0,
        occupied_min: int = 65,
        unknown_blocked: bool = False,
    ) -> List[bool]:
        """返回规划用的布尔阻挡表（``True`` = 不可通行）。

        ``inflate_m`` 按欧氏距离把障碍物向外膨胀，用于给车体留出安全半径。
        """
        width, height = self.width, self.height
        blocked = [False] * (width * height)
        radius = max(0, int(math.ceil(inflate_m / self.resolution)))
        for idx, value in enumerate(self.data):
            if value == UNKNOWN:
                if unknown_blocked:
                    blocked[idx] = True
                continue
            if value < occupied_min:
                continue
            x, y = idx % width, idx // width
            x0, x1 = max(0, x - radius), min(width - 1, x + radius)
            y0, y1 = max(0, y - radius), min(height - 1, y + radius)
            r2 = radius * radius
            for yy in range(y0, y1 + 1):
                dy = yy - y
                for xx in range(x0, x1 + 1):
                    dx = xx - x
                    if dx * dx + dy * dy <= r2:
                        blocked[yy * width + xx] = True
        return blocked

    # ------------------------------------------------------------ 存取
    def save(self, yaml_path: str | Path) -> Path:
        """写成 ROS 2 标准地图（同名 ``.pgm`` + YAML），返回 YAML 路径。"""
        yaml_path = Path(yaml_path)
        yaml_path.parent.mkdir(parents=True, exist_ok=True)
        pgm_path = yaml_path.with_suffix(".pgm")

        # 栅格 -> PGM 灰度；注意图像第 0 行对应地图最上面一行。
        pixels = bytearray(self.width * self.height)
        for r in range(self.height):
            image_row = self.height - 1 - r
            row_off = image_row * self.width
            src = r * self.width
            for c in range(self.width):
                value = self.data[src + c]
                if value == UNKNOWN:
                    gray = 205
                elif value >= int(round(OCCUPIED_THRESH * 100)):
                    gray = 0
                elif value <= int(math.floor(FREE_THRESH * 100)):
                    gray = 254
                else:
                    gray = 205  # 20..64 的中间值落盘后归为未知
                pixels[row_off + c] = gray

        with open(pgm_path, "wb") as fp:
            fp.write(f"P5\n{self.width} {self.height}\n255\n".encode("ascii"))
            fp.write(bytes(pixels))

        yaml_path.write_text(
            "# 由 03_代码/ds/grid_map.py 生成（ROS 2 trinary 地图）\n"
            f"image: {pgm_path.name}\n"
            f"resolution: {self.resolution}\n"
            f"origin: [{self.origin_x}, {self.origin_y}, 0.0]\n"
            "negate: 0\n"
            f"occupied_thresh: {OCCUPIED_THRESH}\n"
            f"free_thresh: {FREE_THRESH}\n"
            "mode: trinary\n",
            encoding="utf-8",
        )
        return yaml_path

    @classmethod
    def load(
        cls,
        yaml_path: str | Path,
        *,
        occupied_thresh: Optional[float] = None,
        free_thresh: Optional[float] = None,
    ) -> "GridMap":
        """读取 ROS 2 标准地图，只产出 ``-1/0/100`` 三值。

        ``occupied_thresh`` / ``free_thresh`` 显式传入时覆盖 YAML 中的值。
        若 YAML 的 ``free_thresh > 0.196``（灰色未知像素的占用度），按 ROS 规则
        它会被判成空闲而不是未知，本方法会发 ``RuntimeWarning``——
        课程仿真器自带的地图正是这种情况，建议传 ``free_thresh=0.196``。
        """
        yaml_path = Path(yaml_path)
        text = yaml_path.read_text(encoding="utf-8")
        fields = _parse_simple_yaml(text)
        image = fields.get("image")
        if not image:
            raise ValueError(f"{yaml_path} 缺少 image 字段")
        resolution = float(fields.get("resolution", 0.05))
        origin = _parse_origin(str(fields.get("origin", "[0.0, 0.0, 0.0]")))
        occ_thresh = (
            float(occupied_thresh)
            if occupied_thresh is not None
            else float(fields.get("occupied_thresh", OCCUPIED_THRESH))
        )
        free_thresh = (
            float(free_thresh)
            if free_thresh is not None
            else float(fields.get("free_thresh", FREE_THRESH))
        )
        if free_thresh > FREE_THRESH + 1e-9:
            warnings.warn(
                f"{yaml_path.name} 的 free_thresh={free_thresh:g} > {FREE_THRESH}，"
                "灰色“未知”像素会被读成“空闲”；建议传入 free_thresh=0.196。",
                RuntimeWarning,
                stacklevel=2,
            )

        pgm_path = (yaml_path.parent / image).resolve()
        width, height, pixels = _read_pgm(pgm_path)

        grid = cls(width, height, resolution, origin=(origin[0], origin[1]))
        for r in range(height):
            image_row = height - 1 - r
            row_off = image_row * width
            dst = r * width
            for c in range(width):
                gray = pixels[row_off + c]
                occupied = (255 - gray) / 255.0
                if occupied >= occ_thresh:
                    value = OCCUPIED
                elif occupied <= free_thresh:
                    value = FREE
                else:
                    value = UNKNOWN
                grid.data[dst + c] = value
        return grid

    def copy(self) -> "GridMap":
        grid = GridMap(
            self.width,
            self.height,
            self.resolution,
            origin=(self.origin_x, self.origin_y),
        )
        grid.data = list(self.data)
        return grid

    # ------------------------------------------------------------ 统计
    def count(self, value: int) -> int:
        return sum(1 for v in self.data if v == value)

    def summary(self) -> str:
        total = self.width * self.height
        unknown = self.count(UNKNOWN)
        occupied = sum(1 for v in self.data if v >= 65)
        free = sum(1 for v in self.data if 0 <= v < 65)
        return (
            f"{self.width}x{self.height} @ {self.resolution} m/格 "
            f"（{self.width_m:.1f}m x {self.height_m:.1f}m）"
            f"  未知 {unknown / total:.1%} / 空闲 {free / total:.1%} / 占据 {occupied / total:.1%}"
        )

    @property
    def width_m(self) -> float:
        return self.width * self.resolution

    @property
    def height_m(self) -> float:
        return self.height * self.resolution

    def __repr__(self) -> str:
        return f"GridMap({self.width}x{self.height}, res={self.resolution}, origin={self.origin_x, self.origin_y})"


# ---------------------------------------------------------------- 文件解析


def _parse_simple_yaml(text: str) -> dict:
    """只解析 ROS 地图 YAML 需要的那几个标量/列表字段（不依赖 PyYAML）。"""
    fields: dict = {}
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        fields[key.strip()] = value.strip()
    return fields


def _parse_origin(value: str) -> Tuple[float, float, float]:
    numbers = re.findall(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?", value)
    if len(numbers) < 2:
        raise ValueError(f"无法解析 origin: {value!r}")
    return float(numbers[0]), float(numbers[1]), float(numbers[2]) if len(numbers) > 2 else 0.0


def _read_pgm(path: Path) -> Tuple[int, int, bytes]:
    """读取 P5 二进制 PGM，返回 (宽, 高, 像素)。"""
    with open(path, "rb") as fp:
        magic = fp.readline().strip()
        if magic != b"P5":
            raise ValueError(f"{path} 不是 P5 PGM（magic={magic!r}）")
        line = fp.readline()
        while line.startswith(b"#"):
            line = fp.readline()
        width, height = (int(v) for v in line.split())
        maxval = int(fp.readline().strip())
        if maxval != 255:
            raise ValueError(f"{path} 只支持 maxval=255，实际 {maxval}")
        pixels = fp.read(width * height)
    if len(pixels) != width * height:
        raise ValueError(f"{path} 像素数据不完整")
    return width, height, pixels
