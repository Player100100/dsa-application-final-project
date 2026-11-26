"""课程项目的核心数据结构与算法包（纯标准库，不依赖 ROS / 网络）。

模块
----
- ``ring_buffer``：定长环形缓冲区（线性表）
- ``grid_map``：二维占据栅格 + ROS 地图读写（数组）
- ``mapping``：激光 → 栅格在线建图
- ``kdtree``：KD-Tree 空间索引与欧氏聚类（树）
- ``astar``：Dijkstra / A* 网格路径规划（图）
"""

from .astar import PathResult, astar, clear_around, dijkstra, path_to_world, smooth_path
from .grid_map import FREE, OCCUPIED, UNKNOWN, GridMap
from .kdtree import KDTree, brute_force_knn
from .mapping import ScanMapper, bresenham
from .ring_buffer import RingBuffer

__all__ = [
    "RingBuffer",
    "GridMap",
    "UNKNOWN",
    "FREE",
    "OCCUPIED",
    "ScanMapper",
    "bresenham",
    "KDTree",
    "brute_force_knn",
    "astar",
    "clear_around",
    "dijkstra",
    "PathResult",
    "path_to_world",
    "smooth_path",
]
