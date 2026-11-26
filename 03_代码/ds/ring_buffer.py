"""定长环形缓冲区（顺序表实现），用于缓存最近 N 帧传感器数据。

对应任务书「线性表 → 传感器数据流管理 → 基础级」。

设计要点
--------
- 容量固定，写满后新数据覆盖最旧数据（FIFO 覆盖），不扩容。
- ``append`` O(1)；``latest`` O(1)；``snapshot`` O(n) 且按时间顺序返回副本。
- ``dropped`` 记录被覆盖掉的帧数，方便在报告里量化“丢帧”。

为什么用顺序表而不是链表：传感器流是“只追加、只取最近”，没有中间插入/删除，
定长数组 + 两个下标即可，缓存局部性也更好。需要任意位置增删的轨迹点请用
双向链表（任务书线性表进阶级，本项目暂未实现）。
"""

from __future__ import annotations

from typing import Generic, Iterator, List, Optional, TypeVar

T = TypeVar("T")


class RingBuffer(Generic[T]):
    """定长环形缓冲区。

    参数
    ----
    capacity:
        最大保存的元素个数，必须为正整数。
    """

    __slots__ = ("_data", "_capacity", "_head", "_size", "_total", "_dropped")

    def __init__(self, capacity: int) -> None:
        if not isinstance(capacity, int) or capacity <= 0:
            raise ValueError("capacity 必须是正整数")
        self._data: List[Optional[T]] = [None] * capacity
        self._capacity = capacity
        self._head = 0          # 下一个写入位置
        self._size = 0          # 当前元素个数
        self._total = 0         # 历史写入总数
        self._dropped = 0       # 因覆盖而丢弃的帧数

    # ------------------------------------------------------------------ 写
    def append(self, item: T) -> None:
        """写入一帧；若已满则覆盖最旧的一帧。"""
        if self._size == self._capacity:
            self._dropped += 1
        else:
            self._size += 1
        self._data[self._head] = item
        self._head = (self._head + 1) % self._capacity
        self._total += 1

    def clear(self) -> None:
        """清空内容，保留容量与累计统计。"""
        self._data = [None] * self._capacity
        self._head = 0
        self._size = 0

    # ------------------------------------------------------------------ 读
    def latest(self, n: int = 1) -> List[T]:
        """返回最近 ``n`` 帧，按时间从旧到新排列；不足则返回现有的全部。"""
        if n <= 0:
            return []
        n = min(n, self._size)
        out: List[T] = []
        start = (self._head - n) % self._capacity
        for i in range(n):
            out.append(self._data[(start + i) % self._capacity])  # type: ignore[arg-type]
        return out

    def newest(self) -> Optional[T]:
        """返回最新一帧；空缓冲区返回 ``None``。"""
        if self._size == 0:
            return None
        return self._data[(self._head - 1) % self._capacity]

    def oldest(self) -> Optional[T]:
        """返回最旧一帧；空缓冲区返回 ``None``。"""
        if self._size == 0:
            return None
        start = (self._head - self._size) % self._capacity
        return self._data[start]

    def snapshot(self) -> List[T]:
        """返回全部内容的列表副本（从旧到新）。"""
        return self.latest(self._size)

    def __len__(self) -> int:
        return self._size

    def __iter__(self) -> Iterator[T]:
        return iter(self.snapshot())

    def __bool__(self) -> bool:
        return self._size > 0

    # -------------------------------------------------------------- 统计
    @property
    def capacity(self) -> int:
        return self._capacity

    @property
    def dropped(self) -> int:
        """被覆盖掉的旧帧总数。"""
        return self._dropped

    @property
    def total_appended(self) -> int:
        """历史写入总帧数。"""
        return self._total

    @property
    def is_full(self) -> bool:
        return self._size == self._capacity

    def __repr__(self) -> str:
        return (
            f"RingBuffer(capacity={self._capacity}, size={self._size}, "
            f"dropped={self._dropped})"
        )
