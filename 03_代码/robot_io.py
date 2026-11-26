"""机器人 I/O 适配层：把厂商 SDK 收窄成项目需要的那几个动作。

为什么要有这一层
----------------
任务书考核的是数据结构与算法，换车/换 SDK 时不应重写算法。所有对
``rosmaster_client`` 的依赖都集中在这里；上层只认 ``RobotIO`` 的接口。

支持两种运行方式
----------------
- ``RobotIO("mock")``：本机起一个假 rosbridge（厂商 SDK 自带），不需要真车/网络。
- ``RobotIO("192.168.x.x", 9090)``：连仿真或真车。

SDK 查找顺序
------------
1. 已安装的 ``rosmaster_client``（``pip install`` 过 wheel）；
2. 环境变量 ``ROSMASTER_SDK_ROOT`` 指向解压后的源码根目录；
3. 工作区内 ``用户传入/数据结构与算法应用/客户端v0.3/.../rosmaster-client-0.3.0``。
"""

from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

_MOCK_NAMES = ("mock", "demo", "fake", "local")


def _candidate_sdk_roots():
    env = os.environ.get("ROSMASTER_SDK_ROOT")
    if env:
        yield Path(env)
    here = Path(__file__).resolve()
    for base in here.parents:
        yield (
            base
            / "用户传入"
            / "数据结构与算法应用"
            / "客户端v0.3"
            / "rosmaster-client-0.3.0-src"
            / "rosmaster-client-0.3.0-src"
            / "rosmaster-client-0.3.0"
        )


def _find_source_root() -> Optional[Path]:
    for root in _candidate_sdk_roots():
        if (root / "src" / "rosmaster_client").is_dir():
            return root
    return None


def ensure_sdk():
    """导入并返回 ``rosmaster_client`` 模块，必要时把源码路径加进 ``sys.path``。

    即使 wheel 已安装，也会把源码根目录放进 ``sys.path``，因为 mock 用的
    ``tools.mock_rosbridge`` 只随源码发布、不在 wheel 里。
    """
    try:
        import rosmaster_client  # type: ignore
    except ImportError:
        root = _find_source_root()
        if root is None:
            raise ImportError(
                "找不到 rosmaster_client。请先 `pip install rosmaster_client-0.3.0-py3-none-any.whl`，"
                "或设置环境变量 ROSMASTER_SDK_ROOT 指向解压后的 SDK 源码根目录。"
            ) from None
        for path in (str(root / "src"), str(root)):
            if path not in sys.path:
                sys.path.insert(0, path)
        import rosmaster_client  # type: ignore

    root = _find_source_root()
    if root is not None and str(root) not in sys.path:
        sys.path.insert(0, str(root))
    return rosmaster_client


class RobotIO:
    """项目内唯一的机器人接口。所有方法都是同步的。"""

    def __init__(
        self,
        host: str = "mock",
        port: int = 9090,
        *,
        settle: float = 0.3,
        timeout: float = 5.0,
        scan_topic: Optional[str] = None,
        odom_topic: Optional[str] = None,
    ) -> None:
        sdk = ensure_sdk()
        self._mock_server = None
        self.is_mock = str(host).lower() in _MOCK_NAMES
        if self.is_mock:
            from tools.mock_rosbridge import MockRosbridgeServer  # type: ignore

            self._mock_server = MockRosbridgeServer(port=0, verbose=False).start()
            host, port = "127.0.0.1", self._mock_server.port

        self.host, self.port = host, port
        extra = {}
        if scan_topic:
            extra["scan_topic"] = scan_topic
        if odom_topic:
            extra["odom_topic"] = odom_topic
        self.client = sdk.RobotClient(
            host, port, settle=settle, timeout=timeout, **extra
        )
        self._laser_offset: Optional[float] = None
        self._laser_mount: Optional[Tuple[float, float]] = None

    # ------------------------------------------------------------ 生命周期
    def close(self) -> None:
        """先发零速停车，再断开连接；重复调用安全。"""
        try:
            if self.client.is_connected:
                self.client.stop()
        except Exception:
            pass
        try:
            self.client.terminate_connect()
        finally:
            if self._mock_server is not None:
                self._mock_server.stop()
                self._mock_server = None

    def __enter__(self) -> "RobotIO":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()

    @property
    def connected(self) -> bool:
        return bool(self.client.is_connected)

    # ------------------------------------------------------------ 数据
    def scan(self):
        """获取并消耗最新一帧 ``/scan``；没有新帧返回 ``None``。"""
        return self.client.get_scan()

    def wait_scan(self, timeout: float = 3.0):
        return self.client.wait_for_scan(timeout)

    def pose(self):
        """最新里程计位姿（``OdomPose``），没有数据返回 ``None``。"""
        return self.client.get_odom_pos()

    def imu(self):
        return self.client.get_imu()

    def laser_mount(self, refresh: bool = False) -> Tuple[float, Tuple[float, float]]:
        """返回 ``(偏航角, (安装 x, 安装 y))``，默认缓存，停车时查一次即可。"""
        if refresh or self._laser_offset is None or self._laser_mount is None:
            tf = self.client.get_transform("base_link", "laser_link")
            q = tf.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            self._laser_offset = yaw
            self._laser_mount = (float(tf.translation.x), float(tf.translation.y))
        return self._laser_offset, self._laser_mount

    def diagnose_text(self) -> str:
        return self.client.diagnose_text()

    # ------------------------------------------------------------ 控制
    def wait_connected(self, timeout: float = 3.0) -> bool:
        """等待底层 WebSocket 重新连上（roslibpy 会自动重连）。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.client.is_connected:
                return True
            time.sleep(0.1)
        return bool(self.client.is_connected)

    def send(self, vx: float, vy: float = 0.0, wz: float = 0.0) -> None:
        """发一帧速度；自动拦掉 NaN/Inf。远端看门狗约 0.5 s 后自动停车。

        网络短暂抖动时先等重连再试一次，避免一拍断开就整场退出。
        """
        for name, value in (("vx", vx), ("vy", vy), ("wz", wz)):
            if not math.isfinite(value):
                raise ValueError(f"{name} 不是有限数值：{value!r}")
        if not self.client.is_connected and not self.wait_connected(3.0):
            raise RuntimeError("与机器人的连接不可用，等待 3 s 仍未重连")
        try:
            self.client.set_speed(vx, vy, wz)
        except Exception:
            if not self.wait_connected(3.0):
                raise
            self.client.set_speed(vx, vy, wz)

    def stop(self) -> None:
        """立即发送零速。"""
        try:
            self.client.stop()
        except Exception:
            pass

    def __repr__(self) -> str:
        kind = "mock" if self.is_mock else "live"
        return f"RobotIO({kind}, {self.host}:{self.port}, connected={self.connected})"
