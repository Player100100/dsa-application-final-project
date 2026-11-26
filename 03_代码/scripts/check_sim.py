#!/usr/bin/env python3
"""仿真/真车链路预检：TCP → WebSocket → SDK → 首帧数据。

在把导航脚本指向虚拟机之前先跑这个，能区分“网络不通”“rosbridge 没起”
和“SDK/话题有问题”，不用靠猜。

用法::

    python scripts/check_sim.py --ip 192.168.150.128
    python scripts/check_sim.py --ip mock          # 用本机假 rosbridge 自测脚本

退出码：0 全通 / 1 TCP 不通 / 2 WebSocket 握手失败 / 3 SDK 或话题异常。
"""

from __future__ import annotations

import argparse
import base64
import math
import os
import socket
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_io import RobotIO, ensure_sdk  # noqa: E402

MOCK_NAMES = ("mock", "demo", "fake", "local")


def tcp_check(host: str, port: int, timeout: float) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            print(f"[ok] TCP {host}:{port} 可连接")
            return True
    except OSError as error:
        print(f"[x] TCP {host}:{port} 连不上：{error}")
        return False


def ws_check(host: str, port: int, timeout: float) -> bool:
    """发一个标准 WebSocket Upgrade 请求，看 rosbridge 是否应答 101。"""
    key = base64.b64encode(os.urandom(16)).decode("ascii")
    request = (
        f"GET / HTTP/1.1\r\n"
        f"Host: {host}:{port}\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\n"
        "Sec-WebSocket-Version: 13\r\n\r\n"
    )
    try:
        with socket.create_connection((host, port), timeout=timeout) as sock:
            sock.settimeout(timeout)
            sock.sendall(request.encode("ascii"))
            reply = b""
            while b"\r\n\r\n" not in reply and len(reply) < 4096:
                chunk = sock.recv(1024)
                if not chunk:
                    break
                reply += chunk
    except OSError as error:
        print(f"[x] WebSocket 握手时断线：{error}")
        return False

    status = reply.split(b"\r\n", 1)[0].decode("latin-1", "replace")
    if " 101 " in status or status.endswith(" 101"):
        print(f"[ok] WebSocket 握手成功（{status}）")
        return True
    print(f"[x] 端口通了，但不是 rosbridge 的 WebSocket 应答：{status!r}")
    print("    常见原因：端口被别的服务占用，或地址写错。")
    return False


def sdk_check(host: str, port: int, timeout: float, scan_topic=None, expect_origin=False) -> bool:
    try:
        robot = RobotIO(host, port, settle=0.3, timeout=timeout, scan_topic=scan_topic)
    except Exception as error:  # noqa: BLE001
        print(f"[x] SDK 连接失败：{error}")
        return False
    try:
        frame = robot.wait_scan(3.0)
        pose = robot.pose()
        print("[i] 诊断：")
        print(robot.diagnose_text())
        if frame is None:
            print("[x] 连上了，但 3 秒内没有 /scan。检查仿真是否启动、传感器话题是否存在。")
            return False
        print(
            f"[ok] /scan：{frame.count} 束，frame_id={frame.frame_id}；"
            f" 位姿={'无' if pose is None else f'({pose.x:+.2f}, {pose.y:+.2f}, {pose.yaw:+.2f})'}"
        )
        if pose is None:
            print("[x] 连上了，但 /odom 没有数据；仿真里程计节点可能没起来。")
            return False
        if expect_origin and math.hypot(pose.x, pose.y) > 0.35:
            print(f"[x] 机器人不在原点附近（{pose.x:+.2f},{pose.y:+.2f}），仿真没有重置干净。")
            return False
        try:
            offset, mount = robot.laser_mount()
            print(f"[ok] 雷达安装：偏航 {offset:+.4f} rad，位置 {mount}")
        except Exception as error:  # noqa: BLE001
            print(f"[!] 坐标变换查询失败（不影响建图，但无法自动换算车头方向）：{error}")
        return True
    finally:
        robot.close()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="仿真/真车链路预检")
    parser.add_argument("--ip", required=True, help="虚拟机/真车 IP；mock 表示本机假 rosbridge")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--scan-topic", default=None, help="激光话题（默认 /scan）")
    parser.add_argument("--expect-origin", action="store_true", help="要求机器人在原点 ±0.35 m 内")
    args = parser.parse_args(argv)

    host, port = args.ip, args.port
    mock_server = None
    if host.lower() in MOCK_NAMES:
        ensure_sdk()
        from tools.mock_rosbridge import MockRosbridgeServer  # type: ignore

        mock_server = MockRosbridgeServer(port=0, verbose=False).start()
        host, port = "127.0.0.1", mock_server.port
        print(f"[i] 使用本机 mock：{host}:{port}")

    try:
        if not tcp_check(host, port, args.timeout):
            return 1
        if not ws_check(host, port, args.timeout):
            return 2
        if not sdk_check(host, port, args.timeout, args.scan_topic, args.expect_origin):
            return 3
        print("\n预检通过：可以把 nav_demo 的 --ip 指向这个地址了。")
        return 0
    finally:
        if mock_server is not None:
            mock_server.stop()


if __name__ == "__main__":
    raise SystemExit(main())
