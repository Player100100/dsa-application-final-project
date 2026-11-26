#!/usr/bin/env python3
"""从 VMware 的 DHCP 租约里找虚拟机的 IP。

虚拟机用 NAT 时，宿主机不需要进系统就能知道它拿到了哪个地址：
读 `.vmx` 里的网卡 MAC，再到 `vmnetdhcp.leases` 里匹配。

用法::

    python scripts/find_vm_ip.py --vmx "F:\\...\\ros2-masterX3.vmx"
    python scripts/find_vm_ip.py --mac 00:0c:29:xx:xx:xx

注意：只有虚拟机**开机并拿到租约**后才有记录；关机后旧租约仍会保留，
所以脚本会按 `starts` 时间排序，最新的一条最可信。
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import List, Tuple

DEFAULT_LEASES = Path(r"C:\ProgramData\VMware\vmnetdhcp.leases")


def mac_from_vmx(vmx_path: Path) -> str:
    text = vmx_path.read_text(encoding="utf-8", errors="replace")
    for key in ("ethernet0.address", "ethernet0.generatedAddress"):
        match = re.search(rf"^{re.escape(key)}\s*=\s*\"([^\"]+)\"", text, re.MULTILINE)
        if match:
            return match.group(1).lower()
    raise ValueError(f"{vmx_path} 里找不到 ethernet0 的 MAC")


def parse_leases(path: Path) -> List[Tuple[str, str, str, str]]:
    """返回 [(starts, ip, mac, hostname), ...]，按开始时间从旧到新排序。"""
    text = path.read_text(encoding="utf-8", errors="replace")
    results: List[Tuple[str, str, str, str]] = []
    for block in re.finditer(r"lease\s+([\d.]+)\s*\{(.*?)\}", text, re.DOTALL):
        ip, body = block.group(1), block.group(2)
        mac_match = re.search(r"hardware ethernet\s+([0-9a-fA-F:]+);", body)
        # 格式为 "starts <星期> YYYY/MM/DD HH:MM:SS;"，星期不能参与排序。
        start_match = re.search(
            r"starts\s+\d+\s+(\d{4}/\d{2}/\d{2}\s+\d{2}:\d{2}:\d{2});", body
        )
        host_match = re.search(r'client-hostname\s+"([^"]*)"', body)
        if not mac_match:
            continue
        results.append(
            (
                start_match.group(1) if start_match else "",
                ip,
                mac_match.group(1).lower(),
                host_match.group(1) if host_match else "",
            )
        )
    results.sort(key=lambda item: item[0])
    return results


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="从 VMware DHCP 租约查虚拟机 IP")
    parser.add_argument("--vmx", help="虚拟机 .vmx 路径")
    parser.add_argument("--mac", help="直接给 MAC（与 --vmx 二选一）")
    parser.add_argument("--leases", default=str(DEFAULT_LEASES), help="租约文件路径")
    args = parser.parse_args(argv)

    if not args.vmx and not args.mac:
        parser.error("需要 --vmx 或 --mac")
    mac = args.mac.lower() if args.mac else mac_from_vmx(Path(args.vmx))
    leases_path = Path(args.leases)
    if not leases_path.exists():
        print(f"[x] 找不到租约文件：{leases_path}")
        return 1

    matches = [item for item in parse_leases(leases_path) if item[2] == mac]
    print(f"[i] MAC = {mac}")
    if not matches:
        print("[x] 没有该 MAC 的租约。检查：虚拟机是否开机、网卡是否连在 NAT。")
        return 2
    for starts, ip, _mac, hostname in matches:
        label = f"  hostname={hostname}" if hostname else ""
        print(f"[ok] {ip}   {starts}{label}")
    print(f"[i] 最新一条：{matches[-1][1]}（以它为准）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
