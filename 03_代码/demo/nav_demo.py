#!/usr/bin/env python3
"""端到端演示：激光建图 → A* 规划 → 10 Hz 闭环跟随。

这是冲刺路线的“主线脚本”，把四个数据结构模块串起来：

    /scan ──► ScanMapper（数组）──► GridMap ──► A*（图）
                        │                              │
    /odom ──────────────┘                              ▼
                                             路径跟随（纯追踪）
                                                       │
                                                  /cmd_vel（10 Hz）

用法::

    python demo/nav_demo.py                              # mock 环境，目标 (2, 2)
    python demo/nav_demo.py --goal 2.0 2.0 --duration 90
    python demo/nav_demo.py --ip 192.168.244.131 --goal 1.5 0.0

注意：``--ip`` 不是 mock 时小车会真的移动，先确认场地与急停。
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from ds.astar import astar, clear_around, path_to_world, smooth_path  # noqa: E402
from ds.cluster_tracker import ClusterTracker  # noqa: E402
from ds.kdtree import KDTree  # noqa: E402
from ds.mapping import ScanMapper  # noqa: E402
from ds.ring_buffer import RingBuffer  # noqa: E402
from robot_io import RobotIO  # noqa: E402


def norm_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def sector_range(frame, yaw_offset: float, center: float, half_angle: float) -> float:
    """车体坐标系下、以 ``center`` 为中心 ±``half_angle`` 扇形内的最近回波。"""
    best = math.inf
    for i in range(frame.count):
        r = frame.ranges[i]
        if not (math.isfinite(r) and frame.range_min <= r <= frame.range_max):
            continue
        bearing = norm_angle(frame.angle_at(i) + yaw_offset - center)
        if abs(bearing) <= half_angle and r < best:
            best = r
    return best


def front_range(frame, yaw_offset: float, half_angle: float = 0.35) -> float:
    """车头前方扇形内最近回波；没有回波返回 ``inf``。"""
    return sector_range(frame, yaw_offset, 0.0, half_angle)


def scan_points_body(frame, yaw_offset: float, mount, stride: int = 4):
    """把有效回波转到车体系（+x 车头、+y 车左），返回点列表。"""
    mx, my = mount
    points = []
    for i in range(0, frame.count, stride):
        r = frame.ranges[i]
        if not (math.isfinite(r) and frame.range_min <= r <= frame.range_max):
            continue
        bearing = frame.angle_at(i) + yaw_offset
        points.append((mx + r * math.cos(bearing), my + r * math.sin(bearing)))
    return points


def nearest_cluster(tree: KDTree, clusters, max_distance: float):
    """返回落在车头 ±0.6 rad、距离 < max_distance 的最近簇中心（没有则 None）。"""
    best = None
    for members in clusters:
        cx = sum(tree.points[i][0] for i in members) / len(members)
        cy = sum(tree.points[i][1] for i in members) / len(members)
        dist = math.hypot(cx, cy)
        if dist < max_distance and abs(math.atan2(cy, cx)) < 0.6:
            if best is None or dist < best[0]:
                best = (dist, cx, cy, len(members))
    return best


def choose_lookahead(path, pose, lookahead: float):
    """在路径上找离车最近点，再向前取 lookahead 距离的点。"""
    if not path:
        return None
    px, py = pose.x, pose.y
    nearest = min(range(len(path)), key=lambda i: (path[i][0] - px) ** 2 + (path[i][1] - py) ** 2)
    if nearest == len(path) - 1:
        return path[-1]
    tx, ty = path[-1]
    for i in range(nearest, len(path)):
        if math.hypot(path[i][0] - px, path[i][1] - py) >= lookahead:
            tx, ty = path[i]
            break
    return tx, ty


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="建图 + A* + 闭环导航（mock 可直接跑）")
    parser.add_argument("--ip", default="mock", help="小车 IP；mock 表示本机假 rosbridge")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--scan-topic", default=None, help="激光话题（默认 /scan；仿真可试 /internal/scan）")
    parser.add_argument("--odom-topic", default=None, help="里程计话题（默认 /odom；仿真可试 /ground_truth/odom）")
    parser.add_argument("--goal", type=float, nargs=2, default=(2.0, 2.0), metavar=("X", "Y"))
    parser.add_argument("--duration", type=float, default=90.0, help="最长运行秒数")
    parser.add_argument("--speed", type=float, default=0.18, help="巡航线速度 m/s")
    parser.add_argument("--resolution", type=float, default=0.05, help="地图分辨率 m/格")
    parser.add_argument("--inflate", type=float, default=0.25, help="规划膨胀半径 m")
    parser.add_argument("--stop-distance", type=float, default=0.35, help="车头多近硬停 m")
    parser.add_argument("--cluster-slow-distance", type=float, default=0.65, help="障碍簇多近减速 m")
    parser.add_argument("--lookahead", type=float, default=0.45, help="纯追踪前视距离 m")
    parser.add_argument("--front-half-angle", type=float, default=0.35, help="前向危险扇区半角 rad")
    parser.add_argument("--min-inflate", type=float, default=0.0, help="规划膨胀下限 m（防止回退到 0 后钻过窄缝）")
    parser.add_argument("--dynamic-speed", type=float, default=0.25, help="判定动态障碍的速度阈值 m/s")
    parser.add_argument("--simulate-dynamic", action="store_true", help="在感知里注入一个横向移动的假障碍（用于测试动态逻辑）")
    parser.add_argument("--trace", default=None, help="把每拍位姿/速度写成 CSV（用于画图/动画）")
    parser.add_argument("--map-out", default="04_实验/地图_导航_模拟.yaml", help="结束时保存的地图")
    parser.add_argument("--no-save", action="store_true", help="不保存地图")
    parser.add_argument("--debug", action="store_true", help="状态行额外打印航向误差/前向距离/硬停保持")
    args = parser.parse_args(argv)

    if str(args.ip).lower() not in ("mock", "demo", "fake", "local"):
        print("[!] 真车模式：3 秒后开始，确认场地空旷、可随时急停 ...")
        time.sleep(3.0)

    if args.debug:
        print(
            f"[debug] ip={args.ip!r} port={args.port!r} "
            f"scan_topic={args.scan_topic!r} odom_topic={args.odom_topic!r}"
        )
    trace_fp = None
    if args.trace:
        trace_path = Path(args.trace)
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        trace_fp = trace_path.open("w", encoding="utf-8", newline="")
        trace_fp.write("t,x,y,yaw,vx,wz\n")

    robot = None
    for attempt in range(1, 4):
        try:
            robot = RobotIO(
                args.ip,
                args.port,
                scan_topic=args.scan_topic,
                odom_topic=args.odom_topic,
            )
            break
        except Exception as error:  # noqa: BLE001 - 启动期 rosbridge 可能还没就绪
            print(f"[!] 连接失败（第 {attempt}/3 次）：{error}")
            time.sleep(3.0)
    if robot is None:
        print("[x] 3 次都没连上 rosbridge，检查仿真/网络")
        return 1
    success = False
    replans = 0
    last_result = None
    mapper = None
    try:
        first = robot.wait_scan(5.0)
        if first is None:
            print("[x] 5 秒内没有 /scan，检查环境")
            return 1
        pose_deadline = time.monotonic() + 5.0
        while robot.pose() is None and time.monotonic() < pose_deadline:
            time.sleep(0.1)
        if robot.pose() is None:
            print("[x] 5 秒内没有 /odom 位姿，仿真里程计可能没起；重启仿真栈再试")
            return 1
        offset, mount = robot.laser_mount()
        print(f"[i] 雷达安装偏航 {offset:+.4f} rad，位置 {mount}")

        mapper = ScanMapper(
            laser_offset_yaw=offset,
            mount_xy=mount,
            resolution=args.resolution,
            width_m=8.0,
            height_m=8.0,
        )
        grid = mapper.grid
        goal_cell = grid.world_to_cell(*args.goal)
        if not grid.in_bounds(*goal_cell):
            print(f"[x] 目标 {args.goal} 超出 8x8 m 地图范围")
            return 1

        scan_history = RingBuffer(10)
        cluster_info = None
        cluster_count = 0
        dynamic_count = 0
        dynamic_track = None
        dynamic_turn_until = 0.0
        tracker = ClusterTracker(dynamic_speed=args.dynamic_speed, gate=0.5, min_hits=3)
        last_cluster = -1.0
        avoid_sign = 1.0         # 避障转向方向（带 0.8 s 保持，防左右抖）
        avoid_until = 0.0
        stop_hold_until = 0.0    # 硬停后保持一小段时间，避免一帧一帧地“挪”
        escape_until = 0.0       # 贴脸时“转-退”交替的节拍
        escape_back = False
        reverse_until = 0.0      # 卡死后短暂倒车
        progress_pose = None     # 用来判断“2 秒没挪窝”
        progress_at = 0.0
        last_good_plan = []      # 最近一次可行的路径
        last_good_plan_at = 0.0
        plan_stale = False

        # 原地热身：先收几帧，把周围环境画进地图
        for _ in range(6):
            frame = robot.wait_scan(1.0)
            pose = robot.pose()
            if frame is not None and pose is not None:
                mapper.update(frame, pose)
        print(f"[i] 热身完成：{mapper.stats()}")
        if args.debug:
            p0 = robot.pose()
            print(
                "[debug] warmup pose="
                + ("None" if p0 is None else f"({p0.x:+.2f},{p0.y:+.2f},{p0.yaw:+.2f})")
            )

        started = time.monotonic()
        next_tick = started
        last_plan = -1.0
        plan_world = []
        path_cost = math.inf
        frames = 0
        dist_goal = math.inf

        while time.monotonic() - started < args.duration:
            now = time.monotonic()
            frame = robot.scan()
            pose = robot.pose()
            if frame is not None:
                scan_history.append(frame)
            if frame is not None and pose is not None:
                mapper.update(frame, pose)
                frames += 1

            if pose is None:
                time.sleep(0.02)
                continue

            dist_goal = math.hypot(pose.x - args.goal[0], pose.y - args.goal[1])
            if dist_goal < 0.30:
                success = True
                print(f"[ok] 到达目标，用时 {time.monotonic() - started:.1f} s，剩余 {dist_goal:.2f} m")
                robot.stop()
                break

            # 每 0.8 秒重规划一次；失败时依次降膨胀、解阻起点邻域
            if now - last_plan >= 0.8:
                last_plan = now
                grid = mapper.build_grid()
                start_cell = grid.world_to_cell(pose.x, pose.y)
                found_plan = []
                inflations = tuple(
                    max(args.min_inflate, v)
                    for v in (args.inflate, max(0.10, args.inflate * 0.6), 0.0)
                )
                for inflate in inflations:
                    blocked = grid.inflated_mask(inflate_m=inflate)
                    last_result = astar(grid, start_cell, goal_cell, blocked=blocked)
                    replans += 1
                    if last_result.found:
                        found_plan = smooth_path(path_to_world(grid, last_result.path))
                        path_cost = last_result.cost * grid.resolution
                        break
                if not found_plan:
                    # 车贴墙时起点 3×3 邻域可能全在膨胀带里：临时解阻再试
                    for inflate in inflations:
                        blocked = grid.inflated_mask(inflate_m=inflate)
                        blocked = clear_around(
                            blocked, grid.width, grid.height, start_cell, radius=1
                        )
                        last_result = astar(grid, start_cell, goal_cell, blocked=blocked)
                        replans += 1
                        if last_result.found:
                            found_plan = smooth_path(path_to_world(grid, last_result.path))
                            path_cost = last_result.cost * grid.resolution
                            break
                if found_plan:
                    plan_world = found_plan
                    last_good_plan = found_plan
                    last_good_plan_at = now
                    plan_stale = False
                elif last_good_plan and now - last_good_plan_at < 2.5:
                    # 规划暂时失败：先沿旧路径走 2.5 s，别立刻原地转
                    plan_world = last_good_plan
                    plan_stale = True
                else:
                    plan_world = []
                    plan_stale = True

            # 控制
            error = 0.0
            if plan_world:
                target = choose_lookahead(plan_world, pose, lookahead=args.lookahead)
                if target is not None:
                    heading = math.atan2(target[1] - pose.y, target[0] - pose.x)
                else:
                    heading = math.atan2(args.goal[1] - pose.y, args.goal[0] - pose.x)
                raw_error = norm_angle(heading - pose.yaw)
                # 小角度死区 + 降增益，减少窄道里的左右摆头
                error = 0.0 if abs(raw_error) < 0.06 else raw_error
                wz = max(-1.0, min(1.0, 1.8 * error))
                vx = args.speed * max(0.25, 1.0 - abs(error) / 1.1)
                if abs(error) > 0.9:
                    vx = 0.0
                if plan_stale:
                    vx *= 0.7
            else:
                # 没有路径：原地转，继续探索
                vx, wz = 0.0, 0.7

            # KD-Tree 障碍物聚类（每 0.5 秒一次，提供“簇”级别的近障判断）
            if frame is not None and now - last_cluster >= 0.5:
                last_cluster = now
                body_points = scan_points_body(frame, offset, mount, stride=6)
                if args.simulate_dynamic:
                    # 车头 0.9 m、左右摆动，速度约 0.5 m/s；只进聚类，不进地图
                    lateral = 0.45 * math.sin(1.2 * (now - started))
                    body_points = list(body_points) + [
                        (0.9, lateral + i * 0.01) for i in range(5)
                    ]
                if body_points:
                    tree = KDTree(body_points)
                    clusters = tree.radius_clusters(0.20, min_points=3)
                    cluster_count = len(clusters)
                    cluster_info = nearest_cluster(tree, clusters, args.cluster_slow_distance)
                    # 跟踪用世界坐标：静止墙不会因为自车运动被误判成动态
                    cos_yaw, sin_yaw = math.cos(pose.yaw), math.sin(pose.yaw)
                    world_points = [
                        (
                            pose.x + bx * cos_yaw - by * sin_yaw,
                            pose.y + bx * sin_yaw + by * cos_yaw,
                        )
                        for bx, by in body_points
                    ]
                    tracks = tracker.update(world_points, clusters, now)
                    dynamic_count = sum(1 for t in tracks if t.dynamic)
                    dynamic_track = tracker.dynamic_near(
                        (pose.x, pose.y), pose.yaw, now, max_distance=1.0
                    )
                else:
                    cluster_count = 0
                    cluster_info = None
                    dynamic_count = 0
                    dynamic_track = None
                    tracker.update([], [], now)

            # 安全层：极近则停（带 0.45 s 保持），有障碍簇则减速
            near = (
                front_range(frame, offset, args.front_half_angle)
                if frame is not None
                else math.inf
            )
            if progress_pose is None:
                progress_pose = (pose.x, pose.y)
                progress_at = now
            elif math.hypot(pose.x - progress_pose[0], pose.y - progress_pose[1]) > 0.03:
                progress_pose = (pose.x, pose.y)
                progress_at = now
            elif (
                now - progress_at > 2.0
                and now >= reverse_until
                and near < max(0.30, args.stop_distance + 0.12)
            ):
                # 2 秒没挪窝且前方有东西：倒车 0.6 s 脱困
                reverse_until = now + 0.6
                progress_at = now

            critical = near < min(0.10, args.stop_distance * 0.6)
            if now < reverse_until:
                vx, wz = -0.08, 0.0
            elif critical:
                # 已经贴脸：转-退交替，先脱困再谈路径
                if now >= escape_until:
                    escape_back = not escape_back
                    escape_until = now + (0.55 if escape_back else 0.4)
                if escape_back:
                    vx, wz = -0.08, 0.0
                else:
                    vx = 0.0
                    if now >= avoid_until:
                        left = sector_range(frame, offset, math.pi / 2, 0.6)
                        right = sector_range(frame, offset, -math.pi / 2, 0.6)
                        avoid_sign = 1.0 if left >= right else -1.0
                        avoid_until = now + 0.6
                    wz = avoid_sign * 1.0
                stop_hold_until = now + 0.15
            elif now < stop_hold_until:
                vx = 0.0
            elif frame is not None and vx > 0.0:
                if near < args.stop_distance:
                    vx = 0.0
                    stop_hold_until = now + 0.45
                    # 优先让路径跟随自己转出去；只有它也说“直行”时才启用避障转向。
                    if abs(wz) < 0.15:
                        if now >= avoid_until:
                            left = sector_range(frame, offset, math.pi / 2, 0.6)
                            right = sector_range(frame, offset, -math.pi / 2, 0.6)
                            avoid_sign = 1.0 if left >= right else -1.0
                            avoid_until = now + 0.8
                        wz = avoid_sign * 0.9
                elif cluster_info is not None:
                    vx *= 0.5  # 障碍簇在车头减速区内：只减速，不硬停

            # 动态障碍：比静态簇更保守，近了直接停车并向反侧转
            if dynamic_track is not None and now >= dynamic_turn_until:
                dx = dynamic_track.centroid[0] - pose.x
                dy = dynamic_track.centroid[1] - pose.y
                dist_dyn = math.hypot(dx, dy)
                # 车体系 y>0 表示障碍在左侧 → 向右转
                rel_y = -math.sin(pose.yaw) * dx + math.cos(pose.yaw) * dy
                if dist_dyn < 0.60:
                    vx = 0.0
                    wz = -1.0 if rel_y > 0 else 1.0
                    dynamic_turn_until = now + 0.5
                elif dist_dyn < 1.00:
                    vx *= 0.4

            try:
                robot.send(vx, 0.0, wz)
            except Exception as error:  # noqa: BLE001 - 网络问题不应带崩整场
                print(f"[x] 控制指令发送失败，结束本次运行：{error}")
                break

            if trace_fp is not None:
                trace_fp.write(
                    f"{now - started:.3f},{pose.x:.4f},{pose.y:.4f},"
                    f"{pose.yaw:.4f},{vx:.4f},{wz:.4f}\n"
                )

            if int((now - started) * 2) != int((now - 0.1 - started) * 2):
                print(
                    f"  t={now - started:5.1f}s  pose=({pose.x:+.2f},{pose.y:+.2f})"
                    f"  goal_d={dist_goal:.2f}  vx={vx:+.2f} wz={wz:+.2f}"
                    f"  {'旧路径' if plan_world and plan_stale else ('有路径' if plan_world else '无路径-探索')}"
                    f"  簇{cluster_count} 动{dynamic_count}"
                    + (
                        f"  err={error:+.2f} near={near:.2f}"
                        f" hold={max(0.0, stop_hold_until - now):.2f}"
                        if args.debug
                        else ""
                    )
                )

            next_tick += 0.1
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

        if not success:
            print(f"[!] 超时未到达，剩余 {dist_goal:.2f} m")
        if mapper is not None:
            final = mapper.build_grid()
            print(f"[i] 最终地图：{final.summary()}")
            print(f"[i] 建图帧 {frames}，重规划 {replans} 次，最后一次路径长 {path_cost:.2f} m"
                  if last_result is not None and last_result.found else f"[i] 建图帧 {frames}，重规划 {replans} 次")
            print(
                f"[i] 扫描缓存：{len(scan_history)}/{scan_history.capacity} 帧，"
                f"累计 {scan_history.total_appended} 帧，覆盖丢弃 {scan_history.dropped} 帧；"
                f"最新一帧 {cluster_count} 个障碍簇"
            )
            if not args.no_save:
                out = Path(args.map_out)
                if not out.is_absolute():
                    out = ROOT.parent / out
                final.save(out)
                print(f"[i] 地图已保存：{out}")
        return 0 if success else 2
    finally:
        if trace_fp is not None:
            trace_fp.close()
        robot.close()


if __name__ == "__main__":
    raise SystemExit(main())
