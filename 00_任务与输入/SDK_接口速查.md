# SDK 接口速查（rosmaster_client v0.3.0）

> 权威文档：[使用手册](../../../用户传入/数据结构与算法应用/客户端v0.3/使用手册.md)、
> [进阶指南](../../../用户传入/数据结构与算法应用/客户端v0.3/进阶指南与技术细节.md)。
> 源码里的 `docs/` 已过时，不要照抄。本文只列项目实际用到的部分。

## 1. 连接与生命周期

```python
import rosmaster_client as car
car.init("192.168.244.131")        # 或 "IP:PORT"
car.terminate_connect()            # 重复调用安全
car.get_client().diagnose_text()   # 排查链路
```

## 2. 控制

| 接口 | 说明 |
|---|---|
| `car.set_speed(vx, vy=0, wz=0)` | 发一帧 `/cmd_vel`，立即返回；**远端保持约 0.5 s 后自动零速** |
| `car.stop()` | 发一帧零速 |
| 建议 | 连续运动按约 10 Hz 循环发；速度上限 \|vx\|=\|vy\|<1、\|wz\|<5 |

## 3. 传感

| 接口 | 返回 | 频率 | 关键字段 |
|---|---|---|---|
| `get_scan()` / `peek_scan()` / `wait_for_scan(t)` | `LaserScanFrame` / None | ~6.5 Hz（真实）/ 10 Hz（mock） | `ranges`、`angle_at(i)`、`angle_min`、`angle_increment`、`range_min/max`、`count`、`frame_id` |
| `get_odom_pos()` | `OdomPose` / None | ~8.9 Hz | `x`、`y`、`yaw`、`quaternion`、`linear`、`angular`、`speed` |
| `get_imu()` / `peek_imu()` | `ImuFrame` | ~8.9 Hz | `angular_velocity`、`linear_acceleration`、`orientation` |

> `get_*` 返回**原始快照**：不做坐标变换、不融合、不滤波。`inf`/`nan` 表示无效回波。
> `received_at` 是本机 `time.monotonic()`。

## 4. 坐标变换

```python
tf = car.get_transform("base_link", "laser_link")  # 把 laser 的点变到 base
tf.translation.x, tf.rotation.w                    # 米 / 单位四元数
```

- 只有 5 个坐标系：`odom`、`base_footprint`、`base_link`、`laser_link`、`imu_link`；
- 方向只由参数顺序决定，`(a,b)` 与 `(b,a)` 互为逆变换；
- 只有**最新**值；RTT 中位数约 15 ms；**停车时查一次缓存**，别放进高频环。

## 5. 标准车的装配事实（实测与文档一致）

- 雷达：`base_link ← laser_link` 平移 ≈ (0.043, 0, 0.110) m，偏航 = **+π（倒装）**；
- IMU：倒置安装，姿态四元数约 180° 倒转；优先用原始角速度/加速度；
- `/scan`：1080 束、±180°（`angle_min=-π`）、0.05–12 m。

## 6. 栅格地图容器

```python
from rosmaster_gridmap import GridMap, FREE, OCCUPIED, UNKNOWN
grid = GridMap(width=100, height=80, resolution=0.05)  # 全 -1
grid.set(10, 20, OCCUPIED)
grid.save("map.yaml")        # 唯一交付动作，写 trinary
same = GridMap.load("map.yaml")
```

约定：`x` 列、`y` 行、`(0,0)` 左下、行主序；内存可存 0–100，**落盘必为三值**；
中间值（20–64）落盘后变未知。本容器不含定位、不含 SLAM。

## 7. 项目里怎么用

本项目不直接调 SDK，统一走 [robot_io.py](../03_代码/robot_io.py)：
`speed/send`、`scan`、`pose`、`imu`、`laser_mount`、`close`。
mock 模式 `RobotIO("mock")` 会自动起厂商自带的假 rosbridge。
