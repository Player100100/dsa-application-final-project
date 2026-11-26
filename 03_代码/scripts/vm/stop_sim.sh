#!/bin/bash
pkill -9 -f "rosapi" >/dev/null 2>&1 || true
pkill -9 -f "rosbridge_websocket" >/dev/null 2>&1 || true
pkill -9 -f "rosmaster_x3_sim.launch.py" >/dev/null 2>&1 || true
pkill -9 -f "rosmaster_gazebo" >/dev/null 2>&1 || true
pkill -9 -f "ign gazebo" >/dev/null 2>&1 || true
pkill -9 -f "ruby.*ign" >/dev/null 2>&1 || true
pkill -9 -f "scan_qos_relay|imu_raw_relay|wheel_state_odometry|cmd_vel_watchdog" >/dev/null 2>&1 || true
pkill -9 -f "parameter_bridge|ros_gz_bridge" >/dev/null 2>&1 || true
pkill -9 -f "controller_manager|spawner" >/dev/null 2>&1 || true
pkill -9 -f "imu_filter_madgwick" >/dev/null 2>&1 || true
pkill -9 -f "ros2 launch|ros2 run" >/dev/null 2>&1 || true
sleep 2
rm -f /dev/shm/fastrtps_* >/dev/null 2>&1 || true
rm -f /dev/shm/sem.* >/dev/null 2>&1 || true
rm -rf /tmp/rosmaster_gazebo_* >/dev/null 2>&1 || true
echo cleaned
