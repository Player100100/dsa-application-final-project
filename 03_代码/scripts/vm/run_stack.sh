#!/bin/bash
# 用法：run_stack.sh sim_ideal.sh | sim_start.sh | sim_empty.sh | sim_stress_gt.sh
SIM="${1:-sim_start.sh}"
/home/ros/vm/stop_sim.sh >/dev/null 2>&1
for i in $(seq 1 20); do
  pgrep -f "rosmaster_x3_sim.launch.py" >/dev/null 2>&1 || break
  sleep 1
done
pkill -9 -f "rosmaster_x3_sim.launch.py" >/dev/null 2>&1 || true
sleep 1
setsid -f "/home/ros/vm/$SIM" > /home/ros/sim.log 2>&1 < /dev/null
setsid -f /home/ros/vm/restart_bridge.sh > /home/ros/bridge.log 2>&1 < /dev/null
echo "stack_started $SIM"
