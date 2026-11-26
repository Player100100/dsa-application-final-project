#!/bin/bash
# 仿真启动脚本的公共前缀（被 sim_*.sh source）
export DISPLAY=:0
export XAUTHORITY=$(ls /run/user/1000/.mutter-Xwaylandauth.* 2>/dev/null | head -1)
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ros/vm/fastdds_udp.xml
source /opt/ros/humble/setup.bash
source /home/ros/Desktop/virtual_car/install/setup.bash
