#!/bin/bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ros/vm/fastdds_udp.xml
source /opt/ros/humble/setup.bash
source /home/ros/Desktop/virtual_car/install/setup.bash
exec ros2 launch --noninteractive rosbridge_server rosbridge_websocket_launch.xml port:=9090
