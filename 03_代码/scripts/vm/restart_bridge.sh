#!/bin/bash
pkill -9 -f "rosapi" >/dev/null 2>&1 || true
pkill -9 -f "rosbridge_websocket" >/dev/null 2>&1 || true
sleep 1
exec /home/ros/vm/bridge_start.sh
