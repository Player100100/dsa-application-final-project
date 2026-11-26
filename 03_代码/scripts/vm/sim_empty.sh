#!/bin/bash
source /home/ros/vm/sim_common.sh
exec ros2 launch --noninteractive yahboom_rosmaster_bringup rosmaster_x3_sim.launch.py \
  gui:=false headless:=true rviz:=false use_camera:=false \
  enable_calculated_odometry:=false enable_ground_truth:=false \
  gazebo_use_llvmpipe:=true gazebo_model_detail:=box \
  gazebo_physics_step_size:=0.002 world:=empty.world \
  spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=0.0 \
  motion_profile:=ideal sensor_profile:=ideal
