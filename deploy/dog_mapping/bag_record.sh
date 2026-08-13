#!/bin/bash
# /state_estimation + /cloud_registered_body + /utlidar/imu -> plio2sm (MOLA map)
source /opt/ros/humble/setup.bash
source /home/unitree/unitree_ros2/setup.sh
source /home/unitree/go2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
# ⛔ ТОЛЬКО ЭТИ ТРИ ТОПИКА, ничего не добавлять. /registered_scan + /tf стояли тут
# раньше — это и уронило Point-LIO на 2.5 км (loc_5_test2): /state_estimation
# просел 7.4 -> 4.1 кГц. Идеальная карта reloc2 писалась тремя.
# /registered_scan при необходимости восстанавливается офлайн: ~/maps/body2world.py
exec ros2 bag record -o "$1" \
  /state_estimation /cloud_registered_body /utlidar/imu
