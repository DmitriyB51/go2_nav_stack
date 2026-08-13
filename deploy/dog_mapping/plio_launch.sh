#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/unitree/unitree_ros2/setup.sh
source /home/unitree/go2_ws/install/setup.bash
exec ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false
