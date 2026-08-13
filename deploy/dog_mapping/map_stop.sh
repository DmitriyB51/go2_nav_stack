#!/bin/bash
NAME=$(cat /home/unitree/maps/.last_map 2>/dev/null)
PCD=/home/unitree/go2_ws/src/point_lio_unilidar/PCD/scans.pcd
BAG=/home/unitree/maps/$NAME
echo 123 | sudo -S systemctl stop go2bag 2>/dev/null    # finalize bag (SIGINT)
echo 123 | sudo -S systemctl stop go2map 2>/dev/null    # save scans.pcd (SIGINT)
sleep 6
[ -f "$PCD" ] && cp "$PCD" "/home/unitree/maps/$NAME.pcd"
source /opt/ros/humble/setup.bash >/dev/null 2>&1
echo "=== plio bag: $BAG ==="
ros2 bag info "$BAG" 2>/dev/null | grep -E "Duration|Messages|Topic:" | sed 's/^/  /'
echo "=== scans.pcd -> /home/unitree/maps/$NAME.pcd ==="
ls -la "/home/unitree/maps/$NAME.pcd" 2>/dev/null || echo "  NO pcd saved!"
