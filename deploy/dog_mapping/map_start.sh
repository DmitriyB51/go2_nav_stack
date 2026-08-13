#!/bin/bash
# Usage: bash map_start.sh [NAME]   -> builds scans.pcd AND a plio bag for MOLA.
NAME="${1:-plio_$(date +%m%d_%H%M)}"
mkdir -p /home/unitree/maps
echo "$NAME" > /home/unitree/maps/.last_map
PCD=/home/unitree/go2_ws/src/point_lio_unilidar/PCD/scans.pcd
BAG=/home/unitree/maps/$NAME
rm -f  "$PCD"
rm -rf "$BAG"                      # ros2 bag record needs a fresh dir
echo 123 | sudo -S systemctl reset-failed go2map go2bag 2>/dev/null
# Point-LIO (accumulates map -> scans.pcd on stop)
echo 123 | sudo -S systemd-run --uid=unitree --gid=unitree --setenv=HOME=/home/unitree \
  -p KillSignal=SIGINT -p TimeoutStopSec=180 --unit=go2map --collect \
  bash /home/unitree/rt2x00_build/plio_launch.sh >/dev/null 2>&1
# plio bag (3 topics for MOLA loop-closure)
echo 123 | sudo -S systemd-run --uid=unitree --gid=unitree --setenv=HOME=/home/unitree \
  -p KillSignal=SIGINT -p TimeoutStopSec=120 --unit=go2bag --collect \
  bash /home/unitree/rt2x00_build/bag_record.sh "$BAG" >/dev/null 2>&1
sleep 7
echo "map name : $NAME"
echo "go2map   : $(systemctl is-active go2map)   (Point-LIO -> scans.pcd)"
echo "go2bag   : $(systemctl is-active go2bag)   (plio bag -> $BAG)"
echo "--- bag already growing? ---"; du -sh "$BAG" 2>/dev/null || echo "  (starting...)"
