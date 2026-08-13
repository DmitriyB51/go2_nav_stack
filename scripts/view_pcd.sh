#!/usr/bin/env bash
# Открыть .pcd в RViz.  ./scripts/view_pcd.sh ~/maps/mapbig_gravity.pcd
PCD="${1:?укажи путь к .pcd}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec env -i HOME="$HOME" USER="$USER" DISPLAY="${DISPLAY:-:0}" \
  XAUTHORITY="$HOME/.Xauthority" TERM="${TERM:-xterm}" PATH=/usr/bin:/bin \
  bash --noprofile --norc -c "
source /opt/ros/humble/setup.bash
trap 'kill 0' EXIT INT TERM
python3 $REPO/go2_navigation/scripts/pcd_publisher.py --ros-args \
    -p pcd:='$PCD' -p frame:=map -p topic:=/prior_map &
rviz2 -d $REPO/go2_navigation/rviz/view_pcd.rviz
"
