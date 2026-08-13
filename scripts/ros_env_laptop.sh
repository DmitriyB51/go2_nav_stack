#!/usr/bin/env bash
# LAPTOP-side ROS env (domain 1, WiFi) — counterpart to the dog's ~/ros_env.sh,
# which is dog-specific (sources ~/unitree_ros2 + ~/go2_ws, binds CycloneDDS to the
# WIRED enP8p1s0). Here we reach the dog through its domain_bridge.
#
# `env -i` because the user's bashrc auto-loads Isaac Sim + conda, which put their
# PYTHONPATH/LD_LIBRARY_PATH ahead of ROS and break rclpy — and `ros2 topic echo` is
# Python ("undefined symbol nav_msgs__msg__goals" or a bare import error).
#
#   ./scripts/ros_env_laptop.sh ros2 topic echo /localization/fitness --field data
#   ./scripts/ros_env_laptop.sh                  # no args -> clean interactive shell
#
# ⚠️ Only topics the bridge forwards are visible (go2_nav_bridge.yaml). For heavy
# unbridged ones (/registered_scan) ssh to the dog and use its ~/ros_env.sh.

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

CLEAN_ENV=(
  HOME="$HOME"
  USER="$USER"
  TERM="${TERM:-xterm}"
  DISPLAY="${DISPLAY:-:0}"
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ROS_DOMAIN_ID=1
  CYCLONEDDS_URI="file://${REPO}/env/cdds_laptop_wifi.xml"
)

if [ "$#" -eq 0 ]; then
  echo "[ros_env_laptop] clean shell: domain=1 iface=wlo1 (exit to return)"
  exec env -i "${CLEAN_ENV[@]}" bash --noprofile --norc -i \
    -c 'source /opt/ros/humble/setup.bash && PS1="(ros1) \w$ " exec bash --noprofile --norc -i'
fi

exec env -i "${CLEAN_ENV[@]}" \
  bash --noprofile --norc -c "source /opt/ros/humble/setup.bash && exec \"\$@\"" _ "$@"
