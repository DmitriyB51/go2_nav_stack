#!/bin/bash
# ⚠️⚠️  THIS IS THE SCRIPT THAT MAKES THE ROBOT ACTUALLY WALK.  ⚠️⚠️
#
# /cmd_vel -> sport_req.Move(vx,vy,vyaw) -> /api/sport/request -> gait -> legs.
#
# Safety built into the node:
#   * 0.5 s /cmd_vel timeout: Nav2 dies or WiFi drops -> the robot STOPS
#   * gamepad /joy override (deadzone 0.05) takes control away from Nav2
#   * speeds capped in nav2_params.yaml
#
# ⚠️ The factory remote does NOT pass through this node — it commands the onboard
#    sport controller directly, in parallel with our Move() calls, and it is NOT an
#    E-STOP (tested). The only stop is Ctrl+C / the 0.5 s timeout.
#
# Before running: run_pointlio / run_matcher / run_nav2 up and healthy, /cmd_vel
# watched and sane, robot in an open area with nobody in front.
#
# (no set -u: ROS setup.bash references unbound vars)

source /opt/ros/humble/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
# go2_sport_api lives in the CMU stack workspace on the dog
if [ -f "$HOME/autonomy_stack_go2/install/setup.bash" ]; then
  source "$HOME/autonomy_stack_go2/install/setup.bash"
else
  source "$HOME/go2_ws/install/setup.bash"
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

read -r -d '' CDDS <<'XMLEOF'
<CycloneDDS>
  <Domain>
    <General><Interfaces>
      <NetworkInterface name="enP8p1s0" priority="default" multicast="default"/>
    </Interfaces></General>
  </Domain>
</CycloneDDS>
XMLEOF
export CYCLONEDDS_URI="$CDDS"

echo "[run_vel_ctrl] ⚠️  LEGS ARE NOW LIVE — /cmd_vel will drive the robot."
echo "[run_vel_ctrl] prints [NAV2]/[JOY]/[STOP] at ~5 Hz so you can see who is driving."
# аргументы идут в узел — подмешка хода подбирается без пересборки:
#   ~/run_vel_ctrl.sh --ros-args -p rotate_assist_vx:=0.0
# 0.0 осмысленно только при большой скорости доворота; на 0.4 рад/с лапы залипали
exec ros2 run go2_sport_api vel_ctrl "$@"
