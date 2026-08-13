#!/bin/bash
# Nav2 on the robot, domain 0, WIRED-ONLY CycloneDDS.
#
# Only the Nav2 layer; the robot does NOT move until run_vel_ctrl.sh is started
# separately — that separation is the safety step.
#
# Order: run_pointlio.sh -> run_matcher.sh -> this -> run_nav_bridge.sh (RViz on
# the laptop) -> run_vel_ctrl.sh (only when the legs should move).
#
# ⛔ NEVER also run the CMU stack (system_real_robot.launch): its pathFollower
#    publishes /cmd_vel AND /api/sport/request -> a second driver fighting us.
#
# (no set -u: ROS setup.bash references unbound vars)

source /opt/ros/humble/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
source $HOME/go2_ws/install/setup.bash      # go2_navigation must be built here too
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

# map:= can be overridden, but the default building_reloc2.yaml is the pair to
# reloc2_gravity.pcd used by the matcher. Do not mix map sessions.
echo "[run_nav2] starting Nav2 (no motion until run_vel_ctrl.sh is started)"
exec ros2 launch go2_navigation nav2_live.launch.py "$@"
