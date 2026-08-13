#!/bin/bash
# Domain bridge for live localization: domain 0 (wired, robot) <-> 1 (WiFi, laptop).
#
# One interface per domain, and not by taste: CycloneDDS advertises on the FIRST
# listed <NetworkInterface> only, and the Unitree bare-DDS lidar app only delivers
# on a single wired binding.
#
# (no set -u: ROS setup.bash references unbound vars)

source /opt/ros/humble/setup.bash
source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# the bridge joins both domains, so it must not have a fixed one
unset ROS_DOMAIN_ID

# WiFi dongle on the dog; verify with `ip -brief addr` if not found
WIFI_IF="wlxacf1df009552"
WIRED_IF="enP8p1s0"

read -r -d '' CDDS <<XMLEOF
<CycloneDDS>
  <Domain id="0">
    <General><Interfaces>
      <NetworkInterface name="${WIRED_IF}" priority="default" multicast="default"/>
    </Interfaces></General>
  </Domain>
  <Domain id="1">
    <General><Interfaces>
      <NetworkInterface name="${WIFI_IF}" priority="default" multicast="default"/>
    </Interfaces></General>
  </Domain>
</CycloneDDS>
XMLEOF
export CYCLONEDDS_URI="$CDDS"

echo "[run_loc_bridge] domain0=${WIRED_IF}  domain1=${WIFI_IF}  cores=0-5"
exec taskset -c 0-5 ros2 run domain_bridge domain_bridge "$HOME/go2_loc_bridge.yaml"
