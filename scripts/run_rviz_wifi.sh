#!/bin/bash
# RViz на ноутбуке (домен 1, WiFi) для живой локализации. Собака в домене 0
# (провод, лидар), связывает их domain_bridge НА СОБАКЕ (~/run_loc_bridge.sh).
#
# env -i обязателен: bashrc пользователя подгружает Isaac Sim и conda, они ломают ROS.
#
# (без set -u: ROS setup.bash обращается к неопределённым переменным)

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# конфиг RViz: nav — навигация ("Nav2 Goal", /map, /plan, costmap);
#               loc — только локализация (прайор-карта, поза, "2D Pose Estimate").
# Нет кнопки "Nav2 Goal" -> взят конфиг loc, целей им не поставить.
case "${1:-nav}" in
  nav) RVIZ_CFG="${REPO}/go2_navigation/rviz/nav2.rviz" ;;
  loc) RVIZ_CFG="${REPO}/go2_localization/rviz/localization.rviz" ;;
  *)   RVIZ_CFG="$1" ;;          # можно передать свой путь к .rviz
esac

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=1
export CYCLONEDDS_URI="file://${REPO}/env/cdds_laptop_wifi.xml"

echo "[run_rviz_wifi] domain=1  iface=wlo1  rviz=${RVIZ_CFG}"

exec env -i \
  HOME="$HOME" \
  DISPLAY="${DISPLAY:-:0}" \
  XAUTHORITY="$HOME/.Xauthority" \
  RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" \
  ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  CYCLONEDDS_URI="$CYCLONEDDS_URI" \
  bash -lc "source /opt/ros/humble/setup.bash && exec rviz2 -d ${RVIZ_CFG}"
