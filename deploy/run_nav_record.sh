#!/usr/bin/env bash
# Запись ЛЁГКИХ управляющих топиков во время живой навигации — для офлайн-разбора
# поведения контроллера (овершут, колебания доворота, фризы).
#
# ⛔⛔ ОБЛАКА ТОЧЕК СЮДА НЕ ДОБАВЛЯТЬ. /registered_scan (~343 КБ/с) и
#    /cloud_registered_body тяжёлые: живая запись отбирает ресурсы у Point-LIO
#    (/state_estimation просел 7.4 -> 4.1 кГц, оценка разошлась на 2.5 км —
#    случай loc_5_test2). Если для replay нужен /registered_scan, его
#    ВОССТАНАВЛИВАЮТ офлайн из /cloud_registered_body (~/maps/body2world.py).
#    Набор ниже — сотни байт в секунду, Point-LIO этого не замечает.
#
#   ~/run_nav_record.sh nav_run1        # отдельное SSH-окно, стек уже поднят
#   Ctrl+C для остановки — НИКОГДА не kill -9, иначе метаданные бага не допишутся
#   rsync -av unitree@172.20.10.3:~/navlogs/nav_run1 ~/maps/
#
# (без set -u: ROS setup.bash обращается к неопределённым переменным)
set -eo pipefail

TAG="${1:-nav_$(date +%Y%m%d_%H%M%S)}"
OUT="$HOME/navlogs/$TAG"
mkdir -p "$HOME/navlogs"

source /opt/ros/humble/setup.bash
source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
source "$HOME/go2_ws/install/setup.bash"

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

#   /tf*        поза робота          /cmd_vel   команды контроллера (ядро разбора)
#   /plan       маршруты              /goal_pose начало каждого заезда
#   /localization/fitness + /pose     отличить потерю захвата от ошибки контроллера
#   /local_costmap/costmap*           разбор фриза (контроллер решил, что перекрыто)
TOPICS=(
  /tf /tf_static
  /plan
  /cmd_vel
  /localization/fitness
  /localization/pose
  /goal_pose
  /local_costmap/costmap
  /local_costmap/costmap_updates
)

echo "[run_nav_record] -> $OUT"
echo "[run_nav_record] топиков: ${#TOPICS[@]}  (облаков точек НЕТ — это намеренно)"
echo "[run_nav_record] Ctrl+C для корректной остановки"
exec ros2 bag record -o "$OUT" "${TOPICS[@]}"
