#!/usr/bin/env bash
# Посмотреть запись: стены из готовой карты + живой скан + робот.
#
#   ./scripts/watch_bag.sh                      # reloc2, 3x
#   ./scripts/watch_bag.sh ~/maps/reloc2 1      # реальное время
#   ./scripts/watch_bag.sh ~/maps/loc_5 5 ~/maps/expF_map.pcd
#
# Слои в RViz: карта (стены), лидар сейчас, след сканов 60 с, оси кадра body.
#
# ⚠️ Наклон стартового кадра Point-LIO (~20°) НЕ убирается — сырой просмотр.
#    Живой скан может не идеально ложиться на серую карту (она выровнена по
#    гравитации, сырая одометрия — нет). Это ожидаемо.

BAG="${1:-$HOME/maps/reloc2}"
RATE="${2:-3.0}"
PCD="${3:-$HOME/maps/reloc2_gravity.pcd}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

[ -d "$BAG" ] || { echo "нет такого бэга: $BAG"; exit 1; }

# env -i: bashrc тянет Isaac Sim и conda, они ломают rclpy
exec env -i HOME="$HOME" USER="$USER" DISPLAY="${DISPLAY:-:0}" \
  XAUTHORITY="$HOME/.Xauthority" TERM="${TERM:-xterm}" PATH=/usr/bin:/bin \
  bash --noprofile --norc -c "
source /opt/ros/humble/setup.bash
echo '[watch_bag] бэг: $BAG'
echo '[watch_bag] скорость: ${RATE}x   карта: $PCD'

# гасим всю группу, иначе RViz и узлы остаются висеть
trap 'kill 0' EXIT INT TERM

python3 $REPO/go2_navigation/scripts/odom_to_tf.py &
if [ -f '$PCD' ]; then
  python3 $REPO/go2_navigation/scripts/pcd_publisher.py --ros-args \
      -p pcd:='$PCD' -p frame:=camera_init -p topic:=/prior_map &
else
  echo '[watch_bag] карты $PCD нет — стены будут только из накопленных сканов'
fi
rviz2 -d $REPO/go2_navigation/rviz/watch_bag.rviz &

sleep 4
ros2 bag play '$BAG' --rate $RATE
echo '[watch_bag] запись кончилась. Ctrl+C для выхода.'
wait
"
