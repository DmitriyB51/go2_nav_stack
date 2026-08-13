#!/usr/bin/env bash
# Температуры моторов Go2: проверка перед заездом и слежение за остыванием.
#
# Робот трижды упал, просто СТОЯ: не софт (ничего нашего не было запущено, load
# 0.2), а перегрев — оба ЗАДНИХ бедренных мотора держали 74 °C при 35-39 у
# остальных. Стоячая поза грузит бедренные непрерывно; на пороге тепловой защиты
# момент срезается и зад складывается.
#
# ⭐ Смотреть на РАЗНИЦУ, а не на абсолют: порога отсечки Unitree у нас нет, зато
#    10 холодных моторов — бесплатный эталон при любой температуре в комнате.
#      разница < 10 °C -> норма;  > 25 °C -> перегрет, ждать
#
#   ~/motor_temp.sh      — одна проба
#   ~/motor_temp.sh 30   — следить каждые 30 с, видно остывание
#
# ⚠️ Без set -e: `read -r -d ''` возвращает ненулевой код при EOF (это нормально),
#    и скрипт молча падал бы прямо там. Соседние deploy-скрипты тоже без set -e/-u.

INTERVAL="${1:-0}"

source /opt/ros/humble/setup.bash
source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
read -r -d '' CDDS <<'XMLEOF'
<CycloneDDS>
  <Domain><General><Interfaces>
    <NetworkInterface name="enP8p1s0" priority="default" multicast="default"/>
  </Interfaces></General></Domain>
</CycloneDDS>
XMLEOF
export CYCLONEDDS_URI="$CDDS"

cat > /tmp/_motor_temp.py <<'PY'
import re, sys, time
txt = open("/tmp/_lowstate.txt").read()
if "motor_state:" not in txt:
    print("  нет данных /lowstate — стек робота выключен?"); sys.exit(1)
block = txt.split("motor_state:")[1].split("bms_state:")[0]
# первый элемент split — пустая строка до первого "- ", поэтому entries[k] = мотор k-1
entries = re.split(r"\n- ", block)
names = ["FR_hip","FR_thigh","FR_calf","FL_hip","FL_thigh","FL_calf",
         "RR_hip","RR_thigh","RR_calf","RL_hip","RL_thigh","RL_calf"]
temps = {}
for k, e in enumerate(entries):
    m = k - 1
    if not (0 <= m <= 11):
        continue
    t = re.search(r"temperature: (\d+)", e)
    if t:
        temps[m] = int(t.group(1))
if len(temps) < 12:
    print("  прочитано моторов:", len(temps), "(ожидалось 12)")

soc = re.search(r"bms_state:.*?soc: (\d+)", txt, re.S)
hips = [temps[i] for i in (0, 3, 6, 9) if i in temps]
cold = sorted(temps.values())[:6]
base = sum(cold) / len(cold) if cold else 0

line = []
worst = 0
for i in range(12):
    if i not in temps:
        continue
    t = temps[i]
    worst = max(worst, t)
    mark = "!" if t - base >= 25 else (":" if t - base >= 10 else " ")
    line.append("%s%s%d" % (names[i], mark, t))
print("  " + "  ".join(line))
print("  фон(6 холодных) %.0f C   макс %d C   разница %+d C   батарея %s%%"
      % (base, worst, worst - base, soc.group(1) if soc else "?"))
d = worst - base
verdict = ("ГОТОВ (разница <10)" if d < 10 else
           "ЖДАТЬ (разница 10-25, короткий заезд)" if d < 25 else
           "НЕ ПОДНИМАТЬ СТОЯ (разница >25 — перегрев)")
print("  ВЕРДИКТ:", verdict)
PY

sample() {
  timeout 15 ros2 topic echo /lowstate --once > /tmp/_lowstate.txt 2>/dev/null || {
    echo "  не удалось прочитать /lowstate"; return 1; }
  printf '[%s]\n' "$(date +%H:%M:%S)"
  python3 /tmp/_motor_temp.py
}

if [ "$INTERVAL" = "0" ]; then
  sample
else
  echo "слежу каждые ${INTERVAL} с, Ctrl+C для выхода"
  while true; do sample || true; sleep "$INTERVAL"; done
fi
