#!/usr/bin/env bash
# Sync laptop -> dog: 3 packages, dog-side scripts, and the reloc2 map trio.
#
# The trio (2D grid + 3D pcd + localizability grid) must ALWAYS move together —
# they live in one map frame, and deploying a subset silently mislocalizes.
#
#   ./deploy/sync_to_dog.sh [host]      # default 172.20.10.3 (phone hotspot)
#   ./deploy/sync_to_dog.sh 192.168.123.18   # on the cable
set -euo pipefail

HOST="${1:-172.20.10.3}"
DOG="unitree@${HOST}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LAPMAPS="$HOME/maps"

say() { printf '\n\033[1;36m== %s\033[0m\n' "$*"; }

say "0. reachability"
ssh -o ConnectTimeout=5 -o BatchMode=yes "$DOG" true 2>/dev/null \
  || { echo "ERROR: cannot ssh $DOG. Put laptop+dog on the same net first."; exit 1; }
echo "ssh $DOG OK"

# Рабочее пространство переименовано dima_ws -> go2_ws. На собаке каталог
# физически остался старым, а в нём лежит и point_lio_unilidar, поэтому просто
# создать go2_ws нельзя — переносим один раз, вместе со сборкой.
# Одноразовая миграция; когда все собаки переедут, этот блок можно удалить.
say "0b. workspace name migration (dima_ws -> go2_ws)"
ssh "$DOG" 'if [ -d ~/dima_ws ] && [ ! -d ~/go2_ws ]; then
              mv ~/dima_ws ~/go2_ws && echo "  moved ~/dima_ws -> ~/go2_ws"
              grep -rl dima_ws ~/*.sh 2>/dev/null | xargs -r sed -i s/dima_ws/go2_ws/g
              # --symlink-install зашивает АБСОЛЮТНЫЕ пути: после mv симлинки в
              # install/ ведут в несуществующий dima_ws. Тут же лежит и
              # point_lio_unilidar, поэтому чинить надо всё, а не наши 3 пакета.
              rm -rf ~/go2_ws/build ~/go2_ws/install ~/go2_ws/log
              echo "  wiped build/install/log — step 7 will do a FULL rebuild (slow)"
            elif [ -d ~/dima_ws ] && [ -d ~/go2_ws ]; then
              echo "  WARNING: both ~/dima_ws and ~/go2_ws exist. Delete the stale one."
            else
              echo "  ~/go2_ws already in place"
            fi'

# all sources must exist locally, or we deploy half a map set
say "1. local source files"
for f in "$LAPMAPS/reloc2_gravity.pcd" \
         "$LAPMAPS/reloc_eval_report/wall_density_reloc2.locgrid" \
         "$REPO/go2_navigation/maps/building_reloc2.pgm" \
         "$REPO/go2_navigation/maps/building_reloc2.yaml"; do
  [ -e "$f" ] || { echo "ERROR missing: $f"; exit 1; }
  printf '  OK %8s  %s\n' "$(du -h "$f" | cut -f1)" "$f"
done

# go2_localization is included even though RUN_NAV_LIVE.md forgot it:
# map_matcher_node.cpp carries the coasting gain + localizability gate
say "2. packages -> ~/go2_ws/src"
rsync -a --info=stats1 "$REPO/go2_navigation/"   "$DOG:go2_ws/src/go2_navigation/"
rsync -a --info=stats1 "$REPO/go2_localization/" "$DOG:go2_ws/src/go2_localization/"

# go2_sport_api = мост /cmd_vel -> лапы. Живёт в ~/autonomy_stack_go2, на собаке
# собирается в go2_ws
SPORT="$HOME/autonomy_stack_go2/src/utilities/unitree_pkgs/go2_sport_api"
if [ -d "$SPORT" ]; then
  rsync -a --info=stats1 "$SPORT/" "$DOG:go2_ws/src/go2_sport_api/"
else
  echo "  WARN: $SPORT нет — go2_sport_api не синхронизирован"
fi

say "3. dog-side scripts -> ~"
rsync -a --info=stats1 "$REPO/deploy/" "$DOG:./"
# chmod по списку файлов из deploy/, а не по маске run_*.sh: маска пропускала
# motor_temp.sh, и rsync -a затирал ему +x правами локальной копии (644).
# Права правятся у источника, здесь только подстраховка.
ssh "$DOG" "chmod +x $(cd "$REPO/deploy" && ls *.sh | sed 's|^|~/|' | tr '\n' ' ') 2>/dev/null || true"

# structure preserved (reloc_eval_report/ kept) so the single rewrite rule
# "/home/dmitriyb51/maps -> /home/unitree/maps" is correct for every path
say "4. reloc2 map trio -> ~/maps"
ssh "$DOG" 'mkdir -p ~/maps/reloc_eval_report'
rsync -a --info=stats1 "$LAPMAPS/reloc2_gravity.pcd" "$DOG:maps/"
rsync -a --info=stats1 "$LAPMAPS/reloc_eval_report/wall_density_reloc2.locgrid" \
                       "$DOG:maps/reloc_eval_report/"
rsync -a --info=stats1 "$REPO/go2_navigation/maps/building_reloc2."* \
                       "$DOG:go2_ws/src/go2_navigation/maps/"

say "5. rewrite laptop paths -> dog paths"
ssh "$DOG" "sed -i 's#/home/dmitriyb51/maps#/home/unitree/maps#g' \
            ~/go2_ws/src/go2_localization/config/localization.yaml"

# A missing localizability grid does NOT fail loudly: the node only WARNs and runs
# with the gate DISABLED (full trust) — the exact failure it was added to prevent.
say "6. verify map paths resolve on the dog"
ssh "$DOG" 'bash -s' <<'EOF'
cfg=~/go2_ws/src/go2_localization/config/localization.yaml
fail=0
for key in map_path localizability_map; do
  p=$(grep -E "^\s*${key}:" "$cfg" | head -1 | awk '{print $2}')
  if [ -e "$p" ]; then printf '  OK   %-18s %s\n' "$key" "$p"
  else                 printf '  FAIL %-18s %s  <-- NOT ON DISK\n' "$key" "$p"; fail=1; fi
done
grep -q '/home/dmitriyb51' "$cfg" && { echo "  FAIL: laptop paths still present"; fail=1; }
ls ~/go2_ws/src/go2_navigation/maps/building_reloc2.yaml >/dev/null 2>&1 \
  && echo "  OK   2D grid          building_reloc2.yaml" || { echo "  FAIL: 2D grid missing"; fail=1; }
# the obstacle costmap is a separate node now: if the package is missing,
# nav2_live.launch.py starts fine and the robot drives blind, silently
source /opt/ros/humble/setup.bash 2>/dev/null
ros2 pkg prefix nav2_costmap_2d >/dev/null 2>&1 \
  && echo "  OK   nav2_costmap_2d   installed (live obstacle costmap)" \
  || { echo "  FAIL: nav2_costmap_2d MISSING -- sudo apt install ros-humble-navigation2"; fail=1; }
exit $fail
EOF

say "7. rebuild on the dog"
# Нет install/ — значит либо первый деплой, либо миграция имени только что его
# снесла: собирать надо ВСЁ, включая point_lio_unilidar. Иначе только наши пакеты.
ssh "$DOG" 'bash -lc "source /opt/ros/humble/setup.bash && cd ~/go2_ws && \
   if [ -d install ]; then \
     colcon build --symlink-install --packages-select go2_localization go2_navigation go2_sport_api; \
   else \
     echo \"[full rebuild — this takes several minutes]\"; \
     colcon build --symlink-install; \
   fi"'

say "DONE — code + reloc2 map trio deployed and verified on $HOST"
echo "Next: RUN_NAV_LIVE.md §1 (jetson_clocks -> run_pointlio -> run_matcher -> run_nav2)"
