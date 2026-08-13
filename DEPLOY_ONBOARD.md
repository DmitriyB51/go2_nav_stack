# Перенос вычислений на собаку (onboard live localization)

Цель: Point-LIO + map-matcher крутятся на Jetson робота в реальном времени.
На ноуте — только RViz. В воздух уходит поза и TF, облака остаются внутри Jetson.

---

## ⚠️ ЭТОТ ФАЙЛ УСТАРЕЛ — живой запуск смотри в `RUN_LIVE.md` / `RUN_NAV_LIVE.md`

Здесь остаётся только то, чего нет в них: почему лидар виден лишь на проводном
интерфейсе, векторизация `cloud_callback` и первичная установка на Jetson.

⛔ **Не выполняй отсюда `taskset`/`chrt` для Point-LIO.** Пиннинг на ядра 6,7 с
SCHED_FIFO 80 ЛОМАЕТ оценку: стоящий робот наматывает круги радиусом ~5 м. Запускать
только `run_pointlio.sh`. Подробности — `RUN_LIVE.md` §1.

⚠️ Имена карт ниже (`final_map_lc.pcd`) тоже устарели — актуальная пара указана в
`go2_localization/config/localization.yaml`.

---

Разведано 2026-07-15. Jetson: aarch64, Ubuntu 22.04, ROS **Humble**, PCL 1.12,
8 ядер, 15 ГБ RAM, 152 ГБ свободно. Дистрибутив совпадает с ноутом → всё собирается нативно.

Адреса: провод `enP8p1s0` = 192.168.123.18 (ноут: eno1 = 192.168.123.222),
wifi `wlxacf1df009552` = 172.20.10.3 (ноут: wlo1 = 172.20.10.2, хотспот "Dima").

**Шаги 1–5 делать по проводу** — быстрее и не зависит от хотспота.

---

## Шаг 1. Доставить зависимости на робота

Четыре apt-пакета, которых на Jetson нет. Единственное место, где нужен sudo (пароль `123`).

```bash
ssh unitree@192.168.123.18
sudo apt update
sudo apt install -y ros-humble-pcl-ros ros-humble-tf-transformations python3-transforms3d \
                    ros-humble-domain-bridge
exit
```

`pcl-ros` — зависимость Point-LIO и матчера. `tf-transformations` + `transforms3d` — для
`transform_sensors` (он на Python). `domain-bridge` — для шага 7.

## Шаг 1b. Калибровка IMU — файл ВНЕ пакета (легко пропустить)

`transform_everything.py` читает калибровку из `~/Desktop/imu_calib_data.yaml`. Файла нет —
узел молча берёт нулевые дефолты и ничего не сообщает (`print` буферизуется и в лог launch
не попадает), а `rsync` пакета его не захватывает.

```bash
scp ~/Desktop/imu_calib_data.yaml unitree@192.168.123.18:~/Desktop/
```

> Что из него реально работает: только гироскопные `ang_bias_*` и проекции `ang_z2x/z2y_proj`.
> `acc_bias_*` применяются и тут же затираются — в конце `imu_callback` линейное ускорение
> обнуляется перед публикацией в `/utlidar/transformed_imu` (осознанный приём CMU, парный к
> `use_imu_as_input: false`). Полное ускорение уходит только в `transformed_raw_imu`.

## Шаг 2. Скопировать исходники

Три пакета в `~/go2_ws/src` (там уже живёт ваш `go2_viz`). `--exclude` отсекает мусор сборки.

```bash
# с ноута
rsync -av --exclude build --exclude install --exclude log \
    ~/autonomy_stack_go2/src/slam/point_lio_unilidar \
    ~/autonomy_stack_go2/src/utilities/transform_sensors \
    ~/ros2_mola_ws/src/go2_localization \
    unitree@192.168.123.18:~/go2_ws/src/
```

`transform_sensors` обязателен: `mapping_utlidar.launch` запускает его первым узлом
(`transform_everything`), без него Point-LIO не стартует.

## Шаг 3. Скопировать карту (133 МБ, ~1-2 мин по проводу)

```bash
scp ~/maps/final_map_lc.pcd unitree@192.168.123.18:~/maps/
```

## Шаг 4. Поправить путь к карте в конфиге

На роботе `/home/dmitriyb51/` не существует — путь в `localization.yaml` прибит гвоздями.

```bash
ssh unitree@192.168.123.18 \
  "sed -i 's|/home/dmitriyb51/maps/|/home/unitree/maps/|' \
   ~/go2_ws/src/go2_localization/config/localization.yaml && \
   grep map_path ~/go2_ws/src/go2_localization/config/localization.yaml"
```

Должно напечатать `map_path: /home/unitree/maps/final_map_lc.pcd`.

## Шаг 4b. ОБЯЗАТЕЛЬНО: векторизовать `cloud_callback` (иначе Point-LIO разваливается)

**Симптом:** Point-LIO уезжает на десяток метров на неподвижном роботе (11 м и растёт,
при `/utlidar/robot_pose` ~0). Матчер честно отбивает всё по health gate (fitness ~6.2).

**Причина.** `transform_everything.py` крутит однопоточный `rclpy.spin()`, а его
`cloud_callback` — чистый Python: `read_points_list` → `.tolist()` → цикл по ~4100 точкам
→ `del` по одному (каждый O(n)) → `create_cloud` со списком. Пока это крутится,
`imu_callback` не выполняется, и очередь IMU (50 ≈ 200 мс при 250 Гц) переполняется на
каждом облаке: `/utlidar/imu` 240 Гц на входе → `transformed_imu` **23 Гц** на выходе,
фильтр голодает и расходится. Облака при этом идут без потерь — душится именно IMU.
На x86-ноуте код успевал, поэтому раньше не всплывало.

**Правка:** заменить цикл на numpy-маски (рабочая копия в `~/go2_ws/src/transform_sensors/`).
Тонкость: `read_points()` отдаёт dtype с `itemsize=point_step` (32 Б), а `create_cloud()`
сравнивает с `dtype_from_fields(fields)` без него (28 Б) — выходной массив создавать с
целевым dtype явно, иначе падает assert.

Проверено побитово на реальном облаке; ускорение 61x. На роботе:
`transformed_imu` 23 → 247 Гц, CPU узла 62.8 → 30.3 %.

> Пакет собран с `--symlink-install`, поэтому правка Python подхватывается **без пересборки** —
> достаточно перезапустить Point-LIO. `egg-link` ведёт в `build/`, а та директория —
> симлинк на `src/`.

## Шаг 5. Собрать на роботе

```bash
ssh unitree@192.168.123.18
source /opt/ros/humble/setup.bash
cd ~/go2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 2
```

**`--parallel-workers 2` обязательно**: Point-LIO — тяжёлый C++ с Eigen, 16-поточная
сборка съела память и подвесила ноутбук. Сборка идёт долго, это нормально.

Проверка:

```bash
source ~/go2_ws/install/setup.bash
ros2 pkg list | grep -E "point_lio_unilidar|transform_sensors|go2_localization"
```

Ожидаем три строки.

---

## ⚠️ ВАЖНО: лидар виден только на проводном интерфейсе (измерено 2026-07-15)

Point-LIO **нельзя** запускать с `setup_wifi.sh`. Измерения на роботе, `ros2 topic hz`:

| Конфиг CycloneDDS процесса | `/utlidar/cloud` | `/utlidar/imu` |
|---|---|---|
| `setup_wifi.sh` (wifi + провод, wifi первым) | тишина (проба 70 с) | тишина |
| `setup.sh` (**только** `enP8p1s0`) | **16.4 Гц** | **248 Гц** |

Лидар публикует не ROS-узел, а нативное приложение Unitree (`_CREATED_BY_BARE_DDS_APP_`)
во внутреннюю сеть `192.168.123.x`. Издатель есть всегда, но данные доходят только до
процесса, у которого CycloneDDS привязан к проводному интерфейсу ЕДИНСТВЕННЫМ. Это не
таймаут дискавери (70 с не помогли) и не QoS (он идентичен у топиков, которые доходят).

> Неразгаданная аномалия: relay `/utlidar/robot_pose → /go2/pose` с двухинтерфейсным
> конфигом работает, а throttle `/utlidar/imu → /go2/imu` в том же окружении не отдаёт
> ничего. Полагаться на это нельзя.

**Отсюда архитектура:** тяжёлые узлы на проводе (домен 0), ноут на wifi (домен 1), между
ними мост, у каждого домена ровно один интерфейс — единственная работающая конфигурация.

```
Jetson, домен 0 (enP8p1s0):  Unitree internals → Point-LIO → матчер
                                      ↓  domain_bridge (поза + TF)
Jetson, домен 1 (wifi)  ←──────────────┘
        ↓
Ноут, домен 1 (wlo1): RViz
```

## Шаг 6. Запуск тяжёлых узлов (домен 0, провод)

Два терминала по ssh. Лучше `tmux` на роботе — переживает обрыв wifi.

**Терминал A — Point-LIO:**

```bash
source ~/unitree_ros2/setup.sh          # ТОЛЬКО провод. НЕ setup_wifi.sh!
source ~/go2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false
```

**`rviz:=false` обязательно** — в launch-файле дефолт `true`, RViz на роботе не нужен.

**Терминал B — матчер:**

```bash
source ~/unitree_ros2/setup.sh
source ~/go2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch go2_localization localization.launch.py rviz:=false
```

Ждём в логе загрузку карты (3.15 М точек → ~116k после вокселя 0.15), затем `fitness`
порядка 0.004–0.01. Робот должен стоять в начале карты — матчер стартует с guess=identity.

## Шаг 7. Мост в wifi (домен 0 → домен 1)

> Этот шаг спроектирован, но ещё не проверен на железе — в отличие от шагов 1–6.

```bash
ssh unitree@192.168.123.18
sudo apt install -y ros-humble-domain-bridge
```

Конфиг `~/go2_bridge.yaml` на роботе — только лёгкое:

```yaml
name: go2_bridge
topics:
  /state_estimation:
    type: nav_msgs/msg/Odometry
    from_domain: 0
    to_domain: 1
  /tf:
    type: tf2_msgs/msg/TFMessage
    from_domain: 0
    to_domain: 1
```

Запуск (у каждого домена — свой единственный интерфейс):

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS>
  <Domain id="0"><General><Interfaces>
    <NetworkInterface name="enP8p1s0" priority="default" multicast="default"/>
  </Interfaces></General></Domain>
  <Domain id="1"><General><Interfaces>
    <NetworkInterface name="wlxacf1df009552" priority="default" multicast="default"/>
  </Interfaces></General></Domain>
</CycloneDDS>'
ros2 run domain_bridge domain_bridge ~/go2_bridge.yaml
```

`/state_estimation` идёт на ~7 кГц — в воздух столько не нужно. Троттлите ДО моста, в домене 0:

```bash
source ~/unitree_ros2/setup.sh
export ROS_DOMAIN_ID=0
ros2 run topic_tools throttle messages /state_estimation 20 /go2/state_estimation
```

…и в `go2_bridge.yaml` мостите `/go2/state_estimation` вместо `/state_estimation`.

**Запасной вариант, если мост не заведётся:** `ros-humble-foxglove-bridge` на роботе в домене 0.
Он отдаёт данные по websocket (обычный TCP), полностью минуя DDS в воздухе — то есть все грабли
этого раздела исчезают. Цена: смотреть придётся в Foxglove Studio, а не в RViz, ваш
`localization.rviz` не переиспользуется.

## Шаг 8. Ноут — только RViz (домен 1)

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=1        # ВНИМАНИЕ: 1, не 0 — домен wifi-стороны моста
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
  <NetworkInterface name="wlo1" priority="default" multicast="default"/>
</Interfaces></General></Domain></CycloneDDS>'
rviz2 -d ~/ros2_mola_ws/src/go2_localization/rviz/localization.rviz
```

Карту в RViz грузите **локально** из своей копии `~/maps/final_map_lc.pcd` — не тяните
133 МБ по воздуху.

> `~/go2_wifi_env.sh` для этого режима не подходит: в нём `ROS_DOMAIN_ID=0`. Либо правьте его
> на 1, либо экспортируйте после него.

---

## Про облака и wifi

Побочный эффект разделения доменов: тяжёлые облака живут в домене 0 и не мостятся — ноут
их физически не видит, поэтому убить wifi одним кликом в RViz нельзя структурно.
Вернёшься к одному домену — помни: DDS шлёт топик только при наличии подписчика, и
включённый дисплей облака мгновенно кладёт канал.

---

## Если что-то не так

**Ноут не видит топики робота.** Проверьте, что оба конца в одном `ROS_DOMAIN_ID=0` и на
одном RMW (`rmw_cyclonedds_cpp`). Ноут — `~/go2_wifi_env.sh`, робот — `setup_wifi.sh`.
Порядок интерфейсов в `CYCLONEDDS_URI` критичен: CycloneDDS анонсируется только на первом.

**Point-LIO не стартует / нет `/registered_scan`.** Проверьте, что лидар жив:
`ros2 topic hz /utlidar/cloud` на роботе. И что `transform_everything` поднялся — он в
логе launch-файла первым.

**Матчер молчит.** Он ждёт накопления окна `window_sec: 1.0` и порога `match_every_m: 0.3`
(матч только после 0.3 м пути). Стоящий на месте робот матчиться не будет — это by design.

**GICP не успевает на Jetson.** Ядра слабее ноутбучных. Крутите в `localization.yaml`:
`gicp_max_iter` 15 → 10, `scan_voxel` 0.2 → 0.3. Выход позы на 50 Гц от этого не страдает —
GICP живёт в отдельном треде и не блокирует таймер.

---

## Потеря лока на развороте — УЖЕ ПОФИКШЕНА

Исторический диагноз: Z-дрейф Point-LIO до −14.5 м, локальный GICP такой разрыв не
перепрыгивает. Починено в коде (`planar_z_hold` + `match_min_interval_s` +
`strong_fitness` + пропуск Z-свипа + MultiThreadedExecutor), лок держится сквозь разворот.
