#!/usr/bin/env python3
"""Контроллер вместо RPP: едет и рулит одновременно, разворот на месте только
если цель почти за спиной.

Почему не RPP: его rotateToHeading релейный (константная ω, обрыв в нуль) и не
настраивается -> овершут. Здесь пропорциональный закон + опережение по задержке
исполнения + тормозной предел.

Из Nav2 остались planner_server, map_server, local_costmap (отдельным узлом);
controller_server выкинут — /cmd_vel шлёт этот узел.

Препятствия: щупаем local_costmap по маршруту -> стоп -> реплан; 30 с не
расходится -> ЗАБЛОКИРОВАН. Это остановка, а не объезд (в global_costmap нет
obstacle_layer). Ближе 0.8 м лидар не метит ничего. Аппаратного E-STOP нет.

Запуск: ros2 run go2_navigation simple_controller.py
"""
import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import Costmap
from std_msgs.msg import Float32
import tf2_ros


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    """Угол в (-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class SimpleController(Node):
    DRIVE = "ЕДУ"
    TURN = "РАЗВОРОТ НА МЕСТЕ"
    ARRIVED = "ПРИЕХАЛ"
    IDLE = "ЖДУ ЦЕЛЬ"
    BLOCKED = "ЗАБЛОКИРОВАН"

    def __init__(self):
        super().__init__("simple_controller")

        # --- ход ---
        self.v_drive = self.declare_parameter("drive_speed", 0.5).value
        # ниже ~0.3 м/с собака ходит плохо (порог исполнения)
        self.v_slow = self.declare_parameter("slow_speed", 0.3).value
        self.err_slow = self.declare_parameter("slow_at_error", 0.6).value

        # --- поворот ---
        self.yaw_gain = self.declare_parameter("yaw_gain", 1.5).value
        self.yaw_max = self.declare_parameter("yaw_max", 1.4).value   # шкала джойстика
        # порог исполнения: на месте команды меньше игнорируются.
        # как ПОЛ использовать нельзя — это и давало овершут
        self.yaw_min = self.declare_parameter("yaw_min", 0.4).value
        # опережение под задержку исполнения, с. Единственный параметр под подбор
        # на роботе; ошибаться в сторону завышения (перелёт опасен, недоворот нет)
        self.turn_lead = self.declare_parameter("turn_lead_sec", 1.0).value
        # мелкую ошибку на месте не правим (физически невозможно) — доберёт руление на ходу
        self.turn_deadband = self.declare_parameter("turn_deadband", 0.10).value
        self.yaw_slew = self.declare_parameter("yaw_slew", 4.0).value  # против скачков позы
        # разворот на месте, с гистерезисом (один порог -> дребезг режима)
        self.in_place_enter = self.declare_parameter("in_place_enter", 1.2).value
        self.in_place_exit = self.declare_parameter("in_place_exit", 0.5).value

        # --- доверие к одометрии при провале локализации ---
        # предел скорости изменения коррекции матчера
        self.corr_slew_m = self.declare_parameter("corr_slew_m", 0.30).value
        self.corr_slew_rad = self.declare_parameter("corr_slew_rad", 0.30).value
        # настоящую релокализацию сглаживать нельзя — принимаем целиком
        self.corr_snap_m = self.declare_parameter("corr_snap_m", 2.0).value
        self.corr_snap_rad = self.declare_parameter("corr_snap_rad", 1.0).value
        # 0.005-0.02 здоровый лок, >0.1 потеря
        self.fitness_bad = self.declare_parameter("fitness_bad", 0.10).value
        # плохая локализация -> СБАВЛЯЕМ ХОД, не встаём: перезахват матчера
        # требует стен, а они только впереди — движение тут часть лечения
        self.coast_max = self.declare_parameter("coast_max_sec", 10.0).value
        # аварийный полный стоп, 0 = никогда (пока локализация мертва, вето по
        # препятствиям слепнет, а E-STOP нет — включать осознанно)
        self.coast_stop = self.declare_parameter("coast_stop_sec", 0.0).value

        self.lookahead = self.declare_parameter("lookahead", 0.5).value
        self.goal_tol = self.declare_parameter("goal_tolerance", 0.25).value
        # реплан: путь строится от позы В МОМЕНТ КЛИКА, матчер потом доводит её до
        # ~1 м -> план от неверной точки. Только в фазе ЕДУ. 0 = выключить
        self.replan_period = self.declare_parameter("replan_period", 2.0).value
        self.turn_timeout = self.declare_parameter("turn_timeout", 10.0).value

        # --- препятствия (живая local_costmap) ---
        self.check_dist = self.declare_parameter("obstacle_check_dist", 1.5).value
        # полуширина корпуса 0.15, берём 0.25 — запас на ошибку локализации
        self.check_halfwidth = self.declare_parameter("obstacle_check_halfwidth", 0.25).value
        # 253 = INSCRIBED_INFLATED, 254 = LETHAL. Ниже брать нельзя: inflation 0.80 м
        # при полуширине коридора 0.88 м накрывает коридор целиком
        self.lethal_cost = self.declare_parameter("obstacle_lethal_cost", 253).value
        # отступ: под роботом остаются старые отметки, иначе блокирует сам себя
        self.probe_start = self.declare_parameter("obstacle_probe_start", 0.3).value
        # проверка "в упор" на время доворота (мост подмешивает vx=0.10)
        self.close_dist = self.declare_parameter("obstacle_close_dist", 0.6).value
        self.costmap_stale_s = self.declare_parameter("costmap_stale_sec", 3.0).value
        self.blocked_timeout = self.declare_parameter("blocked_timeout", 30.0).value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "/plan", 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
        # costmap_raw = всегда полная сетка (на /local_costmap/costmap идут дельты)
        self.create_subscription(Costmap, "/local_costmap/costmap_raw",
                                 self.on_costmap, 1)
        self.create_subscription(Float32, "/localization/fitness",
                                 self.on_fitness, 10)

        self.path = []            # [(x, y)]
        self.goal = None
        self.state = self.IDLE
        self.state_since = self.now()
        self.last_replan = 0.0
        self.plan_pending = False  # иначе запросы накладываются
        self.is_replan = False
        self.turn_target = None   # абсолютный курс доворота
        # ⛔ ω из /state_estimation брать нельзя: Point-LIO всегда пишет туда 0
        self.yaw_hist = []        # [(время, курс)]
        self.yaw_rate = 0.0
        self.last_w = 0.0
        self.corr = None          # сглаженная map->camera_init [x, y, курс]
        self.fitness = None
        self.bad_fit_since = None
        self.costmap = None
        self.costmap_time = 0.0
        self.blocked_since = None
        self.warned_no_costmap = 0.0
        self.create_timer(0.05, self.tick)      # 20 Гц

        self.get_logger().info(
            "контроллер готов: ход %.2f..%.2f м/с, поворот до %.1f рад/с, "
            "опережение %.2f с, разворот на месте при >%.0f°"
            % (self.v_slow, self.v_drive, self.yaw_max, self.turn_lead,
               math.degrees(self.in_place_enter)))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, s):
        if s != self.state:
            self.get_logger().info("-> %s" % s)
            self.state = s
            self.state_since = self.now()
            if s != self.TURN:
                self.turn_target = None   # цель доворота живёт ровно одну фазу

    # --- поза ---
    def tf_xytheta(self, target, source):
        try:
            t = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except Exception:
            return None
        return (t.transform.translation.x,
                t.transform.translation.y,
                yaw_of(t.transform.rotation))

    def robot_pose(self):
        """Сырая поза в карте — согласована с остальным Nav2 (costmap)."""
        return self.tf_xytheta("map", "base_link")

    def update_correction(self):
        """Коррекция матчера через ограничение скорости изменения.

        Поза = map->camera_init (матчер, дёрганая) o camera_init->base_link
        (Point-LIO, гладкая). Пока карта врёт, едем по одометрии, правда вливается
        плавно. Большие прыжки = релокализация, принимаются целиком.
        """
        c = self.tf_xytheta("map", "camera_init")
        if c is None:
            return
        if self.corr is None:
            self.corr = list(c)
            return
        dx, dy = c[0] - self.corr[0], c[1] - self.corr[1]
        dyaw = wrap(c[2] - self.corr[2])
        if math.hypot(dx, dy) > self.corr_snap_m or abs(dyaw) > self.corr_snap_rad:
            self.get_logger().warn(
                "коррекция ушла на %.2f м / %.0f° — принимаю сразу (релокализация)"
                % (math.hypot(dx, dy), math.degrees(dyaw)))
            self.corr = list(c)
            return
        dmax = self.corr_slew_m * 0.05
        d = math.hypot(dx, dy)
        if d > dmax > 0:
            dx, dy = dx * dmax / d, dy * dmax / d
        amax = self.corr_slew_rad * 0.05
        dyaw = max(-amax, min(amax, dyaw))
        self.corr[0] += dx
        self.corr[1] += dy
        self.corr[2] = wrap(self.corr[2] + dyaw)

    def control_pose(self):
        """(x, y, курс_в_карте, курс_по_одометрии). ω считаем по одометрии."""
        o = self.tf_xytheta("camera_init", "base_link")
        if o is None or self.corr is None:
            return None
        cx, cy, cyaw = self.corr
        return (cx + o[0] * math.cos(cyaw) - o[1] * math.sin(cyaw),
                cy + o[0] * math.sin(cyaw) + o[1] * math.cos(cyaw),
                wrap(cyaw + o[2]),
                o[2])

    def loc_healthy(self):
        """(здорова?, сколько секунд уже плохо)"""
        if self.fitness is None:                  # топика нет — не мешаем ехать
            return True, 0.0
        if self.fitness <= self.fitness_bad:
            self.bad_fit_since = None
            return True, 0.0
        if self.bad_fit_since is None:
            self.bad_fit_since = self.now()
        return False, self.now() - self.bad_fit_since

    # --- препятствия ---
    def on_costmap(self, msg: Costmap):
        self.costmap = msg
        self.costmap_time = self.now()   # metadata.update_time приходит нулевым

    def on_fitness(self, msg: Float32):
        self.fitness = float(msg.data)

    def cost_at(self, x, y):
        """None = вне окна 6x6 м; это норма, трактуем как "не знаю"."""
        m = self.costmap.metadata
        col = int((x - m.origin.position.x) / m.resolution)
        row = int((y - m.origin.position.y) / m.resolution)
        if col < 0 or row < 0 or col >= m.size_x or row >= m.size_y:
            return None
        return self.costmap.data[row * m.size_x + col]

    def cross_blocked(self, x, y, heading):
        """Занята ли поперечина корпуса в (x, y) при движении по heading."""
        nx = -math.sin(heading)
        ny = math.cos(heading)
        n = 3
        for k in range(-n, n + 1):
            off = self.check_halfwidth * k / n
            c = self.cost_at(x + nx * off, y + ny * off)
            if c is not None and c >= self.lethal_cost:
                return True
        return False

    def path_blocked(self, x, y):
        """Свободен ли МАРШРУТ на check_dist вперёд (не "прямо по курсу": за
        углом коридора прямая упирается в стену)."""
        if not self.path:
            return False
        d = [(px - x) ** 2 + (py - y) ** 2 for px, py in self.path]
        i0 = d.index(min(d))
        travelled = 0.0
        for i in range(i0, len(self.path) - 1):
            ax, ay = self.path[i]
            bx, by = self.path[i + 1]
            step = math.hypot(bx - ax, by - ay)
            if step < 1e-6:
                continue
            travelled += step
            if travelled < self.probe_start:
                continue
            if travelled > self.check_dist:
                break
            if self.cross_blocked(ax, ay, math.atan2(by - ay, bx - ax)):
                return True
        return False

    def close_blocked(self, x, y, yaw):
        """Что-то в упор по курсу — проверка для фазы доворота."""
        s = self.probe_start
        while s <= self.close_dist:
            if self.cross_blocked(x + math.cos(yaw) * s, y + math.sin(yaw) * s, yaw):
                return True
            s += 0.1
        return False

    def obstacle_check(self, x, y, yaw):
        """(занято?, причина).

        Костмапа не было ни разу -> узел не запущен: едем, но предупреждаем.
        Костмап протух -> неисправность, "нет данных" != "свободно": стоим.
        """
        if self.costmap is None:
            if self.now() - self.warned_no_costmap > 10.0:
                self.warned_no_costmap = self.now()
                self.get_logger().warn(
                    "локальной костмапы НЕТ (/local_costmap/costmap_raw молчит) — "
                    "еду ВСЛЕПУЮ, живой реакции на препятствия не будет")
            return False, ""

        age = self.now() - self.costmap_time
        if age > self.costmap_stale_s:
            return True, "костмап не обновляется %.1f с" % age

        if self.state == self.TURN:
            if self.close_blocked(x, y, yaw):
                return True, "препятствие в упор по курсу"
            return False, ""

        if self.path_blocked(x, y):
            return True, "препятствие на маршруте впереди"
        return False, ""

    def on_blocked(self, why):
        if self.blocked_since is None:
            self.blocked_since = self.now()
            self.get_logger().warn("СТОП: %s" % why)

        if self.now() - self.blocked_since > self.blocked_timeout:
            self.get_logger().error(
                "препятствие не ушло за %.0f с (%s) — стою и жду человека. "
                "Кликни цель заново, когда путь освободится."
                % (self.blocked_timeout, why))
            self.set_state(self.BLOCKED)
            return

        # реплан помогает только если устарел сам маршрут: объехать препятствие
        # планировщик не может (в global_costmap нет obstacle_layer)
        if (self.replan_period > 0.0
                and self.now() - self.last_replan >= self.replan_period):
            self.request_plan(is_replan=True)

    def on_clear(self):
        if self.blocked_since is not None:
            self.get_logger().info("путь свободен, продолжаю")
            self.blocked_since = None

    # --- планирование ---
    def on_goal(self, msg: PoseStamped):
        self.goal = msg
        self.blocked_since = None
        self.request_plan(is_replan=False)

    def request_plan(self, is_replan):
        """is_replan=True — молча подменяем маршрут, фазу не трогаем."""
        if self.goal is None or self.plan_pending:
            return
        # строим от той же (сглаженной) позы, по которой рулим
        self.update_correction()
        pose = self.control_pose() or self.robot_pose()
        if pose is None:
            self.get_logger().error("нет TF map->base_link — локализация запущена?")
            return
        if not self.planner.wait_for_server(timeout_sec=1.0 if is_replan else 3.0):
            if not is_replan:
                self.get_logger().error("planner_server не отвечает")
            return

        start = PoseStamped()
        start.header.frame_id = "map"
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x, start.pose.position.y = pose[0], pose[1]
        start.pose.orientation.w = 1.0

        req = ComputePathToPose.Goal()
        req.start = start
        req.goal = self.goal
        req.use_start = True
        self.is_replan = is_replan
        self.plan_pending = True
        self.last_replan = self.now()
        self.planner.send_goal_async(req).add_done_callback(self._accepted)

    def _accepted(self, fut):
        h = fut.result()
        if not h.accepted:
            self.plan_pending = False
            self.get_logger().error("планировщик отклонил запрос")
            return
        h.get_result_async().add_done_callback(self._planned)

    def _planned(self, fut):
        self.plan_pending = False
        path = fut.result().result.path
        if not path.poses:
            if self.is_replan:
                self.get_logger().warn("пересчёт не дал маршрута — иду по старому",
                                       throttle_duration_sec=10.0)
            else:
                self.get_logger().warn("маршрут не найден (цель за стеной / в неизвестном)")
            return

        self.path = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        self.path_pub.publish(path)

        if self.is_replan:
            self.get_logger().info("маршрут пересчитан: %d точек" % len(self.path),
                                   throttle_duration_sec=10.0)
        else:
            self.get_logger().info("путь построен: %d точек" % len(self.path))
            self.set_state(self.DRIVE)   # курс не тот -> tick сам уйдёт в разворот

    def carrot(self, x, y):
        """Первая точка дальше lookahead, считая от ближайшей (иначе на развороте
        морковка может оказаться сзади)."""
        if not self.path:
            return None
        d = [(px - x) ** 2 + (py - y) ** 2 for px, py in self.path]
        i0 = d.index(min(d))
        for i in range(i0, len(self.path)):
            if math.hypot(self.path[i][0] - x, self.path[i][1] - y) >= self.lookahead:
                return self.path[i]
        return self.path[-1]

    def update_yaw_rate(self, yaw, now):
        """ω по истории курса на базе ~0.3 с (разность соседних тиков = шум
        0.35 рад/с при дрожании позы 1°, на базе 0.3 с — 0.06)."""
        self.yaw_hist.append((now, yaw))
        while len(self.yaw_hist) > 2 and now - self.yaw_hist[0][0] > 0.5:
            self.yaw_hist.pop(0)
        base = None
        for t, y in self.yaw_hist:
            if now - t >= 0.2:
                base = (t, y)
        if base is None:
            return
        dt = now - base[0]
        w = wrap(yaw - base[1]) / dt
        # >2.5 рад/с физически невозможно = прыжок позы, выбрасываем
        if abs(w) > 2.5:
            self.yaw_hist = [(now, yaw)]
            return
        self.yaw_rate = w

    def turn_command(self, err, in_place):
        """Пропорциональный доворот с опережением."""
        err_pred = err - self.yaw_rate * self.turn_lead

        # на месте зона нечувствительности не меньше yaw_min*turn_lead: меньшую
        # ошибку исправить невозможно, любая исполнимая команда её перелетит
        dead = max(self.turn_deadband, self.yaw_min * self.turn_lead) if in_place \
            else self.turn_deadband
        if abs(err_pred) < dead:
            return 0.0

        # тормозной предел — то, что реально убирает овершут: за время задержки
        # команда ω провернёт корпус на ω*turn_lead, значит ω <= остаток/turn_lead.
        # Одного опережения не хватает (модель: 41° перелёта против 15° с пределом)
        cap = min(self.yaw_max, abs(err_pred) / max(self.turn_lead, 1e-3))
        w = max(-cap, min(cap, self.yaw_gain * err_pred))
        if in_place and abs(w) < self.yaw_min:
            w = math.copysign(self.yaw_min, err_pred)
        return w

    def slew(self, w):
        dmax = self.yaw_slew * 0.05
        w = max(self.last_w - dmax, min(self.last_w + dmax, w))
        self.last_w = w
        return w

    def publish(self, vx, wz):
        m = Twist()
        m.linear.x = float(vx)
        m.angular.z = float(wz)
        self.cmd_pub.publish(m)

    # --- главный цикл ---
    def tick(self):
        if self.state in (self.IDLE, self.BLOCKED, self.ARRIVED) or not self.path:
            self.publish(0.0, 0.0)
            self.last_w = 0.0
            return

        self.update_correction()
        cp = self.control_pose()
        if cp is None:
            self.publish(0.0, 0.0)               # нет позы — стоим
            return
        x, y, yaw, odom_yaw = cp

        # локализация провалилась -> едем по одометрии, затянулось -> сбавляем ход
        healthy, bad_for = self.loc_healthy()
        coast_slow = (not healthy) and bad_for > self.coast_max
        if (not healthy) and self.coast_stop > 0.0 and bad_for > self.coast_stop:
            self.publish(0.0, 0.0)
            self.last_w = 0.0
            self.get_logger().warn(
                "локализации нет %.0f с (fitness %.3f) — стою по coast_stop_sec"
                % (bad_for, self.fitness), throttle_duration_sec=5.0)
            return

        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y
        if math.hypot(gx - x, gy - y) <= self.goal_tol:
            self.publish(0.0, 0.0)
            if self.state != self.ARRIVED:
                self.set_state(self.ARRIVED)
                self.get_logger().info("цель достигнута, стою")
            return

        tgt = self.carrot(x, y)
        if tgt is None:
            self.publish(0.0, 0.0)
            return
        err = wrap(math.atan2(tgt[1] - y, tgt[0] - x) - yaw)
        now = self.now()
        self.update_yaw_rate(odom_yaw, now)

        # препятствия — до управления. Щупаем по СЫРОЙ позе: костмап строится по
        # тому же сырому TF, смешивать кадры нельзя
        raw = self.robot_pose() or (x, y, yaw)
        blocked, why = self.obstacle_check(raw[0], raw[1], raw[2])
        if blocked:
            self.publish(0.0, 0.0)
            self.last_w = 0.0
            self.on_blocked(why)
            return
        self.on_clear()

        # режим, с гистерезисом
        if self.state == self.TURN:
            if abs(err) < self.in_place_exit:
                self.set_state(self.DRIVE)
        elif abs(err) > self.in_place_enter:
            self.set_state(self.TURN)

        if self.state == self.TURN:
            # цель разворота фиксируем ОДИН РАЗ: мост подмешивает vx=0.10, робот
            # ползёт вперёд и пеленг на точку в 0.5 м скачет до 180° — иначе
            # ошибка не убывает и разворот не кончается
            if self.turn_target is None:
                self.turn_target = wrap(yaw + err)
            terr = wrap(self.turn_target - yaw)

            # тупик: terr уже в мёртвой зоне, а err ещё велик -> стояли бы до
            # turn_timeout с нулевой командой. Обновляем цель курса
            if abs(terr) <= self.turn_deadband:
                self.turn_target = wrap(yaw + err)
                terr = err

            if now - self.state_since > self.turn_timeout:
                self.publish(0.0, 0.0)
                self.get_logger().error(
                    "разворот не сошёлся за %.0f с (осталось %.0f°) — стою. "
                    "Проверь локализацию и rotate_assist_vx."
                    % (self.turn_timeout, math.degrees(terr)))
                self.set_state(self.ARRIVED)
                return

            w = self.slew(self.turn_command(terr, in_place=True))
            self.publish(0.0, w)
            self.get_logger().info(
                "разворот: осталось %+.0f°, ω изм %+.2f, команда %+.2f"
                % (math.degrees(terr), self.yaw_rate, w),
                throttle_duration_sec=1.0)
            return

        # ЕДУ: скорость падает при большой ошибке курса (замена прежней остановки),
        # но не ниже v_slow — порог исполнения
        k = min(1.0, abs(err) / self.err_slow) if self.err_slow > 0 else 0.0
        v = self.v_drive - (self.v_drive - self.v_slow) * k
        if coast_slow:
            v = min(v, self.v_slow)
            self.get_logger().warn(
                "локализация плохая %.0f с (fitness %.3f) — иду по одометрии на %.2f м/с"
                % (bad_for, self.fitness, v), throttle_duration_sec=3.0)
        w = self.slew(self.turn_command(err, in_place=False))
        self.publish(v, w)
        self.get_logger().info(
            "еду: v %.2f, ошибка курса %+.0f°, ω изм %+.2f, команда %+.2f"
            % (v, math.degrees(err), self.yaw_rate, w),
            throttle_duration_sec=2.0)

        # реплан только в движении, никогда в развороте
        if (self.replan_period > 0.0
                and now - self.last_replan >= self.replan_period):
            self.request_plan(is_replan=True)


_stop_requested = False


def _on_signal(signum, frame):
    global _stop_requested
    _stop_requested = True


def main():
    # свой обработчик Ctrl+C: штатный rclpy гасит контекст первым, и обнуление
    # команды в finally падает с "publisher's context is invalid". Ctrl+C —
    # единственная аварийная остановка, аппаратной нет
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    node = SimpleController()
    # исполнитель создаётся ОДИН раз: spin_once(node) в цикле делает
    # add_node+remove_node каждый такт и перестраивает wait set, а по /tf идут
    # тысячи сообщений в секунду -> такт 20 Гц дрожит и регулятор перелетает
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not _stop_requested:
            executor.spin_once(timeout_sec=0.1)
    finally:
        # ноль шлём 10 раз: одиночный UDP-пакет может потеряться
        if rclpy.ok():
            node.get_logger().info("останавливаюсь по Ctrl+C")
            for _ in range(10):
                node.publish(0.0, 0.0)
                time.sleep(0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
