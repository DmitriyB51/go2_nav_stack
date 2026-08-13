#!/usr/bin/env python3
"""Живой контроль качества записи карты — запускать ВМЕСТЕ с записью.

Ловит порчу на второй минуте вместо 10 минут ходьбы + часа обработки.

Мерит НАКЛОН плоскости траектории. Point-LIO гироскоп-only, крен/тангаж держит
только привязка по лидару; слабеет она — "низ" медленно поворачивается:
    reloc2 (карта идеальная):  27 -> 20 -> 23 -> 19 -> 19°   стоит на месте
    mapbig (карта развалилась): 28 -> 30 -> 37 -> 45 -> 66°  уползает
gravity_align.py снимает ОДИН наклон: постоянный -> остаток 9 см, вращающийся на
38° -> 717 см. Длина маршрута ни при чём (у mapbig уход начался к 150-й секунде).
Виновник — ВРАЩЕНИЕ, не скорость (корреляция +0.88 против +0.42), поэтому в
строке показаны °/с.

Запуск на собаке, ПОСЛЕ старта Point-LIO:
    source ~/ros_env.sh && python3 ~/map_monitor.py

ОК — иди дальше | ⚠ УХОД — перестань крутиться | ✖ ПОРЧА — переснимай.
"""
import math
import sys
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


class MapMonitor(Node):
    def __init__(self):
        super().__init__("map_monitor")

        # 30 с — окно офлайн-разбора, на нём подбирались пороги
        self.win_s = self.declare_parameter("window_sec", 30.0).value
        # уход наклона, градусы
        self.warn_deg = self.declare_parameter("warn_deg", 5.0).value
        self.bad_deg = self.declare_parameter("bad_deg", 10.0).value
        # ниже этого разброса по XY плоскость не подогнать
        self.min_span = self.declare_parameter("min_span_m", 5.0).value
        # опора не с первого окна: ранние подгонки неустойчивы (reloc2 ложно падал)
        self.base_after_m = self.declare_parameter("baseline_after_m", 10.0).value

        self.buf = deque()          # (t, x, y, z, yaw)
        self.tilt_hist = deque(maxlen=7)   # медиана гасит выбросы подгонки
        self.base_samples = []
        self.latest = None
        self.base_tilt = None
        self.t0 = None
        self.worst = 0.0
        self.path = 0.0
        self.prev_xy = None

        # /state_estimation идёт кГц-ами: колбэк только запоминает позу, разбор по таймеру
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        self.create_subscription(Odometry, "/state_estimation", self.on_odom, qos)

        self.create_timer(0.1, self.sample)     # 10 Гц хватает для плоскости
        self.create_timer(1.0, self.report)

        print("=" * 72)
        print(" МОНИТОР ЗАПИСИ КАРТЫ — следит за уходом наклона (порча карты)")
        print(" ОК = иди дальше | УХОД = перестань крутиться | ПОРЧА = переснимай")
        print("=" * 72)
        sys.stdout.flush()

    def on_odom(self, msg):
        self.latest = msg

    def sample(self):
        m = self.latest
        if m is None:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        if self.t0 is None:
            self.t0 = t
        if self.prev_xy is not None:
            self.path += math.hypot(p.x - self.prev_xy[0], p.y - self.prev_xy[1])
        self.prev_xy = (p.x, p.y)

        self.buf.append((t, p.x, p.y, p.z, yaw))
        while self.buf and t - self.buf[0][0] > self.win_s:
            self.buf.popleft()

    def tilt_now(self):
        """Наклон плоскости траектории в окне, градусы. Робот ходит по полу, значит
        его путь и есть пол; этот наклон обязан стоять на месте."""
        if len(self.buf) < 30:
            return None
        a = np.array(self.buf)
        x, y, z = a[:, 1], a[:, 2], a[:, 3]
        # без разброса по XY плоскость вырождается
        if max(x.max() - x.min(), y.max() - y.min()) < self.min_span:
            return None
        A = np.column_stack([x, y, np.ones(len(x))])
        c, *_ = np.linalg.lstsq(A, z, rcond=None)
        n = np.array([-c[0], -c[1], 1.0])
        n /= np.linalg.norm(n)
        return math.degrees(math.acos(min(1.0, abs(n[2]))))

    def rot_rate(self):
        """Средняя |угловая скорость| в окне, °/с — ручка оператора."""
        if len(self.buf) < 5:
            return 0.0
        a = np.array(self.buf)
        yaw = np.unwrap(a[:, 4])
        dt = a[-1, 0] - a[0, 0]
        return math.degrees(np.abs(np.diff(yaw)).sum()) / dt if dt > 0.5 else 0.0

    def report(self):
        tilt = self.tilt_now()
        el = (self.buf[-1][0] - self.t0) if (self.buf and self.t0) else 0.0
        if tilt is None:
            print("[%4.0f с]  набираю данные... (иди, нужно %.0f м разброса)"
                  % (el, self.min_span))
            sys.stdout.flush()
            return

        # одиночная кривая подгонка не должна поднимать тревогу
        self.tilt_hist.append(tilt)
        tilt = float(np.median(self.tilt_hist))

        # опора = медиана нескольких окон, после base_after_m метров пути
        if self.base_tilt is None:
            if self.path < self.base_after_m:
                print("[%4.0f с]  разогрев: путь %.1f/%.0f м (наклон %.1f°)"
                      % (el, self.path, self.base_after_m, tilt))
                sys.stdout.flush()
                return
            self.base_samples.append(tilt)
            if len(self.base_samples) < 8:
                print("[%4.0f с]  фиксирую опору... (%d/8, наклон %.1f°)"
                      % (el, len(self.base_samples), tilt))
                sys.stdout.flush()
                return
            self.base_tilt = float(np.median(self.base_samples))
            print("[%4.0f с]  ОПОРНЫЙ НАКЛОН %.1f° зафиксирован, слежу за уходом"
                  % (el, self.base_tilt))
            sys.stdout.flush()
            return

        drift = abs(tilt - self.base_tilt)
        self.worst = max(self.worst, drift)
        rot = self.rot_rate()

        if drift >= self.bad_deg:
            verdict = "✖ ПОРЧА — переснимай"
        elif drift >= self.warn_deg:
            verdict = "⚠ УХОД — не крутись"
        else:
            verdict = "ОК"
        rmark = "  (много крутишь!)" if rot > 12.0 else ""

        print("[%4.0f с]  наклон %5.1f°  уход %+5.1f°  вращение %4.1f°/с  путь %5.1f м   %s%s"
              % (el, tilt, tilt - self.base_tilt, rot, self.path, verdict, rmark))
        sys.stdout.flush()


def main():
    rclpy.init()
    node = MapMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        print("\n" + "=" * 72)
        print(" ИТОГ: максимальный уход наклона %.1f°, путь %.1f м"
              % (node.worst, node.path))
        if node.worst >= node.bad_deg:
            print(" ✖ Запись ИСПОРЧЕНА — одним поворотом её не выровнять, переснимай.")
        elif node.worst >= node.warn_deg:
            print(" ⚠ Пограничная. Проверь gravity_align.py: остаток плоскости должен")
            print("   быть меньше 20 см, иначе карту не использовать.")
        else:
            print(" ✅ Наклон стоял на месте — карта должна получиться.")
        print("=" * 72)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
