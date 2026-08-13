#!/usr/bin/env python3
"""Логгер для разбора «собака проходит чуть дальше поворота»: в один CSV
собираются /plan, TF map->base_link, /localization/fitness и /cmd_vel за один
заезд через угол, плюс поперечная ошибка xtrack.
(⚠️ /state_estimation .angular.z не берём: Point-LIO всегда пишет туда 0.)

CSV не различает «карта уверенно врёт» и «всё честно» — для этого нужна метка на
полу и взгляд: где стрелка в RViz против того, где робот стоит физически.

Запуск, когда собака уже едет к цели:
  source ~/ros_env.sh && python3 ~/corner_test.py corner_run1
Кликни ОДНУ цель за поворотом, дай доехать, Ctrl+C ->
  corner_run1.csv (лог 20 Гц) + corner_run1_plan.csv (точки /plan).
Разбор: python3 ~/corner_analyze.py corner_run1.csv
"""
import csv
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    """Курс из кватерниона, радианы."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def cross_track(px, py, plan_xy):
    """(расстояние до ближайшего отрезка пути, индекс) — индекс говорит, как далеко
    вдоль пути мы прошли."""
    if len(plan_xy) < 2:
        return float("nan"), -1
    best_d, best_i = float("inf"), -1
    for i in range(len(plan_xy) - 1):
        ax, ay = plan_xy[i]
        bx, by = plan_xy[i + 1]
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        if seg2 < 1e-9:
            continue
        t = ((px - ax) * dx + (py - ay) * dy) / seg2
        t = max(0.0, min(1.0, t))
        cx, cy = ax + t * dx, ay + t * dy
        d = math.hypot(px - cx, py - cy)
        if d < best_d:
            best_d, best_i = d, i
    return best_d, best_i


class CornerTest(Node):
    def __init__(self, tag):
        super().__init__("corner_test")
        self.tag = tag

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)

        # последние значения потоков; таймер собирает их в одну строку
        self.fitness = float("nan")
        self.cmd = (0.0, 0.0, 0.0)      # vx, vy, vyaw
        self.plan_xy = []               # точки пути в кадре map
        self.goal_xy = None             # конец пути = цель

        self.create_subscription(Path, "/plan", self.on_plan, 10)
        self.create_subscription(Float32, "/localization/fitness", self.on_fit, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)

        # пишем построчно с flush — Ctrl+C не теряет данные
        self.FIELDS = ["t", "x", "y", "yaw_deg", "fitness",
                       "cmd_vx", "cmd_vy", "cmd_wz",
                       "xtrack_m", "plan_idx", "dist_to_goal_m"]
        self.n = 0
        self.csv_file = open(f"{tag}.csv", "w", newline="")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.FIELDS)
        self.writer.writeheader(); self.csv_file.flush()

        self.t0 = time.time()
        self.create_timer(0.05, self.tick)   # 20 Гц

        self.get_logger().info(
            f"corner_test пишет в {tag}.csv. Кликни ОДНУ цель, дай доехать, потом Ctrl+C.")

    def on_plan(self, msg: Path):
        self.plan_xy = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if self.plan_xy:
            self.goal_xy = self.plan_xy[-1]
            self.get_logger().info(f"получил /plan: {len(self.plan_xy)} точек")
            with open(f"{self.tag}_plan.csv", "w", newline="") as f:
                w = csv.writer(f); w.writerow(["x", "y"]); w.writerows(self.plan_xy)

    def on_fit(self, msg: Float32):
        self.fitness = float(msg.data)

    def on_cmd(self, msg: Twist):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)

    def tick(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()).transform
        except Exception:
            return   # TF ещё не готов
        x, y = t.translation.x, t.translation.y
        yaw = math.degrees(yaw_of(t.rotation))

        xt, idx = cross_track(x, y, self.plan_xy)
        dgoal = (math.hypot(x - self.goal_xy[0], y - self.goal_xy[1])
                 if self.goal_xy else float("nan"))

        self.writer.writerow({
            "t": round(time.time() - self.t0, 3),
            "x": round(x, 4), "y": round(y, 4), "yaw_deg": round(yaw, 2),
            "fitness": round(self.fitness, 5),
            "cmd_vx": round(self.cmd[0], 4),
            "cmd_vy": round(self.cmd[1], 4),
            "cmd_wz": round(self.cmd[2], 4),
            "xtrack_m": round(xt, 4),
            "plan_idx": idx,
            "dist_to_goal_m": round(dgoal, 4),
        })
        self.csv_file.flush()
        self.n += 1

    def save(self):
        try:
            self.csv_file.close()
        except Exception:
            pass
        print(f"\nсохранил {self.n} строк -> {self.tag}.csv")
        if self.plan_xy:
            print(f"путь ({len(self.plan_xy)} точек) -> {self.tag}_plan.csv")
        else:
            print("⚠️ /plan не приходил — кликнул ли цель? запущен ли goal_to_controller?")
        if self.n == 0:
            print("⚠️ ноль строк — был ли TF map->base_link? (Point-LIO + matcher live?)")


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else "corner_run"
    rclpy.init()
    node = CornerTest(tag)
    # ловим всё: на Ctrl+C rclpy иногда кидает RuntimeError из take_message
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception) as e:
        print(f"\n(остановлено: {type(e).__name__})")
    finally:
        node.save()
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == "__main__":
    main()
