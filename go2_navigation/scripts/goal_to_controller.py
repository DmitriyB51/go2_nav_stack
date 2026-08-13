#!/usr/bin/env python3
"""Этап 4: клик цели -> план -> контроллер (RPP) -> /cmd_vel.

Цепочка: /goal_pose -> поза из TF -> ComputePathToPose -> /plan -> FollowPath.
bt_navigator намеренно не поднимаем, чтобы изолировать слой.

⚠️ Офлайн робот в бэге не слушает /cmd_vel, поэтому через ~10 с progress_checker
объявит "застрял" — это ожидаемо. Смотрим на сами команды: ros2 topic echo /cmd_vel

Поднимается из nav2_stage4.launch.py
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose, FollowPath
import tf2_ros


class GoalToController(Node):
    def __init__(self):
        super().__init__("goal_to_controller")

        # map->base_link = старт для планировщика
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.controller = ActionClient(self, FollowPath, "follow_path")

        # /plan — для RViz
        self.path_pub = self.create_publisher(Path, "/plan", 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)

        # Реплан только когда есть причина И поза устойчива (см. replan_tick).
        # Слепой реплан раз в секунду гнал путь за скачками локализации (до ~1 м)
        # -> робот гонялся за прыгающим путём у углов.
        self.active_goal = None            # None = стоим
        self.current_path = None           # для детектора отклонения
        self.last_pose = None              # (x,y) прошлого такта — детектор скачка
        self.last_pose_t = 0.0
        self.last_replan_t = 0.0
        self.replan_period_s = 1.0
        self.stop_replan_m = 0.35          # ближе этого к цели реплан выключаем
        self.deviation_thresh = 0.4        # реплан, только если сошли дальше этого [м]
        self.max_speed = 0.35              # чуть выше desired_linear_vel
        self.jump_margin = 0.30            # запас к физически возможному шагу позы [м]
        self.min_replan_interval = 1.0
        self.create_timer(self.replan_period_s, self.replan_tick)

        self.get_logger().info(
            "goal_to_controller готов (с перепланированием ~1 Гц): кликай 'Nav2 Goal'.")

    def on_goal(self, goal_msg: PoseStamped):
        self.active_goal = goal_msg
        self.current_path = None
        self.last_pose = None
        self.get_logger().info(
            f"новая цель ({goal_msg.pose.position.x:.2f}, {goal_msg.pose.position.y:.2f})")
        self.plan_and_follow(goal_msg)

    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    # отклонение от пути [м]
    def _cross_track(self, xy):
        if self.current_path is None or not self.current_path.poses:
            return 1e9                       # пути нет -> нужен первый план
        best = 1e9
        for ps in self.current_path.poses:
            dx = ps.pose.position.x - xy[0]
            dy = ps.pose.position.y - xy[1]
            best = min(best, (dx * dx + dy * dy) ** 0.5)
        return best

    def replan_tick(self):
        if self.active_goal is None:
            return
        xy = self._robot_xy()
        if xy is None:
            return
        now = time.time()

        # у цели реплан выключаем — даём последнему FollowPath доехать
        gx, gy = self.active_goal.pose.position.x, self.active_goal.pose.position.y
        if ((gx - xy[0]) ** 2 + (gy - xy[1]) ** 2) ** 0.5 < self.stop_replan_m:
            self.active_goal = None
            self.get_logger().info("у цели — перепланирование остановлено")
            return

        # сдвиг больше физически возможного за такт = скачок оценки, не движение
        if self.last_pose is not None:
            dt = max(now - self.last_pose_t, 1e-3)
            moved = ((xy[0] - self.last_pose[0]) ** 2 + (xy[1] - self.last_pose[1]) ** 2) ** 0.5
            if moved > self.max_speed * dt + self.jump_margin:
                self.get_logger().warn(
                    f"скачок позы {moved:.2f} м за {dt:.1f} с — пропускаю реплан "
                    "(похоже на ошибку локализации, а не движение)")
                self.last_pose, self.last_pose_t = xy, now
                return
        self.last_pose, self.last_pose_t = xy, now

        # на пути — не трогаем
        if self._cross_track(xy) < self.deviation_thresh:
            return

        if now - self.last_replan_t < self.min_replan_interval:
            return
        self.last_replan_t = now
        self.get_logger().info("сошли с пути — перепланирую от текущей позы")
        self.plan_and_follow(self.active_goal)

    def plan_and_follow(self, goal_msg: PoseStamped):
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(
                f"не могу найти робота в TF (map->base_link): {e}. "
                "Запущены ли matcher и tf_setup?")
            return

        start = PoseStamped()
        start.header.frame_id = "map"
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = tf.transform.translation.x
        start.pose.position.y = tf.transform.translation.y
        start.pose.orientation = tf.transform.rotation

        self.get_logger().info(
            f"цель ({goal_msg.pose.position.x:.2f}, {goal_msg.pose.position.y:.2f}) — "
            "строю путь", throttle_duration_sec=5.0)

        if not self.planner.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("planner_server не отвечает — он 'active'?")
            return

        request = ComputePathToPose.Goal()
        request.start = start
        request.goal = goal_msg
        request.use_start = True
        self.planner.send_goal_async(request).add_done_callback(self.on_plan_accepted)

    def on_plan_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("планировщик отклонил запрос")
            return
        handle.get_result_async().add_done_callback(self.on_plan_result)

    def on_plan_result(self, future):
        path = future.result().result.path
        if len(path.poses) == 0:
            self.get_logger().warn("маршрут НЕ найден — цель за стеной/в неизвестном")
            return

        self.current_path = path
        self.path_pub.publish(path)
        self.get_logger().info(
            f"путь построен ({len(path.poses)} точек) — отдаю контроллеру, "
            "поехали командами /cmd_vel", throttle_duration_sec=5.0)

        if not self.controller.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("controller_server не отвечает — он 'active'?")
            return

        follow = FollowPath.Goal()
        follow.path = path
        follow.controller_id = "FollowPath"              # плагин RPP
        follow.goal_checker_id = "general_goal_checker"
        self.controller.send_goal_async(
            follow, feedback_callback=self.on_feedback
        ).add_done_callback(self.on_follow_accepted)

    def on_follow_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("контроллер отклонил путь")
            return
        self.get_logger().info("контроллер принял путь — смотри /cmd_vel",
                               throttle_duration_sec=5.0)
        handle.get_result_async().add_done_callback(self.on_follow_result)

    def on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"скорость {fb.speed:.2f} м/с, до цели {fb.distance_to_goal:.2f} м",
            throttle_duration_sec=1.0)

    def on_follow_result(self, future):
        # 4 = SUCCEEDED. При реплане старые FollowPath вытесняются и возвращают
        # не-4 (CANCELED/ABORTED) — это норма
        if future.result().status == 4:
            self.active_goal = None
            self.get_logger().info("ЦЕЛЬ ДОСТИГНУТА — стоп")


def main():
    rclpy.init()
    node = GoalToController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
