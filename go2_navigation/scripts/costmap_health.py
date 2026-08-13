#!/usr/bin/env python3
"""Не врёт ли живая costmap?

Локальная costmap живёт в кадре map и наследует скачки коррекции матчера: отметки
размазываются быстрее, чем их стирает raytrace-очистка, окно заполняется, робот
оказывается замурован посреди пустого коридора (сорванный заезд 2026-07-24).

Число летальных клеток об этом не говорит ничего — говорит то, ГДЕ они лежат.
Живой лидар видит в основном те же стены, что и статичная карта:
    клетка на стене карты   -> настоящая стена;
    клетка в свободном месте -> новое препятствие ИЛИ фантом размазывания.
Здоровый замер: 83 % на стенах, 5.2 % в свободном. 30 % и выше = размазывается.

Мало фантомов -> можно добавить obstacle_layer в global_costmap (планировщик
начнёт объезжать, а не только стоять). Много -> копить препятствия в кадре
одометрии camera_init, где нет скачков матчера.

Запуск, когда подняты Point-LIO + матчер + run_nav2.sh:
  source ~/ros_env.sh
  ros2 run go2_navigation costmap_health.py [--ros-args -p period:=5.0]

⚠️ Мерить, когда робот СТОИТ НА НОГАХ: у сидящей собаки лидар видит землю и
   собственные лапы во все стороны, замер бессмыслен.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import Costmap


class CostmapHealth(Node):
    def __init__(self):
        super().__init__("costmap_health")
        # 253 = INSCRIBED, 254 = LETHAL; раздутая зона (<253) есть везде
        self.lethal = self.declare_parameter("lethal_cost", 253).value
        period = self.declare_parameter("period", 2.0).value
        # ближе этого к стене карты клетка = "съехавшая стена", не фантом (3 клетки)
        self.near_wall_m = self.declare_parameter("near_wall_m", 0.15).value

        self.static_map = None
        self.local = None

        # /map защёлкнута (transient_local): обычная подписка её не получит,
        # если мы подключились позже map_server
        latched = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
        self.create_subscription(Costmap, "/local_costmap/costmap_raw",
                                 self.on_local, 1)
        self.create_timer(period, self.report)
        self.get_logger().info("слежу за костмапом, отчёт раз в %.1f с" % period)

    def on_map(self, msg):
        self.static_map = msg

    def on_local(self, msg):
        self.local = msg

    def near_wall(self, x, y):
        """Есть ли стена статичной карты в радиусе near_wall_m.

        Разделение обязательно: стены в 2D-сетке толщиной 1-2 клетки, и сдвига
        позы на 5-10 см хватает, чтобы попадания по настоящей стене съехали в
        соседнюю свободную клетку. Это смещение (вопрос к локализации), а не
        фантом. Настоящее размазывание — отметки ВДАЛИ от любой стены.
        """
        m = self.static_map.info
        r = int(self.near_wall_m / m.resolution)
        col0 = int((x - m.origin.position.x) / m.resolution)
        row0 = int((y - m.origin.position.y) / m.resolution)
        for row in range(row0 - r, row0 + r + 1):
            if row < 0 or row >= m.height:
                continue
            base = row * m.width
            for col in range(col0 - r, col0 + r + 1):
                if col < 0 or col >= m.width:
                    continue
                v = self.static_map.data[base + col]
                if v >= 65:
                    return True
        return False

    def static_at(self, x, y):
        """'стена' / 'свободно' / 'неизвестно' / None (вне карты)."""
        m = self.static_map.info
        col = int((x - m.origin.position.x) / m.resolution)
        row = int((y - m.origin.position.y) / m.resolution)
        if col < 0 or row < 0 or col >= m.width or row >= m.height:
            return None
        v = self.static_map.data[row * m.width + col]
        if v < 0:
            return "неизвестно"
        # 65 = occupied_thresh наших .yaml карт
        return "стена" if v >= 65 else "свободно"

    def report(self):
        if self.static_map is None:
            self.get_logger().warn("нет /map — map_server поднят?")
            return
        if self.local is None:
            self.get_logger().warn("нет /local_costmap/costmap_raw — узел костмапа поднят?")
            return

        m = self.local.metadata
        counts = {"стена": 0, "свободно": 0, "неизвестно": 0, "вне карты": 0}
        near = 0        # из "свободных" — льнущие к стене (сдвиг, не фантом)
        total = 0
        for row in range(m.size_y):
            base = row * m.size_x
            for col in range(m.size_x):
                if self.local.data[base + col] < self.lethal:
                    continue
                total += 1
                x = m.origin.position.x + (col + 0.5) * m.resolution
                y = m.origin.position.y + (row + 0.5) * m.resolution
                where = self.static_at(x, y) or "вне карты"
                counts[where] += 1
                if where == "свободно" and self.near_wall(x, y):
                    near += 1

        if total == 0:
            self.get_logger().info("занятых клеток нет — лидар ничего не метит "
                                   "(проверь /cloud_registered_body и высоты отсечки)")
            return

        pct = {k: 100.0 * v / total for k, v in counts.items()}
        # вердикт только по "вдали от стен": на стенах так и должно быть, в
        # неизвестном — реальные стены вне 2D-карты (reloc2 снята одним проходом),
        # у стены — сдвиг на пару клеток
        far = counts["свободно"] - near
        pct_far = 100.0 * far / total
        pct_near = 100.0 * near / total
        verdict = ("здорово" if pct_far < 10 else
                   "терпимо" if pct_far < 25 else
                   "ПЛОХО: похоже на размазывание")
        self.get_logger().info(
            "клеток %d | стены %.0f%% | у стены (сдвиг) %.0f%% | ВДАЛИ ОТ СТЕН %.0f%% | "
            "неизвестное %.0f%% | вне карты %.0f%% -> %s"
            % (total, pct["стена"], pct_near, pct_far, pct["неизвестно"],
               pct["вне карты"], verdict))


def main():
    rclpy.init()
    node = CostmapHealth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.ok(): Ctrl+C гасит контекст раньше нас, повторный shutdown падает
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
