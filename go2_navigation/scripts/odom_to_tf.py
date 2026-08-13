#!/usr/bin/env python3
"""TF из /state_estimation, чтобы плио-бэг можно было просто посмотреть: в нём нет
/tf, а облака лежат в кадре "body", и без camera_init->body RViz не покажет ничего.

Колбэк только запоминает позу, публикует таймер: /state_estimation идёт ~7400 Гц
(на скорости 3x — за 20 тысяч в секунду), Python не успевает, TF выходит рывками и
RViz сыплет "Message Filter dropping message: frame 'body' ..." (формулировка
обманчива — дело не в старых данных, а в том, что TF в этот момент не тот).

Наклон стартового кадра (~20°) не убирается — сырой просмотр как есть.
  ros2 run go2_navigation odom_to_tf.py
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTf(Node):
    def __init__(self):
        super().__init__("odom_to_tf")
        self.br = TransformBroadcaster(self)
        self.latest = None

        rate = self.declare_parameter("rate", 100.0).value

        # depth=1 + BEST_EFFORT: пусть DDS выбрасывает старое сам, до разбора в Python
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        self.create_subscription(Odometry, "/state_estimation", self.cb, qos)

        self.create_timer(1.0 / rate, self.publish_tf)
        self.get_logger().info(
            "публикую TF camera_init->body из /state_estimation, %.0f Гц" % rate)

    def cb(self, msg: Odometry):
        self.latest = msg

    def publish_tf(self):
        msg = self.latest
        if msg is None:
            return
        t = TransformStamped()
        # штамп из одометрии, не now(): облака ищут TF по времени из бэга
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "camera_init"
        t.child_frame_id = "body"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    try:
        rclpy.spin(OdomToTf())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
