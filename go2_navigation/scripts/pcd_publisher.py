#!/usr/bin/env python3
""".pcd как фон в RViz. Свой, а не pcl_ros/pcd_to_pointcloud: тот публикует
VOLATILE, и RViz, подключившийся позже, не получает ничего. Здесь TRANSIENT_LOCAL.

  ros2 run go2_navigation pcd_publisher.py --ros-args -p pcd:=/home/.../reloc2_gravity.pcd
"""
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def read_pcd_xyz(path):
    """.pcd (ascii или binary, x y z float32) -> Nx3."""
    with open(path, "rb") as f:
        raw = f.read()
    end = raw.find(b"DATA")
    line_end = raw.find(b"\n", end)
    header = raw[:line_end].decode("ascii", "replace")
    fmt = header.split("DATA")[-1].strip()

    fields, size, count, npts = [], [], [], 0
    for line in header.splitlines():
        p = line.split()
        if not p:
            continue
        if p[0] == "FIELDS":
            fields = p[1:]
        elif p[0] == "SIZE":
            size = [int(v) for v in p[1:]]
        elif p[0] == "COUNT":
            count = [int(v) for v in p[1:]]
        elif p[0] == "POINTS":
            npts = int(p[1])

    if fmt == "ascii":
        arr = np.loadtxt(path, skiprows=header.count("\n") + 1, usecols=(0, 1, 2))
        return arr.astype(np.float32)

    if fmt != "binary":
        raise RuntimeError("поддерживаются только ascii и binary .pcd, здесь: " + fmt)

    stride = sum(s * c for s, c in zip(size, count))
    body = raw[line_end + 1: line_end + 1 + npts * stride]
    off = 0
    for name, s, c in zip(fields, size, count):
        if name == "x":
            break
        off += s * c
    buf = np.frombuffer(body, dtype=np.uint8).reshape(npts, stride)
    return buf[:, off:off + 12].copy().view(np.float32).reshape(npts, 3)


class PcdPublisher(Node):
    def __init__(self):
        super().__init__("pcd_publisher")
        path = self.declare_parameter("pcd", "").value
        frame = self.declare_parameter("frame", "camera_init").value
        topic = self.declare_parameter("topic", "/prior_map").value
        if not path:
            self.get_logger().error("не задан параметр pcd"); return

        pts = read_pcd_xyz(path)
        self.get_logger().info("карта %s: %d точек -> %s (кадр %s)"
                               % (path, len(pts), topic, frame))

        # защёлка: RViz получит карту, даже если запустится позже
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(PointCloud2, topic, qos)

        msg = PointCloud2()
        msg.header = Header(frame_id=frame)
        msg.height, msg.width = 1, len(pts)
        msg.fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate(("x", "y", "z"))]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(pts)
        msg.is_dense = True
        msg.data = pts.astype(np.float32).tobytes()
        self.msg = msg
        # переиздание раз в 2 с страхует от гонок при старте RViz
        self.create_timer(2.0, lambda: self.pub.publish(self.msg))
        self.pub.publish(msg)


def main():
    rclpy.init()
    try:
        rclpy.spin(PcdPublisher())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
