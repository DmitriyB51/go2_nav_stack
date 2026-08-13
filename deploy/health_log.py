#!/usr/bin/env python3
"""Live health logger: one CSV row per second, so a divergence can be pinned to a
moment instead of guessed at afterwards.

    t_wall    epoch seconds
    dist_m    Point-LIO estimate from its OWN origin — THE key number: tens of
              metres indoors is normal, hundreds/thousands = divergence
    dz_m      Z of the estimate (Point-LIO's weak axis)
    odom_hz   /state_estimation rate; sagging = Point-LIO starved of CPU
    scan_hz   /registered_scan rate (expect ~15)
    scan_pts  points in the last scan; a sudden drop = nothing to match against
    fitness   0.006-0.02 healthy, >0.3 matcher rejecting, huge = no correspondence
    cpu_pct   total across all cores
    load1     1-minute load average

Deliberately a plain subscriber doing no heavy work.
"""

import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32


class HealthLogger(Node):
    def __init__(self, out_path):
        super().__init__("health_logger")

        self.odom_count = 0
        self.scan_count = 0
        self.last_pos = (0.0, 0.0, 0.0)
        self.last_scan_pts = 0
        self.last_fitness = float("nan")

        fast_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Odometry, "/state_estimation", self.on_odom, fast_qos)
        self.create_subscription(PointCloud2, "/registered_scan", self.on_scan, fast_qos)
        self.create_subscription(Float32, "/localization/fitness", self.on_fitness, 10)

        self.f = open(out_path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow([
            "t_wall", "dist_m", "dz_m", "odom_hz", "scan_hz",
            "scan_pts", "fitness", "cpu_pct", "load1",
        ])
        self.f.flush()

        # CPU % = diff of /proc/stat between ticks
        self.prev_cpu = self.read_cpu()

        self.create_timer(1.0, self.tick)
        self.get_logger().info(f"health logging to {out_path}")

    def on_odom(self, msg):
        self.odom_count += 1
        p = msg.pose.pose.position
        self.last_pos = (p.x, p.y, p.z)

    def on_scan(self, msg):
        self.scan_count += 1
        self.last_scan_pts = msg.width * msg.height

    def on_fitness(self, msg):
        self.last_fitness = msg.data

    def read_cpu(self):
        """(idle, total) jiffies from /proc/stat's aggregate cpu line."""
        with open("/proc/stat") as fh:
            parts = fh.readline().split()[1:]
        vals = [int(v) for v in parts]
        idle = vals[3] + vals[4]          # idle + iowait
        return idle, sum(vals)

    def tick(self):
        idle, total = self.read_cpu()
        pidle, ptotal = self.prev_cpu
        d_total = total - ptotal
        cpu_pct = 100.0 * (1.0 - (idle - pidle) / d_total) if d_total > 0 else float("nan")
        self.prev_cpu = (idle, total)

        x, y, z = self.last_pos
        dist = math.sqrt(x * x + y * y + z * z)
        load1 = os.getloadavg()[0]

        self.w.writerow([
            f"{time.time():.1f}", f"{dist:.2f}", f"{z:.2f}",
            self.odom_count, self.scan_count, self.last_scan_pts,
            f"{self.last_fitness:.4f}", f"{cpu_pct:.1f}", f"{load1:.2f}",
        ])
        self.f.flush()

        # visible in the log tail without opening the CSV
        if dist > 200.0:
            self.get_logger().error(f"DIVERGED: Point-LIO {dist:.0f} m from origin")

        self.odom_count = 0
        self.scan_count = 0


def main():
    out = os.path.expanduser(f"~/health_{time.strftime('%H%M%S')}.csv")
    rclpy.init()
    node = HealthLogger(out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
