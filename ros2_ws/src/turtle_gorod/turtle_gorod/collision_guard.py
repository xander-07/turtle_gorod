#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class CollisionGuard(Node):
    """Fail-safe layer between planner/teleop and the motor bridge."""

    def __init__(self):
        super().__init__("collision_guard")

        self.declare_parameter("stop_distance_m", 0.22)
        self.declare_parameter("slow_distance_m", 0.38)
        self.declare_parameter("front_sector_deg", 32.0)
        self.declare_parameter("rear_sector_deg", 32.0)
        self.declare_parameter("scan_timeout_sec", 0.35)
        self.declare_parameter("require_scan", True)
        self.declare_parameter("min_scale", 0.18)

        self.stop_distance = float(self.get_parameter("stop_distance_m").value)
        self.slow_distance = float(self.get_parameter("slow_distance_m").value)
        self.front_sector = math.radians(
            float(self.get_parameter("front_sector_deg").value)
        )
        self.rear_sector = math.radians(
            float(self.get_parameter("rear_sector_deg").value)
        )
        self.scan_timeout = float(self.get_parameter("scan_timeout_sec").value)
        self.require_scan = bool(self.get_parameter("require_scan").value)
        self.min_scale = float(self.get_parameter("min_scale").value)

        self._scan: Optional[LaserScan] = None
        self._scan_time_ns: Optional[int] = None
        self._warned_stale = False

        self.pub = self.create_publisher(Twist, "cmd_vel_safe", 20)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 20)
        self.create_subscription(LaserScan, "scan", self._on_scan, 10)

    def _on_scan(self, msg: LaserScan):
        self._scan = msg
        self._scan_time_ns = self.get_clock().now().nanoseconds
        self._warned_stale = False

    @staticmethod
    def _norm_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _sector_min(self, center: float, half_width: float) -> Optional[float]:
        scan = self._scan
        if scan is None:
            return None

        best = None
        angle = scan.angle_min
        for rng in scan.ranges:
            rel = abs(self._norm_angle(angle - center))
            if rel <= half_width and math.isfinite(rng):
                if scan.range_min <= rng <= scan.range_max:
                    best = rng if best is None else min(best, rng)
            angle += scan.angle_increment
        return best

    def _scan_is_fresh(self) -> bool:
        if self._scan_time_ns is None:
            return False
        age = (
            self.get_clock().now().nanoseconds - self._scan_time_ns
        ) * 1e-9
        return age <= self.scan_timeout

    def _on_cmd(self, msg: Twist):
        out = Twist()
        out.linear.x = msg.linear.x
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = msg.angular.z

        if self.require_scan and not self._scan_is_fresh():
            if not self._warned_stale:
                self.get_logger().warning(
                    "LaserScan is missing/stale: motion is blocked by safety guard"
                )
                self._warned_stale = True
            self.pub.publish(Twist())
            return

        if self._scan is None:
            self.pub.publish(out)
            return

        if msg.linear.x > 1e-4:
            distance = self._sector_min(0.0, self.front_sector)
        elif msg.linear.x < -1e-4:
            distance = self._sector_min(math.pi, self.rear_sector)
        else:
            distance = self._sector_min(0.0, math.pi)

        if distance is None:
            self.pub.publish(out)
            return

        if distance <= self.stop_distance:
            self.pub.publish(Twist())
            return

        if (
            abs(msg.linear.x) > 1e-4
            and self.stop_distance < distance < self.slow_distance
        ):
            span = max(self.slow_distance - self.stop_distance, 1e-3)
            scale = (distance - self.stop_distance) / span
            scale = max(self.min_scale, min(1.0, scale))
            out.linear.x *= scale
            out.angular.z *= max(scale, 0.35)

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
