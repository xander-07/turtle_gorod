#!/usr/bin/env python3
import math
import threading
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

try:
    import serial
    from serial import SerialException
except ImportError as exc:
    raise RuntimeError(
        "pyserial is not installed. Run: sudo apt install python3-serial"
    ) from exc


def yaw_to_quaternion(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class SerialBridge(Node):
    """Bridge ROS 2 cmd_vel <-> Arduino Uno wheel controller."""

    def __init__(self):
        super().__init__("serial_bridge")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("wheel_base_m", 0.185)
        self.declare_parameter("max_wheel_speed_mps", 0.45)
        self.declare_parameter("cmd_timeout_sec", 0.30)
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("cmd_vel_topic", "cmd_vel")

        self.port = str(self.get_parameter("port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.wheel_base_m = float(self.get_parameter("wheel_base_m").value)
        self.max_wheel_speed_mps = float(
            self.get_parameter("max_wheel_speed_mps").value
        )
        self.cmd_timeout_sec = float(self.get_parameter("cmd_timeout_sec").value)
        self.command_rate_hz = float(self.get_parameter("command_rate_hz").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        self._serial: Optional[serial.Serial] = None
        self._serial_lock = threading.Lock()
        self._rx_buffer = bytearray()

        self._last_cmd_ns: Optional[int] = None
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0

        self._last_warn_no_serial_ns = 0
        self._last_telemetry_ns: Optional[int] = None
        self._connected_ns: Optional[int] = None

        self.odom_pub = self.create_publisher(Odometry, "odom", 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 20)
        self.create_service(Trigger, "base/reset_odom", self._on_reset_odom)

        period = 1.0 / max(self.command_rate_hz, 1.0)
        self.create_timer(period, self._command_tick)
        self.create_timer(0.01, self._serial_read_tick)
        self.create_timer(1.0, self._reconnect_tick)

        self._open_serial()

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _open_serial(self):
        if self._serial is not None and self._serial.is_open:
            return

        try:
            ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.0,
                write_timeout=0.05,
            )
            ser.reset_input_buffer()
            with self._serial_lock:
                self._serial = ser
            self._connected_ns = self._now_ns()
            self._rx_buffer.clear()
            self.get_logger().info(
                f"Connected to lower level: {self.port} @ {self.baudrate}"
            )
        except (SerialException, OSError) as exc:
            self._serial = None
            self._connected_ns = None
            self.get_logger().warning(
                f"Cannot open lower level {self.port}: {exc}"
            )

    def _close_serial(self):
        with self._serial_lock:
            ser = self._serial
            self._serial = None
        self._connected_ns = None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def _reconnect_tick(self):
        if self._serial is None or not self._serial.is_open:
            self._open_serial()

    def _on_cmd_vel(self, msg: Twist):
        self._cmd_linear = float(msg.linear.x)
        self._cmd_angular = float(msg.angular.z)
        self._last_cmd_ns = self._now_ns()

    def _compute_wheels(self):
        now_ns = self._now_ns()
        stale = (
            self._last_cmd_ns is None
            or (now_ns - self._last_cmd_ns) * 1e-9 > self.cmd_timeout_sec
        )

        if stale:
            v = 0.0
            w = 0.0
        else:
            v = self._cmd_linear
            w = self._cmd_angular

        half_base = 0.5 * self.wheel_base_m
        left = v - w * half_base
        right = v + w * half_base

        peak = max(abs(left), abs(right))
        if peak > self.max_wheel_speed_mps > 0.0:
            scale = self.max_wheel_speed_mps / peak
            left *= scale
            right *= scale

        return left, right

    def _write_line(self, line: str) -> bool:
        payload = (line.rstrip() + "\n").encode("ascii", errors="strict")
        try:
            with self._serial_lock:
                if self._serial is None or not self._serial.is_open:
                    return False
                self._serial.write(payload)
            return True
        except (SerialException, OSError):
            self._close_serial()
            return False

    def _command_tick(self):
        left, right = self._compute_wheels()
        self._write_line(
            f"SET_WHEELS_SPEED {left * 1000.0:.2f} {right * 1000.0:.2f}"
        )

    def _serial_read_tick(self):
        ser = self._serial
        if ser is None or not ser.is_open:
            return

        try:
            waiting = ser.in_waiting
            if waiting <= 0:
                return
            data = ser.read(min(waiting, 4096))
        except (SerialException, OSError):
            self._close_serial()
            return

        if not data:
            return

        self._rx_buffer.extend(data)

        while True:
            pos = self._rx_buffer.find(b"\n")
            if pos < 0:
                break
            raw = self._rx_buffer[:pos]
            del self._rx_buffer[: pos + 1]
            line = raw.decode("ascii", errors="ignore").strip()
            if line:
                self._handle_line(line)

        if len(self._rx_buffer) > 8192:
            self.get_logger().warning("Serial RX buffer overflow; clearing buffer")
            self._rx_buffer.clear()

    def _handle_line(self, line: str):
        # Opening an Arduino Uno serial port toggles DTR and normally resets the
        # ATmega328P. If it resets while a telemetry line is in flight, the old
        # partial TEL line and the new READY banner can arrive as one line. This
        # is expected during connection and should not be reported as bad data.
        if "READY turtle_gorod_low_level" in line:
            self.get_logger().info("Lower level is ready")
            return

        if not line.startswith("TEL,"):
            if line.startswith("ERR"):
                self.get_logger().warning(f"Lower level: {line}")
            return

        fields = line.split(",")
        if len(fields) != 12:
            in_startup_grace = (
                self._connected_ns is not None
                and (self._now_ns() - self._connected_ns) < int(2e9)
            )
            if not in_startup_grace:
                self.get_logger().warning(
                    f"Malformed telemetry ({len(fields)} fields): {line}"
                )
            return

        try:
            _ms = int(fields[1])
            x_m = float(fields[2]) / 1000.0
            y_m = float(fields[3]) / 1000.0
            theta = float(fields[4])
            _enc_l = int(fields[5])
            _enc_r = int(fields[6])
            vl_mps = float(fields[7]) / 1000.0
            vr_mps = float(fields[8]) / 1000.0
            _target_l = float(fields[9])
            _target_r = float(fields[10])
            watchdog = int(fields[11])
        except ValueError:
            in_startup_grace = (
                self._connected_ns is not None
                and (self._now_ns() - self._connected_ns) < int(2e9)
            )
            if not in_startup_grace:
                self.get_logger().warning(f"Cannot parse telemetry: {line}")
            return

        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(theta)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        msg.pose.pose.position.x = x_m
        msg.pose.pose.position.y = y_m
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = 0.5 * (vl_mps + vr_mps)
        if abs(self.wheel_base_m) > 1e-6:
            msg.twist.twist.angular.z = (vr_mps - vl_mps) / self.wheel_base_m

        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.03
        msg.twist.covariance[0] = 0.02
        msg.twist.covariance[7] = 0.02
        msg.twist.covariance[35] = 0.05

        self.odom_pub.publish(msg)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = x_m
            tf.transform.translation.y = y_m
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

        self._last_telemetry_ns = self._now_ns()

        if watchdog:
            now_ns = self._now_ns()
            if now_ns - self._last_warn_no_serial_ns > int(2e9):
                self._last_warn_no_serial_ns = now_ns
                self.get_logger().warning("Lower-level command watchdog is active")

    def _on_reset_odom(self, _request, response):
        if self._write_line("SET_POSE 0 0 0"):
            response.success = True
            response.message = "Lower-level odometry reset requested"
        else:
            response.success = False
            response.message = "Lower level is not connected"
        return response

    def destroy_node(self):
        try:
            self._write_line("STOP")
        finally:
            self._close_serial()
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
