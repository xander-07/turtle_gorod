#!/usr/bin/env python3
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
Ручное управление turtle_gorod

Основное управление (удобно через SSH):
  ↑ / 8       вперёд
  ↓ / 2       назад
  ← / 4       поворот влево
  → / 6       поворот вправо
  Space / 5   стоп

Дополнительно:
  W/S/A/D     движение (английская раскладка)
  Ц/Ы/Ф/В     те же клавиши в русской раскладке
  + / =       увеличить скорость
  - / _       уменьшить скорость
  Q / Esc     выход

Клавишу движения можно держать зажатой. Команды публикуются в /cmd_vel
и проходят через collision_guard. Если поток нажатий пропал более чем на
1.5 с, робот автоматически останавливается.
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")

        # 0.10 м/с — скорость, на которой нижний уровень уже откалиброван
        # и уверенно едет прямо. 0.70 рад/с оставляет колеса выше зоны
        # низкоскоростного заедания при развороте на месте.
        self.declare_parameter("linear_speed", 0.10)
        self.declare_parameter("angular_speed", 0.70)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("deadman_timeout_sec", 1.50)

        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.deadman_timeout = float(self.get_parameter("deadman_timeout_sec").value)

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_motion_key_time = 0.0
        self.running = True
        self.last_status = None

    def _print_status(self, text: str):
        if text == self.last_status:
            return
        self.last_status = text
        print(f"\r{text:<70}", end="", flush=True)

    def stop(self):
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.pub.publish(Twist())
        self._print_status("СТОП")

    def publish_current(self):
        now = time.monotonic()
        moving = (
            abs(self.target_linear) > 1e-6
            or abs(self.target_angular) > 1e-6
        )

        if moving and (now - self.last_motion_key_time > self.deadman_timeout):
            self.target_linear = 0.0
            self.target_angular = 0.0
            self._print_status("СТОП: не поступают клавиши управления")

        msg = Twist()
        msg.linear.x = self.target_linear
        msg.angular.z = self.target_angular
        self.pub.publish(msg)

    def set_motion(self, linear: float, angular: float, label: str):
        self.target_linear = linear
        self.target_angular = angular
        self.last_motion_key_time = time.monotonic()
        self._print_status(
            f"{label}: linear={linear:+.3f} м/с, angular={angular:+.3f} рад/с"
        )

    def change_speed(self, factor: float):
        self.linear_speed = max(0.04, min(0.25, self.linear_speed * factor))
        self.angular_speed = max(0.40, min(1.20, self.angular_speed * factor))
        self._print_status(
            f"Скорость: linear={self.linear_speed:.3f} м/с, "
            f"angular={self.angular_speed:.3f} рад/с"
        )


def read_key(timeout: float = 0.0):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return None

    ch = sys.stdin.read(1)
    if ch != "\x1b":
        return ch

    # Стрелки обычно приходят как ESC [ A/B/C/D.
    # Если после ESC продолжения нет, считаем это отдельной клавишей Esc.
    seq = ch
    for _ in range(2):
        ready, _, _ = select.select([sys.stdin], [], [], 0.03)
        if not ready:
            break
        seq += sys.stdin.read(1)
    return seq


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    if not sys.stdin.isatty():
        node.get_logger().error(
            "keyboard_teleop нужно запускать в интерактивном терминале"
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    old_settings = termios.tcgetattr(sys.stdin)
    period = 1.0 / max(node.publish_rate_hz, 1.0)

    print(HELP)
    print(
        f"Стартовая скорость: {node.linear_speed:.3f} м/с, "
        f"поворот: {node.angular_speed:.3f} рад/с"
    )

    try:
        tty.setcbreak(sys.stdin.fileno())
        next_publish = time.monotonic()

        while rclpy.ok() and node.running:
            key = read_key(0.01)

            if key is not None:
                lower = key.lower() if len(key) == 1 else key

                # Вперёд: стрелка, NumPad/цифра 8, W или русская Ц.
                if lower in ("w", "ц", "8", "\x1b[A"):
                    node.set_motion(node.linear_speed, 0.0, "ВПЕРЁД")

                # Назад: стрелка, NumPad/цифра 2, S или русская Ы.
                elif lower in ("s", "ы", "2", "\x1b[B"):
                    node.set_motion(-node.linear_speed, 0.0, "НАЗАД")

                # Влево: стрелка, NumPad/цифра 4, A или русская Ф.
                elif lower in ("a", "ф", "4", "\x1b[D"):
                    node.set_motion(0.0, node.angular_speed, "ВЛЕВО")

                # Вправо: стрелка, NumPad/цифра 6, D или русская В.
                elif lower in ("d", "в", "6", "\x1b[C"):
                    node.set_motion(0.0, -node.angular_speed, "ВПРАВО")

                elif key in (" ", "5"):
                    node.stop()
                elif key in ("+", "="):
                    node.change_speed(1.15)
                elif key in ("-", "_"):
                    node.change_speed(1.0 / 1.15)
                elif lower == "q" or key in ("\x03", "\x1b"):
                    node.running = False
                    break

            now = time.monotonic()
            if now >= next_publish:
                node.publish_current()
                rclpy.spin_once(node, timeout_sec=0.0)
                next_publish = now + period

    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
            time.sleep(0.05)
            node.stop()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            print("\nРучное управление остановлено.")
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
