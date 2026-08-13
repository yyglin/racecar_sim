#!/usr/bin/env python3

import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


ARROW_KEYS = {
    b"\x1b[A": "up",
    b"\x1b[B": "down",
    b"\x1b[C": "right",
    b"\x1b[D": "left",
    b"\x1bOA": "up",
    b"\x1bOB": "down",
    b"\x1bOC": "right",
    b"\x1bOD": "left",
}


class TerminalReader:
    def __init__(self):
        if not sys.stdin.isatty():
            raise RuntimeError(
                "keyboard_teleop needs an interactive terminal on stdin"
            )
        self._fd = sys.stdin.fileno()
        self._settings = None

    def __enter__(self):
        self._settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self._settings is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._settings)

    def read_key(self, timeout):
        readable, _, _ = select.select([self._fd], [], [], timeout)
        if not readable:
            return None

        first = os.read(self._fd, 1)
        if first != b"\x1b":
            return first.decode(errors="ignore")

        sequence = first
        deadline = time.monotonic() + 0.03
        while len(sequence) < 3:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            readable, _, _ = select.select([self._fd], [], [], remaining)
            if not readable:
                break
            sequence += os.read(self._fd, 1)

        return ARROW_KEYS.get(sequence, "escape")


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_step", 0.25)
        self.declare_parameter("angular_step", 0.15)
        self.declare_parameter("max_linear_speed", 2.0)
        self.declare_parameter("max_angular_speed", 0.60)
        self.declare_parameter("command_timeout", 0.80)
        self.declare_parameter("publish_rate", 20.0)

        topic = self.get_parameter("cmd_vel_topic").value
        self.linear_step = float(self.get_parameter("linear_step").value)
        self.angular_step = float(self.get_parameter("angular_step").value)
        self.max_linear = float(
            self.get_parameter("max_linear_speed").value
        )
        self.max_angular = float(
            self.get_parameter("max_angular_speed").value
        )
        self.command_timeout = float(
            self.get_parameter("command_timeout").value
        )
        self.publish_rate = float(self.get_parameter("publish_rate").value)

        values = {
            "linear_step": self.linear_step,
            "angular_step": self.angular_step,
            "max_linear_speed": self.max_linear,
            "max_angular_speed": self.max_angular,
            "command_timeout": self.command_timeout,
            "publish_rate": self.publish_rate,
        }
        invalid = [name for name, value in values.items() if value <= 0.0]
        if invalid:
            raise ValueError(
                "Parameters must be greater than zero: " + ", ".join(invalid)
            )

        self.publisher = self.create_publisher(Twist, topic, 10)
        self.linear = 0.0
        self.angular = 0.0
        self.last_key_time = None
        self.auto_stopped = False

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    def handle_key(self, key):
        if key == "up":
            self.linear = self._clamp(
                self.linear + self.linear_step, self.max_linear
            )
        elif key == "down":
            self.linear = self._clamp(
                self.linear - self.linear_step, self.max_linear
            )
        elif key == "left":
            self.angular = self._clamp(
                self.angular + self.angular_step, self.max_angular
            )
        elif key == "right":
            self.angular = self._clamp(
                self.angular - self.angular_step, self.max_angular
            )
        elif key in {" ", "s", "S"}:
            self.stop()
        elif key in {"c", "C"}:
            self.angular = 0.0
        elif key in {"q", "Q", "escape"}:
            self.stop()
            self.publish()
            return False
        else:
            return True

        self.last_key_time = time.monotonic()
        self.auto_stopped = False
        self.publish()
        self.show_status()
        return True

    def apply_timeout(self):
        if self.last_key_time is None or self.auto_stopped:
            return
        if time.monotonic() - self.last_key_time < self.command_timeout:
            return
        self.stop()
        self.auto_stopped = True
        self.publish()
        self.show_status("输入超时，已自动停车")

    def stop(self):
        self.linear = 0.0
        self.angular = 0.0

    def publish(self):
        message = Twist()
        message.linear.x = self.linear
        message.angular.z = self.angular
        self.publisher.publish(message)

    def show_status(self, note=""):
        suffix = f"  {note}" if note else ""
        text = (
            f"\r速度 linear.x={self.linear:+.2f} m/s  "
            f"转向 angular.z={self.angular:+.2f} rad/s{suffix}"
        )
        sys.stdout.write(text.ljust(100))
        sys.stdout.flush()


HELP = """
方向键控制:
  ↑ / ↓      增加 / 减少速度（可进入倒车）
  ← / →      增加左转 / 右转角速度
  C          转向回中
  空格或 S   立即停车
  Q 或 Esc   停车并退出

需要保持按键或连续按键；释放按键后会因安全超时自动停车。
请让运行本节点的终端窗口保持键盘焦点。
"""


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0

    try:
        node = KeyboardTeleop()
        print(HELP)
        node.show_status()

        with TerminalReader() as reader:
            keep_running = True
            period = 1.0 / node.publish_rate
            while rclpy.ok() and keep_running:
                key = reader.read_key(period)
                if key is not None:
                    keep_running = node.handle_key(key)
                node.apply_timeout()
                node.publish()
                rclpy.spin_once(node, timeout_sec=0.0)

    except (KeyboardInterrupt, EOFError):
        pass
    except (RuntimeError, ValueError) as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f"keyboard_teleop: {error}", file=sys.stderr)
        exit_code = 1
    finally:
        if node is not None:
            node.stop()
            for _ in range(3):
                node.publish()
                rclpy.spin_once(node, timeout_sec=0.01)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
