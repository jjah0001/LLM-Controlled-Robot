#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import sys
import termios
import tty
import threading

LINEAR_SPEED = 0.2   # m/s
ANGULAR_SPEED = 1.0  # rad/s


class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber = self.create_subscription(
            Range, 'ultrasonic/range', self.listener_callback, 10
        )

        self.get_logger().info(
            'Keyboard Controller Ready.\n'
            '  W / ↑  : forward\n'
            '  S / ↓  : backward\n'
            '  A / ←  : turn left\n'
            '  D / →  : turn right\n'
            '  X      : stop\n'
            '  Q      : quit'
        )

        self._key_thread = threading.Thread(target=self.key_loop, daemon=True)
        self._key_thread.start()

    def key_loop(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                msg = Twist()

                if key in ('w', 'W', '\x1b[A'):
                    msg.linear.x = LINEAR_SPEED
                elif key in ('s', 'S', '\x1b[B'):
                    msg.linear.x = -LINEAR_SPEED
                elif key in ('a', 'A', '\x1b[D'):
                    msg.angular.z = ANGULAR_SPEED
                elif key in ('d', 'D', '\x1b[C'):
                    msg.angular.z = -ANGULAR_SPEED
                elif key in ('x', 'X'):
                    pass  # zero Twist → stop
                elif key in ('q', 'Q', '\x03'):
                    self.get_logger().info('Quitting...')
                    rclpy.shutdown()
                    return
                else:
                    continue

                self.publisher.publish(msg)
                self.get_logger().info(
                    f'cmd_vel: linear.x={msg.linear.x:.2f}  angular.z={msg.angular.z:.2f}'
                )

        except Exception as e:
            self.get_logger().error(f'Keyboard loop crashed: {e}')
            rclpy.shutdown()

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch1 = sys.stdin.read(1)
            if ch1 == '\x1b':
                ch2 = sys.stdin.read(1)
                ch3 = sys.stdin.read(1)
                return ch1 + ch2 + ch3
            return ch1
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def listener_callback(self, msg: Range):
        self.get_logger().info(f'Ultrasonic: {msg.range:.2f} m', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
