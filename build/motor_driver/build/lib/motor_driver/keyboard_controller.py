#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.msg import Float64

import sys
import termios
import tty
import threading


class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')

        self.publisher = self.create_publisher(String, 'motor_command', 10)

        # FIX 1: correct message type for ultrasonic_data (matches your publisher)
        self.subscriber = self.create_subscription(
            Float64,
            'ultrasonic_data',
            self.listener_callback,
            10
        )

        self.get_logger().info('Keyboard Controller Ready. Use keys: W/A/S/D/X, I/K or ↑/↓. Press Q to quit.')

        # FIX 2: do NOT block in __init__. Run key loop in a thread.
        self._key_thread = threading.Thread(target=self.key_loop, daemon=True)
        self._key_thread.start()

    def key_loop(self):
        try:
            while rclpy.ok():
                key = self.get_key()

                # Normalise letters
                if isinstance(key, str):
                    key_lower = key.lower()
                else:
                    key_lower = key

                cmd = None

                if key_lower == 'w':
                    cmd = 'forward'
                elif key_lower == 's':
                    cmd = 'backward'
                elif key_lower == 'a':
                    cmd = 'left'
                elif key_lower == 'd':
                    cmd = 'right'
                elif key_lower == 'x':
                    cmd = 'stop'
                elif key_lower in ('i', '\x1b[A'):   # i or Up arrow
                    cmd = 'increase'
                elif key_lower in ('k', '\x1b[B'):   # k or Down arrow
                    cmd = 'decrease'
                elif key_lower in ('q',):            # quit cleanly
                    self.get_logger().info('Quitting...')
                    rclpy.shutdown()
                    return
                elif key == '\x03':  # Ctrl+C
                    rclpy.shutdown()
                    return
                else:
                    continue  # ignore unknown keys

                msg = String()
                msg.data = cmd
                self.get_logger().info(f'Sending: {msg.data}')
                self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Keyboard loop crashed: {e}')
            rclpy.shutdown()

    def get_key(self):
        """Raw key capture (Unix). Supports arrow keys by reading escape sequences."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)
            ch1 = sys.stdin.read(1)

            # Arrow keys come as escape sequences: \x1b [ A/B/C/D
            if ch1 == '\x1b':
                ch2 = sys.stdin.read(1)
                ch3 = sys.stdin.read(1)
                return ch1 + ch2 + ch3  # e.g. '\x1b[A'
            return ch1

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def listener_callback(self, msg: Float64):
        distance = msg.data
        self.get_logger().info(f'Ultrasonic Reading: {distance:.2f} cm')


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
