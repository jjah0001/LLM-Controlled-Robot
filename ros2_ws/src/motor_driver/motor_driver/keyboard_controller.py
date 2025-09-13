import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher = self.create_publisher(String, 'motor_command', 10)
        self.get_logger().info('Keyboard Controller Ready. Use keys: W/A/S/D/X')

        self.key_loop()

    def key_loop(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                msg = String()

                if key == 'w':
                    msg.data = 'forward'
                elif key == 's':
                    msg.data = 'backward'
                elif key == 'a':
                    msg.data = 'left'
                elif key == 'd':
                    msg.data = 'right'
                elif key == 'x':
                    msg.data = 'stop'
                elif key == '\x03':  # Ctrl+C
                    break
                else:
                    continue  # Ignore unknown keys

                self.get_logger().info(f'Sending: {msg.data}')
                self.publisher.publish(msg)
        except KeyboardInterrupt:
            pass

    def get_key(self):
        # Raw key capture for single key press (Unix only)
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        return key

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

