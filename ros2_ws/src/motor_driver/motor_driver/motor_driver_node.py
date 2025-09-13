import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        self.subscription = self.create_subscription(
            String,
            'motor_command',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Motor pin setup
        self.motor_pins = [17, 22, 23, 24]
        GPIO.setmode(GPIO.BCM)
        for pin in self.motor_pins:
            GPIO.setup(pin, GPIO.OUT)

        self.get_logger().info('Motor driver node started.')

    def listener_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f'Received command: {command}')
        if command == 'forward':
            self.forward(0.5)
        elif command == 'backward':
            self.backward(0.5)
        elif command == 'left':
            self.left(0.5)
        elif command == 'right':
            self.right(0.5)
        elif command == 'stop':
            self.stop()
        else:
            self.get_logger().warn('Unknown command!')

    def forward(self, sec):
        GPIO.output(17, False)
        GPIO.output(22, True)
        GPIO.output(23, True)
        GPIO.output(24, False)
        time.sleep(sec)
        self.stop()

    def backward(self, sec):
        GPIO.output(17, True)
        GPIO.output(22, False)
        GPIO.output(23, False)
        GPIO.output(24, True)
        time.sleep(sec)
        self.stop()

    def left(self, sec):
        GPIO.output(17, True)
        GPIO.output(22, False)
        GPIO.output(23, True)
        GPIO.output(24, False)
        time.sleep(sec)
        self.stop()

    def right(self, sec):
        GPIO.output(17, False)
        GPIO.output(22, True)
        GPIO.output(23, False)
        GPIO.output(24, True)
        time.sleep(sec)
        self.stop()

    def stop(self):
        for pin in self.motor_pins:
            GPIO.output(pin, False)

    def destroy_node(self):
        self.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

