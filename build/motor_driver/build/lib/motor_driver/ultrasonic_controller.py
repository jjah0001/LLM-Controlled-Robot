import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

# Motor Pins
M1A, M1B = 17, 22
M2A, M2B = 23, 24

# Ultrasonic Pins
TRIG, ECHO = 4, 27
STOP_DISTANCE = 20.0  # cm

class UltrasonicController(Node):
    def __init__(self):
        super().__init__('ultrasonic_controller')
        self.subscription = self.create_subscription(
            String,
            'motor_command',
            self.listener_callback,
            10
        )
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([M1A, M1B, M2A, M2B], GPIO.OUT)
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)
        self.get_logger().info('Ultrasonic Controller Node started')

    def get_distance(self):
        GPIO.output(TRIG, False)
        time.sleep(0.0002)
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()
        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()

        duration = pulse_end - pulse_start
        distance = round(duration * 17150, 2)
        return distance

    def stop(self):
        GPIO.output(M1A, False)
        GPIO.output(M1B, False)
        GPIO.output(M2A, False)
        GPIO.output(M2B, False)

    def move_forward(self, duration=2.0):
        start = time.time()
        while time.time() - start < duration:
            dist = self.get_distance()
            self.get_logger().info(f'Distance: {dist} cm')
            if dist < STOP_DISTANCE:
                self.get_logger().warn('Obstacle detected! Stopping.')
                break
            GPIO.output(M1A, False)
            GPIO.output(M1B, True)
            GPIO.output(M2A, True)
            GPIO.output(M2B, False)
            time.sleep(0.1)
        self.stop()

    def move_left(self, duration=0.5):
        GPIO.output(M1A, True)
        GPIO.output(M1B, False)
        GPIO.output(M2A, True)
        GPIO.output(M2B, False)
        time.sleep(duration)
        self.stop()

    def move_right(self, duration=0.5):
        GPIO.output(M1A, False)
        GPIO.output(M1B, True)
        GPIO.output(M2A, False)
        GPIO.output(M2B, True)
        time.sleep(duration)
        self.stop()

    def move_backwards(self, duration=1.5):
        GPIO.output(M1A, True)
        GPIO.output(M1B, False)
        GPIO.output(M2A, False)
        GPIO.output(M2B, True)
        time.sleep(duration)
        self.stop()

    def listener_callback(self, msg):
        cmd = msg.data.lower()
        if cmd == "forward":
            self.move_forward()
        elif cmd == "left":
            self.move_left()
        elif cmd == "right":
            self.move_right()
        elif cmd == "backward":
            self.move_backwards()
        elif cmd == "stop":
            self.stop()
        else:
            self.get_logger().warn(f'Unknown command: {cmd}')

    def destroy_node(self):
        self.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()
