#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
from example_interfaces.msg import Float64
TRIG, ECHO = 4, 27

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')
        self.publisher = self.create_publisher(Float64, 'ultrasonic_data', 10)
        self.temperature_timer_ = self.create_timer(
            0.5, self.get_distance)
        
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)

        self.get_logger().info('Ultrasonic Node Ready')

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

        msg = Float64()
        msg.data = distance
        self.publisher.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

