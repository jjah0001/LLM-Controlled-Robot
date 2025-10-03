# pi_robot/pi_robot/ultrasonic_publisher.py
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

import RPi.GPIO as GPIO

TRIG = 4  # BCM
ECHO = 27   # BCM

class UltrasonicPublisher(Node):
    def __init__(self):
        super().__init__('ultrasonic_publisher')
        self.pub = self.create_publisher(Range, 'ultrasonic/range', 10)

        # Parameters
        self.declare_parameter('rate_hz', 10.0)
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)
        GPIO.output(TRIG, GPIO.LOW)
        time.sleep(0.05)

        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info('Ultrasonic publisher started (TRIG=27, ECHO=4).')

    def destroy_node(self):
        try:
            GPIO.cleanup([TRIG, ECHO])
        except Exception:
            pass
        super().destroy_node()

    @staticmethod
    def _measure_once(timeout_s: float = 0.03) -> float:
        """Return distance in meters, or float('inf') on timeout."""
        # Trigger 10 us pulse
        GPIO.output(TRIG, GPIO.LOW)
        time.sleep(0.000002)
        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.000010)
        GPIO.output(TRIG, GPIO.LOW)

        # Wait for echo HIGH
        start = time.time()
        while GPIO.input(ECHO) == 0:
            if (time.time() - start) > timeout_s:
                return float('inf')
        t_rise = time.time()

        # Wait for echo LOW
        while GPIO.input(ECHO) == 1:
            if (time.time() - t_rise) > timeout_s:
                return float('inf')
        t_fall = time.time()

        pulse = t_fall - t_rise  # seconds
        # Speed of sound ~343 m/s => distance (m) = pulse * 343 / 2
        distance_m = (pulse * 343.0) / 2.0
        return max(distance_m, 0.0)

    def _tick(self):
        # Take a small median-of-3 to reduce jitter
        readings = []
        for _ in range(3):
            d = self._measure_once()
            readings.append(d)
            time.sleep(0.003)

        distance = sorted(readings)[1]

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_link'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26  # ~15°
        msg.min_range = 0.02
        msg.max_range = 4.0
        msg.range = distance if distance != float('inf') else msg.max_range + 1.0

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = UltrasonicPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
