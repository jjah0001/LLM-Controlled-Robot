# ultrasonic_control/ultrasonic_publisher.py
"""HC-SR04 ultrasonic sensor -> sensor_msgs/Range on ultrasonic/range.

Pins (trig_pin / echo_pin) and frame_id are parameters, so a second sensor can run as
another instance with a remapped topic.

Follows REP 117: when no echo comes back within range the node publishes +inf
("nothing in range") instead of skipping the reading, so consumers can tell a
clear path from a dead sensor (which publishes nothing and goes stale).
"""
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO

TRIG = 4    # BCM, default pins of the front sensor
ECHO = 27   # BCM (through a 5V -> 3.3V divider!)
SPEED_OF_SOUND = 343.0  # m/s at ~20 C


class UltrasonicPublisher(Node):
    def __init__(self):
        super().__init__('ultrasonic_publisher')
        self.pub = self.create_publisher(Range, 'ultrasonic/range', 10)

        self.declare_parameter('trig_pin', TRIG)
        self.declare_parameter('echo_pin', ECHO)
        self.declare_parameter('frame_id', 'ultrasonic_link')
        self.declare_parameter('rate_hz', 15.0)
        self.declare_parameter('timeout_s', 0.025)   # max echo length (~4.3 m round trip)
        self.declare_parameter('min_range', 0.02)
        self.declare_parameter('max_range', 4.0)
        self.trig = int(self.get_parameter('trig_pin').value)
        self.echo = int(self.get_parameter('echo_pin').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.timeout_s = float(self.get_parameter('timeout_s').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.echo, GPIO.IN)
        time.sleep(0.05)

        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(
            f'Ultrasonic publisher started (TRIG={self.trig}, ECHO={self.echo}) '
            f'on {self.pub.topic_name}.')

    def destroy_node(self):
        try:
            GPIO.cleanup([self.trig, self.echo])
        except Exception:
            pass
        super().destroy_node()

    def _measure_once(self):
        """Distance in metres, math.inf if the echo is longer than range, None on fault."""
        GPIO.output(self.trig, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trig, GPIO.LOW)

        t0 = time.perf_counter()
        while GPIO.input(self.echo) == 0:
            if time.perf_counter() - t0 > self.timeout_s:
                return None          # echo never started: wiring/sensor fault
        t_rise = time.perf_counter()

        while GPIO.input(self.echo) == 1:
            if time.perf_counter() - t_rise > self.timeout_s:
                return math.inf      # no echo within range: path is clear
        pulse = time.perf_counter() - t_rise
        return pulse * SPEED_OF_SOUND / 2.0

    def _tick(self):
        # Median of 3 to reject single bad echoes; skip the cycle on sensor faults.
        vals = []
        for _ in range(3):
            d = self._measure_once()
            if d is not None:
                vals.append(d)
            time.sleep(0.01)         # let the previous ping die out
        if len(vals) < 2:
            self.get_logger().warn('Ultrasonic not responding', throttle_duration_sec=5.0)
            return
        distance = sorted(vals)[len(vals) // 2]

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26     # ~15 degrees
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        if distance > self.max_range:
            msg.range = math.inf
        else:
            msg.range = max(distance, self.min_range)
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
