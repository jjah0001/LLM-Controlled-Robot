#!/usr/bin/env python3
import time
from collections import deque
from statistics import median
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO

class HCSR04Publisher(Node):
    def __init__(self):
        super().__init__('hcsr04_publisher')
        self.declare_parameter('trig_pin', 27)         # BCM
        self.declare_parameter('echo_pin', 4)          # BCM (use level shifter!)
        self.declare_parameter('range_topic', '/ultrasonic/front')
        self.declare_parameter('frame_id', 'ultra_front')
        self.declare_parameter('publish_hz', 15)
        self.declare_parameter('temperature_c', 20.0)
        self.declare_parameter('median_window', 3)
        self.declare_parameter('timeout_s', 0.03)

        self.trig = int(self.get_parameter('trig_pin').value)
        self.echo = int(self.get_parameter('echo_pin').value)
        self.topic = self.get_parameter('range_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.hz = int(self.get_parameter('publish_hz').value)
        self.temp = float(self.get_parameter('temperature_c').value)
        self.win = max(1, int(self.get_parameter('median_window').value))
        self.timeout = float(self.get_parameter('timeout_s').value)

        self.min_range = 0.02
        self.max_range = 4.0
        self.fov = 0.26  # ~15 deg

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.echo, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.pub = self.create_publisher(Range, self.topic, 10)
        self.timer = self.create_timer(1.0/max(self.hz,1), self.tick)
        self.buf = deque(maxlen=self.win)
        self.get_logger().info(f"HC-SR04 on TRIG={self.trig} ECHO={self.echo} → {self.topic}")

    def destroy_node(self):
        try:
            GPIO.cleanup([self.trig, self.echo])
        finally:
            super().destroy_node()

    def c(self):  # speed of sound (m/s)
        return 331.3 + 0.606 * self.temp

    def measure_once(self):
        GPIO.output(self.trig, GPIO.LOW); time.sleep(2e-6)
        GPIO.output(self.trig, GPIO.HIGH); time.sleep(10e-6)
        GPIO.output(self.trig, GPIO.LOW)

        t0 = time.perf_counter()
        while GPIO.input(self.echo) == 0:
            if time.perf_counter() - t0 > self.timeout:
                return None
        tr = time.perf_counter()
        while GPIO.input(self.echo) == 1:
            if time.perf_counter() - tr > self.timeout:
                return None
        tf = time.perf_counter()

        dist = (self.c() * (tf - tr)) / 2.0
        if not (self.min_range <= dist <= self.max_range):
            return None
        return dist

    def tick(self):
        d = self.measure_once()
        if d is not None:
            self.buf.append(d)
        d_pub = median(self.buf) if len(self.buf) else float("nan")

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = float(self.fov)
        msg.min_range = float(self.min_range)
        msg.max_range = float(self.max_range)
        msg.range = float(d_pub)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    n = HCSR04Publisher()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
