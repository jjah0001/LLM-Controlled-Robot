# ultrasonic_control/ultrasonic_publisher.py
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO

TRIG = 4    # BCM
ECHO = 27   # BCM

class UltrasonicPublisher(Node):
    def __init__(self):
        super().__init__('ultrasonic_publisher')
        self.pub = self.create_publisher(Range, 'ultrasonic/range', 10)

        # Params
        self.declare_parameter('rate_hz', 10.0)          # publish rate
        self.declare_parameter('timeout_s', 0.02)        # per pulse listen timeout (~20 ms)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.timeout_s = float(self.get_parameter('timeout_s').value)

        # GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)
        GPIO.output(TRIG, GPIO.LOW)
        time.sleep(0.05)

        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info('Ultrasonic publisher started (TRIG=4, ECHO=27).')

    def destroy_node(self):
        try:
            GPIO.cleanup([TRIG, ECHO])
        except Exception:
            pass
        super().destroy_node()

    def _measure_once(self) -> float | None:
        """Return distance in meters, or None on timeout (no publish)."""
        # 10 µs trigger
        GPIO.output(TRIG, GPIO.LOW); time.sleep(0.000002)
        GPIO.output(TRIG, GPIO.HIGH); time.sleep(0.000010)
        GPIO.output(TRIG, GPIO.LOW)

        t0 = time.time()
        while GPIO.input(ECHO) == 0:
            if (time.time() - t0) > self.timeout_s:
                return None
        t_rise = time.time()

        while GPIO.input(ECHO) == 1:
            if (time.time() - t_rise) > self.timeout_s:
                return None
        t_fall = time.time()

        pulse = t_fall - t_rise
        distance = max((pulse * 343.0) / 2.0, 0.0)  # meters
        self.get_logger().info(f'Distance in metres (m): {distance}')
        return distance

    def _tick(self):
        # Median-of-3 valid readings; if not enough valid samples, skip this cycle
        vals = []
        for _ in range(3):
            d = self._measure_once()
            if d is not None:
                vals.append(d)
            time.sleep(0.003)
        if len(vals) < 2:
            return  # don’t publish junk

        distance = sorted(vals)[len(vals)//2]

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_link'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26     # ~15°
        msg.min_range = 0.02
        msg.max_range = 4.00
        # Clamp to sensor spec; no special “inf” values
        msg.range = min(max(distance, msg.min_range), msg.max_range)
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
