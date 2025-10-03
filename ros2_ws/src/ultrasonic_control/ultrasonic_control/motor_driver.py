# pi_robot/pi_robot/motor_driver.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

import RPi.GPIO as GPIO

# L298N pins (BCM)
IN1, IN2 = 17, 22
IN3, IN4 = 23, 24
ENA, ENB = 18, 19  # PWM-capable

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # Parameters
        self.declare_parameter('stop_distance_m', 0.2)    # stop when nearer than this
        self.declare_parameter('hysteresis_m', 0.05)       # start again when farther than stop + hysteresis
        self.declare_parameter('speed_percent', 80.0)      # 0..100
        self.declare_parameter('pwm_hz', 1000.0)

        self.stop_distance = float(self.get_parameter('stop_distance_m').value)
        self.hyst = float(self.get_parameter('hysteresis_m').value)
        self.speed = float(self.get_parameter('speed_percent').value)
        self.pwm_hz = float(self.get_parameter('pwm_hz').value)

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        for pin in (IN1, IN2, IN3, IN4, ENA, ENB):
            GPIO.setup(pin, GPIO.OUT)
        self.pwm_a = GPIO.PWM(ENA, self.pwm_hz)
        self.pwm_b = GPIO.PWM(ENB, self.pwm_hz)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

        self.moving_forward = False
        self._stop_motors()

        self.sub = self.create_subscription(Range, 'ultrasonic/range', self._range_cb, 10)
        self.get_logger().info(
            f"Motor driver ready (IN1={IN1}, IN2={IN2}, IN3={IN3}, IN4={IN4}, ENA={ENA}, ENB={ENB}). "
            f"stop_distance={self.stop_distance:.2f} m, speed={self.speed:.0f}%"
        )

    def destroy_node(self):
        try:
            self._stop_motors()
            self.pwm_a.stop()
            self.pwm_b.stop()
            GPIO.cleanup([IN1, IN2, IN3, IN4, ENA, ENB])
        except Exception:
            pass
        super().destroy_node()

    # ---- low-level motor helpers ----
    def _apply_speed(self, percent: float):
        percent = max(0.0, min(100.0, percent))
        self.pwm_a.ChangeDutyCycle(percent)
        self.pwm_b.ChangeDutyCycle(percent)

    def _forward(self, percent: float):
        # Typical L298N forward: A = IN1 HIGH, IN2 LOW ; B = IN3 HIGH, IN4 LOW
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        self._apply_speed(percent)
        self.moving_forward = True

    def _stop_motors(self):
        self._apply_speed(0.0)
        # Brake (both inputs HIGH) or coast (both LOW). We'll coast to reduce current draw.
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
        self.moving_forward = False

    # ---- callback ----
    def _range_cb(self, msg: Range):
        # Normalize reading
        d = msg.range
        # If publisher signalled "no echo" (> max_range), treat as very far
        if d > msg.max_range:
            d = float('inf')

        # Control with hysteresis
        if self.moving_forward:
            if d <= self.stop_distance:
                self.get_logger().info(f'Obstacle at {d:.3f} m → STOP')
                self._stop_motors()
        else:
            if d >= (self.stop_distance + self.hyst) or d == float('inf'):
                self.get_logger().info(f'Path clear ({("inf" if d == float("inf") else f"{d:.3f}")} m) → FORWARD')
                self._forward(self.speed)

def main():
    rclpy.init()
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
