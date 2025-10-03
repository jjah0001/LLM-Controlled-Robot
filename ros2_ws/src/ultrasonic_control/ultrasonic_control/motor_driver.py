# ultrasonic_control/motor_driver.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO

# L298N pins (BCM)
IN1, IN2 = 17, 22
IN3, IN4 = 23, 24
ENA, ENB = 18, 19  # PWM

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # Params
        self.declare_parameter('stop_distance_m', 0.25)  # stop if <= this
        self.declare_parameter('speed_percent', 60.0)    # 0..100
        self.declare_parameter('pwm_hz', 1000.0)
        self.declare_parameter('start_moving', True)     # begin in forward motion

        self.stop_distance = float(self.get_parameter('stop_distance_m').value)
        self.speed = float(self.get_parameter('speed_percent').value)
        self.pwm_hz = float(self.get_parameter('pwm_hz').value)
        self.start_moving = bool(self.get_parameter('start_moving').value)

        # GPIO
        GPIO.setmode(GPIO.BCM)
        for p in (IN1, IN2, IN3, IN4, ENA, ENB):
            GPIO.setup(p, GPIO.OUT)
        self.pwm_a = GPIO.PWM(ENA, self.pwm_hz)
        self.pwm_b = GPIO.PWM(ENB, self.pwm_hz)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

        self.moving_forward = False
        if self.start_moving:
            self._forward(self.speed)
            self.get_logger().info(f"Starting FORWARD at {self.speed:.0f}% duty.")
        else:
            self._stop_motors()

        self.sub = self.create_subscription(Range, 'ultrasonic/range', self._range_cb, 10)
        self.get_logger().info(
            f"Motor driver ready. stop_distance={self.stop_distance:.2f} m, speed={self.speed:.0f}%"
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

    # --- low-level ---
    def _apply_speed(self, percent: float):
        percent = max(0.0, min(100.0, percent))
        self.pwm_a.ChangeDutyCycle(percent)
        self.pwm_b.ChangeDutyCycle(percent)

    def _forward(self, percent: float):
        GPIO.output(IN1, GPIO.LOW); GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.HIGH); GPIO.output(IN4, GPIO.LOW)
        self._apply_speed(percent)
        self.moving_forward = True

    def _stop_motors(self):
        self._apply_speed(0.0)
        GPIO.output(IN1, GPIO.LOW); GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW); GPIO.output(IN4, GPIO.LOW)
        self.moving_forward = False

    # --- callback ---
    def _range_cb(self, msg: Range):
        # Clamp to the message’s stated limits; treat as plain number only.
        d = min(max(msg.range, msg.min_range), msg.max_range)

        if d <= self.stop_distance:
            if self.moving_forward:
                self.get_logger().info(f'Obstacle at {d:.3f} m → STOP')
                self._stop_motors()
        else:
            if not self.moving_forward:
                self.get_logger().info(f'Clear ({d:.3f} m) → FORWARD')
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
