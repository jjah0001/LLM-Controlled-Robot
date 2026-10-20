#!/usr/bin/env python3
"""L298N differential-drive motor driver: /cmd_vel (m/s, rad/s) -> PWM on the Pi's GPIO.

Converts the body twist into left/right wheel speeds with the robot's track width,
scales both wheels together when one would saturate (so the curve is preserved),
and maps the result onto [min_duty, max_duty] so speed scales smoothly above the
duty where the motors stall. Velocity ramping is done upstream in motion_controller.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import RPi.GPIO as GPIO

from motor_driver.kinematics import fraction_to_duty, twist_to_wheels

# BCM pins. Motor A = left, motor B = right.
IN1, IN2, ENA = 17, 22, 18
IN3, IN4, ENB = 23, 24, 19
PINS = (IN1, IN2, IN3, IN4, ENA, ENB)


class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        p = self.declare_parameter
        p('max_wheel_speed_mps', 0.30)
        p('track_width_m', 0.16)
        p('min_duty', 60.0)
        p('max_duty', 100.0)
        p('deadband', 0.02)
        p('pwm_hz', 1000.0)
        p('cmd_timeout_s', 0.5)
        p('invert_left', False)
        p('invert_right', False)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.max_wheel = float(g('max_wheel_speed_mps'))
        self.track = float(g('track_width_m'))
        self.min_duty = float(g('min_duty'))
        self.max_duty = float(g('max_duty'))
        self.deadband = float(g('deadband'))
        self.timeout = float(g('cmd_timeout_s'))
        self.inv_l = -1.0 if g('invert_left') else 1.0
        self.inv_r = -1.0 if g('invert_right') else 1.0

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for pin in PINS:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        self.pwm_a = GPIO.PWM(ENA, float(g('pwm_hz')))
        self.pwm_b = GPIO.PWM(ENB, float(g('pwm_hz')))
        self.pwm_a.start(0)
        self.pwm_b.start(0)

        self._last_cmd = self.get_clock().now()
        self._stopped = True
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_timer(0.1, self._watchdog)
        self.get_logger().info(
            f'Motor driver ready: {self.max_wheel:.2f} m/s at 100% duty, '
            f'track {self.track:.3f} m, duty {self.min_duty:.0f}-{self.max_duty:.0f}%.')

    def _cmd_cb(self, msg: Twist):
        self._last_cmd = self.get_clock().now()
        left, right = twist_to_wheels(msg.linear.x, msg.angular.z, self.track, self.max_wheel)
        self._drive(left * self.inv_l, right * self.inv_r)

    def _drive(self, left, right):
        # Forward = IN2 high on the left motor and IN3 high on the right (as wired).
        self._set(self.pwm_a, IN2, IN1, left)
        self._set(self.pwm_b, IN3, IN4, right)
        self._stopped = left == 0.0 and right == 0.0

    def _set(self, pwm, pin_fwd, pin_rev, fraction):
        duty = fraction_to_duty(fraction, self.min_duty, self.max_duty, self.deadband)
        if duty == 0.0:
            GPIO.output(pin_fwd, GPIO.LOW)
            GPIO.output(pin_rev, GPIO.LOW)
        elif fraction > 0:
            GPIO.output(pin_fwd, GPIO.HIGH)
            GPIO.output(pin_rev, GPIO.LOW)
        else:
            GPIO.output(pin_fwd, GPIO.LOW)
            GPIO.output(pin_rev, GPIO.HIGH)
        pwm.ChangeDutyCycle(duty)

    def _watchdog(self):
        elapsed = (self.get_clock().now() - self._last_cmd).nanoseconds / 1e9
        if elapsed > self.timeout and not self._stopped:
            self.get_logger().warn('No /cmd_vel for %.1fs: stopping motors.' % elapsed)
            self._drive(0.0, 0.0)

    def destroy_node(self):
        try:
            self._drive(0.0, 0.0)
            self.pwm_a.stop()
            self.pwm_b.stop()
            GPIO.cleanup(list(PINS))
        except Exception as e:
            self.get_logger().error(f'Cleanup error: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
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
