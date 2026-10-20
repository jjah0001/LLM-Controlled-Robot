#!/usr/bin/env python3
"""L298N differential-drive motor driver: /cmd_vel (m/s, rad/s) -> PWM on the Pi's GPIO.

Converts the body twist into left/right wheel speeds with the robot's track width,
scales both wheels together when one would saturate (so the curve is preserved),
and maps the result onto [min_duty, max_duty] so speed scales smoothly above the
duty where the motors stall. Velocity ramping is done upstream in motion_controller.

The PWM settings can be changed while driving, e.g. on carpet:
    ros2 param set /motor_driver min_duty 80.0
    ros2 param set /motor_driver kick_duty 100.0
    ros2 param set /motor_driver kick_s 0.25
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
import RPi.GPIO as GPIO

from motor_driver.kinematics import StartKick, fraction_to_duty, twist_to_wheels

# BCM pins. Motor A = left, motor B = right.
IN1, IN2, ENA = 17, 22, 18
IN3, IN4, ENB = 23, 24, 19
PINS = (IN1, IN2, IN3, IN4, ENA, ENB)

DEFAULTS = {
    'max_wheel_speed_mps': 0.30,
    'track_width_m': 0.16,
    'min_duty': 60.0,
    'max_duty': 100.0,
    'deadband': 0.02,
    'kick_duty': 100.0,
    'kick_s': 0.0,
    'pwm_hz': 1000.0,
    'cmd_timeout_s': 0.5,
    'invert_left': False,
    'invert_right': False,
}
DUTY_PARAMS = ('min_duty', 'max_duty', 'kick_duty')


class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)
        self._cfg = {n: self.get_parameter(n).value for n in DEFAULTS}
        error = self._validate(self._cfg)
        if error:
            raise ValueError(f'motor_driver parameters: {error}')

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for pin in PINS:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        self.pwm_a = GPIO.PWM(ENA, float(self._cfg['pwm_hz']))
        self.pwm_b = GPIO.PWM(ENB, float(self._cfg['pwm_hz']))
        self.pwm_a.start(0)
        self.pwm_b.start(0)
        self._kick_a, self._kick_b = StartKick(), StartKick()

        self._last_cmd = self.get_clock().now()
        self._wheels = (0.0, 0.0)
        self._stopped = True
        self.add_on_set_parameters_callback(self._on_params)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_timer(0.1, self._watchdog)
        self._log_settings('Motor driver ready')

    @staticmethod
    def _validate(cfg):
        for name in DUTY_PARAMS:
            if not 0.0 <= float(cfg[name]) <= 100.0:
                return f'{name} must be between 0 and 100 (got {cfg[name]})'
        if float(cfg['min_duty']) > float(cfg['max_duty']):
            return 'min_duty must not exceed max_duty'
        if not 0.0 <= float(cfg['deadband']) < 1.0:
            return 'deadband must be in [0, 1)'
        if float(cfg['kick_s']) < 0.0:
            return 'kick_s must be >= 0'
        for name in ('max_wheel_speed_mps', 'track_width_m', 'pwm_hz', 'cmd_timeout_s'):
            if float(cfg[name]) <= 0.0:
                return f'{name} must be > 0'
        return None

    def _on_params(self, params):
        cfg = dict(self._cfg)
        for param in params:
            if param.name in cfg:
                cfg[param.name] = param.value
        error = self._validate(cfg)
        if error:
            return SetParametersResult(successful=False, reason=error)
        if cfg['pwm_hz'] != self._cfg['pwm_hz']:
            self.pwm_a.ChangeFrequency(float(cfg['pwm_hz']))
            self.pwm_b.ChangeFrequency(float(cfg['pwm_hz']))
        self._cfg = cfg
        self._log_settings('Motor settings updated')
        return SetParametersResult(successful=True)

    def _log_settings(self, prefix):
        c = self._cfg
        kick = (f'kick {c["kick_duty"]:.0f}% for {c["kick_s"]:.2f}s' if c['kick_s'] > 0
                else 'no start kick')
        self.get_logger().info(
            f'{prefix}: {c["max_wheel_speed_mps"]:.2f} m/s at 100% duty, '
            f'track {c["track_width_m"]:.3f} m, duty {c["min_duty"]:.0f}-{c["max_duty"]:.0f}%, '
            f'{kick}.')

    def _cmd_cb(self, msg: Twist):
        self._last_cmd = self.get_clock().now()
        c = self._cfg
        left, right = twist_to_wheels(msg.linear.x, msg.angular.z,
                                      float(c['track_width_m']),
                                      float(c['max_wheel_speed_mps']))
        self._wheels = (left * (-1.0 if c['invert_left'] else 1.0),
                        right * (-1.0 if c['invert_right'] else 1.0))
        self._drive(*self._wheels)

    def _drive(self, left, right):
        # Forward = IN2 high on the left motor and IN3 high on the right (as wired).
        now = time.monotonic()
        self._set(self.pwm_a, IN2, IN1, left, self._kick_a, now)
        self._set(self.pwm_b, IN3, IN4, right, self._kick_b, now)
        self._stopped = left == 0.0 and right == 0.0

    def _set(self, pwm, pin_fwd, pin_rev, fraction, kick, now):
        c = self._cfg
        duty = fraction_to_duty(fraction, float(c['min_duty']), float(c['max_duty']),
                                float(c['deadband']))
        duty = kick.apply(duty, fraction, now, float(c['kick_duty']), float(c['kick_s']))
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
        if self._stopped:
            return
        if elapsed > float(self._cfg['cmd_timeout_s']):
            self.get_logger().warn('No /cmd_vel for %.1fs: stopping motors.' % elapsed)
            self._wheels = (0.0, 0.0)
        # Re-apply the last command so a start kick ends on time between /cmd_vel messages.
        self._drive(*self._wheels)

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
