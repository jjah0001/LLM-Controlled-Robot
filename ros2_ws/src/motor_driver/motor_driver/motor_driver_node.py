#!/usr/bin/env python3
"""L298N differential-drive motor driver: /cmd_vel (m/s, rad/s) -> PWM on the Pi's GPIO.

Converts the body twist into left/right wheel speeds with the robot's track width,
scales both wheels together when one would saturate (so the curve is preserved),
and maps the result onto [min_duty, max_duty] so speed scales smoothly above the
duty where the motors stall. Velocity ramping is done upstream in motion_controller.

Motor power (PWM duty) can be changed while running, e.g. for carpet:
  ros2 param set /motor_driver surface carpet        # use the carpet preset
  ros2 param set /motor_driver carpet_min_duty 85.0  # tune a preset
  ros2 param set /motor_driver min_duty 75.0         # override any preset (-1 = off)
  ros2 param set /motor_driver kick_s 0.25           # full-power burst when a wheel starts
  ros2 topic pub --once /motor_driver/surface std_msgs/String "{data: carpet}"
The active setting is published (latched) on /motor_driver/power as JSON.
"""
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
import RPi.GPIO as GPIO

from motor_driver.kinematics import (
    SURFACES, StartKick, duty_range, fraction_to_duty, twist_to_wheels)

# BCM pins. Motor A = left, motor B = right.
IN1, IN2, ENA = 17, 22, 18
IN3, IN4, ENB = 23, 24, 19
PINS = (IN1, IN2, IN3, IN4, ENA, ENB)


def check_kick(kick_duty, kick_s):
    kick_duty, kick_s = float(kick_duty), float(kick_s)
    if not 0.0 <= kick_duty <= 100.0:
        raise ValueError('kick_duty must be between 0 and 100 %')
    if not 0.0 <= kick_s <= 2.0:
        raise ValueError('kick_s must be between 0 and 2 s')
    return kick_duty, kick_s


class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        p = self.declare_parameter
        p('max_wheel_speed_mps', 0.30)
        p('track_width_m', 0.16)
        p('surface', 'hard_floor')          # hard_floor | carpet
        p('hard_floor_min_duty', 60.0)      # % duty where wheels just turn on a hard floor
        p('carpet_min_duty', 80.0)          # carpet needs more power to get moving
        p('min_duty', -1.0)                 # >= 0 overrides the surface preset
        p('max_duty', 100.0)
        p('deadband', 0.02)
        p('kick_duty', 100.0)               # % duty for a short burst when a wheel starts
        p('kick_s', 0.0)                    # burst length in s; 0 = off
        p('pwm_hz', 1000.0)
        p('cmd_timeout_s', 0.5)
        p('invert_left', False)
        p('invert_right', False)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.max_wheel = float(g('max_wheel_speed_mps'))
        self.track = float(g('track_width_m'))
        self.surface = g('surface')
        self.presets = {'hard_floor': float(g('hard_floor_min_duty')),
                        'carpet': float(g('carpet_min_duty'))}
        self.min_override = float(g('min_duty'))
        self.min_duty, self.max_duty = duty_range(
            self.surface, self.presets, self.min_override, g('max_duty'))
        self.deadband = float(g('deadband'))
        self.kick_duty, self.kick_s = check_kick(g('kick_duty'), g('kick_s'))
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
        self._kick_a, self._kick_b = StartKick(), StartKick()

        self._last_cmd = self.get_clock().now()
        self._wheels = (0.0, 0.0)
        self._stopped = True
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._power_pub = self.create_publisher(String, '/motor_driver/power', latched)
        self.add_on_set_parameters_callback(self._on_params)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(String, '/motor_driver/surface', self._surface_cb, 10)
        self.create_timer(0.1, self._watchdog)
        self.get_logger().info(
            f'Motor driver ready: {self.max_wheel:.2f} m/s at 100% duty, '
            f'track {self.track:.3f} m.')
        self._publish_power()

    # ── Motor power (PWM duty) ────────────────────────────────────────────────

    def _on_params(self, params):
        """Validate and apply live changes to the surface / duty parameters."""
        surface, presets = self.surface, dict(self.presets)
        override, max_duty = self.min_override, self.max_duty
        kick_duty, kick_s = self.kick_duty, self.kick_s
        for prm in params:
            if prm.name == 'surface':
                surface = str(prm.value)
            elif prm.name == 'hard_floor_min_duty':
                presets['hard_floor'] = float(prm.value)
            elif prm.name == 'carpet_min_duty':
                presets['carpet'] = float(prm.value)
            elif prm.name == 'min_duty':
                override = float(prm.value)
            elif prm.name == 'max_duty':
                max_duty = float(prm.value)
            elif prm.name == 'kick_duty':
                kick_duty = float(prm.value)
            elif prm.name == 'kick_s':
                kick_s = float(prm.value)
        try:
            min_d, max_d = duty_range(surface, presets, override, max_duty)
            kick_duty, kick_s = check_kick(kick_duty, kick_s)
        except (TypeError, ValueError) as e:
            return SetParametersResult(successful=False, reason=str(e))
        changed = ((surface, min_d, max_d, kick_duty, kick_s)
                   != (self.surface, self.min_duty, self.max_duty, self.kick_duty, self.kick_s))
        self.surface, self.presets, self.min_override = surface, presets, override
        self.min_duty, self.max_duty = min_d, max_d
        self.kick_duty, self.kick_s = kick_duty, kick_s
        if changed:
            self._publish_power()
        return SetParametersResult(successful=True)

    def _surface_cb(self, msg: String):
        surface = msg.data.strip()
        if surface == self.surface:
            return
        if surface not in SURFACES:
            self.get_logger().warn(f'Ignoring unknown surface {surface!r}; use {SURFACES}.')
            return
        result = self.set_parameters([Parameter('surface', Parameter.Type.STRING, surface)])[0]
        if not result.successful:
            self.get_logger().warn(f'Could not switch to {surface}: {result.reason}')

    def _publish_power(self):
        self.get_logger().info(
            f'Motor power: surface={self.surface}, duty {self.min_duty:.0f}-{self.max_duty:.0f}%'
            + (' (min_duty override)' if self.min_override >= 0 else '')
            + (f', start kick {self.kick_duty:.0f}% for {self.kick_s:.2f}s'
               if self.kick_s > 0 else ''))
        self._power_pub.publish(String(data=json.dumps({
            'surface': self.surface, 'min_duty': self.min_duty, 'max_duty': self.max_duty,
            'override': self.min_override >= 0, 'kick_s': self.kick_s})))

    def _cmd_cb(self, msg: Twist):
        self._last_cmd = self.get_clock().now()
        left, right = twist_to_wheels(msg.linear.x, msg.angular.z, self.track, self.max_wheel)
        self._wheels = (left * self.inv_l, right * self.inv_r)
        self._drive(*self._wheels)

    def _drive(self, left, right):
        # Forward = IN2 high on the left motor and IN3 high on the right (as wired).
        now = time.monotonic()
        self._set(self.pwm_a, IN2, IN1, left, self._kick_a, now)
        self._set(self.pwm_b, IN3, IN4, right, self._kick_b, now)
        self._stopped = left == 0.0 and right == 0.0

    def _set(self, pwm, pin_fwd, pin_rev, fraction, kick, now):
        duty = fraction_to_duty(fraction, self.min_duty, self.max_duty, self.deadband)
        duty = kick.apply(duty, fraction, now, self.kick_duty, self.kick_s)
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
        if elapsed > self.timeout:
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
