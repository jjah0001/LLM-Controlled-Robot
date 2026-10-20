#!/usr/bin/env python3
"""High-rate local controller: sensor fusion + smooth, safe execution of Claude's intents.

Subscribes:
  /scan              sensor_msgs/LaserScan  (Witmotion/M1CT D05 dTOF LiDAR, m1ct_d2 driver)
  ultrasonic/range   sensor_msgs/Range      (front HC-SR04)
  /claude/intent     std_msgs/String        (JSON from claude_agent_node)
Publishes:
  /cmd_vel               geometry_msgs/Twist  (every tick, keeps the motor watchdog fed)
  /perception/summary    std_msgs/String      (JSON world summary consumed by the agent)
"""
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String

from claude_ros import control, intent as intent_mod, perception


class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')

        p = self.declare_parameter
        p('control_rate_hz', 20.0)
        p('summary_rate_hz', 5.0)
        p('scan_timeout_s', 0.6)
        p('ultrasonic_timeout_s', 0.5)
        p('max_intent_duration_s', 6.0)
        # LiDAR mounting / robot footprint
        p('lidar_yaw_offset_deg', 0.0)
        p('lidar_reverse', False)
        p('lidar_self_filter_m', 0.05)
        p('lidar_to_front_m', 0.10)
        p('lidar_to_rear_m', 0.10)
        p('robot_half_width_m', 0.10)
        p('corridor_margin_m', 0.05)
        p('ultrasonic_to_front_m', 0.0)
        # Motion limits
        defaults = control.ControlLimits()
        for name in defaults.__dataclass_fields__:
            p(name, getattr(defaults, name))

        g = self.get_parameter
        self.geom = perception.ScanGeometry(
            yaw_offset_rad=math.radians(g('lidar_yaw_offset_deg').value),
            reverse=bool(g('lidar_reverse').value),
            self_filter_m=float(g('lidar_self_filter_m').value),
            to_front_m=float(g('lidar_to_front_m').value),
            to_rear_m=float(g('lidar_to_rear_m').value),
            half_width_m=float(g('robot_half_width_m').value),
            corridor_margin_m=float(g('corridor_margin_m').value),
        )
        self.limits = control.ControlLimits(
            **{n: float(g(n).value) for n in defaults.__dataclass_fields__})
        self.scan_timeout = float(g('scan_timeout_s').value)
        self.us_timeout = float(g('ultrasonic_timeout_s').value)
        self.max_intent = float(g('max_intent_duration_s').value)
        self.us_to_front = float(g('ultrasonic_to_front_m').value)

        self._scan = None
        self._scan_t = -math.inf
        self._us = None
        self._us_t = -math.inf
        self._intent = control.Intent()
        self._state = control.ControllerState()
        self._world = {}
        self._last_tick = time.monotonic()

        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Range, 'ultrasonic/range', self._range_cb, 10)
        self.create_subscription(String, '/claude/intent', self._intent_cb, 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._summary_pub = self.create_publisher(String, '/perception/summary', 10)

        self.create_timer(1.0 / float(g('control_rate_hz').value), self._tick)
        self.create_timer(1.0 / float(g('summary_rate_hz').value), self._publish_summary)
        self.get_logger().info(
            f'Motion controller ready: max {self.limits.max_linear:.2f} m/s, '
            f'stop at {self.limits.stop_distance:.2f} m clearance.')

    # ── Inputs ────────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        self._scan = msg
        self._scan_t = time.monotonic()

    def _range_cb(self, msg: Range):
        r = msg.range
        if math.isinf(r) and r > 0:
            self._us = math.inf            # REP 117: +inf = nothing within range
        elif math.isfinite(r) and r >= msg.min_range:
            self._us = r if r <= msg.max_range else math.inf
        else:
            return
        self._us_t = time.monotonic()

    def _intent_cb(self, msg: String):
        try:
            d = intent_mod.decode(msg.data, max_duration=self.max_intent)
        except ValueError as e:
            self.get_logger().warn(f'Ignoring malformed intent ({e}): {msg.data!r}')
            return
        self._intent = control.Intent(
            linear=d['linear'], angular=d['angular'],
            expires_at=time.monotonic() + d['duration_s'], reason=d['reason'])

    # ── Fusion ────────────────────────────────────────────────────────────────

    def _fuse(self, now: float) -> control.Perception:
        world = {'lidar_ok': False, 'ultrasonic_ok': False}
        lidar_front = lidar_rear = side_l = side_r = None

        scan_ok = self._scan is not None and now - self._scan_t <= self.scan_timeout
        if scan_ok:
            s = self._scan
            ang, rng = perception.scan_to_polar(
                s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max, self.geom)
            lidar_front = perception.corridor_clearance(ang, rng, self.geom, forward=True)
            lidar_rear = perception.corridor_clearance(ang, rng, self.geom, forward=False)
            side_l, side_r = perception.side_clearances(
                ang, rng, self.limits.slow_distance, self.geom)
            world['lidar_ok'] = True
            world['lidar_sectors_m'] = perception.sector_distances(ang, rng, s.range_max)

        us_ok = self._us is not None and now - self._us_t <= self.us_timeout
        us = self._us if us_ok else None
        world['ultrasonic_ok'] = us_ok
        world['ultrasonic_m'] = None if us is None else (
            'clear' if math.isinf(us) else round(us, 2))

        front = perception.fuse_front(lidar_front, us, self.us_to_front)
        world['front_clearance_m'] = None if front is None else round(front, 2)
        world['rear_clearance_m'] = None if lidar_rear is None else round(lidar_rear, 2)
        self._world = world
        return control.Perception(
            front=front, rear=lidar_rear, side_left=side_l, side_right=side_r,
            front_known=scan_ok or us_ok, rear_known=scan_ok)

    # ── Control loop ──────────────────────────────────────────────────────────

    def _tick(self):
        now = time.monotonic()
        dt = min(0.2, max(1e-3, now - self._last_tick))
        self._last_tick = now

        p = self._fuse(now)
        prev_flags = set(self._state.flags)
        self._state = control.step(self._state, self._intent, p, self.limits, now, dt)
        for f in set(self._state.flags) - prev_flags:
            if f.startswith(('emergency', 'blocked', 'no_')):
                front = self._world.get('front_clearance_m')
                self.get_logger().warn(f'Safety: {f} (front clearance={front})')

        msg = Twist()
        msg.linear.x = float(self._state.linear)
        msg.angular.z = float(self._state.angular)
        self._cmd_pub.publish(msg)

    def _publish_summary(self):
        now = time.monotonic()
        summary = dict(self._world)
        summary['velocity'] = {'linear_mps': round(self._state.linear, 3),
                               'angular_radps': round(self._state.angular, 3)}
        summary['controller_flags'] = self._state.flags
        summary['active_intent'] = {
            'linear': self._intent.linear, 'angular': self._intent.angular,
            'remaining_s': round(max(0.0, self._intent.expires_at - now), 1),
            'reason': self._intent.reason,
        }
        summary['limits'] = {'max_linear_mps': self.limits.max_linear,
                             'max_angular_radps': self.limits.max_angular,
                             'stop_distance_m': self.limits.stop_distance}
        self._summary_pub.publish(String(data=json.dumps(summary)))

    def stop(self):
        self._cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
