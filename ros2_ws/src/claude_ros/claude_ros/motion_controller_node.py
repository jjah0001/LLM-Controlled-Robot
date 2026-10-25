#!/usr/bin/env python3
"""High-rate local controller: sensor fusion + smooth, safe execution of Claude's intents.

Subscribes:
  /scan              sensor_msgs/LaserScan  (Witmotion/M1CT D05 dTOF LiDAR, m1ct_d2 driver)
  ultrasonic/range   sensor_msgs/Range      (front HC-SR04)
  /claude/intent     std_msgs/String        (JSON from claude_agent_node)
  /cliff             std_msgs/String        (JSON from cliff_publisher, when use_cliff_sensors)
Publishes:
  /cmd_vel               geometry_msgs/Twist  (every tick, keeps the motor watchdog fed)
  /perception/summary    std_msgs/String      (JSON world summary consumed by the agent)
"""
import collections
import dataclasses
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String

from claude_ros import control, intent as intent_mod, perception


def self_mask_triples(flat) -> tuple:
    """Group the flat lidar_self_mask parameter into (from_deg, to_deg, max_m) triples."""
    v = [float(x) for x in flat]
    return tuple(tuple(v[i:i + 3]) for i in range(0, len(v) - len(v) % 3, 3))


class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')

        p = self.declare_parameter
        p('control_rate_hz', 20.0)
        p('summary_rate_hz', 5.0)
        p('scan_timeout_s', 0.6)
        p('ultrasonic_timeout_s', 0.5)
        p('max_intent_duration_s', 6.0)
        p('max_turn_duration_s', 12.0)  # a fixed turn that hasn't finished by then is dropped
        p('turn_window_s', 20.0)        # heading.turned_recently_deg covers this many seconds
        p('turn_rate_scale', 3.0)       # real / commanded turn rate to start from (then learned)
        p('turn_lead_deg', 25.0)        # how far a turn coasts after stopping (then learned)
        p('use_cliff_sensors', False)   # True: no travel unless /cliff is fresh and clear
        p('cliff_timeout_s', 0.3)
        # LiDAR mounting / robot footprint
        p('lidar_yaw_offset_deg', 0.0)
        p('lidar_reverse', False)
        p('lidar_self_filter_m', 0.05)
        p('lidar_self_mask', [0.0])      # flat list of (from_deg, to_deg, max_m) triples
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
            self_mask=self_mask_triples(g('lidar_self_mask').value),
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
        self.max_turn = float(g('max_turn_duration_s').value)
        self.turn_window = float(g('turn_window_s').value)
        self._turn_scale = float(g('turn_rate_scale').value)
        self._turn_lead = math.radians(float(g('turn_lead_deg').value))
        self.us_to_front = float(g('ultrasonic_to_front_m').value)
        self.use_cliff = bool(g('use_cliff_sensors').value)
        self.cliff_timeout = float(g('cliff_timeout_s').value)

        self._scan = None
        self._scan_t = -math.inf
        self._us = None
        self._us_t = -math.inf
        self._cliff = {}
        self._cliff_t = -math.inf
        self._intent = control.Intent()
        self._state = control.ControllerState()
        self._world = {}
        self._last_tick = time.monotonic()
        # Turning is measured by matching each LiDAR scan against the previous one: the motors
        # are open loop and turn far faster than commanded at carpet duty, so the commanded
        # rate alone can be out by a factor of three. Between scans, and with no LiDAR, the
        # commanded rotation times a learned scale fills in.
        self._heading = 0.0               # rad since start, unwrapped (+ = left)
        self._heading_log = collections.deque()   # (time, heading) for the recent-turn window
        self._profile = None              # previous scan as a range profile
        self._profile_t = -math.inf
        self._turned = 0.0                # rad measured since the current intent arrived
        self._cmd_since_scan = 0.0        # rad commanded since the last scan was matched
        self._heading_source = 'commanded'
        # The fixed turn in progress: what was asked, how early it stops to allow for coasting,
        # and when it stopped (to measure the coast once the robot has come to rest).
        self._turn_target = 0.0           # cleared once the turn has been logged
        self._turn_asked = 0.0            # the current intent's turn, for the summary
        self._turn_lead_used = 0.0
        self._turn_stopped_t = 0.0

        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Range, 'ultrasonic/range', self._range_cb, 10)
        self.create_subscription(String, '/claude/intent', self._intent_cb, 10)
        if self.use_cliff:
            self.create_subscription(String, '/cliff', self._cliff_cb, 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._summary_pub = self.create_publisher(String, '/perception/summary', 10)

        self.create_timer(1.0 / float(g('control_rate_hz').value), self._tick)
        self.create_timer(1.0 / float(g('summary_rate_hz').value), self._publish_summary)
        self.get_logger().info(
            f'Motion controller ready: max {self.limits.max_linear:.2f} m/s, '
            f'stop at {self.limits.stop_distance:.2f} m clearance, '
            f'cliff sensors {"on" if self.use_cliff else "OFF (edges are not detected)"}.')

    # ── Inputs ────────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        now = time.monotonic()
        self._scan = msg
        self._scan_t = now
        ang, rng = perception.scan_to_polar(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
            self.geom)
        profile = perception.range_profile(ang, rng)
        rot = None
        if self._profile is not None and now - self._profile_t <= self.scan_timeout:
            rot = perception.estimate_rotation(self._profile, profile)
        self._profile, self._profile_t = profile, now
        cmd = self._cmd_since_scan
        self._cmd_since_scan = 0.0
        if rot is None:
            self._heading_source = 'commanded'
            self._add_rotation(cmd * self._turn_scale)
            return
        self._heading_source = 'lidar'
        if abs(cmd) < math.radians(0.5) and abs(rot) < math.radians(1.0):
            rot = 0.0                   # standing still: don't let scan noise add up
        elif abs(cmd) >= math.radians(2.0) and rot * cmd > 0.0:
            ratio = rot / cmd
            if 0.2 < ratio < 6.0:
                self._turn_scale += 0.2 * (ratio - self._turn_scale)
        self._add_rotation(rot)

    def _add_rotation(self, rot: float):
        self._heading += rot
        self._turned += rot

    def _range_cb(self, msg: Range):
        r = msg.range
        if math.isinf(r) and r > 0:
            self._us = math.inf            # REP 117: +inf = nothing within range
        elif math.isfinite(r) and r >= msg.min_range:
            self._us = r if r <= msg.max_range else math.inf
        else:
            return
        self._us_t = time.monotonic()

    def _cliff_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
            cliff = {'front': bool(d['front']), 'rear': bool(d['rear']),
                     'sensors': dict(d.get('sensors', {}))}
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().warn(f'Ignoring malformed /cliff message ({e})',
                                   throttle_duration_sec=5.0)
            return
        self._cliff, self._cliff_t = cliff, time.monotonic()
        # Don't wait for the next control tick when driving toward a fresh edge.
        v = self._state.linear
        if (cliff['front'] and v > 0.0) or (cliff['rear'] and v < 0.0):
            self._tick()

    def _intent_cb(self, msg: String):
        try:
            d = intent_mod.decode(msg.data, max_duration=self.max_intent)
        except ValueError as e:
            self.get_logger().warn(f'Ignoring malformed intent ({e}): {msg.data!r}')
            return
        now = time.monotonic()
        self._finish_turn(now, final=True)
        turn = math.radians(d['turn_deg'])
        duration = d['duration_s']
        command = 0.0
        if turn:
            # Long enough to finish the turn whatever duration_s says, with room for ramping.
            rate = abs(d['angular']) * self.limits.max_angular
            duration = min(self.max_turn, max(duration, 1.5 * abs(turn) / rate + 1.0))
            # Stop early by the distance the robot coasts, so it ends up at the angle asked.
            lead = min(self._turn_lead, 0.7 * abs(turn))
            command = math.copysign(abs(turn) - lead, turn)
            self._turn_lead_used = lead
        self._turn_target = self._turn_asked = turn
        self._turn_stopped_t = 0.0
        self._intent = control.Intent(
            linear=d['linear'], angular=d['angular'], turn=command,
            expires_at=now + duration, reason=d['reason'])
        self._turned = 0.0
        self._state = dataclasses.replace(self._state, turned=0.0)

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
        cliff_front = cliff_rear = False
        cliff_known = True
        if self.use_cliff:
            cliff_known = now - self._cliff_t <= self.cliff_timeout
            if cliff_known:
                cliff_front, cliff_rear = self._cliff['front'], self._cliff['rear']
                world['cliff'] = {'edge_ahead': cliff_front, 'edge_behind': cliff_rear,
                                  'sensors': self._cliff['sensors']}
            else:
                world['cliff'] = 'sensors silent: travel stopped'
        else:
            world['cliff'] = 'no cliff sensors installed: watch the camera for edges'
        self._world = world
        return control.Perception(
            front=front, rear=lidar_rear, side_left=side_l, side_right=side_r,
            front_known=scan_ok or us_ok, rear_known=scan_ok,
            cliff_front=cliff_front, cliff_rear=cliff_rear, cliff_known=cliff_known)

    # ── Control loop ──────────────────────────────────────────────────────────

    def _tick(self):
        now = time.monotonic()
        dt = min(0.2, max(1e-3, now - self._last_tick))
        self._last_tick = now

        p = self._fuse(now)
        prev_flags = set(self._state.flags)
        if now - self._scan_t > self.scan_timeout and self._cmd_since_scan:
            self._heading_source = 'commanded'          # no LiDAR: dead reckoning only
            self._add_rotation(self._cmd_since_scan * self._turn_scale)
            self._cmd_since_scan = 0.0
        # Measured turn so far, plus a prediction for the time since the last scan.
        self._state = dataclasses.replace(
            self._state, turned=self._turned + self._cmd_since_scan * self._turn_scale)
        self._state = control.step(self._state, self._intent, p, self.limits, now, dt)
        self._cmd_since_scan += self._state.angular * dt
        if 'turn_complete' in self._state.flags and 'turn_complete' not in prev_flags:
            self._turn_stopped_t = now
        self._finish_turn(now)
        for f in set(self._state.flags) - prev_flags:
            if f.startswith(('emergency', 'blocked', 'no_', 'cliff')):
                front = self._world.get('front_clearance_m')
                self.get_logger().warn(f'Safety: {f} (front clearance={front})')

        msg = Twist()
        msg.linear.x = float(self._state.linear)
        msg.angular.z = float(self._state.angular)
        self._cmd_pub.publish(msg)

    def _finish_turn(self, now: float, final: bool = False):
        """Once a fixed turn has stopped and settled, log it and learn how far it coasted."""
        if not self._turn_target or not self._turn_stopped_t:
            return
        if not final and now - self._turn_stopped_t < 0.8:
            return
        asked, turned = self._turn_target, self._turned
        self._turn_target = 0.0
        if self._heading_source == 'lidar':
            along = math.copysign(abs(turned), turned * asked)   # + = in the asked direction
            coast = along - (abs(asked) - self._turn_lead_used)
            self._turn_lead = max(0.0, min(math.radians(60.0), self._turn_lead + 0.5 * (
                coast - self._turn_lead)))
        self.get_logger().info(
            f'Turn done: asked {math.degrees(asked):+.0f} deg, turned '
            f'{math.degrees(turned):+.0f} deg ({self._heading_source}; turn rate scale '
            f'{self._turn_scale:.2f}, stops {math.degrees(self._turn_lead):.0f} deg early)')

    def _publish_summary(self):
        now = time.monotonic()
        log = self._heading_log
        log.append((now, self._heading))
        while len(log) > 1 and now - log[1][0] >= self.turn_window:
            log.popleft()
        summary = dict(self._world)
        summary['velocity'] = {'linear_mps': round(self._state.linear, 3),
                               'angular_radps': round(self._state.angular, 3)}
        wrapped = math.degrees(math.atan2(math.sin(self._heading), math.cos(self._heading)))
        summary['heading'] = {
            'heading_deg': round(wrapped),
            'turned_recently_deg': round(math.degrees(self._heading - log[0][1])),
            'window_s': self.turn_window,
            'measured_by': self._heading_source,
        }
        summary['controller_flags'] = self._state.flags
        active = {
            'linear': self._intent.linear, 'angular': self._intent.angular,
            'remaining_s': round(max(0.0, self._intent.expires_at - now), 1),
            'turned_deg': round(math.degrees(self._state.turned)),
            'reason': self._intent.reason,
        }
        if self._intent.turn:
            active['turn_deg'] = round(math.degrees(self._turn_asked))
            active['turn_done'] = (
                'turn_complete' in self._state.flags or now >= self._intent.expires_at)
        summary['active_intent'] = active
        summary['limits'] = {'max_linear_mps': self.limits.max_linear,
                             'max_angular_radps': self.limits.max_angular,
                             'stop_distance_m': self.limits.stop_distance}
        self._summary_pub.publish(String(data=json.dumps(summary)))

    def stop(self):
        if rclpy.ok():                  # Ctrl+C may already have shut ROS down
            self._cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
