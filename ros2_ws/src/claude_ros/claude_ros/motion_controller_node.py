#!/usr/bin/env python3
"""High-rate local controller: sensor fusion + smooth, safe execution of Claude's intents.

Subscribes:
  /scan              sensor_msgs/LaserScan  (Witmotion/M1CT D05 dTOF LiDAR, m1ct_d2 driver)
  ultrasonic/range   sensor_msgs/Range      (front HC-SR04)
  /claude/intent     std_msgs/String        (JSON from claude_agent_node)
  /cliff             std_msgs/String        (JSON from cliff_publisher, when use_cliff_sensors)
  /motor_driver/power std_msgs/String       (latched JSON: which floor mode the motors are in)
Publishes:
  /cmd_vel               geometry_msgs/Twist  (every tick, keeps the motor watchdog fed)
  /perception/summary    std_msgs/String      (JSON world summary consumed by the agent)
  /motor_driver/surface  std_msgs/String      (switches to carpet power if a goal stalls)

Fixed turns (turn_deg) and fixed distances (distance_m) are goals: the controller measures
them with the LiDAR, stops early by the coast it has learned for the current floor, and once
the robot has settled it checks the result and makes up to max_corrections small, slow moves
to close any remaining error. What it learns per floor is saved to calibration_file.
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
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String

from claude_ros import calibration as cal_mod, control, intent as intent_mod, perception

BLOCKING_FLAGS = ('blocked', 'emergency', 'cliff', 'no_forward', 'no_cliff')


def self_mask_triples(flat) -> tuple:
    """Group the flat lidar_self_mask parameter into (from_deg, to_deg, max_m) triples."""
    v = [float(x) for x in flat]
    return tuple(tuple(v[i:i + 3]) for i in range(0, len(v) - len(v) % 3, 3))


@dataclasses.dataclass
class Goal:
    """A fixed turn or distance in progress, across its first move and any corrections."""

    kind: str                   # 'turn' (rad) or 'drive' (m)
    asked: float                # signed total, from where the goal started
    start: float                # heading (turn) or odometer (drive) when the goal started
    reason: str = ''
    attempt: int = 0            # 0 = first move, then 1.. for corrections
    command: float = 0.0        # this move's command (asked remainder minus the coast lead)
    move_start: float = 0.0     # heading / odometer when this move started
    stopped_t: float = 0.0      # when the controller reported this move complete
    blocked: bool = False       # the safety layer held this move back
    measured: bool = True       # every update during this goal came from the LiDAR
    predicted: float = 0.0      # turn: rad the commands should have turned during this move
    key: object = None          # the LiDAR profile from where the goal started, to match against
    key_heading: float = 0.0
    key_odom: float = 0.0


class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')

        p = self.declare_parameter
        p('control_rate_hz', 20.0)
        p('summary_rate_hz', 5.0)
        p('scan_timeout_s', 0.6)
        p('ultrasonic_timeout_s', 0.5)
        p('max_intent_duration_s', 6.0)
        p('max_turn_duration_s', 12.0)  # a fixed turn/distance move unfinished by then is dropped
        p('turn_window_s', 20.0)        # heading.turned_recently_deg covers this many seconds
        # Starting calibration for a floor the robot hasn't learned yet (then learned per floor).
        p('turn_rate_scale', 3.0)       # real / commanded turn rate
        p('turn_lead_deg', 25.0)        # how far a turn coasts after stopping
        p('drive_rate_scale', 1.0)      # real / commanded forward speed
        p('drive_lead_m', 0.03)         # how far a drive coasts after stopping
        p('calibration_file', '~/.ros/robot_memory/motion_calibration.json')  # '' = don't save
        # Goal accuracy: after a move settles, correct any error bigger than these.
        p('settle_s', 0.8)
        p('turn_correct_deg', 3.0)
        p('drive_correct_m', 0.025)
        p('max_corrections', 3)
        p('max_real_turn_rate', 1.5)    # rad/s: cap on how fast the robot really spins (learned
        #                                 scale), so the LiDAR can follow it between scans
        p('stall_window_s', 1.5)        # moving for this long with no measured motion = stalled
        p('auto_carpet_on_stall', True)  # switch the motors to carpet power when a goal stalls
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
        self.settle_s = float(g('settle_s').value)
        self.turn_correct = math.radians(float(g('turn_correct_deg').value))
        self.drive_correct = float(g('drive_correct_m').value)
        self.max_corrections = int(g('max_corrections').value)
        self.stall_window = float(g('stall_window_s').value)
        self.max_real_turn = float(g('max_real_turn_rate').value)
        self.auto_carpet = bool(g('auto_carpet_on_stall').value)
        self.us_to_front = float(g('ultrasonic_to_front_m').value)
        self.use_cliff = bool(g('use_cliff_sensors').value)
        self.cliff_timeout = float(g('cliff_timeout_s').value)
        self.calibrations = cal_mod.CalibrationStore(
            str(g('calibration_file').value), cal_mod.Calibration(
                turn_scale=float(g('turn_rate_scale').value),
                turn_lead=math.radians(float(g('turn_lead_deg').value)),
                drive_scale=float(g('drive_rate_scale').value),
                drive_lead=float(g('drive_lead_m').value)))
        if self.calibrations.load_error:
            self.get_logger().warn(f'Could not read {self.calibrations.path} '
                                   f'({self.calibrations.load_error}); relearning.')

        self._scan = None
        self._scan_t = -math.inf
        self._us = None
        self._us_t = -math.inf
        self._cliff = {}
        self._cliff_t = -math.inf
        self._surface = 'hard_floor'      # until /motor_driver/power says otherwise
        self._intent = control.Intent()
        self._state = control.ControllerState()
        self._world = {}
        self._last_tick = time.monotonic()
        # Motion is measured with the LiDAR, because the motors are open loop and run far
        # faster than commanded at carpet duty. Turning: each scan is matched against the
        # previous one. Driving a fixed distance: each scan is matched against the scan from
        # where the drive started. Between scans, and with no LiDAR, the commanded motion
        # times the floor's learned scale fills in.
        self._heading = 0.0               # rad since start, unwrapped (+ = left)
        self._odom = 0.0                  # m driven along the body's x axis since start
        self._heading_log = collections.deque()   # (time, heading) for the recent-turn window
        self._profile = None              # previous scan as a range profile
        self._profile_t = -math.inf
        self._cmd_rot = 0.0               # rad commanded since the last scan was matched
        self._cmd_lin = 0.0               # m commanded since the last scan was matched
        self._heading_source = 'commanded'
        self._drive_source = 'commanded'
        self._intent_heading = 0.0        # heading / odometer when the current intent started
        self._intent_odom = 0.0
        self._goal = None                 # the fixed turn / distance in progress
        self._last_goal = None            # how the last one ended, for Claude
        self._stall_log = collections.deque()     # (time, progress) while a goal is moving
        self._stalled_t = -math.inf

        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Range, 'ultrasonic/range', self._range_cb, 10)
        self.create_subscription(String, '/claude/intent', self._intent_cb, 10)
        if self.use_cliff:
            self.create_subscription(String, '/cliff', self._cliff_cb, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/motor_driver/power', self._power_cb, latched)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._summary_pub = self.create_publisher(String, '/perception/summary', 10)
        self._surface_pub = self.create_publisher(String, '/motor_driver/surface', 10)

        self.create_timer(1.0 / float(g('control_rate_hz').value), self._tick)
        self.create_timer(1.0 / float(g('summary_rate_hz').value), self._publish_summary)
        self.get_logger().info(
            f'Motion controller ready: max {self.limits.max_linear:.2f} m/s, '
            f'stop at {self.limits.stop_distance:.2f} m clearance, '
            f'cliff sensors {"on" if self.use_cliff else "OFF (edges are not detected)"}, '
            f'calibrated floors: {sorted(self.calibrations.surfaces) or "none yet"}.')

    def _cal(self) -> cal_mod.Calibration:
        return self.calibrations.get(self._surface)

    def _limits(self) -> control.ControlLimits:
        """The motion limits, with the turn rate capped so the real spin stays trackable.

        On carpet the robot spins several times faster than commanded. Spinning much more
        than about 45 degrees between two scans makes the scan match ambiguous (it can lose
        count of whole turns), and smears each scan, so the commanded rate is scaled down by
        the learned turn_rate_scale to keep the real rate under max_real_turn_rate.
        """
        lim = self.limits
        cap = self.max_real_turn / max(1e-3, self._cal().turn_scale)
        return dataclasses.replace(
            lim, max_angular=max(lim.min_turn_rate, min(lim.max_angular, cap)))

    # ── Inputs ────────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        now = time.monotonic()
        self._scan = msg
        self._scan_t = now
        ang, rng = perception.scan_to_polar(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
            self.geom)
        profile = perception.range_profile(ang, rng)
        cal = self._cal()
        cmd_rot, cmd_lin = self._cmd_rot, self._cmd_lin
        self._cmd_rot = self._cmd_lin = 0.0
        # A scan can't tell a 10 degree turn from a 370 degree one, or a fast 60 degree step
        # from a -300 one, so search around the turn predicted from the command, not around 0.
        fresh = self._profile is not None and now - self._profile_t <= self.scan_timeout
        rot = perception.estimate_rotation(
            self._profile, profile, guess=cmd_rot * cal.turn_scale) if fresh else None
        self._profile, self._profile_t = profile, now
        if self._goal is not None:
            self._goal.predicted += abs(cmd_rot) * cal.turn_scale

        goal = self._goal
        if goal is not None and goal.kind == 'drive':
            self._measure_drive(goal, ang, rng, profile, cmd_rot, cmd_lin)
            return
        if goal is not None and goal.key is not None:
            # A fixed turn: match against the turn's first scan, around the chained estimate,
            # so per-scan errors don't add up over the turn.
            step = rot if rot is not None else cmd_rot * cal.turn_scale
            guess = self._heading + step - goal.key_heading
            total = perception.estimate_rotation(goal.key, profile, max_deg=15.0, guess=guess)
            if total is not None:
                self._heading_source = 'lidar'
                if abs(cmd_rot) >= math.radians(2.0):
                    measured = goal.key_heading + total - self._heading
                    cal.turn_scale = cal_mod.learn_scale(cal.turn_scale, measured, cmd_rot)
                self._heading = goal.key_heading + total
                self._odom += cmd_lin * cal.drive_scale
                return
        if rot is None:
            self._heading_source = 'commanded'
            self._heading += cmd_rot * cal.turn_scale
            if goal is not None and cmd_rot:
                goal.measured = False
        else:
            self._heading_source = 'lidar'
            if abs(cmd_rot) < math.radians(0.5) and abs(rot) < math.radians(1.0):
                rot = 0.0               # standing still: don't let scan noise add up
            elif abs(cmd_rot) >= math.radians(2.0):
                cal.turn_scale = cal_mod.learn_scale(cal.turn_scale, rot, cmd_rot)
            self._heading += rot
        self._odom += cmd_lin * cal.drive_scale

    def _measure_drive(self, goal: Goal, ang, rng, profile, cmd_rot: float, cmd_lin: float):
        """Distance and heading on a fixed drive, both matched against the drive's first scan."""
        cal = self._cal()
        if goal.key is None:
            goal.key, goal.key_heading, goal.key_odom = profile, self._heading, self._odom
        driven = self._odom - goal.key_odom
        turned = self._heading - goal.key_heading + cmd_rot * cal.turn_scale
        est, rot = perception.estimate_drive(
            goal.key, ang, rng, rotation=turned, guess=driven + cmd_lin * cal.drive_scale)
        self._heading = goal.key_heading + rot
        if est is None:
            self._drive_source = self._heading_source = 'commanded'
            goal.measured = False
            self._odom += cmd_lin * cal.drive_scale
            return
        self._drive_source = self._heading_source = 'lidar'
        if abs(cmd_lin) >= 0.01:
            cal.drive_scale = cal_mod.learn_scale(cal.drive_scale, est - driven, cmd_lin,
                                                  gain=0.1)
        self._odom = goal.key_odom + est

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

    def _power_cb(self, msg: String):
        try:
            surface = str(json.loads(msg.data)['surface'])
        except (ValueError, KeyError, TypeError):
            return
        if surface != self._surface:
            self._surface = surface
            known = 'learned' if surface in self.calibrations.surfaces else 'default'
            self.get_logger().info(f'Floor is now {surface}: using its {known} calibration.')

    def _intent_cb(self, msg: String):
        try:
            d = intent_mod.decode(msg.data, max_duration=self.max_intent)
        except ValueError as e:
            self.get_logger().warn(f'Ignoring malformed intent ({e}): {msg.data!r}')
            return
        now = time.monotonic()
        if self._goal is not None:
            self._end_goal('replaced')
        turn = math.radians(d['turn_deg'])
        fresh = self._profile is not None and now - self._profile_t <= self.scan_timeout
        if turn:
            self._goal = Goal('turn', turn, self._heading, reason=d['reason'])
            if fresh:
                self._goal.key = self._profile
                self._goal.key_heading, self._goal.key_odom = self._heading, self._odom
            self._start_move(now, turn, abs(d['angular']), d['duration_s'])
        elif d['distance_m']:
            self._goal = Goal('drive', d['distance_m'], self._odom, reason=d['reason'])
            if fresh:
                self._goal.key = self._profile
                self._goal.key_heading, self._goal.key_odom = self._heading, self._odom
            self._start_move(now, d['distance_m'], abs(d['linear']), d['duration_s'])
        else:
            self._begin_intent(control.Intent(
                linear=d['linear'], angular=d['angular'],
                expires_at=now + d['duration_s'], reason=d['reason']))

    # ── Goals: fixed turns and distances ──────────────────────────────────────

    def _progress(self, goal: Goal) -> float:
        return self._heading if goal.kind == 'turn' else self._odom

    def _begin_intent(self, intent: control.Intent):
        self._intent = intent
        self._intent_heading, self._intent_odom = self._heading, self._odom
        self._state = dataclasses.replace(self._state, turned=0.0, driven=0.0)
        self._stall_log.clear()

    def _start_move(self, now: float, remaining: float, pace: float, duration: float):
        """Start the goal's next move: ``remaining`` rad or m at ``pace`` (fraction of max)."""
        goal, cal = self._goal, self._cal()
        if goal.kind == 'turn':
            command = cal_mod.plan(remaining, cal.turn_lead, 1.5 * self.limits.turn_tolerance)
            # Long enough to finish at the real (learned) rate whatever duration_s says, with
            # room for ramping.
            rate = pace * self._limits().max_angular * max(0.3, min(1.0, cal.turn_scale))
            duration = min(self.max_turn, max(duration, 2.0 * abs(command) / rate + 1.0))
            intent = control.Intent(angular=math.copysign(pace, command), turn=command,
                                    expires_at=now + duration, reason=goal.reason)
        else:
            command = cal_mod.plan(remaining, cal.drive_lead, 1.5 * self.limits.drive_tolerance)
            speed = pace * self.limits.max_linear * max(0.3, min(1.0, cal.drive_scale))
            duration = min(self.max_turn, max(duration, 2.0 * abs(command) / speed + 1.5))
            intent = control.Intent(linear=math.copysign(pace, command), distance=command,
                                    expires_at=now + duration, reason=goal.reason)
        goal.command, goal.stopped_t, goal.blocked, goal.predicted = command, 0.0, False, 0.0
        goal.move_start = self._progress(goal)
        self._begin_intent(intent)

    def _settle_goal(self, now: float):
        """The move has stopped and settled: learn from it, then correct or finish."""
        goal, cal = self._goal, self._cal()
        moved = self._progress(goal) - goal.move_start
        if goal.measured and goal.attempt == 0:
            # Trust the first measurement on a new floor more than the starting guess.
            if goal.kind == 'turn':
                cal.turn_lead = cal_mod.learn_lead(cal.turn_lead, goal.command, moved,
                                                   cal_mod.MAX_TURN_LEAD,
                                                   0.8 if cal.turns_learned == 0 else 0.5)
                cal.turns_learned += 1
            else:
                cal.drive_lead = cal_mod.learn_lead(cal.drive_lead, goal.command, moved,
                                                    cal_mod.MAX_DRIVE_LEAD,
                                                    0.8 if cal.drives_learned == 0 else 0.5)
                cal.drives_learned += 1
            self._save_calibration()
        error = goal.asked - (self._progress(goal) - goal.start)
        tolerance = self.turn_correct if goal.kind == 'turn' else self.drive_correct
        if goal.measured and abs(error) > tolerance and goal.attempt < self.max_corrections:
            goal.attempt += 1
            pace = intent_mod.TURN_RATE_MIN if goal.kind == 'turn' else intent_mod.DRIVE_PACE_MIN
            self._start_move(now, error, pace, 0.0)
            return
        self._end_goal('done')

    def _end_goal(self, result: str):
        goal, self._goal = self._goal, None
        achieved = self._progress(goal) - goal.start
        error = goal.asked - achieved
        source = 'lidar' if goal.measured else 'commanded (rough)'
        if goal.kind == 'turn':
            self._last_goal = {'kind': 'turn', 'asked_deg': round(math.degrees(goal.asked)),
                               'turned_deg': round(math.degrees(achieved)),
                               'error_deg': round(math.degrees(error))}
            text = (f'asked {math.degrees(goal.asked):+.0f} deg, turned '
                    f'{math.degrees(achieved):+.0f} deg')
        else:
            self._last_goal = {'kind': 'drive', 'asked_m': round(goal.asked, 2),
                               'driven_m': round(achieved, 2), 'error_m': round(error, 2)}
            text = f'asked {goal.asked:+.2f} m, drove {achieved:+.2f} m'
        self._last_goal.update(result=result, corrections=goal.attempt, measured_by=source)
        cal = self._cal()
        self.get_logger().info(
            f'{goal.kind.capitalize()} {result}: {text} after {goal.attempt} correction(s) '
            f'({source}; {self._surface}: turn scale {cal.turn_scale:.2f}, turn coast '
            f'{math.degrees(cal.turn_lead):.0f} deg, drive scale {cal.drive_scale:.2f}, '
            f'drive coast {100 * cal.drive_lead:.0f} cm)')

    def _save_calibration(self):
        try:
            self.calibrations.save()
        except OSError as e:
            self.get_logger().warn(f'Could not save calibration: {e}', throttle_duration_sec=60.0)

    def _update_goal(self, now: float):
        goal = self._goal
        if goal is None:
            return
        flags = self._state.flags
        if goal.stopped_t == 0.0:
            if any(f.startswith(BLOCKING_FLAGS) for f in flags):
                goal.blocked = True
            if ('turn_complete' if goal.kind == 'turn' else 'drive_complete') in flags:
                goal.stopped_t = now
                self._stall_log.clear()
            elif now >= self._intent.expires_at:
                if goal.blocked:
                    self._end_goal('blocked')
                    return
                goal.stopped_t = now        # ran out of time: settle, then check and correct
                self._stall_log.clear()
            elif goal.kind == 'turn' and goal.predicted > 3.0 * abs(goal.command) + math.pi / 2:
                # The commands should have turned the robot far more than the LiDAR says: it
                # has probably lost count of whole turns. Stop rather than keep spinning.
                self.get_logger().warn(
                    f'Turn stopped: commanded about {math.degrees(goal.predicted):.0f} deg of '
                    f'spin for a {math.degrees(goal.command):+.0f} deg move, but the LiDAR '
                    f'measured {math.degrees(self._progress(goal) - goal.move_start):+.0f}.')
                goal.measured = False
                self._begin_intent(control.Intent(reason='turn lost count'))
                self._end_goal('lost_count')
                return
            else:
                self._check_stall(now, goal)
                return
        # Settled once the LiDAR has seen no motion for a moment (it can coast for a while on
        # some floors), or after a few seconds whatever happens.
        log = self._stall_log
        log.append((now, self._progress(goal)))
        while now - log[0][0] > 0.4:
            log.popleft()
        still = abs(log[-1][1] - log[0][1]) < (0.003 if goal.kind == 'drive'
                                               else math.radians(0.5))
        waited = now - goal.stopped_t
        if (waited >= self.settle_s and still) or waited >= self.settle_s + 2.5:
            self._stall_log.clear()
            self._settle_goal(now)

    def _check_stall(self, now: float, goal: Goal):
        """Wheels commanded to move but the LiDAR sees no motion: likely too little power."""
        moving = (abs(self._state.linear) >= 0.04 if goal.kind == 'drive'
                  else abs(self._state.angular) >= 0.3)
        source = self._drive_source if goal.kind == 'drive' else self._heading_source
        if not moving or goal.blocked or source != 'lidar':
            self._stall_log.clear()
            return
        log = self._stall_log
        log.append((now, self._progress(goal)))
        if now - log[0][0] < self.stall_window:
            return
        moved = abs(log[-1][1] - log[0][1])
        while now - log[0][0] > self.stall_window:
            log.popleft()
        if moved >= (0.01 if goal.kind == 'drive' else math.radians(3.0)):
            return
        log.clear()
        self._stalled_t = now
        if self.auto_carpet and self._surface != 'carpet':
            self.get_logger().warn('Wheels stalled during a fixed move: switching to carpet '
                                   'power.')
            self._surface_pub.publish(String(data='carpet'))
        else:
            self.get_logger().warn('Wheels stalled during a fixed move.',
                                   throttle_duration_sec=5.0)

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
        cal = self._cal()
        if now - self._scan_t > self.scan_timeout:      # no LiDAR: dead reckoning only
            if self._cmd_rot or self._cmd_lin:
                self._heading_source = self._drive_source = 'commanded'
                if self._goal is not None:
                    self._goal.measured = False
            self._heading += self._cmd_rot * cal.turn_scale
            self._odom += self._cmd_lin * cal.drive_scale
            self._cmd_rot = self._cmd_lin = 0.0
        # Measured motion so far, plus a prediction for the time since the last scan.
        self._state = dataclasses.replace(
            self._state,
            turned=self._heading - self._intent_heading + self._cmd_rot * cal.turn_scale,
            driven=self._odom - self._intent_odom + self._cmd_lin * cal.drive_scale)
        self._state = control.step(self._state, self._intent, p, self._limits(), now, dt)
        self._cmd_rot += self._state.angular * dt
        self._cmd_lin += self._state.linear * dt
        self._update_goal(now)
        for f in set(self._state.flags) - prev_flags:
            if f.startswith(('emergency', 'blocked', 'no_', 'cliff')):
                front = self._world.get('front_clearance_m')
                self.get_logger().warn(f'Safety: {f} (front clearance={front})')

        msg = Twist()
        msg.linear.x = float(self._state.linear)
        msg.angular.z = float(self._state.angular)
        self._cmd_pub.publish(msg)

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
        flags = list(self._state.flags)
        if now - self._stalled_t < 3.0:
            flags.append('stalled')
        summary['controller_flags'] = flags
        summary['active_intent'] = {
            'linear': self._intent.linear, 'angular': self._intent.angular,
            'remaining_s': round(max(0.0, self._intent.expires_at - now), 1),
            'turned_deg': round(math.degrees(self._state.turned)),
            'driven_m': round(self._state.driven, 2),
            'reason': self._intent.reason,
        }
        goal = self._goal
        if goal is not None:
            so_far = self._progress(goal) - goal.start
            if goal.kind == 'turn':
                active = {'kind': 'turn', 'asked_deg': round(math.degrees(goal.asked)),
                          'so_far_deg': round(math.degrees(so_far))}
            else:
                active = {'kind': 'drive', 'asked_m': round(goal.asked, 2),
                          'so_far_m': round(so_far, 2)}
            active['state'] = ('settling' if goal.stopped_t else
                               'correcting' if goal.attempt else 'moving')
            summary['goal'] = active
        else:
            summary['goal'] = None
        summary['last_goal'] = self._last_goal
        cal = self._cal()
        summary['calibration'] = {
            'floor': self._surface,
            'learned_from': f'{cal.turns_learned} turns, {cal.drives_learned} drives',
            'turn_rate_scale': round(cal.turn_scale, 2),
            'drive_speed_scale': round(cal.drive_scale, 2),
        }
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
