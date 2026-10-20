"""Local motion control: turns Claude's slow, coarse intents into smooth, safe velocity.

Pure Python so the behaviour can be unit tested without ROS or hardware.

Pipeline each control tick (default 20 Hz):
  1. target   = latest intent (or zero once it has expired)
  2. limit    = slow down in proportion to the free space ahead/behind, stop inside stop_distance,
                and never travel toward an edge (drop) seen by the cliff sensors
  3. steer    = add a small bias away from walls that are close beside the robot
  4. ramp     = move the commanded velocity toward the target under acceleration limits
  5. e-stop   = bypass the ramp and zero forward/backward speed inside hard_stop_distance
                or as soon as a cliff sensor sees an edge in the direction of travel
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class ControlLimits:
    max_linear: float = 0.25          # m/s
    max_angular: float = 1.2          # rad/s
    accel_linear: float = 0.35        # m/s^2 when speeding up
    decel_linear: float = 0.8         # m/s^2 when slowing down
    accel_angular: float = 2.5        # rad/s^2
    stop_distance: float = 0.20       # m of clearance at which travel in that direction stops
    slow_distance: float = 0.70       # m of clearance at which slowing down begins
    hard_stop_distance: float = 0.10  # m of clearance that triggers an immediate stop
    avoid_distance: float = 0.15      # m side gap below which steering bias kicks in
    avoid_gain: float = 0.6           # rad/s of bias at zero side gap, at full speed
    max_turn_while_blocked: float = 0.8  # rad/s cap for turning in place near obstacles


@dataclass
class Intent:
    linear: float = 0.0     # fraction of max_linear, -1..1
    angular: float = 0.0    # fraction of max_angular, -1..1 (+ = left / CCW)
    expires_at: float = 0.0  # monotonic seconds
    reason: str = ''


@dataclass
class Perception:
    front: float | None = None       # fused clearance ahead (m), None = nothing seen
    rear: float | None = None        # clearance behind (m), None = nothing seen
    side_left: float | None = None   # gap beside left flank (m)
    side_right: float | None = None
    front_known: bool = False        # True when at least one fresh forward sensor exists
    rear_known: bool = False
    cliff_front: bool = False        # a cliff sensor sees an edge (drop) ahead
    cliff_rear: bool = False         # a cliff sensor sees an edge behind
    cliff_known: bool = True         # False when cliff sensors are enabled but silent


@dataclass
class ControllerState:
    linear: float = 0.0
    angular: float = 0.0
    flags: list = field(default_factory=list)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def clearance_speed_cap(clearance, limits: ControlLimits) -> float:
    """Highest speed allowed for a given clearance, linear between stop and slow distance."""
    if clearance is None:
        return limits.max_linear
    if clearance <= limits.stop_distance:
        return 0.0
    span = max(1e-6, limits.slow_distance - limits.stop_distance)
    return limits.max_linear * clamp((clearance - limits.stop_distance) / span, 0.0, 1.0)


def avoidance_bias(side_left, side_right, speed_fraction, limits: ControlLimits) -> float:
    """Angular bias (rad/s) steering away from the closer side wall.

    Positive (turn left) when the right side is tight, and vice versa. Scales with
    forward speed so it never spins the robot while stopped.
    """
    def push(gap):
        if gap is None or gap >= limits.avoid_distance:
            return 0.0
        return 1.0 - max(0.0, gap) / limits.avoid_distance

    return limits.avoid_gain * speed_fraction * (push(side_right) - push(side_left))


def ramp(current: float, target: float, accel: float, decel: float, dt: float) -> float:
    """Rate-limit ``current`` toward ``target``; slowing down may use ``decel``."""
    slowing = abs(target) < abs(current) or (target * current < 0.0)
    step = (decel if slowing else accel) * dt
    if target > current:
        return min(target, current + step)
    return max(target, current - step)


def step(state: ControllerState, intent: Intent, p: Perception, limits: ControlLimits,
         now: float, dt: float) -> ControllerState:
    """Advance the controller one tick and return the new state."""
    flags = []

    if now < intent.expires_at:
        v_target = clamp(intent.linear, -1.0, 1.0) * limits.max_linear
        w_target = clamp(intent.angular, -1.0, 1.0) * limits.max_angular
    else:
        v_target = w_target = 0.0
        if intent.expires_at > 0.0:
            flags.append('intent_expired')

    # 2. Clearance-based speed limits.
    if v_target > 0.0:
        cap = clearance_speed_cap(p.front, limits) if p.front_known else 0.0
        if not p.front_known:
            flags.append('no_forward_sensor')
        elif cap < v_target:
            flags.append('blocked_front' if cap == 0.0 else 'slowing_front')
        v_target = min(v_target, cap)
    elif v_target < 0.0:
        # Reversing without a rear sensor is allowed only at a crawl.
        cap = clearance_speed_cap(p.rear, limits) if p.rear_known else 0.3 * limits.max_linear
        if p.rear_known and cap < -v_target:
            flags.append('blocked_rear' if cap == 0.0 else 'slowing_rear')
        v_target = max(v_target, -cap)

    # Edges: no travel toward a drop, and no travel at all if the cliff sensors are silent.
    # Turning in place stays allowed (the wheels don't move), and so does backing away.
    edge_ahead = p.cliff_front or not p.cliff_known
    edge_behind = p.cliff_rear or not p.cliff_known
    if v_target != 0.0 and not p.cliff_known:
        flags.append('no_cliff_sensor')
    if v_target > 0.0 and edge_ahead:
        v_target = 0.0
        if p.cliff_front:
            flags.append('cliff_front')
    elif v_target < 0.0 and edge_behind:
        v_target = 0.0
        if p.cliff_rear:
            flags.append('cliff_rear')

    near_edge = p.cliff_front or p.cliff_rear
    if v_target == 0.0 and abs(w_target) > limits.max_turn_while_blocked and (
            near_edge or (p.front is not None and p.front < limits.stop_distance)):
        w_target = math.copysign(limits.max_turn_while_blocked, w_target)

    # 3. Steering bias away from close side walls while moving forward.
    if v_target > 0.0:
        bias = avoidance_bias(p.side_left, p.side_right, v_target / limits.max_linear, limits)
        if bias:
            flags.append('steering_away_from_wall')
        w_target = clamp(w_target + bias, -limits.max_angular, limits.max_angular)

    # 4. Acceleration limits -> smooth motion.
    v = ramp(state.linear, v_target, limits.accel_linear, limits.decel_linear, dt)
    w = ramp(state.angular, w_target, limits.accel_angular, limits.accel_angular * 2.0, dt)

    # 5. Emergency stop overrides the ramp.
    if v > 0.0 and p.front is not None and p.front <= limits.hard_stop_distance:
        v = 0.0
        flags.append('emergency_stop_front')
    if v < 0.0 and p.rear is not None and p.rear <= limits.hard_stop_distance:
        v = 0.0
        flags.append('emergency_stop_rear')
    if v > 0.0 and edge_ahead:
        v = 0.0
        flags.append('emergency_stop_cliff_front')
    if v < 0.0 and edge_behind:
        v = 0.0
        flags.append('emergency_stop_cliff_rear')

    return ControllerState(linear=v, angular=w, flags=flags)
