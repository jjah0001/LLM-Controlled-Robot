import math

from claude_ros import control as c

LIM = c.ControlLimits()
CLEAR = c.Perception(front=None, rear=None, front_known=True, rear_known=True)


def _run(intent, perception, seconds, state=None, dt=0.05, t0=0.0):
    state = state or c.ControllerState()
    t = t0
    trace = []
    while t < t0 + seconds:
        t += dt
        state = c.step(state, intent, perception, LIM, t, dt)
        trace.append(state)
    return state, trace


def test_ramps_smoothly_to_target():
    intent = c.Intent(linear=1.0, expires_at=100.0)
    state, trace = _run(intent, CLEAR, 2.0)
    assert abs(state.linear - LIM.max_linear) < 1e-9
    steps = [b.linear - a.linear for a, b in zip(trace, trace[1:])]
    assert max(steps) <= LIM.accel_linear * 0.05 + 1e-9   # never jumps


def test_expired_intent_decays_to_zero():
    intent = c.Intent(linear=1.0, expires_at=1.0)
    state, _ = _run(intent, CLEAR, 2.0)
    assert state.linear == 0.0
    assert 'intent_expired' in state.flags


def test_slows_then_stops_near_obstacle():
    intent = c.Intent(linear=1.0, expires_at=100.0)
    slow = c.Perception(front=0.45, front_known=True)
    state, _ = _run(intent, slow, 3.0)
    assert 0.0 < state.linear < LIM.max_linear
    blocked = c.Perception(front=0.15, front_known=True)
    state, _ = _run(intent, blocked, 3.0, state=state)
    assert state.linear == 0.0
    assert 'blocked_front' in state.flags


def test_hard_stop_bypasses_ramp():
    moving = c.ControllerState(linear=LIM.max_linear)
    intent = c.Intent(linear=1.0, expires_at=100.0)
    state = c.step(moving, intent, c.Perception(front=0.05, front_known=True), LIM, 1.0, 0.05)
    assert state.linear == 0.0
    assert 'emergency_stop_front' in state.flags


def test_can_turn_in_place_when_blocked():
    intent = c.Intent(linear=0.0, angular=1.0, expires_at=100.0)
    state, _ = _run(intent, c.Perception(front=0.1, front_known=True), 2.0)
    assert state.linear == 0.0
    assert abs(state.angular - LIM.max_turn_while_blocked) < 1e-9


def test_no_forward_motion_without_sensors():
    intent = c.Intent(linear=1.0, expires_at=100.0)
    state, _ = _run(intent, c.Perception(), 1.0)
    assert state.linear == 0.0
    assert 'no_forward_sensor' in state.flags


def test_steers_away_from_close_right_wall():
    intent = c.Intent(linear=1.0, expires_at=100.0)
    p = c.Perception(front=None, side_right=0.03, front_known=True)
    state, _ = _run(intent, p, 2.0)
    assert state.angular > 0.0


def test_cliff_ahead_stops_at_once_but_allows_turning_and_reversing():
    moving = c.ControllerState(linear=LIM.max_linear)
    edge = c.Perception(front_known=True, rear_known=True, cliff_front=True)
    state = c.step(moving, c.Intent(linear=1.0, expires_at=100.0), edge, LIM, 1.0, 0.05)
    assert state.linear == 0.0
    assert 'emergency_stop_cliff_front' in state.flags and 'cliff_front' in state.flags
    state, _ = _run(c.Intent(angular=1.0, expires_at=100.0), edge, 2.0)
    assert state.linear == 0.0 and abs(state.angular - LIM.max_turn_while_blocked) < 1e-9
    state, _ = _run(c.Intent(linear=-1.0, expires_at=100.0), edge, 2.0)
    assert state.linear < 0.0


def test_cliff_behind_blocks_reversing():
    edge = c.Perception(front_known=True, rear_known=True, cliff_rear=True)
    state, _ = _run(c.Intent(linear=-1.0, expires_at=100.0), edge, 1.0)
    assert state.linear == 0.0 and 'cliff_rear' in state.flags


def test_silent_cliff_sensors_block_travel():
    blind = c.Perception(front_known=True, rear_known=True, cliff_known=False)
    for linear in (1.0, -1.0):
        state, _ = _run(c.Intent(linear=linear, expires_at=100.0), blind, 1.0)
        assert state.linear == 0.0 and 'no_cliff_sensor' in state.flags


def test_fixed_turn_stops_at_the_angle():
    for degrees, rate in ((90.0, 0.5), (-45.0, 0.5), (180.0, 1.0), (10.0, 0.3)):
        turn = math.radians(degrees)
        intent = c.Intent(linear=0.0, angular=math.copysign(rate, turn), turn=turn,
                          expires_at=100.0)
        state, trace = _run(intent, CLEAR, 8.0)
        assert state.angular == 0.0 and state.linear == 0.0
        assert 'turn_complete' in state.flags
        assert abs(state.turned - turn) <= LIM.turn_tolerance + math.radians(1.0)
        assert all(abs(s.angular) <= rate * LIM.max_angular + 1e-9 for s in trace)


def test_fixed_turn_ignores_linear_and_stops_when_it_expires():
    intent = c.Intent(linear=1.0, angular=0.5, turn=math.radians(360.0), expires_at=2.0)
    state, trace = _run(intent, CLEAR, 4.0)
    assert all(s.linear == 0.0 for s in trace)
    assert state.angular == 0.0 and 'intent_expired' in state.flags


def test_fixed_turn_continues_from_the_angle_already_turned():
    turn = math.radians(90.0)
    intent = c.Intent(angular=0.5, turn=turn, expires_at=100.0)
    half = c.ControllerState(turned=turn - math.radians(1.0))
    state = c.step(half, intent, CLEAR, LIM, 1.0, 0.05)
    assert state.angular == 0.0 and 'turn_complete' in state.flags


def test_fixed_distance_stops_at_the_distance_without_overshoot():
    for metres, pace in ((1.0, 0.6), (0.3, 1.0), (-0.4, -0.4), (0.05, 0.4)):
        intent = c.Intent(linear=pace, distance=metres, expires_at=100.0)
        state, trace = _run(intent, CLEAR, 15.0)
        assert 'drive_complete' in state.flags and state.linear == 0.0
        assert abs(state.driven - metres) <= LIM.drive_tolerance + 0.005
        assert abs(max((s.driven for s in trace), key=abs)) <= abs(metres) + 0.005


def test_fixed_distance_holds_its_heading():
    intent = c.Intent(linear=0.5, distance=1.0, expires_at=100.0)
    drifted = c.ControllerState(turned=math.radians(10.0))
    state = c.step(drifted, intent, CLEAR, LIM, 0.05, 0.05)
    assert state.angular < 0.0                      # drifted left, so steer right


def test_fixed_distance_still_obeys_the_safety_layer():
    intent = c.Intent(linear=0.5, distance=1.0, expires_at=100.0)
    state, _ = _run(intent, c.Perception(front=0.15, front_known=True), 2.0)
    assert state.linear == 0.0 and 'blocked_front' in state.flags
    state, _ = _run(intent, c.Perception(cliff_front=True, front_known=True), 2.0)
    assert state.linear == 0.0
