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


def test_edge_stops_forward_at_once_but_allows_reverse_and_turning():
    intent = c.Intent(linear=1.0, expires_at=100.0)
    state, _ = _run(intent, CLEAR, 2.0)
    assert state.linear > 0.0
    edge = c.Perception(front=None, rear=None, front_known=True, rear_known=True,
                        edge_front=True)
    state = c.step(state, intent, edge, LIM, 2.05, 0.05)
    assert state.linear == 0.0                         # no ramp-down over the edge
    assert 'emergency_stop_edge' in state.flags
    back = c.Intent(linear=-0.3, angular=0.5, expires_at=100.0)
    state, _ = _run(back, edge, 1.0, state=state, t0=2.05)
    assert state.linear < 0.0 and state.angular > 0.0
