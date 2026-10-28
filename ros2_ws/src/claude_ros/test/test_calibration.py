import math

from claude_ros import calibration as cal


def test_plan_stops_early_by_the_coast_but_never_swallows_a_move():
    assert math.isclose(cal.plan(math.radians(90), math.radians(20)), math.radians(70))
    assert math.isclose(cal.plan(-1.0, 0.1), -0.9)
    assert math.isclose(cal.plan(0.1, 0.5), 0.1 * 0.3)            # lead capped at 70 %
    assert math.isclose(cal.plan(0.04, 0.5, min_command=0.015), 0.015)
    assert math.isclose(cal.plan(0.01, 0.5, min_command=0.015), 0.01)


def test_learn_lead_moves_toward_the_measured_coast():
    lead = cal.learn_lead(0.2, command=1.0, achieved=1.4, max_lead=1.0, gain=0.5)
    assert math.isclose(lead, 0.3)                    # coasted 0.4, was expecting 0.2
    lead = cal.learn_lead(0.2, command=-1.0, achieved=-1.0, max_lead=1.0, gain=1.0)
    assert lead == 0.0                                # stopped dead: no lead needed
    assert cal.learn_lead(0.2, 1.0, 5.0, max_lead=0.5, gain=1.0) == 0.5


def test_learn_scale_ignores_nonsense():
    assert math.isclose(cal.learn_scale(1.0, measured=3.0, commanded=1.0, gain=0.5), 2.0)
    assert cal.learn_scale(1.0, measured=-1.0, commanded=1.0) == 1.0     # wrong way
    assert cal.learn_scale(1.0, measured=50.0, commanded=1.0) == 1.0     # implausible


def test_store_keeps_floors_apart_and_survives_a_restart(tmp_path):
    path = str(tmp_path / 'sub' / 'calibration.json')
    store = cal.CalibrationStore(path, cal.Calibration(turn_scale=2.0))
    store.get('carpet').turn_scale = 3.5
    assert store.get('hard_floor').turn_scale == 2.0
    store.save()
    again = cal.CalibrationStore(path, cal.Calibration())
    assert again.get('carpet').turn_scale == 3.5
    assert again.get('hard_floor').turn_scale == 2.0


def test_store_starts_fresh_from_a_corrupt_file(tmp_path):
    path = tmp_path / 'calibration.json'
    path.write_text('{not json')
    store = cal.CalibrationStore(str(path), cal.Calibration())
    assert store.load_error is not None and store.get('carpet') == cal.Calibration()
