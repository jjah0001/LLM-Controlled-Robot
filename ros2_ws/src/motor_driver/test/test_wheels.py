import pytest

from motor_driver.kinematics import StartKick, duty_range, fraction_to_duty, twist_to_wheels


def test_straight_and_turn():
    assert twist_to_wheels(0.15, 0.0, 0.16, 0.3) == (0.5, 0.5)
    left, right = twist_to_wheels(0.0, 1.0, 0.16, 0.3)
    assert left < 0 < right and abs(left + right) < 1e-9


def test_saturation_preserves_curvature():
    left, right = twist_to_wheels(0.3, 2.0, 0.16, 0.3)
    assert right == 1.0
    raw_l, raw_r = 0.3 - 2.0 * 0.08, 0.3 + 2.0 * 0.08
    assert abs(left / right - raw_l / raw_r) < 1e-9


def test_duty_mapping():
    assert fraction_to_duty(0.01, 40, 100, 0.02) == 0.0
    assert fraction_to_duty(-1.0, 40, 100, 0.02) == 100.0
    assert fraction_to_duty(0.5, 40, 100, 0.02) == 70.0


def test_duty_range_presets_and_override():
    presets = {'hard_floor': 60.0, 'carpet': 80.0}
    assert duty_range('hard_floor', presets, -1, 100) == (60.0, 100.0)
    assert duty_range('carpet', presets, -1, 100) == (80.0, 100.0)
    assert duty_range('carpet', presets, 70, 90) == (70.0, 90.0)


def test_duty_range_rejects_bad_values():
    presets = {'hard_floor': 60.0, 'carpet': 80.0}
    for args in (('grass', presets, -1, 100), ('carpet', presets, -1, 70),
                 ('hard_floor', presets, 120, 100), ('hard_floor', presets, -1, 0)):
        with pytest.raises(ValueError):
            duty_range(*args)


def test_start_kick_only_on_start_and_reverse():
    kick = StartKick()
    assert kick.apply(65.0, 0.1, 0.00, 90.0, 0.2) == 90.0   # starting from rest
    assert kick.apply(65.0, 0.1, 0.10, 90.0, 0.2) == 90.0   # still inside the kick
    assert kick.apply(65.0, 0.1, 0.25, 90.0, 0.2) == 65.0   # kick over
    assert kick.apply(65.0, -0.1, 0.30, 90.0, 0.2) == 90.0  # reversing kicks again
    assert kick.apply(0.0, 0.0, 0.35, 90.0, 0.2) == 0.0     # stop
    assert kick.apply(95.0, 0.9, 0.40, 90.0, 0.2) == 95.0   # never lowers the duty


def test_start_kick_disabled():
    assert StartKick().apply(65.0, 0.1, 0.0, 90.0, 0.0) == 65.0
