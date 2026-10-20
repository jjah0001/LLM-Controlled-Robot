from motor_driver.kinematics import fraction_to_duty, twist_to_wheels


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
