"""Differential-drive maths, kept free of ROS/GPIO imports so it can be unit tested."""


def twist_to_wheels(linear, angular, track_width, max_wheel_speed):
    """Return (left, right) wheel commands as fractions in [-1, 1]."""
    left = (linear - angular * track_width / 2.0) / max_wheel_speed
    right = (linear + angular * track_width / 2.0) / max_wheel_speed
    peak = max(1.0, abs(left), abs(right))
    return left / peak, right / peak


def fraction_to_duty(fraction, min_duty, max_duty, deadband):
    """Map |fraction| in (deadband, 1] linearly onto [min_duty, max_duty] percent."""
    mag = abs(fraction)
    if mag <= deadband:
        return 0.0
    return min_duty + (max_duty - min_duty) * min(1.0, mag)
