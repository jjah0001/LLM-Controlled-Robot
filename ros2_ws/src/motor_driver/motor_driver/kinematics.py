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


class StartKick:
    """Briefly raise one wheel's duty when it starts from rest or reverses.

    Static friction (carpet, rugs) needs more torque to get a wheel moving than to keep
    it moving, so a short kick lets min_duty stay low for gentle cruising.
    """

    def __init__(self):
        self._sign = 0
        self._since = 0.0

    def apply(self, duty, fraction, now, kick_duty, kick_s):
        """Return the duty to use at time `now` (seconds) for a wheel command."""
        sign = (fraction > 0) - (fraction < 0) if duty > 0.0 else 0
        if sign == 0:
            self._sign = 0
            return 0.0
        if sign != self._sign:
            self._sign, self._since = sign, now
        if kick_s > 0.0 and now - self._since < kick_s:
            return max(duty, kick_duty)
        return duty
