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


SURFACES = ('hard_floor', 'carpet')


def duty_range(surface, presets, min_override, max_duty):
    """Resolve the (min_duty, max_duty) pair for a floor surface.

    ``presets`` maps surface name -> minimum duty. A non-negative ``min_override``
    replaces the preset. Raises ValueError for unknown surfaces or impossible ranges.
    """
    if surface not in presets:
        raise ValueError(f'unknown surface {surface!r}, expected one of {sorted(presets)}')
    min_duty = float(min_override) if min_override >= 0 else float(presets[surface])
    max_duty = float(max_duty)
    if not 0.0 <= min_duty <= 100.0 or not 0.0 < max_duty <= 100.0:
        raise ValueError('duty values must be between 0 and 100 %')
    if min_duty > max_duty:
        raise ValueError(f'min duty {min_duty:.0f}% is above max duty {max_duty:.0f}%')
    return min_duty, max_duty


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
