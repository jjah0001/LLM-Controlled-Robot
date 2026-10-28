"""The contract between the Claude agent and the motion controller.

Claude answers with a JSON object matching ``DECISION_SCHEMA`` (enforced by the
API's structured outputs). The motion part (linear, angular, turn_deg, distance_m,
duration_s, reason) is published on ``/claude/intent`` as a ``std_msgs/String`` and the motion
controller executes it smoothly until it expires or a newer one arrives. A non-zero
``turn_deg`` asks for an in-place turn by that angle: the controller turns, then stops
by itself, so the turn doesn't depend on when the next decision arrives. A non-zero
``distance_m`` likewise asks for a straight drive by that distance, then a stop. ``say``
goes to ``/claude/say``, ``note`` into the robot's memory, and ``floor`` switches
the motor driver's power mode (``/motor_driver/surface``).
"""
from __future__ import annotations

import json
import math

DECISION_SCHEMA = {
    'type': 'object',
    'properties': {
        'linear': {
            'type': 'number',
            'description': 'Forward speed as a fraction of max speed, -1.0 (full reverse) '
                           'to 1.0 (full forward).',
        },
        'angular': {
            'type': 'number',
            'description': 'Turn rate as a fraction of max turn rate, -1.0 (full right) '
                           'to 1.0 (full left).',
        },
        'turn_deg': {
            'type': 'number',
            'description': 'Turn in place by this many degrees and then stop, -180 (right) '
                           'to 180 (left). 0 = no fixed turn; use linear/angular instead.',
        },
        'distance_m': {
            'type': 'number',
            'description': 'Drive straight by this many metres and then stop, -1 (backward) '
                           'to 2 (forward). 0 = no fixed distance; use linear/angular instead.',
        },
        'duration_s': {
            'type': 'number',
            'description': 'How long to keep this motion if no newer decision arrives.',
        },
        'reason': {
            'type': 'string',
            'description': 'One short sentence explaining the choice.',
        },
        'say': {
            'type': 'string',
            'description': 'Something to say out loud to people nearby, or empty.',
        },
        'note': {
            'type': 'string',
            'description': 'A new observation worth remembering long term, or empty.',
        },
        'floor': {
            'type': 'string',
            'enum': ['', 'hard_floor', 'carpet'],
            'description': 'Switch motor power for this floor type, or empty to keep it.',
        },
    },
    'required': ['linear', 'angular', 'turn_deg', 'distance_m', 'duration_s', 'reason', 'say',
                 'note', 'floor'],
    'additionalProperties': False,
}


def _finite(v, default=0.0):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return default
    return f if math.isfinite(f) else default


# A fixed turn uses |angular| as its rate, within these bounds (fraction of max turn rate).
TURN_RATE_DEFAULT = 0.5
TURN_RATE_MIN = 0.3
MAX_TURN_DEG = 180.0
# A fixed distance uses |linear| as its pace, within these bounds (fraction of max speed).
DRIVE_PACE_DEFAULT = 0.4
DRIVE_PACE_MIN = 0.2
MAX_DRIVE_M = 2.0
MAX_REVERSE_M = 1.0
MIN_DRIVE_M = 0.02


def normalise(data: dict, min_duration: float, max_duration: float) -> dict:
    """Clamp a decision into safe ranges. Never raises on odd values.

    A fixed turn (``turn_deg`` != 0) is always in place: linear becomes 0 and angular
    becomes the turn rate, signed in the turn's direction. A fixed distance (``distance_m``
    != 0, ignored during a fixed turn) is always straight: angular becomes 0 and linear
    becomes the pace, signed in the direction of travel.
    """
    linear = max(-1.0, min(1.0, _finite(data.get('linear'))))
    angular = max(-1.0, min(1.0, _finite(data.get('angular'))))
    turn = max(-MAX_TURN_DEG, min(MAX_TURN_DEG, _finite(data.get('turn_deg'))))
    distance = max(-MAX_REVERSE_M, min(MAX_DRIVE_M, _finite(data.get('distance_m'))))
    if abs(distance) < MIN_DRIVE_M:
        distance = 0.0
    if turn:
        rate = max(TURN_RATE_MIN, abs(angular)) if angular else TURN_RATE_DEFAULT
        linear, angular, distance = 0.0, math.copysign(rate, turn), 0.0
    elif distance:
        pace = max(DRIVE_PACE_MIN, abs(linear)) if linear else DRIVE_PACE_DEFAULT
        linear, angular = math.copysign(pace, distance), 0.0
    return {
        'linear': linear,
        'angular': angular,
        'turn_deg': turn,
        'distance_m': distance,
        'duration_s': max(min_duration, min(max_duration,
                                            _finite(data.get('duration_s'), min_duration))),
        'reason': str(data.get('reason', ''))[:200],
    }


def extras(data: dict) -> tuple:
    """The non-motion parts of a decision: (say, note), each trimmed, '' when absent."""
    def text(key, limit):
        return ' '.join(str(data.get(key) or '').split())[:limit]
    return text('say', 200), text('note', 300)


def stop_intent(reason: str) -> dict:
    return {'linear': 0.0, 'angular': 0.0, 'turn_deg': 0.0, 'distance_m': 0.0,
            'duration_s': 0.5, 'reason': reason}


def encode(intent: dict) -> str:
    return json.dumps(intent, separators=(',', ':'))


def decode(text: str, min_duration: float = 0.2, max_duration: float = 10.0) -> dict:
    """Parse an intent message; raises ValueError on malformed JSON."""
    data = json.loads(text)
    if not isinstance(data, dict):
        raise ValueError('intent must be a JSON object')
    return normalise(data, min_duration, max_duration)
