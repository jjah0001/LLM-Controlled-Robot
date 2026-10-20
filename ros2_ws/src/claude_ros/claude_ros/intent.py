"""The contract between the Claude agent and the motion controller.

Claude answers with a JSON object matching ``DECISION_SCHEMA`` (enforced by the
API's structured outputs). The motion part (linear, angular, duration_s, reason)
is published on ``/claude/intent`` as a ``std_msgs/String`` and the motion
controller executes it smoothly until it expires or a newer one arrives. ``say``
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
    'required': ['linear', 'angular', 'duration_s', 'reason', 'say', 'note', 'floor'],
    'additionalProperties': False,
}


def _finite(v, default=0.0):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return default
    return f if math.isfinite(f) else default


def normalise(data: dict, min_duration: float, max_duration: float) -> dict:
    """Clamp a decision into safe ranges. Never raises on odd values."""
    return {
        'linear': max(-1.0, min(1.0, _finite(data.get('linear')))),
        'angular': max(-1.0, min(1.0, _finite(data.get('angular')))),
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
    return {'linear': 0.0, 'angular': 0.0, 'duration_s': 0.5, 'reason': reason}


def encode(intent: dict) -> str:
    return json.dumps(intent, separators=(',', ':'))


def decode(text: str, min_duration: float = 0.2, max_duration: float = 10.0) -> dict:
    """Parse an intent message; raises ValueError on malformed JSON."""
    data = json.loads(text)
    if not isinstance(data, dict):
        raise ValueError('intent must be a JSON object')
    return normalise(data, min_duration, max_duration)
