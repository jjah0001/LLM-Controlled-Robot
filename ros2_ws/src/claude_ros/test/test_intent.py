import json

import pytest

from claude_ros import intent


def test_normalise_clamps_and_survives_garbage():
    d = intent.normalise({'linear': 3, 'angular': 'x', 'duration_s': 99, 'reason': 1}, 1.0, 6.0)
    assert d == {'linear': 1.0, 'angular': 0.0, 'turn_deg': 0.0, 'duration_s': 6.0,
                 'reason': '1'}
    d = intent.normalise({'linear': float('nan'), 'angular': -2}, 1.0, 6.0)
    assert d['linear'] == 0.0 and d['angular'] == -1.0 and d['duration_s'] == 1.0


def test_round_trip():
    msg = intent.encode({'linear': 0.5, 'angular': -0.2, 'duration_s': 3.0, 'reason': 'ok'})
    assert intent.decode(msg)['angular'] == -0.2


def test_decode_rejects_non_objects():
    with pytest.raises(ValueError):
        intent.decode('[1, 2]')
    with pytest.raises(json.JSONDecodeError):
        intent.decode('not json')


def test_schema_is_strict():
    s = intent.DECISION_SCHEMA
    assert s['additionalProperties'] is False
    assert set(s['required']) == set(s['properties'])


def test_extras_trim_and_default():
    assert intent.extras({'say': '  hello\n there ', 'note': None}) == ('hello there', '')
    assert intent.extras({}) == ('', '')


def test_fixed_turn_is_in_place_with_a_signed_rate():
    d = intent.normalise({'linear': 0.8, 'angular': 0.0, 'turn_deg': -90}, 1.0, 6.0)
    assert d['linear'] == 0.0 and d['turn_deg'] == -90.0
    assert d['angular'] == -intent.TURN_RATE_DEFAULT
    d = intent.normalise({'angular': -0.1, 'turn_deg': 45}, 1.0, 6.0)
    assert d['angular'] == intent.TURN_RATE_MIN          # too slow to turn on carpet
    d = intent.normalise({'angular': 0.9, 'turn_deg': 720}, 1.0, 6.0)
    assert d['turn_deg'] == 180.0 and d['angular'] == 0.9


def test_no_fixed_turn_leaves_motion_alone():
    d = intent.normalise({'linear': 0.4, 'angular': 0.3, 'turn_deg': 0}, 1.0, 6.0)
    assert (d['linear'], d['angular'], d['turn_deg']) == (0.4, 0.3, 0.0)
    assert intent.decode('{"linear": 0.4}')['turn_deg'] == 0.0    # older senders still work
