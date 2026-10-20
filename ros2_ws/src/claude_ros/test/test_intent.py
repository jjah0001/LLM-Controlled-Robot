import json

import pytest

from claude_ros import intent


def test_normalise_clamps_and_survives_garbage():
    d = intent.normalise({'linear': 3, 'angular': 'x', 'duration_s': 99, 'reason': 1}, 1.0, 6.0)
    assert d == {'linear': 1.0, 'angular': 0.0, 'duration_s': 6.0, 'reason': '1'}
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
