from ultrasonic_control.cliff import CliffDebounce, summarise


def test_starts_unsafe_and_needs_steady_floor():
    d = CliffDebounce(clear_count=3)
    assert d.drop
    assert d.update(False) and d.update(False)
    assert not d.update(False)


def test_drop_reported_immediately():
    d = CliffDebounce(clear_count=3)
    for _ in range(3):
        d.update(False)
    assert d.update(True)
    assert d.update(False)          # one floor reading is not enough to clear it


def test_summarise_by_end():
    s = summarise({'front_left': False, 'front_right': True, 'rear': False})
    assert s['front'] and not s['rear']
    assert summarise({'front_left': False})['rear'] is False
