import datetime

from claude_ros.memory import NoteStore


def test_notes_persist_and_skip_repeats(tmp_path):
    path = tmp_path / 'mem' / 'notes.jsonl'
    store = NoteStore(str(path))
    assert store.add('Kitchen is left of the start.')
    assert not store.add('Kitchen is  left of the start.')   # same note again
    assert not store.add('   ')
    assert store.add('Cable across the hallway.')
    again = NoteStore(str(path))
    assert [n['note'] for n in again.recent(10)] == [
        'Kitchen is left of the start.', 'Cable across the hallway.']
    assert again.recent(0) == []


def test_oldest_notes_dropped(tmp_path):
    path = tmp_path / 'notes.jsonl'
    store = NoteStore(str(path), max_notes=3)
    t = datetime.datetime(2026, 1, 1)
    for i in range(5):
        store.add(f'note {i}', now=t)
    assert [n['note'] for n in NoteStore(str(path), max_notes=3).recent(10)] == [
        'note 2', 'note 3', 'note 4']


def test_corrupt_file_does_not_crash(tmp_path):
    path = tmp_path / 'notes.jsonl'
    path.write_text('{not json\n')
    store = NoteStore(str(path))
    assert len(store) == 0
    assert store.add('fresh start')
