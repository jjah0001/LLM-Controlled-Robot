"""The robot's long-term memory: short notes about what it has seen, kept on disk.

Notes are appended to a JSON Lines file so they survive restarts, and the most
recent ones are handed back to Claude with every decision. Pure Python, no ROS.
"""
from __future__ import annotations

import datetime
import json
import os


class NoteStore:
    def __init__(self, path: str, max_notes: int = 500):
        self.path = os.path.expanduser(path)
        self.max_notes = max_notes
        self._notes = self._load()

    def _load(self):
        try:
            with open(self.path, encoding='utf-8') as f:
                notes = [json.loads(line) for line in f if line.strip()]
        except FileNotFoundError:
            return []
        except (OSError, json.JSONDecodeError):
            return []   # a corrupt file must not stop the robot; it is rewritten on next add
        return [n for n in notes if isinstance(n, dict) and n.get('note')][-self.max_notes:]

    def add(self, text: str, now: datetime.datetime | None = None) -> bool:
        """Store a note; returns False for empty text or a repeat of the last note."""
        text = ' '.join(str(text).split())[:300]
        if not text or (self._notes and self._notes[-1]['note'] == text):
            return False
        now = now or datetime.datetime.now()
        self._notes.append({'time': now.isoformat(timespec='seconds'), 'note': text})
        os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)
        if len(self._notes) > self.max_notes:
            self._notes = self._notes[-self.max_notes:]
            self._rewrite()
        else:
            with open(self.path, 'a', encoding='utf-8') as f:
                f.write(json.dumps(self._notes[-1]) + '\n')
        return True

    def _rewrite(self):
        tmp = self.path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            for n in self._notes:
                f.write(json.dumps(n) + '\n')
        os.replace(tmp, self.path)

    def recent(self, n: int):
        return list(self._notes[-n:]) if n > 0 else []

    def __len__(self):
        return len(self._notes)
