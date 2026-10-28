"""How Ratangga's motors really behave on each floor, learned from LiDAR-measured motion.

Pure Python so it unit-tests without ROS. The motors are open loop (no encoders), so the same
command turns or drives the robot a different amount on carpet than on a hard floor. The
motion controller measures what really happened with the LiDAR and keeps, per floor type:

  * turn_scale / drive_scale: real rate / commanded rate, used to predict motion between
    LiDAR scans and when the LiDAR is down
  * turn_lead / drive_lead: how far the robot coasts after the command drops to zero, so a
    fixed turn or distance stops that much early and ends up on target

The numbers are saved to a JSON file so the robot starts calibrated after a restart.
"""
from __future__ import annotations

import dataclasses
import json
import math
import os
from dataclasses import dataclass

MAX_TURN_LEAD = math.radians(60.0)
MAX_DRIVE_LEAD = 0.15
SCALE_RANGE = (0.2, 6.0)


@dataclass
class Calibration:
    turn_scale: float = 3.0                   # real / commanded turn rate
    turn_lead: float = math.radians(25.0)     # rad a fixed turn coasts after stopping
    drive_scale: float = 1.0                  # real / commanded forward speed
    drive_lead: float = 0.03                  # m a fixed distance coasts after stopping
    turns_learned: int = 0
    drives_learned: int = 0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def learn_scale(scale: float, measured: float, commanded: float, gain: float = 0.2) -> float:
    """Move ``scale`` toward measured / commanded; ignores samples that disagree in sign."""
    if commanded == 0.0 or measured * commanded <= 0.0:
        return scale
    ratio = measured / commanded
    if not SCALE_RANGE[0] < ratio < SCALE_RANGE[1]:
        return scale
    return scale + gain * (ratio - scale)


def learn_lead(lead: float, command: float, achieved: float, max_lead: float,
               gain: float = 0.5) -> float:
    """Update the coast estimate from one move that was told to stop after ``command``.

    ``achieved`` is how far it really went (same units, signed). The coast is how far past
    the command it went; a negative coast (it stopped short) lowers the lead.
    """
    if command == 0.0:
        return lead
    coast = math.copysign(1.0, command) * achieved - abs(command)
    return clamp(lead + gain * (coast - lead), 0.0, max_lead)


def plan(remaining: float, lead: float, min_command: float = 0.0,
         max_fraction: float = 0.7) -> float:
    """The command for a move of ``remaining``: stop early by the expected coast.

    Never shortens a move by more than ``max_fraction`` of itself, so a wrong lead can't
    swallow a small move entirely, nor below ``min_command`` (the smallest command the
    controller acts on), so a small correction still moves.
    """
    early = min(lead, max_fraction * abs(remaining))
    size = max(abs(remaining) - early, min(abs(remaining), min_command))
    return math.copysign(size, remaining)


class CalibrationStore:
    """Per-floor calibrations, loaded from and saved to a JSON file ('' = memory only)."""

    def __init__(self, path: str, defaults: Calibration):
        self.path = os.path.expanduser(path) if path else ''
        self.defaults = defaults
        self.surfaces: dict[str, Calibration] = {}
        self.load_error = None
        if self.path and os.path.exists(self.path):
            try:
                with open(self.path) as f:
                    data = json.load(f)
                fields = {f.name for f in dataclasses.fields(Calibration)}
                for surface, values in data.items():
                    known = {k: v for k, v in values.items() if k in fields}
                    self.surfaces[surface] = dataclasses.replace(defaults, **known)
            except (OSError, ValueError, TypeError, AttributeError) as e:
                self.load_error = e
                self.surfaces = {}

    def get(self, surface: str) -> Calibration:
        if surface not in self.surfaces:
            self.surfaces[surface] = dataclasses.replace(self.defaults)
        return self.surfaces[surface]

    def save(self):
        if not self.path:
            return
        os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump({s: dataclasses.asdict(c) for s, c in self.surfaces.items()}, f, indent=1)
        os.replace(tmp, self.path)
