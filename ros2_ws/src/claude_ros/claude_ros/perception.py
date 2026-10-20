"""Sensor fusion helpers (pure Python + numpy, no ROS imports so they unit-test anywhere).

Conventions (REP 103, robot body frame):
  * x forward, y left, angles in radians, counter-clockwise positive, 0 = straight ahead.
  * "Clearance" is the free distance between the robot's bumper and the nearest obstacle
    in the direction of travel, not the raw sensor range.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# Eight 45 degree sectors around the robot, centre angle in degrees (CCW positive).
SECTORS = (
    ('front', 0.0),
    ('front_left', 45.0),
    ('left', 90.0),
    ('back_left', 135.0),
    ('back', 180.0),
    ('back_right', -135.0),
    ('right', -90.0),
    ('front_right', -45.0),
)
SECTOR_HALF_WIDTH_DEG = 22.5


@dataclass
class ScanGeometry:
    """How the LiDAR is mounted on the robot."""

    yaw_offset_rad: float = 0.0     # added to every beam angle so 0 rad points forward
    reverse: bool = False           # True if the driver orders beams clockwise
    self_filter_m: float = 0.05     # ignore returns closer than this (robot's own body)
    to_front_m: float = 0.10        # LiDAR centre -> front bumper along +x
    to_rear_m: float = 0.10         # LiDAR centre -> rear bumper along -x
    half_width_m: float = 0.10      # half the robot's width
    corridor_margin_m: float = 0.05  # extra side margin for the drive corridor


def wrap_angle(a):
    """Wrap angle(s) to (-pi, pi]."""
    return np.arctan2(np.sin(a), np.cos(a))


def scan_to_polar(ranges, angle_min, angle_increment, range_min, range_max,
                  geometry: ScanGeometry):
    """Return (angles, ranges) of valid returns, angles in the robot frame.

    The D05/M1CT driver reports "no return" as 0.0, other drivers use inf/nan;
    both are dropped here instead of being treated as obstacles at 0 m.
    """
    r = np.asarray(ranges, dtype=np.float64)
    if r.size == 0:
        return np.empty(0), np.empty(0)
    ang = angle_min + np.arange(r.size, dtype=np.float64) * angle_increment
    if geometry.reverse:
        ang = -ang
    ang = wrap_angle(ang + geometry.yaw_offset_rad)
    lo = max(float(range_min), geometry.self_filter_m)
    hi = float(range_max) if range_max and range_max > 0 else math.inf
    valid = np.isfinite(r) & (r > lo) & (r <= hi)
    return ang[valid], r[valid]


def _robust_min(values: np.ndarray):
    """Second-smallest value, so one speckle can't stop the robot. None if empty."""
    if values.size == 0:
        return None
    if values.size == 1:
        return float(values[0])
    return float(np.partition(values, 1)[1])


def sector_distances(angles: np.ndarray, ranges: np.ndarray, range_max: float):
    """Nearest obstacle per 45 degree sector, in metres from the LiDAR.

    A sector with no returns is reported as ``range_max`` (nothing seen).
    """
    out = {}
    half = math.radians(SECTOR_HALF_WIDTH_DEG)
    for name, centre_deg in SECTORS:
        diff = np.abs(wrap_angle(angles - math.radians(centre_deg)))
        d = _robust_min(ranges[diff <= half])
        out[name] = round(d if d is not None else float(range_max), 2)
    return out


def corridor_clearance(angles: np.ndarray, ranges: np.ndarray, geometry: ScanGeometry,
                       forward: bool = True):
    """Free distance from the bumper to the nearest point inside the drive corridor.

    The corridor is the rectangle the robot would sweep driving straight forward
    (or backward). Returns None when nothing is in the corridor.
    """
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    half = geometry.half_width_m + geometry.corridor_margin_m
    if forward:
        mask = (x > 0.0) & (np.abs(y) <= half)
        d = _robust_min(x[mask] - geometry.to_front_m)
    else:
        mask = (x < 0.0) & (np.abs(y) <= half)
        d = _robust_min(-x[mask] - geometry.to_rear_m)
    return None if d is None else max(0.0, d)


def side_clearances(angles: np.ndarray, ranges: np.ndarray, look_ahead_m: float,
                    geometry: ScanGeometry):
    """Sideways gap between the robot's flanks and obstacles just ahead of it.

    Looks at points beside the drive corridor from the rear bumper to
    ``look_ahead_m`` past the front bumper. Used for gentle steering away from
    walls while driving. Returns (left, right) gaps in metres, None when clear.
    """
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    ahead = (x > -geometry.to_rear_m) & (x < geometry.to_front_m + look_ahead_m)
    gap = np.abs(y) - geometry.half_width_m
    beside = ahead & (np.abs(y) > geometry.half_width_m + geometry.corridor_margin_m)
    left = _robust_min(gap[beside & (y > 0.0)])
    right = _robust_min(gap[beside & (y < 0.0)])
    return left, right


def fuse_front(lidar_front, ultrasonic_m, ultrasonic_to_front_m: float = 0.0):
    """Combine LiDAR corridor clearance and the front ultrasonic reading.

    The more pessimistic (smaller) of the two wins: the ultrasonic catches glass,
    thin chair legs and objects below the LiDAR plane; the LiDAR covers the width.
    Either may be None (no data / nothing seen).
    """
    candidates = []
    if lidar_front is not None:
        candidates.append(lidar_front)
    if ultrasonic_m is not None and math.isfinite(ultrasonic_m):
        candidates.append(max(0.0, ultrasonic_m - ultrasonic_to_front_m))
    return min(candidates) if candidates else None


def edge_ahead(floor_m, max_floor_m: float) -> bool:
    """True when a downward-facing sensor at the front bumper shows no floor below it.

    ``floor_m`` is its latest reading: metres to the floor, ``math.inf`` when no echo
    came back (nothing within range, i.e. a drop), or None when the sensor is stale.
    A stale sensor counts as an edge, so a dead sensor stops the robot instead of
    letting it drive blind towards stairs.
    """
    if floor_m is None or not math.isfinite(floor_m):
        return True
    return floor_m > max_floor_m
