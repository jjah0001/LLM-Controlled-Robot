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
    # Parts of the robot that stick up into the scan plane: (from_deg, to_deg, max_m) in the
    # robot frame. Returns inside that arc and closer than max_m are dropped.
    self_mask: tuple = ()


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
    for start_deg, end_deg, max_m in geometry.self_mask:
        start = math.radians(start_deg)
        width = (math.radians(end_deg) - start) % (2 * math.pi)
        inside = ((ang - start) % (2 * math.pi)) <= width
        valid &= ~(inside & (r <= max_m))
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


def range_profile(angles: np.ndarray, ranges: np.ndarray, bin_deg: float = 1.0):
    """Nearest return per ``bin_deg`` slice all round the robot; nan where a slice is empty.

    Puts every scan on the same angle grid, whatever the driver's beam count, so two scans
    can be compared by shifting one against the other.
    """
    n = int(round(360.0 / bin_deg))
    idx = np.floor(np.mod(np.degrees(angles), 360.0) / bin_deg).astype(int) % n
    profile = np.full(n, np.inf)
    np.minimum.at(profile, idx, ranges)
    profile[np.isinf(profile)] = np.nan
    return profile


def estimate_rotation(prev: np.ndarray, cur: np.ndarray, max_deg: float = 45.0,
                      bin_deg: float = 1.0, min_overlap: float = 0.3,
                      outlier_m: float = 0.3, guess: float = 0.0):
    """How far the robot turned (rad, + = left) between two range profiles.

    Turning left by a moves everything a clockwise in the robot frame, so the current
    profile matches the previous one read a further a round: cur[i] ~ prev[i + a]. Tries
    every whole-bin shift up to ``max_deg`` and refines the best one to a fraction of a
    bin. Assumes the robot turned more or less in place (true for fixed turns; a little
    off while driving). Returns None when too few slices overlap to tell. ``guess`` (rad)
    centres the search, so a scan from the start of a long turn can be matched within a
    narrow window around the predicted angle.
    """
    n = cur.size
    centre = int(round(math.degrees(guess) / bin_deg))
    shifts = centre + np.arange(-int(max_deg / bin_deg), int(max_deg / bin_deg) + 1)
    shifted = prev[(np.arange(n)[None, :] + shifts[:, None]) % n]
    diff = np.abs(cur[None, :] - shifted)
    ok = ~np.isnan(diff)
    count = ok.sum(axis=1)
    # Mean mismatch per shift, each slice capped so a moving person can't dominate it.
    cost = np.where(ok, np.minimum(np.nan_to_num(diff), outlier_m), 0.0).sum(axis=1)
    cost = np.where(count >= min_overlap * n, cost / np.maximum(count, 1), np.inf)
    best = int(np.argmin(cost))
    if not np.isfinite(cost[best]):
        return None
    offset = 0.0
    if 0 < best < shifts.size - 1 and np.isfinite(cost[best - 1]) and np.isfinite(
            cost[best + 1]):
        left, mid, right = cost[best - 1], cost[best], cost[best + 1]
        curve = left - 2.0 * mid + right
        if curve > 1e-12:
            offset = _clamp_half(0.5 * (left - right) / curve)
    return math.radians((shifts[best] + offset) * bin_deg)


def _clamp_half(v: float) -> float:
    return max(-0.5, min(0.5, v))


def _profile_cost(key: np.ndarray, xs: np.ndarray, ys: np.ndarray, bin_deg: float,
                  min_overlap: float, outlier_m: float) -> np.ndarray:
    """Mismatch between ``key`` and each row of candidate points (xs, ys), in metres."""
    n = key.size
    a = np.arctan2(ys, xs)
    r = np.hypot(xs, ys)
    idx = np.floor(np.mod(np.degrees(a), 360.0) / bin_deg).astype(int) % n
    profiles = np.full((xs.shape[0], n), np.inf)
    rows = np.broadcast_to(np.arange(xs.shape[0])[:, None], idx.shape)
    np.minimum.at(profiles, (rows, idx), r)
    profiles[np.isinf(profiles)] = np.nan
    diff = np.abs(profiles - key[None, :])
    ok = ~np.isnan(diff)
    count = ok.sum(axis=1)
    cost = np.where(ok, np.minimum(np.nan_to_num(diff), outlier_m), 0.0).sum(axis=1)
    return np.where(count >= min_overlap * n, cost / np.maximum(count, 1), np.inf)


def _best_candidate(cost: np.ndarray, values: np.ndarray, min_contrast: float):
    """The candidate value with the lowest cost, refined between grid points; None if unsure."""
    best = int(np.argmin(cost))
    if not np.isfinite(cost[best]) or best == 0 or best == cost.size - 1:
        return None         # no overlap, or the true value lies outside the search window
    left, mid, right = cost[best - 1], cost[best], cost[best + 1]
    if not (np.isfinite(left) and np.isfinite(right)):
        return None
    near = cost[max(0, best - 3):best + 4]
    if np.max(near[np.isfinite(near)]) - mid < min_contrast:
        return None         # flat cost: the scene doesn't pin the value down
    curve = left - 2.0 * mid + right
    offset = _clamp_half(0.5 * (left - right) / curve) if curve > 1e-12 else 0.0
    return float(values[best] + offset * (values[1] - values[0]))


def estimate_translation(key: np.ndarray, angles: np.ndarray, ranges: np.ndarray,
                         rotation: float = 0.0, guess: float = 0.0, span: float = 0.15,
                         step_m: float = 0.01, bin_deg: float = 1.0, min_overlap: float = 0.3,
                         outlier_m: float = 0.3, min_contrast: float = 0.004):
    """How far forward (m, - = backward) the robot is from where ``key`` was scanned.

    ``key`` is the range profile taken at the start of a straight drive; ``angles`` /
    ``ranges`` are the current scan's returns and ``rotation`` (rad) how far the robot has
    turned since. Each candidate distance near ``guess`` moves the current points back into
    the key scan's frame and compares the resulting profile with ``key``. Matching against
    one fixed scan, rather than chaining scan to scan, keeps errors from adding up over a
    drive. Returns None when the match can't tell distances apart, e.g. in a long, bare
    corridor with nothing ahead or behind, so the caller can fall back to dead reckoning.
    """
    if angles.size == 0:
        return None
    x = ranges * np.cos(angles + rotation)
    y = ranges * np.sin(angles + rotation)
    shifts = guess + np.arange(-span, span + step_m / 2, step_m)
    xs = x[None, :] + shifts[:, None]
    ys = np.broadcast_to(y, xs.shape)
    cost = _profile_cost(key, xs, ys, bin_deg, min_overlap, outlier_m)
    return _best_candidate(cost, shifts, min_contrast)


def estimate_drive(key: np.ndarray, angles: np.ndarray, ranges: np.ndarray,
                   rotation: float = 0.0, guess: float = 0.0, span_deg: float = 6.0,
                   step_deg: float = 0.5, bin_deg: float = 1.0, min_overlap: float = 0.3,
                   outlier_m: float = 0.3):
    """(distance m or None, rotation rad) of the robot since ``key`` was scanned on a drive.

    Chaining ``estimate_rotation`` scan to scan while driving drifts several degrees per
    metre, because it assumes the robot only turned. This instead fits the distance and the
    heading together against the drive's starting scan: distance first at the predicted
    heading, then the heading at that distance, then the distance again. The heading stays
    at ``rotation`` when the scene can't pin it down.
    """
    dist = estimate_translation(key, angles, ranges, rotation, guess, bin_deg=bin_deg,
                                min_overlap=min_overlap, outlier_m=outlier_m)
    if dist is None or angles.size == 0:
        return dist, rotation
    rots = rotation + np.radians(np.arange(-span_deg, span_deg + step_deg / 2, step_deg))
    a = angles[None, :] + rots[:, None]
    xs = ranges[None, :] * np.cos(a) + dist
    ys = ranges[None, :] * np.sin(a)
    cost = _profile_cost(key, xs, ys, bin_deg, min_overlap, outlier_m)
    rot = _best_candidate(cost, rots, 0.002)
    if rot is None:
        return dist, rotation
    again = estimate_translation(key, angles, ranges, rot, dist, span=0.05, bin_deg=bin_deg,
                                 min_overlap=min_overlap, outlier_m=outlier_m)
    return (dist if again is None else again), rot
