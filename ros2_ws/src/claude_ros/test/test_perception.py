import math

import numpy as np

from claude_ros import perception as pc

GEOM = pc.ScanGeometry(to_front_m=0.1, to_rear_m=0.1, half_width_m=0.1, corridor_margin_m=0.05)


def _scan(n=360, default=0.0):
    """A scan from 0 to 2*pi like the m1ct_d2 driver, 0.0 meaning 'no return'."""
    return [default] * n, 0.0, 2 * math.pi / n


def test_zero_ranges_are_not_obstacles():
    ranges, amin, inc = _scan()
    ang, rng = pc.scan_to_polar(ranges, amin, inc, 0.1, 10.0, GEOM)
    assert rng.size == 0
    assert pc.corridor_clearance(ang, rng, GEOM) is None
    assert pc.sector_distances(ang, rng, 10.0)['front'] == 10.0


def test_wall_ahead_gives_bumper_clearance():
    ranges, amin, inc = _scan()
    for i in list(range(0, 6)) + list(range(355, 360)):
        ranges[i] = 1.0
    ang, rng = pc.scan_to_polar(ranges, amin, inc, 0.1, 10.0, GEOM)
    front = pc.corridor_clearance(ang, rng, GEOM)
    assert 0.85 < front < 0.91        # 1.0 m minus the 0.1 m lidar->bumper offset
    assert pc.corridor_clearance(ang, rng, GEOM, forward=False) is None
    assert pc.sector_distances(ang, rng, 10.0)['front'] == 1.0


def test_yaw_offset_and_reverse():
    ranges, amin, inc = _scan()
    ranges[90] = 0.5                  # beam at +90 deg (left in REP 103)
    ang, rng = pc.scan_to_polar(ranges, amin, inc, 0.1, 10.0, GEOM)
    assert pc.sector_distances(ang, rng, 10.0)['left'] == 0.5
    rev = pc.ScanGeometry(reverse=True)
    ang, rng = pc.scan_to_polar(ranges, amin, inc, 0.1, 10.0, rev)
    assert pc.sector_distances(ang, rng, 10.0)['right'] == 0.5
    rot = pc.ScanGeometry(yaw_offset_rad=-math.pi / 2)
    ang, rng = pc.scan_to_polar(ranges, amin, inc, 0.1, 10.0, rot)
    assert pc.sector_distances(ang, rng, 10.0)['front'] == 0.5


def test_single_speckle_is_ignored():
    ranges, amin, inc = _scan()
    ranges[0] = 0.3                   # one noisy beam
    ranges[1] = ranges[359] = 2.0
    ang, rng = pc.scan_to_polar(ranges, amin, inc, 0.1, 10.0, GEOM)
    assert pc.corridor_clearance(ang, rng, GEOM) > 1.5


def test_side_gap():
    # Wall 0.3 m to the right, parallel to the robot.
    xs = np.linspace(-0.1, 0.8, 40)
    ang = np.arctan2(-0.3, xs)
    rng = np.hypot(xs, 0.3)
    left, right = pc.side_clearances(ang, rng, 0.7, GEOM)
    assert left is None
    assert abs(right - 0.2) < 1e-6


def test_fuse_front_takes_most_pessimistic():
    assert pc.fuse_front(1.0, 0.4) == 0.4
    assert pc.fuse_front(0.3, math.inf) == 0.3
    assert pc.fuse_front(None, 0.5, ultrasonic_to_front_m=0.1) == 0.4
    assert pc.fuse_front(None, None) is None


def _room(n=400, seed=1):
    """An irregular room around the robot, as a raw scan (0.0 = no return)."""
    rng = np.random.default_rng(seed)
    a = np.arange(n) * 2 * math.pi / n
    r = 1.5 + 0.6 * np.sin(3 * a) + 0.4 * np.cos(5 * a + 1.0) + rng.normal(0, 0.01, n)
    r[(a > 2.0) & (a < 2.4)] = 0.0              # an open doorway: no returns
    r[(a > 4.0) & (a < 4.1)] = 0.4              # a chair leg
    return r, a


def test_estimate_rotation_finds_left_and_right_turns():
    base, a = _room()
    inc = a[1] - a[0]
    prev = pc.range_profile(*pc.scan_to_polar(base, 0.0, inc, 0.05, 8.0, GEOM))
    for turn_deg in (0.0, 7.3, -12.0, 30.0, -44.0):
        # Turning left by turn_deg: the same beam index now sees what was turn_deg further on.
        shift = turn_deg / 360.0 * a.size
        cur_r = np.interp((np.arange(a.size) + shift) % a.size, np.arange(a.size + 1),
                          np.append(base, base[0]))
        cur_r[(base == 0.0)[np.round(np.arange(a.size) + shift).astype(int) % a.size]] = 0.0
        cur = pc.range_profile(*pc.scan_to_polar(cur_r, 0.0, inc, 0.05, 8.0, GEOM))
        est = math.degrees(pc.estimate_rotation(prev, cur))
        assert abs(est - turn_deg) < 1.0, (turn_deg, est)


def test_estimate_rotation_respects_reversed_lidars():
    base, a = _room()
    inc = a[1] - a[0]
    rev = pc.ScanGeometry(reverse=True)
    prev = pc.range_profile(*pc.scan_to_polar(base, 0.0, inc, 0.05, 8.0, rev))
    cur = pc.range_profile(*pc.scan_to_polar(np.roll(base, -20), 0.0, inc, 0.05, 8.0, rev))
    assert abs(math.degrees(pc.estimate_rotation(prev, cur)) + 18.0) < 1.0


def test_estimate_rotation_needs_enough_returns():
    empty = np.full(360, np.nan)
    assert pc.estimate_rotation(empty, empty) is None


def test_self_mask_drops_only_close_returns_in_its_arc():
    ranges, amin, inc = _scan()
    for i in range(185, 190):          # behind, slightly right: -175..-170 deg
        ranges[i] = 0.10
    ranges[192] = 0.10                 # -168 deg is outside the arc
    ranges[0] = 0.10                   # straight ahead, same distance
    geom = pc.ScanGeometry(self_mask=((-180.0, -169.0, 0.12),))
    ang, rng = pc.scan_to_polar(ranges, amin, inc, 0.01, 10.0, geom)
    assert sorted(round(math.degrees(a)) for a in ang) == [-168, 0]
    ranges[186] = 0.50                 # a real obstacle further out in the arc stays
    ang, rng = pc.scan_to_polar(ranges, amin, inc, 0.01, 10.0, geom)
    assert 0.50 in rng


# A 4 x 3 m room with a box and a bin in it, for the drive-measurement tests.
_ROOM = np.array([[-1.5, -1.2, 2.5, -1.2], [2.5, -1.2, 2.5, 1.8], [2.5, 1.8, -1.5, 1.8],
                  [-1.5, 1.8, -1.5, -1.2], [0.8, 0.5, 1.2, 0.5], [1.2, 0.5, 1.2, 0.9],
                  [-0.6, -0.6, -0.3, -0.8]])


def _room_scan(x, y, yaw, walls=_ROOM, n=450, noise=0.01):
    """Returns (angles, ranges) seen by a robot at (x, y) facing yaw."""
    rng = np.random.default_rng(0)
    ang = np.linspace(-math.pi, math.pi, n, endpoint=False)
    dx, dy = np.cos(ang + yaw)[:, None], np.sin(ang + yaw)[:, None]
    ex, ey = (walls[:, 2] - walls[:, 0])[None], (walls[:, 3] - walls[:, 1])[None]
    wx, wy = (walls[:, 0] - x)[None], (walls[:, 1] - y)[None]
    det = ex * dy - dx * ey
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (ex * wy - wx * ey) / det
        s = (dx * wy - dy * wx) / det
    t = np.where((np.abs(det) > 1e-9) & (t > 0) & (s >= 0) & (s <= 1), t, np.inf)
    r = t.min(axis=1)
    keep = np.isfinite(r)
    return ang[keep], r[keep] + rng.normal(0.0, noise, keep.sum())


def test_estimate_translation_measures_forward_and_backward_moves():
    key = pc.range_profile(*_room_scan(0.0, 0.0, 0.0))
    for moved, yaw in ((0.05, 0.0), (0.3, 0.03), (-0.2, -0.05)):
        ang, rng = _room_scan(moved * math.cos(yaw), moved * math.sin(yaw), yaw)
        est = pc.estimate_translation(key, ang, rng, rotation=yaw, guess=0.9 * moved)
        assert abs(est - moved) < 0.01


def test_estimate_translation_gives_up_in_a_bare_corridor():
    corridor = np.array([[-20.0, -0.6, 20.0, -0.6], [-20.0, 0.6, 20.0, 0.6]])
    key = pc.range_profile(*_room_scan(0.0, 0.0, 0.0, corridor))
    ang, rng = _room_scan(0.3, 0.0, 0.0, corridor)
    assert pc.estimate_translation(key, ang, rng, guess=0.25) is None


def test_estimate_drive_tracks_distance_and_heading_over_a_metre():
    key = pc.range_profile(*_room_scan(0.0, 0.0, 0.0))
    dist, rot = 0.0, 0.0
    for i in range(1, 41):
        yaw = math.radians(3.0) * math.sin(i / 10.0)          # skid steer wobbles a little
        x = 0.025 * i
        d, rot = pc.estimate_drive(key, *_room_scan(x, 0.0, yaw), rotation=rot,
                                   guess=dist + 0.025)
        dist = d if d is not None else dist + 0.025
        assert abs(dist - x) < 0.015
    assert abs(rot - yaw) < math.radians(1.0)


def test_estimate_rotation_can_search_around_a_guess():
    key = pc.range_profile(*_room_scan(0.0, 0.0, 0.0))
    cur = pc.range_profile(*_room_scan(0.0, 0.0, math.radians(120.0)))
    est = pc.estimate_rotation(key, cur, max_deg=15.0, guess=math.radians(110.0))
    assert abs(math.degrees(est) - 120.0) < 1.5
