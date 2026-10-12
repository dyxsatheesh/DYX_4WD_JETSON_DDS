#!/usr/bin/env python3
"""Compare C++17 math to the *live reference* Python RPP implementation.
Runs with Python stdlib only; no Jetson or ROS needed.
"""
from __future__ import annotations
import math
from pathlib import Path
import subprocess
import sys

PROBE = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else None
if PROBE is None or not PROBE.is_file():
    raise SystemExit("Usage: python python_parity.py <absolute path to native rpp_probe executable>")

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "rpp_controller"))
from rpp_controller.path_geometry import (  # noqa: E402
    GeometrySegment, Point2D, project_onto_segment, wrap_angle)
from rpp_controller.guidance import (  # noqa: E402
    GuidanceConfig, adaptive_lookahead_m, limit_bearing_to_moving_cone)
from rpp_controller.speed_regulator import (  # noqa: E402
    braking_distance, allowable_speed_for_distance)

counts = {"tested": 0}

def check(operation, args, expected):
    cmd = [str(PROBE), operation, *[repr(float(v)) for v in args]]
    result = subprocess.run(cmd, text=True, capture_output=True, check=True)
    actual = [float(x) for x in result.stdout.strip().split()]
    expected = list(expected) if isinstance(expected, (list, tuple)) else [expected]
    if len(expected) != len(actual):
        raise AssertionError(f"{cmd}: got {actual}, expected {expected}")
    for got, target in zip(actual, expected):
        if not math.isclose(got, float(target), rel_tol=1e-11, abs_tol=1e-11):
            raise AssertionError(f"{cmd}: got {got}, expected {target}")
    counts["tested"] += 1

for rad in [-11.0, -4 * math.pi, -3 * math.pi, -math.pi, -0.1, 0.0,
            0.1, math.pi, 3 * math.pi, 11.0]:
    check("wrap", [rad], wrap_angle(rad))

cases = [
    ((0, 0), (4, 0), (2, 0.02)),
    ((0, 0), (4, 0), (2, -0.02)),
    ((0, 0), (4, 0), (6, 0.02)),
    ((0, 0), (4, 0), (-1, 0.02)),
    ((4, 0), (0, 0), (2, 0.02)),
    ((0, 0), (3, 4), (4, 1)),
    ((-2, 1), (1, -2), (3, -3)),
    ((0, 0), (0.5, 0.5), (0, 0)),
]
for start, end, point in cases:
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    seg = GeometrySegment(
        segment_index=7, raw_start_index=0, raw_end_index=1,
        start=Point2D(*start), end=Point2D(*end), length=length,
        s_start=10.0, s_end=10.0 + length,
        heading_rad=math.atan2(end[1] - start[1], end[0] - start[0]), leg_id=0)
    projected = project_onto_segment(Point2D(*point), seg)
    check("projection", [*start, *end, *point],
          [projected.t, projected.point.x, projected.point.y,
           projected.signed_cross_track_m, projected.distance_m, projected.s])

for min_, max_, time_, gain, speed, xtrack in [
    (0.2, 1.0, 0.55, 0.0, 0.6, 0.025),
    (0.2, 1.0, 0.55, 0.0, 1.0, -0.100),
    (0.2, 1.0, 0.55, 0.0, 0.0, 0.00),
    (0.2, 1.0, 0.55, 0.5, 0.6, 0.1),
    (0.3, 1.2, 0.40, 1.0, 1.0, -0.2),
    (0.2, 0.2, 0.55, 0.0, 1.0, 0.1),
]:
    cfg = GuidanceConfig(lookahead_min_m=min_, lookahead_max_m=max_,
                         lookahead_time_s=time_, xtrack_lookahead_gain=gain)
    check("lookahead", [min_, max_, time_, gain, speed, xtrack],
          adaptive_lookahead_m(cfg, speed_mps=speed, signed_cross_track_m=xtrack))

for desired, yaw, cone in [
    (0.0, 0.0, math.radians(30)),
    (math.radians(90), 0, math.radians(30)),
    (math.radians(-90), 0, math.radians(30)),
    (math.radians(170), math.radians(-170), math.radians(30)),
    (math.radians(-170), math.radians(170), math.radians(30)),
    (math.radians(40), math.radians(35), math.radians(30)),
    (math.pi, 0.0, math.pi),
]:
    got_bearing, got_clamp = limit_bearing_to_moving_cone(desired, yaw, cone)
    check("cone", [desired, yaw, cone], [got_bearing, int(got_clamp)])

for v, target, decel, latency, margin in [
    (0.6, 0.0, 0.75, 0.1, 0.05),
    (1.0, 0.15, 0.75, 0.1, 0.05),
    (0.15, 0.15, 0.30, 0.1, 0.01),
    (0.0, 0.0, 0.30, 0.0, 0.0),
    (0.2, 0.5, 0.75, 0.2, 0.02),
]:
    check("brake", [v, target, decel, latency, margin],
          braking_distance(v, target, decel, latency, margin))

for distance, target, decel, latency, margin in [
    (0.8, 0.15, 0.75, 0.1, 0.05),
    (0.05, 0.15, 0.75, 0.1, 0.05),
    (0.02, 0.0, 0.3, 0.1, 0.01),
    (0.0, 0.0, 0.3, 0.0, 0.0),
    (2.0, 0.3, 0.75, 0.1, 0.05),
]:
    check("allowable", [distance, target, decel, latency, margin],
          allowable_speed_for_distance(distance, target, decel, latency, margin))

print(f"PASS: {counts['tested']} C++17 / original Python primitive parity cases")
