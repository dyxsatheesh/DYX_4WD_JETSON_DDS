#!/usr/bin/env python3
"""Compare native C++17 stationary detector with original Python RPP implementation.
Uses the Python module in THIS repository, never a hardcoded expected table.
"""
from __future__ import annotations
import math
from pathlib import Path
import random
import subprocess
import sys

PROBE = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else None
if PROBE is None or not PROBE.is_file():
    raise SystemExit("Usage: stationary_detector_parity.py <native probe>")
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "rpp_controller"))
from rpp_controller.terminal_stop_regulator import PositionWindowStationaryDetector


def run_case(config, actions):
    py = PositionWindowStationaryDetector(
        window_sec=config[0], displacement_limit_m=config[1],
        yaw_rate_limit_radps=config[2], maximum_sample_gap_sec=config[3])
    lines, expected = [], []
    for action in actions:
        if action is None:
            lines.append("reset")
            py.reset()
            continue
        lines.append("update " + " ".join(repr(float(v)) for v in action))
        result = py.update(position_sample_time_sec=action[0], x_m=action[1],
                           y_m=action[2], yaw_rate_radps=action[3])
        expected.append((int(result.stationary), result.held_sec,
                         result.maximum_displacement_m,
                         result.maximum_abs_yaw_rate_radps, result.sample_count))
    cmd = [str(PROBE), *(repr(v) for v in config)]
    proc = subprocess.run(cmd, input="\n".join(lines) + "\n", text=True,
                          capture_output=True, check=True)
    rows = proc.stdout.splitlines()
    assert len(rows) == len(expected), (len(rows), len(expected), proc.stderr)
    for i, (actual, want) in enumerate(zip(rows, expected)):
        parts = actual.split()
        assert len(parts) == 5, (i, parts)
        got = (int(parts[0]), float(parts[1]), float(parts[2]),
               float(parts[3]), int(parts[4]))
        for col, (a, b) in enumerate(zip(got, want)):
            if col in (0, 4):
                assert a == b, (config, i, col, a, b)
            else:
                assert math.isclose(a, b, rel_tol=1e-11, abs_tol=1e-11), (
                    config, i, col, a, b)
    return len(expected)


rng = random.Random(20261013)
actions = []
# Standard stationary lock followed by a duplicate timestamp.
for i in range(20):
    actions.append((i * 0.05, 0.001, -0.001, 0.01))
actions.append((19 * 0.05, 100.0, 100.0, 2.0))  # must ignore duplicate timestamp
# A movement larger than displacement tolerance clears window.
actions.append((1.0, 0.01, 0.0, 0.02))
# A yaw-rate spike clears window and delays certification.
actions.append((1.05, 0.01, 0.0, 0.07))
for i in range(1, 22):
    actions.append((1.05 + i * 0.05, 0.01, 0.0, 0.01))
# Large telemetry gap resets samples; NaN input resets safely.
actions.extend([(3.0, 0.01, 0, 0), (3.05, float("nan"), 0, 0),
                (3.1, 0, 0, 0), (-1.0, 0, 0, 0), None])
# randomized bounded positions and motion bursts
for burst in range(5):
    t0 = 4.0 + burst * 2.5
    for i in range(35):
        x = rng.uniform(-0.0007, 0.0007)
        y = rng.uniform(-0.0007, 0.0007)
        yaw = rng.uniform(-0.02, 0.02)
        if i == 22 and burst % 2 == 0:
            x += 0.015
        if i == 19 and burst % 2:
            yaw = 0.08
        actions.append((t0 + i * 0.05, x, y, yaw))
    actions.append(None)

count = 0
for c in [(0.5, 0.005, 0.05, 0.20),
          (0.3, 0.002, 0.04, 0.10),
          (0.8, 0.008, 0.10, 0.25)]:
    count += run_case(c, actions)
print(f"PASS: {count} native/Python RPP stationary-window evidence parity cases")
