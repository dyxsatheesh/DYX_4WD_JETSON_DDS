#!/usr/bin/env python3
"""Stateful parity replay against the ORIGINAL Python RPP speed regulator.

The Python reference is imported from the user's current repository checkout;
no copied or simplified Python calculations are used as the oracle.
"""
from __future__ import annotations

from dataclasses import fields
import math
from pathlib import Path
import random
import subprocess
import sys

PROBE = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else None
if not PROBE or not PROBE.is_file():
    raise SystemExit("Usage: speed_regulator_parity.py <speed_regulator_probe executable>")
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "rpp_controller"))
from rpp_controller.speed_regulator import (  # noqa: E402
    LongitudinalRegulator, LongitudinalRegulatorConfig, SpeedRegulatorInput,
)

FIELD_NAMES = tuple(f.name for f in fields(SpeedRegulatorInput))
assert len(FIELD_NAMES) == 15, FIELD_NAMES


def make(**overrides):
    values = dict(mission_speed_ceiling_mps=1.0, measured_speed_mps=0.0,
                  last_commanded_speed_mps=0.0, dt_sec=0.05,
                  along_track_progress_m=0.0, heading_error_rad=0.0,
                  cross_track_error_m=0.0)
    values.update(overrides)
    return SpeedRegulatorInput(**values)


def out_fields(result):
    caps = result.caps
    return (result.requested_speed_mps, result.winning_cap_owner.value,
            result.effective_speed_mps, result.bounded_dt_sec,
            result.acceleration_gate_scale, result.acceleration_progress_m,
            int(result.recovery_active), result.recovery_transition,
            result.recovery_exit_dwell_sec,
            result.recovery_exit_dwell_required_sec,
            result.corner_required_braking_distance_m,
            result.terminal_required_braking_distance_m,
            caps.mission_mps, caps.hardware_mps, caps.recovery_mps,
            caps.acceleration_mps, caps.heading_mps, caps.cross_track_mps,
            caps.tracking_mps, caps.corner_mps, caps.terminal_mps,
            caps.curvature_mps)


def token(value):
    if value is None:
        return "n"
    if isinstance(value, bool):
        return str(int(value))
    return repr(float(value))


def compare(actual_line, expected, context):
    actual = actual_line.split()
    assert len(actual) == len(expected), f"{context}: shape {actual} vs {expected}"
    for idx, (got, reference) in enumerate(zip(actual, expected)):
        if reference is None:
            assert got == "n", f"{context}: field {idx}: {got} != None"
        elif isinstance(reference, str):
            assert got == reference, f"{context}: field {idx}: {got} != {reference}"
        elif isinstance(reference, int):
            assert int(got) == reference, f"{context}: field {idx}: {got} != {reference}"
        else:
            assert math.isclose(float(got), reference, rel_tol=1e-10, abs_tol=1e-10), (
                f"{context}: field {idx}: C++={got} Python={reference}"
            )


def run(curvature_enabled, actions):
    regulator = LongitudinalRegulator(LongitudinalRegulatorConfig(
        curvature_enabled=curvature_enabled))
    lines, expected = [], []
    for action in actions:
        if action[0] == "reset":
            _, progress, initial = action
            regulator.reset(along_track_progress_m=progress, initial_speed_mps=initial)
            lines.append(f"reset {progress!r} {initial!r}")
        else:
            request = action[1]
            lines.append("step " + " ".join(token(getattr(request, name)) for name in FIELD_NAMES))
            expected.append(out_fields(regulator.resolve(request)))
    proc = subprocess.run([str(PROBE), "curvature" if curvature_enabled else "normal"],
                          input="\n".join(lines) + "\n", text=True,
                          capture_output=True, check=True)
    actual = proc.stdout.splitlines()
    assert len(actual) == len(expected), (len(actual), len(expected), proc.stderr)
    for i, (line, want) in enumerate(zip(actual, expected)):
        compare(line, want, f"curvature={curvature_enabled} case={i}")
    return len(expected)


rng = random.Random(20261013)
actions = [("reset", 0.0, 0.0)]
# A straight leg, controlled launch and a 0.8 m braking region.
for i in range(30):
    s = i * 0.05
    actions.append(("step", make(along_track_progress_m=s,
        measured_speed_mps=min(i * 0.025, 0.65),
        last_commanded_speed_mps=min(i * 0.027, 0.70),
        distance_to_terminal_m=max(0.0, 1.50 - s))))
actions.append(("reset", 2.0, 0.0))
# Trigger recovery; hold until the hysteresis dwell is satisfied.
for i in range(16):
    actions.append(("step", make(along_track_progress_m=2.0 + 0.06 * i,
        measured_speed_mps=0.4, last_commanded_speed_mps=0.3,
        cross_track_error_m=0.12 if i < 5 else 0.0,
        heading_error_rad=math.radians(18.0) if i < 3 else 0.0)))
# Hard corners, terminal overrides, tracking gate and disabled acceleration.
for i in range(50):
    actions.append(("step", make(along_track_progress_m=2.7 + i * 0.04,
      dt_sec=rng.uniform(-0.01, 0.15),
      measured_speed_mps=rng.uniform(-0.5, 0.9),
      last_commanded_speed_mps=rng.uniform(-0.5, 0.9),
      heading_error_rad=rng.uniform(-0.3, 0.3),
      cross_track_error_m=rng.uniform(-0.15, 0.15),
      mission_speed_ceiling_mps=rng.choice([0.25, 0.6, 1.0]),
      distance_to_corner_m=rng.choice([None, 0.0, 0.1, 0.6, 1.4]),
      corner_angle_rad=rng.choice([None, 0.1, math.pi / 2, -math.pi / 3]),
      distance_to_terminal_m=rng.choice([None, 0.0, 0.05, 0.8, 2.0]),
      terminal_target_speed_override_mps=rng.choice([None, 0.0, 0.12, 0.15]),
      curvature_inv_m=rng.choice([None, 0.0, 0.05, 1.5, -2.0]),
      tracking_acceleration_allowed=rng.choice([True, False]),
      tracking_speed_cap_mps=rng.choice([None, 0.0, 0.15, 0.5]),
      hard_zero=rng.choice([False, False, False, True]))))

total = run(False, actions) + run(True, actions)
print(f"PASS: {total} longitudinal RPP Python/C++17 stateful parity cycles")
