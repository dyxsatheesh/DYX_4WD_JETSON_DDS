#!/usr/bin/env python3
"""Replay stateful C++17 tracking stability against LIVE Python RPP source.

This imports tracking_control.py from the user's checkout, not a copied oracle.
Run under CTest from the repository root; stdlib only, no ROS2 dependency.
"""
from __future__ import annotations

import math
from pathlib import Path
import random
import subprocess
import sys

PROBE = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else None
if PROBE is None or not PROBE.is_file():
    raise SystemExit("Usage: tracking_stability_parity.py <tracking_stability_probe executable>")
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "rpp_controller"))
from rpp_controller.tracking_control import (  # noqa: E402
    TrackingControlConfig,
    TrackingControlInput,
    TrackingStabilityController,
)


def sample(**overrides):
    fields = dict(
        path_identity="path-a", projection_s_m=1.0,
        signed_cross_track_m=0.0, heading_error_rad=0.0,
        commanded_speed_mps=0.6, measured_speed_mps=0.6,
        bearing_clamped=False, telemetry_fresh=True, dt_sec=0.05,
        reset=False,
    )
    fields.update(overrides)
    return TrackingControlInput(**fields)


def float_token(value):
    if math.isnan(value):
        return "nan"
    if value == math.inf:
        return "inf"
    if value == -math.inf:
        return "-inf"
    return repr(float(value))


def tokenized_input(s):
    return " ".join((
        "step", s.path_identity,
        float_token(s.projection_s_m),
        float_token(s.signed_cross_track_m),
        float_token(s.heading_error_rad),
        float_token(s.commanded_speed_mps),
        float_token(s.measured_speed_mps),
        str(int(s.bearing_clamped)), str(int(s.telemetry_fresh)),
        float_token(s.dt_sec), str(int(s.reset)),
    ))


def compare(actual, expected, label):
    # Exact state, booleans, transition reasons and invalid reason; numerical tolerance
    # only for floating-point fields, including recapture dwell accumulation.
    want = (
        expected.state.value,
        int(expected.valid),
        int(expected.acceleration_allowed),
        expected.recovery_speed_scale,
        expected.transition_reason,
        expected.bounded_dt_sec,
        expected.stable_dwell_sec,
        int(expected.reset_applied),
        expected.invalid_reason or "n",
    )
    got = actual.split()
    if len(got) != len(want):
        raise AssertionError(f"{label}: C++ output shape {got}; Python {want}")
    for field, (g, w) in enumerate(zip(got, want)):
        if field in (3, 5, 6):
            if not math.isclose(float(g), float(w), rel_tol=1e-12, abs_tol=1e-12):
                raise AssertionError(f"{label}: field {field}: C++={g}, Python={w}")
        elif field in (1, 2, 7):
            if int(g) != w:
                raise AssertionError(f"{label}: field {field}: C++={g}, Python={w}")
        elif g != w:
            raise AssertionError(f"{label}: field {field}: C++={g}, Python={w}")


def run(actions, *, bearing_clamp_entry):
    reference = TrackingStabilityController(TrackingControlConfig(
        recovery_enter_on_bearing_clamp=bearing_clamp_entry))
    commands, expected = [], []
    for action in actions:
        if isinstance(action, tuple) and action[0] == "reset":
            identity = action[1]
            reference.reset(identity)
            commands.append(f"reset {identity if identity is not None else 'n'}")
        else:
            s = action
            expected.append(reference.step(s))
            commands.append(tokenized_input(s))
    result = subprocess.run(
        [str(PROBE)] if bearing_clamp_entry else [str(PROBE), "no_clamp_entry"],
        input="\n".join(commands) + "\n",
        capture_output=True,
        text=True,
        check=True,
    )
    got = result.stdout.splitlines()
    if len(got) != len(expected):
        raise AssertionError(f"output count C++={len(got)} Python={len(expected)}; {result.stderr}")
    for index, (line, ref) in enumerate(zip(got, expected)):
        compare(line, ref, f"clamp_entry={bearing_clamp_entry}, cycle={index}")
    return len(got)


trace = [("reset", None)]
# Exactly-at-boundary entry/exit, heading, clamp, path identity, faults and dwell.
trace += [sample(dt_sec=0.1)] * 4
trace += [sample(signed_cross_track_m=0.050)]
trace += [sample(signed_cross_track_m=0.021)]
trace += [sample(signed_cross_track_m=0.020, dt_sec=0.1)] * 4
trace += [sample(heading_error_rad=math.radians(15.0))]
trace += [sample(heading_error_rad=math.radians(5.0), dt_sec=0.1)] * 4
trace += [sample(bearing_clamped=True)]
trace += [sample(dt_sec=0.1)] * 4
trace += [sample(path_identity="path-b", dt_sec=0.01)]
trace += [sample(path_identity="path-b", dt_sec=0.2)] * 5
trace += [sample(path_identity="path-b", telemetry_fresh=False)]
trace += [sample(path_identity="path-b", telemetry_fresh=True, dt_sec=0.1)] * 4
trace += [sample(reset=True)]
trace += [sample(signed_cross_track_m=0.12, heading_error_rad=0.35, bearing_clamped=True)]
trace += [("reset", "path-a")]
trace += [sample(heading_error_rad=math.nan)]
trace += [sample(signed_cross_track_m=math.inf)]
trace += [sample(commanded_speed_mps=-0.1)]
trace += [sample(measured_speed_mps=-0.1)]
trace += [sample(projection_s_m=-0.1)]
trace += [sample(dt_sec=-0.1)]
trace += [sample(telemetry_fresh=False, dt_sec=math.nan)]

rng = random.Random(20261013)
for i in range(450):
    trace.append(sample(
        path_identity=rng.choice(["path-a", "path-a", "path-b", "path-c"]),
        projection_s_m=rng.choice([0.0, 0.1, float(i)*0.03, -0.1]),
        signed_cross_track_m=rng.choice([0.0, 0.015, -0.020, 0.050, -0.080, 0.12]),
        heading_error_rad=rng.choice([0.0, math.radians(3.0),
                                      math.radians(5.0), math.radians(15.0),
                                      math.radians(25.0), -math.radians(20.0)]),
        commanded_speed_mps=rng.choice([0.0, 0.6, 1.0, -0.1]),
        measured_speed_mps=rng.choice([0.0, 0.3, 0.8, -0.1]),
        bearing_clamped=rng.choice([True, False]),
        telemetry_fresh=rng.choice([True, True, True, False]),
        dt_sec=rng.choice([0.0, 0.01, 0.05, 0.1, 0.2, -0.01]),
        reset=rng.choice([False, False, False, True]),
    ))

total = run(trace, bearing_clamp_entry=True) + run(trace, bearing_clamp_entry=False)
print(f"PASS: {total} stateful tracking stability Python/C++17 parity cycles")
