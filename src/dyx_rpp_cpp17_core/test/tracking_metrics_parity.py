#!/usr/bin/env python3
"""Phase 2C4: compare C++17 diagnostics against the live Python tracking_control.py.

The Python oracle is imported from the user's own checkout. This test does not
create ROS2 nodes or publish motor commands. Replays samples and diagnostics.
"""
from __future__ import annotations

from dataclasses import fields
import math
from pathlib import Path
import random
import subprocess
import sys

PROBE = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else None
if PROBE is None or not PROBE.is_file():
    raise SystemExit("Usage: tracking_metrics_parity.py <tracking_metrics_probe executable>")
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "rpp_controller"))
from rpp_controller.tracking_control import (  # noqa: E402
    TrackingControlConfig, TrackingControlInput, TrackingControlState,
    TrackingMetricsAccumulator, TrackingMetricsSnapshot,
)


def make(**kw):
    data = dict(path_identity="path-a", projection_s_m=0.0,
                signed_cross_track_m=0.0, heading_error_rad=0.0,
                commanded_speed_mps=0.6, measured_speed_mps=0.5,
                bearing_clamped=False, telemetry_fresh=True, dt_sec=0.05)
    data.update(kw)
    return TrackingControlInput(**data)


def float_token(value):
    return repr(float(value))


def reference_fields(snapshot):
    return [getattr(snapshot, f.name) for f in fields(TrackingMetricsSnapshot)]


def compare(line, accepted, snapshot, context):
    values = line.strip().split()
    expected = [accepted, *reference_fields(snapshot)]
    if len(values) != len(expected):
        raise AssertionError(f"{context}: fields {len(values)} != {len(expected)}: {values}")
    for index, (got, want) in enumerate(zip(values, expected)):
        if want is None:
            valid = got == "n"
        elif isinstance(want, bool):
            valid = got == str(int(want))
        elif isinstance(want, str):
            valid = got == want
        elif isinstance(want, int):
            valid = int(got) == want
        else:
            valid = math.isclose(float(got), float(want), rel_tol=1e-11, abs_tol=1e-11)
        if not valid:
            raise AssertionError(f"{context}: field {index}: C++={got!r} Python={want!r}")


def run(mode, actions):
    kwargs = {}
    if mode == "short":
        kwargs.update(metrics_quantile_window_capacity=5,
                      metrics_histogram_bin_width_m=0.01,
                      metrics_histogram_max_m=0.10)
    oracle = TrackingMetricsAccumulator(TrackingControlConfig(**kwargs))
    inputs, expected = [], []
    for action in actions:
        operation = action[0]
        if operation == "reset":
            _, mission, path = action
            oracle.reset(mission, path)
            command = f"reset {mission or 'n'} {path or 'n'}"
            accepted = "n"
        elif operation == "discontinuity":
            reason = action[1]
            oracle.note_discontinuity(reason)
            command = "discontinuity " + reason
            accepted = "n"
        elif operation == "snapshot":
            command = "snapshot"
            accepted = "n"
        else:
            _, mission, state, sample = action
            accepted = str(int(oracle.add(sample, state, mission_identity=mission)))
            tokens = ["add", mission or "n", sample.path_identity,
                float_token(sample.projection_s_m),
                float_token(sample.signed_cross_track_m),
                float_token(sample.heading_error_rad),
                float_token(sample.commanded_speed_mps),
                float_token(sample.measured_speed_mps),
                str(int(sample.bearing_clamped)), str(int(sample.telemetry_fresh)),
                float_token(sample.dt_sec), state.value]
            command = " ".join(tokens)
        inputs.append(command)
        expected.append((accepted, oracle.snapshot()))
    proc = subprocess.run([str(PROBE), mode], input="\n".join(inputs) + "\n",
                          text=True, capture_output=True, check=True)
    actual = proc.stdout.splitlines()
    if len(actual) != len(expected):
        raise AssertionError(f"{mode}: output lines {len(actual)} != {len(expected)}; {proc.stderr}")
    for i, (output, pair) in enumerate(zip(actual, expected)):
        compare(output, *pair, context=f"{mode} step {i} {inputs[i][:100]}")
    return len(actual)


actions = [("snapshot",), ("reset", "mission-a", "path-a")]
# Straight driving + the 20-mm cross-track acceptance threshold.
for i in range(45):
    actions.append(("add", "mission-a", TrackingControlState.TRACKING,
                    make(projection_s_m=i * 0.04,
                         signed_cross_track_m=[0.0, 0.005, 0.019, 0.020, -0.022][i % 5],
                         commanded_speed_mps=[0.6, 0.8, 1.0][i % 3],
                         measured_speed_mps=0.55, dt_sec=0.12 if i % 3 == 0 else 0.05)))
actions.extend([
    ("snapshot",),
    ("add", "mission-a", TrackingControlState.RECOVERY,
     make(projection_s_m=0.1, signed_cross_track_m=0.12)),
    ("add", "mission-a", TrackingControlState.RECOVERY,
     make(projection_s_m=0.15, telemetry_fresh=False)),
    ("add", "mission-a", TrackingControlState.RECAPTURE_STABLE,
     make(projection_s_m=0.20, signed_cross_track_m=math.inf)),
    ("discontinuity", "  localization_jump  "),
    ("snapshot",),
    ("add", "mission-a", TrackingControlState.RECAPTURE_STABLE,
     make(projection_s_m=0.16, signed_cross_track_m=0.03)),
    ("discontinuity", "   "),
    ("snapshot",),
    ("reset", None, None),
])

rng = random.Random(20261013)
for i in range(540):
    if i and i % 80 == 0:
        actions.append(("discontinuity", " rtcm_reacquire "))
    if i and i % 130 == 0:
        actions.append(("reset", "mission-a", "path-a"))
    path = "path-b" if i % 165 == 164 else "path-a"
    mission = "mission-b" if i % 210 == 209 else "mission-a"
    actions.append(("add", mission,
        rng.choice([TrackingControlState.TRACKING, TrackingControlState.RECOVERY,
                    TrackingControlState.RECAPTURE_STABLE]),
        make(path_identity=path, projection_s_m=max(0, i * 0.02 + rng.uniform(-0.08, 0.08)),
             signed_cross_track_m=rng.choice([0.0, 0.005, -0.015, 0.020,
                 0.050, -0.095, 0.10, 0.11, 0.9, 1.01, 2.0]),
             heading_error_rad=rng.choice([0.0, 0.05, -0.2, 0.8]),
             commanded_speed_mps=rng.choice([0.0, 0.6, 0.8, 1.0, -0.1]),
             measured_speed_mps=rng.choice([0.0, 0.4, 0.7, 1.0, -0.1]),
             telemetry_fresh=rng.choice([True, True, True, False]),
             dt_sec=rng.choice([0.0, 0.01, 0.05, 0.2, -0.1]))))
    if i % 23 == 0:
        actions.append(("snapshot",))
actions.append(("snapshot",))

count = run("default", actions) + run("short", actions)
print(f"PASS: {count} C++17/Python tracking metric snapshots match")
