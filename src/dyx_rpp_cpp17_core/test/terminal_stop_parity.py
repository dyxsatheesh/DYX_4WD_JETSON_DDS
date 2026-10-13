#!/usr/bin/env python3
"""Exact stateful replay against THIS checkout's original Python radial-stop FSM.

Runs only pure computations; no PX4, Jetson DDS, ROS node or motor actuation.
"""
from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import subprocess
import sys

PROBE = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else None
if PROBE is None or not PROBE.is_file():
    raise SystemExit("Usage: terminal_stop_parity.py <terminal_stop_probe executable>")
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "rpp_controller"))
from rpp_controller.terminal_stop_regulator import (  # noqa: E402
    TerminalStopRegulator, RadialStopInput,
)


def sample(t, *, pt=None, active=True, identity="P0001", along=1.0,
           cross=0.0, x=4.0, y=1.0, pspeed=0.0, yaw=0.01,
           speed=0.3, fresh=True):
    return RadialStopInput(
        monotonic_time_sec=t,
        position_sample_time_sec=t if pt is None else pt,
        active=active, terminal_identity=identity,
        along_remaining_m=along, cross_error_m=cross,
        position_x_m=x, position_y_m=y,
        position_derived_speed_mps=pspeed,
        measured_yaw_rate_radps=yaw,
        tracking_speed_command_mps=speed, telemetry_fresh=fresh)


def to_line(s):
    identity = s.terminal_identity if s.terminal_identity is not None else "_"
    assert not any(c.isspace() for c in identity), identity
    values = (s.monotonic_time_sec, s.position_sample_time_sec, int(s.active),
              identity, s.along_remaining_m, s.cross_error_m, s.position_x_m,
              s.position_y_m, s.position_derived_speed_mps,
              s.measured_yaw_rate_radps, s.tracking_speed_command_mps,
              int(s.telemetry_fresh))
    return "step " + " ".join(str(v) if isinstance(v, str) else repr(v) for v in values)


def expected_fields(o):
    answer = [o.previous_state.value, o.state.value, o.motion_direction.value,
              o.forward_speed_command_mps, int(o.hold_zero),
              o.effective_braking_speed_mps, o.stop_distance_m,
              o.profile_speed_mps, o.radial_error_m, int(o.stationary),
              o.stationary_window_sec, o.failure.value,
              int(o.certificate is not None)]
    if o.certificate is not None:
        c = o.certificate
        answer += [c.version, c.terminal_identity, c.certified_timestamp_sec,
                   c.radial_error_m, c.along_error_m, c.cross_error_m,
                   c.final_position_x_m, c.final_position_y_m,
                   c.stationary_window_sec, c.maximum_stationary_displacement_m,
                   c.maximum_abs_yaw_rate_radps, c.speed_source, c.truth_frame]
    return answer


def compare(got, want, scenario, step):
    values = got.split()
    if len(values) != len(want):
        raise AssertionError((scenario, step, "shape", values, want))
    for idx, (text, expected) in enumerate(zip(values, want)):
        if isinstance(expected, str):
            match = text == expected
        elif isinstance(expected, int):
            match = int(text) == expected
        else:
            actual = float(text)
            match = math.isclose(actual, expected, rel_tol=1e-10, abs_tol=1e-10)
        if not match:
            raise AssertionError((scenario, step, idx, text, expected))


def run(name, actions):
    reference = TerminalStopRegulator()
    lines = []
    expected = []
    for action in actions:
        if action in ("reset", "cancel"):
            getattr(reference, action)()
            lines.append(action)
        else:
            lines.append(to_line(action))
            expected.append(expected_fields(reference.step(action)))
    proc = subprocess.run([str(PROBE)], input="\n".join(lines) + "\n",
                          capture_output=True, text=True, check=True)
    actual = proc.stdout.splitlines()
    if len(actual) != len(expected):
        raise AssertionError((name, "line count", len(actual), len(expected), proc.stderr))
    for idx, (got, want) in enumerate(zip(actual, expected)):
        compare(got, want, name, idx)
    print(f"PASS {name}: {len(expected)} cycles")
    return len(expected)


# A complete approach, 20 mm radial stop, settle, certificate and held zero.
normal = [sample(0.0, active=False, identity=None),
          sample(0.05, active=False),
          sample(0.10, along=2.0),
          sample(0.15, along=0.60),
          sample(0.20, along=0.15, pspeed=0.5, speed=0.5),
          sample(0.25, along=0.10, pspeed=0.3, speed=0.3),
          sample(0.30, along=0.012, cross=0.001, speed=0.05)]
for i in range(1, 18):
    normal.append(sample(0.30 + 0.05*i, along=0.012, cross=0.001,
                         speed=0.0, x=4.001, y=1.001))
normal.append(sample(1.20, active=False, identity=None, along=float("nan")))

# Existing certificate and failure latches survive further changed inputs.
identity = [sample(0.0, along=1.0), sample(0.05, along=0.4),
            sample(0.10, along=0.3, identity="P0002"),
            sample(0.15, active=False, identity=None, speed=-1.0),
            "cancel", sample(1.0, along=0.3, fresh=False),
            sample(1.05, along=0.2),
            "reset", sample(2.0, along=0.3),
            sample(2.05, active=False),
            sample(2.10, active=True, identity=None),
            "reset", sample(3.0, active=True, speed=-0.1),
            "reset", sample(4.0, active=True, along=float("nan")),
            "reset", sample(5.0, active=True, pspeed=float("inf"))]

# Goal-plane crossing causes one-way zero latch even when lateral error is large;
# after stable stationarity it must fail rather than certify outside 20 mm.
overshoot = [sample(0.0, along=0.25),
             sample(0.05, along=-0.01, cross=0.10)]
for i in range(1, 15):
    overshoot.append(sample(0.05 + 0.05*i, along=-0.01, cross=0.10,
                            speed=0.0))
overshoot.append(sample(1.0, along=0.01, cross=0.0, speed=0.0))

# Changing time, duplicate odometry samples, yaw spikes and loss-of-dwell.
window = [sample(0.0, along=0.01)]
for i in range(1, 34):
    t=i*0.05
    pt=0.20 if 5 <= i <= 8 else t
    # duplicate position timestamp must not certify prematurely.
    if i == 9:
        pt=0.25
    if i == 12:
        yaw=0.08
    else:
        yaw=0.01
    # Move out of tolerance temporarily but back inside before eventual settle.
    remaining=0.012 if i != 17 else 0.04
    window.append(sample(t, pt=pt, along=remaining, yaw=yaw, speed=0.0,
                         x=4.0 if i<15 else 4.002))

# Timed-out active terminal; missing terminal after engagement; settle timeout.
timeouts = [sample(0.0, along=0.9), sample(15.10, along=0.9),
            "reset", sample(16.0, along=0.02),
            sample(21.1, along=0.02, pt=16.0, speed=0.0),
            "reset", sample(25.0, along=0.3),
            sample(25.1, active=False)]

# A finite sample with time advancing but odometry sample timestamp regressing
# becomes INVALID_MEASUREMENT, rather than throwing the exception to the caller.
backward_position = [sample(0.0, along=0.015),
                     sample(0.1, pt=0.1, along=0.015),
                     sample(0.2, pt=0.05, along=0.015)]

runs = [("normal_certificate", normal), ("fail_closed_latches", identity),
        ("goal_plane_overshoot", overshoot), ("stationarity_replay", window),
        ("timeouts", timeouts), ("odometry_time_regression", backward_position)]
total = sum(run(name, actions) for name, actions in runs)
print(f"PASS: {total} original-Python vs native-C++17 terminal-stop FSM parity cycles")
