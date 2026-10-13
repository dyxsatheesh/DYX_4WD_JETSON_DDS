#!/usr/bin/env python3
"""Direct state-by-state replay against the original ROS-free Python pivot FSM.

This imports src/rpp_controller/rpp_controller/motion_state_machine.py from the
user's repository. A re-implemented Python oracle is never used.
"""
from __future__ import annotations

from pathlib import Path
import math
import random
import subprocess
import sys

EXE = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else None
if EXE is None or not EXE.is_file():
    raise SystemExit("Usage: pivot_motion_parity.py <native pivot_motion_probe>")
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "rpp_controller"))
from rpp_controller.motion_state_machine import (  # noqa: E402
    PivotMotionInput, VerifiedPivotStateMachine,
)


def sample(t, **kwargs):
    defaults = dict(monotonic_time_sec=t, dt_sec=0.05, anchor_radial_error_m=0.005,
                    measured_linear_speed_mps=0.0, measured_yaw_rate_radps=0.0,
                    heading_error_rad=0.75, telemetry_fresh=True,
                    pivot_requested=True, brake_to_anchor_requested=False,
                    recapture_complete=False)
    defaults.update(kwargs)
    return PivotMotionInput(**defaults)


def evidence_values(cert, released=False):
    if released:
        return (int(cert.position_ok), int(cert.heading_ok), int(cert.linear_speed_ok),
                int(cert.yaw_rate_ok), int(cert.telemetry_fresh), cert.held_sec,
                cert.required_sec, int(cert.valid))
    return (int(cert.position_ok), int(cert.linear_speed_ok), int(cert.yaw_rate_ok),
            int(cert.telemetry_fresh), cert.held_sec, cert.required_sec,
            int(cert.valid))


def values(result):
    return (result.state.value, result.previous_state.value, result.directive.value,
            result.transition_reason or "-",
            *evidence_values(result.stop_certificate),
            *evidence_values(result.release_certificate, released=True),
            result.max_pivot_drift_m, result.recenter_attempts,
            result.bounded_dt_sec, result.state_elapsed_sec, int(result.failed))


def wire_action(action):
    if isinstance(action, tuple):
        return f"reset {action[1]!r}"
    return "step " + " ".join(
        (str(int(getattr(action, name))) if isinstance(getattr(action, name), bool)
         else repr(float(getattr(action, name))))
        for name in ("monotonic_time_sec", "dt_sec", "anchor_radial_error_m",
                     "measured_linear_speed_mps", "measured_yaw_rate_radps",
                     "heading_error_rad", "telemetry_fresh", "pivot_requested",
                     "brake_to_anchor_requested", "recapture_complete")
    )


def compare(got, expected, scenario, index):
    actual = got.split()
    assert len(actual) == len(expected), (scenario, index, len(actual), len(expected))
    for i, (one, other) in enumerate(zip(actual, expected)):
        if isinstance(other, str):
            assert one == other, (scenario, index, i, one, other)
        elif isinstance(other, int):
            assert int(one) == other, (scenario, index, i, one, other)
        else:
            assert math.isclose(float(one), other, abs_tol=1e-10, rel_tol=1e-10), (
                scenario, index, i, one, other)


def run(scenario, actions):
    py = VerifiedPivotStateMachine()
    inputs, expected = [], []
    for action in actions:
        inputs.append(wire_action(action))
        if isinstance(action, tuple):
            py.reset(monotonic_time_sec=action[1])
        else:
            expected.append(values(py.step(action)))
    cp = subprocess.run([str(EXE)], input="\n".join(inputs) + "\n",
                        text=True, capture_output=True, check=True)
    rows = cp.stdout.splitlines()
    assert len(rows) == len(expected), (scenario, len(rows), len(expected), cp.stderr)
    for i, (a, b) in enumerate(zip(rows, expected)):
        compare(a, b, scenario, i)
    return len(expected)


# 1. Nominal stop -> pivot -> settle -> capture -> track; repeat at second leg.
normal = [("reset", 0.0)]
for i in range(16):
    t = i * 0.05
    normal.append(sample(t,
        brake_to_anchor_requested=i >= 1,
        heading_error_rad=0.75 if i < 10 else 0.01,
        recapture_complete=i >= 15))
for i in range(16, 39):
    t = i * 0.05
    normal.append(sample(t,
        brake_to_anchor_requested=True,
        heading_error_rad=0.01,
        recapture_complete=i >= 18))
normal.append(("reset", 5.0))
for i in range(45):
    normal.append(sample(5.0 + i * 0.05,
        brake_to_anchor_requested=True,
        heading_error_rad=0.55 if i < 10 else 0.01,
        recapture_complete=i >= 24))

# 2. Stale telemetry must suppress all motion directives; lost request and
#    large radial drift must re-enter recenter or hold-fail, never go unnoticed.
adverse = [("reset", 0.0)]
for i in range(110):
    t=i*0.1
    adverse.append(sample(t, dt_sec=0.15,
        brake_to_anchor_requested=i>=1,
        anchor_radial_error_m=(0.30 if 8<=i<11 else 0.005),
        telemetry_fresh=not (20<=i<28 or i>=95),
        heading_error_rad=(0.5 if i<15 else (0.0 if i<40 else -0.7)),
        measured_linear_speed_mps=0.07 if 24<=i<29 else 0.0,
        measured_yaw_rate_radps=0.10 if 16<=i<19 else 0.0,
        recapture_complete=i>=40))
adverse.append(("reset", 12.0))
for i in range(90):
    adverse.append(sample(12.0+i*0.1, dt_sec=0.05, anchor_radial_error_m=0.8,
        brake_to_anchor_requested=i>=1, heading_error_rad=1.0,
        pivot_requested=i<4, telemetry_fresh=i<70))

# 3. Deterministic pseudo-random control input stresses dwell, cancellation,
#    retries and timeouts without changing config or commanding real hardware.
rng=random.Random(20261013)
random_actions=[("reset", 0.0)]
for i in range(650):
    t=i*0.05
    random_actions.append(sample(t,dt_sec=rng.choice([0.0,0.01,0.05,0.11]),
        anchor_radial_error_m=rng.choice([0.0,0.005,0.02,0.021,0.08]),
        measured_linear_speed_mps=rng.choice([0.0,0.029,0.031,0.2,-0.3]),
        measured_yaw_rate_radps=rng.choice([0.0,0.05,0.051,-0.15]),
        heading_error_rad=rng.choice([0.0,0.01,0.035,0.20,-2.0,2*math.pi+0.01]),
        telemetry_fresh=rng.random()>0.1,
        pivot_requested=rng.random()>0.10,
        brake_to_anchor_requested=rng.random()>0.4,
        recapture_complete=rng.random()>0.7))

total=sum([run("normal",normal),run("adverse",adverse),run("random",random_actions)])
print(f"PASS: {total} original Python/C++17 verified-pivot FSM state cycles")
