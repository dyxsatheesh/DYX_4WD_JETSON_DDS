"""radial20 must re-approach a goal after stopping inside the terminal zone.

2026-09-30 (mission.csv_20260930_185447): an RTK FLOAT pause stopped the
rover 111/42/97 mm short of P0002/P0009/P0012. On auto-resume the regulator
sat in TERMINAL_GUIDANCE passing through the previous cycle's published
speed (0 after the pause), RPP published 0 again, and each goal failed with
terminal_timeout 15 s later without the wheels turning.
"""

import ast
import math
from pathlib import Path
from types import SimpleNamespace

from rpp_controller.terminal_stop_regulator import (
    RadialStopConfig,
    RadialStopInput,
    RadialStopState,
    TerminalStopRegulator,
)

NODE_PATH = Path(__file__).resolve().parents[1] / "rpp_controller" / "rpp_controller_node.py"
NODE_SOURCE = NODE_PATH.read_text(encoding="utf-8")
FLOOR_MPS = 0.15  # deceleration_floor_speed_mps (TERMINAL_FLOOR_SPEED_MPS)


def _step_method():
    tree = ast.parse(NODE_SOURCE)
    controller = next(
        n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "RPPController"
    )
    method = next(
        n
        for n in controller.body
        if isinstance(n, ast.FunctionDef) and n.name == "_step_radial20_terminal_for_cycle"
    )
    namespace = {
        "math": math,
        "RadialStopInput": RadialStopInput,
        "RadialStopState": RadialStopState,
    }
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(NODE_PATH), "exec"), namespace)
    return namespace["_step_radial20_terminal_for_cycle"]


def _node(clock):
    config = RadialStopConfig(
        radial_tolerance_m=0.020,
        terminal_guidance_distance_m=0.75,
        conservative_decel_mps2=0.75,
        brake_margin_m=0.003,
    )
    node = SimpleNamespace(
        radial_stop_regulator=TerminalStopRegulator(config),
        deceleration_floor_speed=FLOOR_MPS,
        precision_last_published_translational_speed_mps=0.0,  # after the pause
        radial_stop_telemetry_timeout_sec=0.25,
        last_odom_time=0.0,
        current_yaw_rate_radps=0.0,
        current_x=0.0,
        current_y=0.0,
    )
    node._current_precision_terminal_identity = lambda: ("RUN|GOAL:P0009", {})
    node._precision_now_sec = lambda: clock["t"]
    node.is_fresh = lambda *_: True
    node._radial_stop_position_sample_time_sec = lambda: clock["t"]
    node._radial_stop_position_derived_speed_mps = lambda _t: 0.0
    node._publish_radial20_heartbeat = lambda *_: None
    return node


def test_resume_from_standstill_inside_terminal_zone_moves_toward_goal():
    step = _step_method()
    clock = {"t": 100.0}
    node = _node(clock)
    commands = []
    for _ in range(50):  # 1 s at 50 Hz, rover stationary 97 mm short (P0012)
        result = step(node, along_remaining=0.097, cross_error=-0.006)
        assert result is not None
        commands.append(result.forward_speed_command_mps)
        node.precision_last_published_translational_speed_mps = (
            result.forward_speed_command_mps
        )
        clock["t"] += 0.02
    assert max(commands) >= FLOOR_MPS
    assert node.radial_stop_regulator.state is not RadialStopState.HOLD_FAIL


def test_moving_approach_speed_is_passed_through_unchanged():
    step = _step_method()
    clock = {"t": 100.0}
    node = _node(clock)
    node.precision_last_published_translational_speed_mps = 0.6
    result = step(node, along_remaining=0.70, cross_error=0.0)
    assert math.isclose(result.forward_speed_command_mps, 0.6)
