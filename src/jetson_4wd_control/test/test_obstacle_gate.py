"""cmd_vel_bridge second obstacle layer: same contract as mission_manager."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest  # noqa: E402

from jetson_4wd_control.obstacle_gate import obstacle_blocks_motion  # noqa: E402

DANGER = {"enabled": True, "healthy": True, "zone": "danger", "distance_mm": 900}


def test_fresh_healthy_enabled_danger_blocks():
    assert obstacle_blocks_motion(DANGER, 0.1)


@pytest.mark.parametrize(
    "change,age",
    [
        ({"enabled": False}, 0.1),
        ({"healthy": False}, 0.1),        # dead sensor: warning only
        ({"zone": "warning"}, 0.1),
        ({"zone": "clear"}, 0.1),
        ({"zone": "unknown"}, 0.1),
        ({}, 1.5),                        # stale status: warning only
        ({}, float("inf")),               # never received
    ],
)
def test_everything_else_does_not_block(change, age):
    assert not obstacle_blocks_motion({**DANGER, **change}, age)


def test_bridge_gates_on_obstacle_right_after_estop():
    source = (Path(__file__).resolve().parents[1] / "jetson_4wd_control" / "cmd_vel_bridge.py").read_text()
    estop = source.index('reason = "emergency_stop"')
    obstacle = source.index('reason = "obstacle_danger"')
    mission = source.index('reason = "mission_disabled"')
    assert estop < obstacle < mission


def test_bridge_sends_zero_on_the_danger_message_itself():
    source = (Path(__file__).resolve().parents[1] / "jetson_4wd_control" / "cmd_vel_bridge.py").read_text()
    callback = source[source.index("def _obstacle_callback"):source.index("def _heartbeat_callback")]
    assert "was_blocking" in callback
    assert '_evaluate_and_publish(from_timer=False, source="gate_change")' in callback
