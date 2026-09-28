"""LASER marking tool: AUX6 ON at a verified point, held until NEXT, then OFF."""

from __future__ import annotations

import json
import math
import time
from types import SimpleNamespace

import pytest

from test_manual_spray import SprayController, _Future, build_node


def _accepted() -> SimpleNamespace:
    return SimpleNamespace(success=True, result=SprayController.MAV_RESULT_ACCEPTED)


def _ack_last(node) -> None:
    node.command_client.futures[-1]._done = True
    node.command_client.futures[-1]._result = _accepted()


def _last_request(node):
    return node.command_client.requests[-1]


def laser_mission_node(tmp_path) -> SprayController:
    """A controller at a verified, marking-active point of a LASER mission."""
    node = build_node(tmp_path)
    now = time.monotonic()
    node.mission_marking_tool = SprayController.TOOL_LASER
    node.mission_state = "RUNNING"
    node.mission_run_id = "run-1"
    node.current_point_id = "P0001"
    node.current_point_index = 0
    node.current_point_state = "ACTIVE"
    node.marking_active = True
    node.status_marking_active = True
    node.marking_active_since = now - 1.0
    node.mission_enable = True
    node.px4_mode = "OFFBOARD"
    node._spray_gate_ok = lambda _now: True
    return node


def laser_on_node(tmp_path) -> SprayController:
    """Drive a LASER point to LASER_ON through the real control loop."""
    node = laser_mission_node(tmp_path)
    node._control_loop()
    _ack_last(node)
    node._control_loop()
    assert node.state == SprayController.STATE_LASER_ON
    return node


def test_laser_press_drives_actuator_set_2_and_leaves_spray_alone(tmp_path):
    node = laser_mission_node(tmp_path)

    node._control_loop()

    request = _last_request(node)
    assert node.state == SprayController.STATE_WAIT_PRESS_ACK
    assert node.transaction_tool == SprayController.TOOL_LASER
    assert request.param2 == pytest.approx(1.0)
    assert math.isnan(request.param1)
    for name in ("param3", "param4", "param5", "param6"):
        assert math.isnan(getattr(request, name))


def test_spray_tool_still_drives_actuator_set_1(tmp_path):
    node = laser_mission_node(tmp_path)
    node.mission_marking_tool = SprayController.TOOL_SPRAY

    node._control_loop()

    request = _last_request(node)
    assert request.param1 == pytest.approx(node.press_value)
    assert math.isnan(request.param2)


def test_laser_on_ack_reports_success_once_and_holds(tmp_path):
    node = laser_on_node(tmp_path)

    results = [json.loads(m.data) for m in node.spray_result_pub.messages]
    assert [r["result"] for r in results] == ["SUCCESS"]
    assert [m.data for m in node.spray_complete_pub.messages] == ["P0001"]
    assert node.press_may_be_active is True

    # mission_manager takes over: point complete, hold published, WAITING.
    node.mission_state = "WAITING_FOR_NEXT"
    node.current_point_id = "P0002"
    node.marking_active = False
    node.laser_hold_run_id = "run-1"
    node.laser_hold_point_id = "P0001"
    node.laser_hold_mode = SprayController.LASER_STEADY
    node.laser_on_since = time.monotonic() - 600.0  # no time limit on a hold
    requests_before = len(node.command_client.requests)

    node._control_loop()

    assert node.state == SprayController.STATE_LASER_ON
    assert len(node.command_client.requests) == requests_before


def test_hold_cleared_turns_laser_off_on_set_2_then_idle_without_fault(tmp_path):
    node = laser_on_node(tmp_path)
    node.mission_state = "WAITING_FOR_NEXT"
    node.laser_hold_run_id = "run-1"
    node.laser_hold_point_id = "P0001"
    node.laser_hold_mode = SprayController.LASER_STEADY
    node._control_loop()

    node.laser_hold_run_id = None
    node.laser_hold_point_id = None  # operator pressed NEXT
    node._control_loop()

    request = _last_request(node)
    assert node.state == SprayController.STATE_WAIT_RELEASE_ACK
    assert request.param2 == pytest.approx(-1.0)
    assert math.isnan(request.param1)

    _ack_last(node)
    node._control_loop()

    assert node.state == SprayController.STATE_IDLE
    assert node.fault_latched is False
    assert node.press_may_be_active is False
    assert node.release_confirmed is True
    results = [json.loads(m.data)["result"] for m in node.spray_result_pub.messages]
    assert results == ["SUCCESS"]


def test_handoff_keeps_laser_on_briefly_then_releases(tmp_path):
    node = laser_on_node(tmp_path)
    requests_before = len(node.command_client.requests)

    # Still RUNNING on the same point, no hold yet: inside the handoff.
    node._control_loop()
    assert node.state == SprayController.STATE_LASER_ON
    assert len(node.command_client.requests) == requests_before

    node.laser_on_since = time.monotonic() - 2.0
    node._control_loop()
    assert node.state == SprayController.STATE_WAIT_RELEASE_ACK


@pytest.mark.parametrize(
    "change",
    ["estop", "disarm", "offboard_lost", "disabled", "status_stale", "tool"],
)
def test_safety_loss_turns_laser_off_without_fault(tmp_path, change):
    node = laser_on_node(tmp_path)
    node.mission_state = "WAITING_FOR_NEXT"
    node.laser_hold_run_id = "run-1"
    node.laser_hold_point_id = "P0001"
    node.laser_hold_mode = SprayController.LASER_STEADY

    if change == "estop":
        node.emergency_stop = True
    elif change == "disarm":
        node.px4_armed = False
    elif change == "offboard_lost":
        node.px4_mode = "MANUAL"
    elif change == "disabled":
        node.enabled = False
    elif change == "status_stale":
        node.last_mission_status_rx = time.monotonic() - 10.0
    elif change == "tool":
        node.mission_marking_tool = SprayController.TOOL_SPRAY

    node._control_loop()
    assert node.state == SprayController.STATE_WAIT_RELEASE_ACK
    assert _last_request(node).param2 == pytest.approx(-1.0)

    _ack_last(node)
    node._control_loop()
    assert node.state == SprayController.STATE_IDLE
    assert node.fault_latched is False


def test_estop_callback_releases_laser_immediately(tmp_path):
    node = laser_on_node(tmp_path)

    node._emergency_stop_callback(SimpleNamespace(data=True))

    assert node.state == SprayController.STATE_WAIT_RELEASE_ACK
    assert _last_request(node).param2 == pytest.approx(-1.0)


def test_status_reports_laser_on_point(tmp_path):
    node = laser_on_node(tmp_path)

    node._publish_status()
    payload = json.loads(node.spray_status_pub.messages[-1].data)

    assert payload["laser_on"] is True
    assert payload["laser_on_point_id"] == "P0001"
    assert payload["marking_tool"] == "LASER"
    assert payload["laser_actuator_set"] == 2


def test_mission_status_parses_tool_and_hold(tmp_path):
    node = build_node(tmp_path)

    node._mission_status_callback(
        SimpleNamespace(
            data=json.dumps(
                {
                    "state": "WAITING_FOR_NEXT",
                    "mission_run_id": "run-1",
                    "marking_tool": "laser",
                    "laser_hold": {"mission_run_id": "run-1", "point_id": "P0003"},
                }
            )
        )
    )

    assert node.mission_marking_tool == SprayController.TOOL_LASER
    assert node.laser_hold_point_id == "P0003"

    node._mission_status_callback(SimpleNamespace(data=json.dumps({"state": "READY"})))
    assert node.mission_marking_tool == SprayController.TOOL_SPRAY
    assert node.laser_hold_point_id is None


def test_restart_with_laser_journal_turns_laser_off_without_fault(tmp_path):
    node = build_node(tmp_path)
    node.transaction_tool = SprayController.TOOL_LASER
    node.transaction_kind = SprayController.TRANSACTION_MISSION
    node.transaction_run_id = "run-1"
    node.transaction_point_id = "P0001"
    node.press_may_be_active = True
    node.failure_after_release = "NODE_RESTART_DURING_UNCERTAIN_SPRAY"
    node.state = SprayController.STATE_RECOVERY_RELEASE
    node._write_journal(state=SprayController.JOURNAL_PRESSED)

    node._control_loop()
    assert _last_request(node).param2 == pytest.approx(-1.0)
    _ack_last(node)
    node._control_loop()

    assert node.state == SprayController.STATE_IDLE
    assert node.fault_latched is False
    assert not node.journal_path.exists()


def test_laser_press_rejected_by_px4_still_fails_the_point(tmp_path):
    node = laser_mission_node(tmp_path)
    node._control_loop()
    node.command_client.futures[-1]._done = True
    node.command_client.futures[-1]._result = SimpleNamespace(success=True, result=4)

    node._control_loop()  # rejected -> release
    _ack_last(node)
    node._control_loop()

    results = [json.loads(m.data) for m in node.spray_result_pub.messages]
    assert results[-1]["result"] == "FAILED"
    assert node.spray_complete_pub.messages == []


def _laser_msg(request_id: str, on, off) -> SimpleNamespace:
    return SimpleNamespace(
        data=json.dumps(
            {"request_id": request_id, "laser_on_value": on, "laser_off_value": off}
        )
    )


def test_laser_config_is_saved_and_applied_on_restart(tmp_path):
    node = build_node(tmp_path)
    node._spray_duration_config_callback(
        SimpleNamespace(
            data=json.dumps({"request_id": "d", "spray_duration_sec": 0.4})
        )
    )

    node._laser_config_callback(_laser_msg("l1", 0.8, -0.8))

    assert node._last_config_result == "ACCEPTED"
    saved = json.loads(node.spray_settings_path.read_text())
    assert saved["laser_on_value"] == pytest.approx(0.8)
    assert saved["spray_duration_sec"] == pytest.approx(0.4)  # merged, not lost

    restarted = build_node(tmp_path)
    restarted._apply_saved_settings()
    assert restarted.laser_on_value == pytest.approx(0.8)
    assert restarted.laser_off_value == pytest.approx(-0.8)
    assert restarted.laser_value_source == "saved"
    assert restarted.spray_duration_sec == pytest.approx(0.4)


@pytest.mark.parametrize("on,off", [(1.5, -1.0), (0.5, 0.5), ("x", -1.0)])
def test_laser_config_rejects_invalid(tmp_path, on, off):
    node = build_node(tmp_path)

    node._laser_config_callback(_laser_msg("l2", on, off))

    assert node._last_config_result == "REJECTED"
    assert node.laser_on_value == pytest.approx(1.0)


def test_laser_config_rejected_while_laser_on(tmp_path):
    node = laser_on_node(tmp_path)

    node._laser_config_callback(_laser_msg("l3", 0.5, -0.5))

    assert node._last_config_reason == "SPRAY_CONTROLLER_NOT_IDLE"


def test_shutdown_releases_spray_and_laser_in_one_command(tmp_path, monkeypatch):
    node = build_node(tmp_path)
    sent = []
    node.command_client.call_async = lambda request: sent.append(request) or _Future(
        done=True, result=_accepted()
    )
    monkeypatch.setattr(
        "spray_controller.spray_controller_node.rclpy.spin_until_future_complete",
        lambda *a, **k: None,
        raising=False,
    )

    node.best_effort_release_on_shutdown()

    assert sent[0].param1 == pytest.approx(node.release_value)
    assert sent[0].param2 == pytest.approx(node.laser_off_value)


# ------------------------------------------------------------ BLINK


def blink_hold_node(tmp_path) -> SprayController:
    """WAITING_FOR_NEXT after a raw-GNSS measured miss at P0001."""
    node = build_node(tmp_path)
    node.mission_marking_tool = SprayController.TOOL_LASER
    node.mission_state = "WAITING_FOR_NEXT"
    node.mission_run_id = "run-1"
    node.current_point_id = "P0002"
    node.px4_mode = "OFFBOARD"
    node.laser_hold_run_id = "run-1"
    node.laser_hold_point_id = "P0001"
    node.laser_hold_mode = SprayController.LASER_BLINK
    return node


def _values(node):
    return [r.param2 for r in node.command_client.requests]


def _half_period_elapses(node):
    node.laser_blink_next_toggle_at = time.monotonic() - 0.001


def test_blink_hold_starts_blinking_from_idle_with_laser_on(tmp_path):
    node = blink_hold_node(tmp_path)

    node._control_loop()

    assert node.state == SprayController.STATE_LASER_BLINK
    assert _values(node) == [pytest.approx(1.0)]
    assert math.isnan(_last_request(node).param1)
    assert node.press_may_be_active is True
    assert json.loads(node.journal_path.read_text())["state"] == (
        SprayController.JOURNAL_PRESS_COMMAND_SENT
    )
    node._publish_status()
    status = json.loads(node.spray_status_pub.messages[-1].data)
    assert status["laser_blinking"] is True and status["laser_on"] is True
    # A failed point never reports a marking result.
    assert node.spray_result_pub.messages == []


def test_blink_toggles_every_half_period_only_after_each_ack(tmp_path):
    node = blink_hold_node(tmp_path)
    node._control_loop()                      # ON sent

    _half_period_elapses(node)
    node._control_loop()                      # ON not ACKed yet: no new command
    assert _values(node) == [pytest.approx(1.0)]

    _ack_last(node)
    node._control_loop()                      # ACK: next toggle scheduled +0.5 s
    assert node.laser_blink_is_on is True
    assert node.laser_blink_next_toggle_at == pytest.approx(
        node.laser_blink_sent_at + 0.5
    )
    assert len(_values(node)) == 1

    _half_period_elapses(node)
    node._control_loop()                      # OFF
    _ack_last(node)
    node._control_loop()
    _half_period_elapses(node)
    node._control_loop()                      # ON again

    assert _values(node) == [
        pytest.approx(1.0),
        pytest.approx(-1.0),
        pytest.approx(1.0),
    ]
    assert node.state == SprayController.STATE_LASER_BLINK


def test_next_clears_blink_hold_laser_off_and_idle_without_fault(tmp_path):
    node = blink_hold_node(tmp_path)
    node._control_loop()
    _ack_last(node)
    node._control_loop()

    node.laser_hold_run_id = node.laser_hold_point_id = node.laser_hold_mode = None
    node._control_loop()

    assert node.state == SprayController.STATE_WAIT_RELEASE_ACK
    assert _values(node)[-1] == pytest.approx(-1.0)
    _ack_last(node)
    node._control_loop()

    assert node.state == SprayController.STATE_IDLE
    assert node.fault_latched is False
    assert node.press_may_be_active is False
    assert not node.journal_path.exists()
    assert node.spray_result_pub.messages == []

    node._control_loop()  # no hold -> does not start again
    assert node.state == SprayController.STATE_IDLE


def test_estop_during_inflight_toggle_waits_then_turns_off(tmp_path):
    node = blink_hold_node(tmp_path)
    node._control_loop()                      # ON in flight

    node._emergency_stop_callback(SimpleNamespace(data=True))
    assert len(_values(node)) == 1            # never overlaps the ON in flight

    _ack_last(node)
    node._control_loop()

    assert node.state == SprayController.STATE_WAIT_RELEASE_ACK
    assert _values(node)[-1] == pytest.approx(-1.0)


def test_manual_off_during_blink_turns_laser_off(tmp_path):
    node = blink_hold_node(tmp_path)
    node._control_loop()
    _ack_last(node)
    node._control_loop()

    response = node._manual_spray_service(SimpleNamespace(data=False), SimpleNamespace(success=False, message=""))

    assert response.success is True
    assert node.state == SprayController.STATE_WAIT_RELEASE_ACK
    assert _values(node)[-1] == pytest.approx(-1.0)


def test_rejected_toggle_turns_laser_off(tmp_path):
    node = blink_hold_node(tmp_path)
    node._control_loop()
    node.command_client.futures[-1]._done = True
    node.command_client.futures[-1]._result = SimpleNamespace(success=True, result=4)

    node._control_loop()

    assert node.state == SprayController.STATE_WAIT_RELEASE_ACK
    assert _values(node)[-1] == pytest.approx(-1.0)


@pytest.mark.parametrize("change", ["run", "tool", "estop", "disarmed", "steady"])
def test_blink_does_not_start_when_not_requested_or_unsafe(tmp_path, change):
    node = blink_hold_node(tmp_path)
    if change == "run":
        node.laser_hold_run_id = "old-run"
    elif change == "tool":
        node.mission_marking_tool = SprayController.TOOL_SPRAY
    elif change == "estop":
        node.emergency_stop = True
    elif change == "disarmed":
        node.px4_armed = False
    elif change == "steady":
        node.laser_hold_mode = SprayController.LASER_STEADY

    node._control_loop()

    assert node.state == SprayController.STATE_IDLE
    assert node.command_client.requests == []


def test_hold_switching_to_steady_ends_a_blink(tmp_path):
    node = blink_hold_node(tmp_path)
    node._control_loop()
    _ack_last(node)
    node._control_loop()

    node.laser_hold_mode = SprayController.LASER_STEADY
    node._control_loop()

    assert node.state == SprayController.STATE_WAIT_RELEASE_ACK
