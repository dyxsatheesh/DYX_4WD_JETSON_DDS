"""Focused safety tests for bounded manual spray control."""

from __future__ import annotations

import importlib
import json
import sys
import time
import types
from pathlib import Path
from types import SimpleNamespace

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


def _install_ros_stubs() -> None:
    class _Anything:
        def __init__(self, *args, **kwargs):
            pass

        def __getattr__(self, _name):
            return _Anything()

        def __call__(self, *args, **kwargs):
            return _Anything()

    class _Msg:
        pass

    class _Srv:
        Request = _Msg
        Response = _Msg

    def module(name: str, **attrs):
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        sys.modules[name] = mod

    module("rclpy", ok=lambda: True, init=lambda *a, **k: None)
    module("rclpy.node", Node=type("Node", (), {}))
    module(
        "rclpy.qos",
        DurabilityPolicy=_Anything(),
        HistoryPolicy=_Anything(),
        QoSProfile=_Anything,
        ReliabilityPolicy=_Anything(),
    )
    module("mavros_msgs")
    module("mavros_msgs.msg", State=_Msg)
    module("mavros_msgs.srv", CommandLong=_Srv)
    module("std_msgs")
    module("std_msgs.msg", Bool=_Msg, String=_Msg)
    module("std_srvs")
    module("std_srvs.srv", SetBool=_Srv, Trigger=_Srv)


try:
    import rclpy  # noqa: F401
except ImportError:
    _install_ros_stubs()

node_module = importlib.import_module("spray_controller.spray_controller_node")
SprayController = node_module.SprayController


class _Publisher:
    def __init__(self) -> None:
        self.messages: list = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


class _Logger:
    def warn(self, *_args, **_kwargs) -> None:
        pass

    def error(self, *_args, **_kwargs) -> None:
        pass


class _Future:
    def __init__(self, *, done: bool = False, result=None) -> None:
        self._done = done
        self._result = result

    def done(self) -> bool:
        return self._done

    def result(self):
        return self._result


class _CommandClient:
    def __init__(self, *, ready: bool = True) -> None:
        self.ready = ready
        self.requests: list = []
        self.futures: list[_Future] = []

    def service_is_ready(self) -> bool:
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        future = _Future()
        self.futures.append(future)
        return future


def _response() -> SimpleNamespace:
    return SimpleNamespace(success=False, message="")


def _request(value: bool) -> SimpleNamespace:
    return SimpleNamespace(data=value)


def build_node(tmp_path) -> SprayController:
    """Build a controller instance without starting a ROS executor."""
    node = object.__new__(SprayController)
    now = time.monotonic()

    node.enabled = True
    node.press_value = -1.0
    node.release_value = 0.090909
    node.spray_duration_sec = 0.2
    node.pre_spray_stable_sec = 0.05
    node.command_timeout_sec = 1.0
    node.release_retry_interval_sec = 0.25
    node.hard_press_timeout_sec = 5.0
    node.manual_override_timeout_sec = 3.0
    node.mavros_state_timeout_sec = 2.5
    node.mission_status_timeout_sec = 1.0
    node.marking_active_timeout_sec = 0.5
    node.raw_gnss_max_radial_mm = 30.0
    node.require_px4_armed = True
    node.require_px4_offboard = True
    node.require_raw_gnss_spray_gate = True

    node.marking_active = False
    node.mission_enable = False
    node.emergency_stop = False
    node.mavros_connected = True
    node.px4_armed = True
    node.px4_mode = "MANUAL"
    node.mission_state = "READY"
    node.mission_run_id = None
    node.current_point_id = None
    node.current_point_index = None
    node.current_point_state = None
    node.status_marking_active = False
    node.raw_gnss_spray_gate = None
    node.last_marking_active_rx = None
    node.last_mission_status_rx = now
    node.last_mavros_state_rx = now
    node.marking_active_since = None

    node.state = SprayController.STATE_IDLE
    node.transaction_kind = None
    node.transaction_run_id = None
    node.transaction_point_id = None
    node.transaction_point_index = None
    node.transaction_timing = {}
    node.manual_requested = False
    node.manual_deadline_at = None
    node.command_future = None
    node.command_kind = None
    node.command_sent_at = None
    node.press_may_be_active = False
    node.press_command_sent_at = None
    node.spray_started_at = None
    node.failure_after_release = None
    node.next_release_retry_at = 0.0
    node.preserve_fault_after_release = False
    node.release_confirmed = True
    node.fault_latched = False
    node.fault_reason = None
    node.result_sequence = 0
    node.last_result = None
    node._last_completed_replay_key = None
    node._last_config_request_id = None
    node._last_config_result = None
    node._last_config_reason = None

    node.journal_path = tmp_path / "journal.json"
    node.spray_settings_path = tmp_path / "spray_settings.json"
    node.spray_duration_source = "launch"
    node.laser_value_source = "launch"
    node.spray_actuator_set = 1
    node.laser_actuator_set = 2
    node.laser_on_value = 1.0
    node.laser_off_value = -1.0
    node.laser_handoff_timeout_sec = 1.5
    node.mission_marking_tool = SprayController.TOOL_SPRAY
    node.laser_hold_run_id = None
    node.laser_hold_point_id = None
    node.transaction_tool = None
    node.laser_on_since = None
    node.laser_hold_mode = None
    node.laser_blink_half_period_sec = 0.5
    node.laser_blink = False
    node.laser_blink_target = None
    node.laser_blink_is_on = False
    node.laser_blink_sent_at = None
    node.laser_blink_next_toggle_at = 0.0
    node.journal = None
    node.command_client = _CommandClient()
    node.spray_active_pub = _Publisher()
    node.spray_complete_pub = _Publisher()
    node.spray_result_pub = _Publisher()
    node.spray_status_pub = _Publisher()
    node.get_logger = lambda: _Logger()
    return node


@pytest.mark.parametrize(
    ("mutate", "reason"),
    [
        (lambda n: setattr(n, "enabled", False), "SPRAY_CONTROLLER_DISABLED"),
        (lambda n: setattr(n, "fault_latched", True), "SPRAY_FAULT_LATCHED"),
        (
            lambda n: setattr(n, "state", SprayController.STATE_WAIT_MARKING_CLEAR),
            "SPRAY_CONTROLLER_NOT_IDLE",
        ),
        (lambda n: setattr(n, "emergency_stop", True), "EMERGENCY_STOP_ACTIVE"),
        (lambda n: setattr(n, "mission_state", "RUNNING"), "MISSION_ACTIVE"),
        (lambda n: setattr(n, "last_mission_status_rx", None), "MISSION_STATUS_STALE"),
        (lambda n: setattr(n, "mavros_connected", False), "PX4_STATE_UNSAFE"),
        (lambda n: setattr(n, "px4_armed", False), "PX4_STATE_UNSAFE"),
        (
            lambda n: setattr(n.command_client, "ready", False),
            "MAVROS_COMMAND_SERVICE_UNAVAILABLE",
        ),
    ],
)
def test_manual_on_fails_closed_for_each_acceptance_gate(
    tmp_path, mutate, reason
):
    node = build_node(tmp_path)
    mutate(node)

    response = node._manual_spray_service(_request(True), _response())

    assert not response.success
    assert reason in response.message
    assert not node.command_client.requests


def test_manual_on_accepts_without_offboard_or_mission_only_gates(tmp_path):
    node = build_node(tmp_path)
    node.mission_enable = False
    node.marking_active = False
    node.status_marking_active = False
    node.raw_gnss_spray_gate = None
    node.px4_mode = "MANUAL"

    response = node._manual_spray_service(_request(True), _response())

    assert response.success
    assert node.transaction_kind == SprayController.TRANSACTION_MANUAL
    assert node.state == SprayController.STATE_WAIT_PRESS_ACK
    assert node.manual_requested and node.press_may_be_active
    assert node.manual_deadline_at > node.press_command_sent_at
    assert len(node.command_client.requests) == 1


def test_manual_keepalive_does_not_extend_deadline(tmp_path):
    node = build_node(tmp_path)
    node._manual_spray_service(_request(True), _response())
    original_deadline = node.manual_deadline_at

    response = node._manual_spray_service(_request(True), _response())

    assert response.success
    assert "deadline unchanged" in response.message
    assert node.manual_deadline_at == original_deadline
    assert len(node.command_client.requests) == 1


def test_manual_release_never_completes_a_mission_point(tmp_path):
    node = build_node(tmp_path)
    node.transaction_kind = SprayController.TRANSACTION_MANUAL
    node.state = SprayController.STATE_WAIT_RELEASE_ACK
    node.press_may_be_active = True
    node.release_confirmed = False
    node._write_journal(state=SprayController.JOURNAL_RELEASE_UNCONFIRMED)

    node._release_succeeded()

    assert node.state == SprayController.STATE_IDLE
    assert node.release_confirmed and not node.press_may_be_active
    assert not node.spray_complete_pub.messages
    assert not node.spray_result_pub.messages
    assert not node.journal_path.exists()


def test_manual_transaction_uses_press_ack_hold_and_release_ack_fsm(tmp_path):
    node = build_node(tmp_path)
    node._manual_spray_service(_request(True), _response())

    press_future = node.command_future
    press_future._done = True
    press_future._result = SimpleNamespace(success=True, result=0)
    node._control_loop()
    assert node.state == SprayController.STATE_SPRAYING

    node.manual_deadline_at = time.monotonic() - 0.01
    node._control_loop()
    assert node.state == SprayController.STATE_WAIT_RELEASE_ACK

    release_future = node.command_future
    release_future._done = True
    release_future._result = SimpleNamespace(success=True, result=0)
    node._control_loop()

    assert node.state == SprayController.STATE_IDLE
    assert node.release_confirmed and not node.press_may_be_active
    assert not node.spray_complete_pub.messages
    assert not node.spray_result_pub.messages


def test_manual_off_is_idempotent_and_requests_physical_release(tmp_path):
    node = build_node(tmp_path)
    node.release_confirmed = False

    first = node._manual_spray_service(_request(False), _response())
    second = node._manual_spray_service(_request(False), _response())

    assert first.success and second.success
    assert node.state == SprayController.STATE_WAIT_RELEASE_ACK
    assert not node.release_confirmed
    assert len(node.command_client.requests) == 1


def test_off_after_mission_release_preserves_point_journal_and_wait_state(tmp_path):
    node = build_node(tmp_path)
    node.transaction_kind = SprayController.TRANSACTION_MISSION
    node.transaction_run_id = "run-1"
    node.transaction_point_id = "P0001"
    node.state = SprayController.STATE_WAIT_MARKING_CLEAR
    node._write_journal(state=SprayController.JOURNAL_COMPLETED)

    response = node._manual_spray_service(_request(False), _response())

    assert response.success
    assert node.state == SprayController.STATE_WAIT_MARKING_CLEAR
    assert node.transaction_kind == SprayController.TRANSACTION_MISSION
    assert node.journal["state"] == SprayController.JOURNAL_COMPLETED
    assert not node.command_client.requests


def test_manual_off_preserves_latched_mission_fault_and_failed_journal(tmp_path):
    node = build_node(tmp_path)
    node.transaction_kind = SprayController.TRANSACTION_MISSION
    node.transaction_run_id = "run-1"
    node.transaction_point_id = "P0001"
    node.transaction_point_index = 0
    node.release_confirmed = False
    node._set_fault_without_press("MISSION_PRESS_PRECONDITION_FAILED")
    original_journal = node.journal.copy()

    response = node._manual_spray_service(_request(False), _response())
    assert response.success
    assert node.state == SprayController.STATE_WAIT_RELEASE_ACK
    assert node.transaction_kind == SprayController.TRANSACTION_MISSION
    assert node.journal == original_journal

    release_future = node.command_future
    release_future._done = True
    release_future._result = SimpleNamespace(success=True, result=0)
    node._control_loop()

    assert node.state == SprayController.STATE_FAULT
    assert node.fault_latched
    assert node.fault_reason == "MISSION_PRESS_PRECONDITION_FAILED"
    assert node.transaction_kind == SprayController.TRANSACTION_MISSION
    assert node.journal == original_journal
    assert json.loads(node.journal_path.read_text(encoding="utf-8")) == original_journal

    restarted = object.__new__(SprayController)
    restarted.journal_path = node.journal_path
    restarted.get_logger = lambda: _Logger()
    loaded = restarted._load_journal()
    assert loaded["state"] == SprayController.JOURNAL_FAILED_SAFE
    assert loaded["reason"] == "MISSION_PRESS_PRECONDITION_FAILED"


@pytest.mark.parametrize("release_request", ["manual_off", "disable"])
def test_fault_journal_survives_release_timeout_rejection_and_retry(
    tmp_path, release_request
):
    node = build_node(tmp_path)
    node.transaction_kind = SprayController.TRANSACTION_MISSION
    node.transaction_run_id = "run-1"
    node.transaction_point_id = "P0001"
    node.transaction_point_index = 0
    node.release_confirmed = False
    node._set_fault_without_press("MISSION_PRESS_PRECONDITION_FAILED")
    original_journal = node.journal.copy()

    if release_request == "manual_off":
        node._manual_spray_service(_request(False), _response())
    else:
        node._set_enabled_service(_request(False), _response())

    # First RELEASE times out.
    node.command_sent_at = time.monotonic() - node.command_timeout_sec - 0.1
    node._control_loop()
    assert node.state == SprayController.STATE_RECOVERY_RELEASE
    assert node.journal == original_journal

    # Retry is explicitly rejected by PX4.
    node.next_release_retry_at = time.monotonic() - 0.1
    node._control_loop()
    rejected = node.command_future
    rejected._done = True
    rejected._result = SimpleNamespace(success=False, result=4)
    node._control_loop()
    assert node.state == SprayController.STATE_RECOVERY_RELEASE
    assert node.journal == original_journal

    # A later retry succeeds and returns to the original FAULT state.
    node.next_release_retry_at = time.monotonic() - 0.1
    node._control_loop()
    accepted = node.command_future
    accepted._done = True
    accepted._result = SimpleNamespace(success=True, result=0)
    node._control_loop()

    assert node.state == SprayController.STATE_FAULT
    assert node.fault_latched
    assert node.fault_reason == "MISSION_PRESS_PRECONDITION_FAILED"
    assert node.transaction_kind == SprayController.TRANSACTION_MISSION
    assert node.journal == original_journal
    assert json.loads(node.journal_path.read_text(encoding="utf-8")) == original_journal


def test_disable_initiates_release_and_enable_reopens_master_gate(tmp_path):
    node = build_node(tmp_path)
    node._manual_spray_service(_request(True), _response())
    # Model PRESS ACK completion so RELEASE can be sent immediately.
    node.command_future = None

    disabled = node._set_enabled_service(_request(False), _response())

    assert disabled.success and not node.enabled
    assert node.failure_after_release == "CONTROLLER_DISABLED_DURING_SPRAY"
    assert node.state == SprayController.STATE_WAIT_RELEASE_ACK
    assert len(node.command_client.requests) == 2

    enabled = node._set_enabled_service(_request(True), _response())
    assert enabled.success and node.enabled


def test_disable_preserves_latched_mission_fault_and_failed_journal(tmp_path):
    node = build_node(tmp_path)
    node.transaction_kind = SprayController.TRANSACTION_MISSION
    node.transaction_run_id = "run-1"
    node.transaction_point_id = "P0001"
    node.transaction_point_index = 0
    node.release_confirmed = False
    node._set_fault_without_press("MISSION_PRESS_PRECONDITION_FAILED")
    original_journal = node.journal.copy()

    response = node._set_enabled_service(_request(False), _response())

    assert response.success and not node.enabled
    assert node.state == SprayController.STATE_WAIT_RELEASE_ACK
    assert node.preserve_fault_after_release
    assert node.transaction_kind == SprayController.TRANSACTION_MISSION
    assert node.journal == original_journal

    release_future = node.command_future
    release_future._done = True
    release_future._result = SimpleNamespace(success=True, result=0)
    node._control_loop()

    assert node.state == SprayController.STATE_FAULT
    assert node.fault_latched
    assert node.fault_reason == "MISSION_PRESS_PRECONDITION_FAILED"
    assert node.transaction_kind == SprayController.TRANSACTION_MISSION
    assert node.journal == original_journal


def test_manual_release_cleanup_failure_latches_fault_without_escaping(tmp_path):
    node = build_node(tmp_path)
    node.transaction_kind = SprayController.TRANSACTION_MANUAL
    node.state = SprayController.STATE_WAIT_RELEASE_ACK
    node.press_may_be_active = True
    node.release_confirmed = False
    node._write_journal(state=SprayController.JOURNAL_RELEASE_UNCONFIRMED)

    def fail_cleanup() -> None:
        raise OSError("read-only filesystem")

    node._clear_journal = fail_cleanup
    node._release_succeeded()

    assert node.release_confirmed and not node.press_may_be_active
    assert node.state == SprayController.STATE_FAULT
    assert node.fault_latched
    assert node.fault_reason.startswith("MANUAL_JOURNAL_CLEANUP_FAILED:")
    assert node.journal["state"] == SprayController.JOURNAL_FAILED_SAFE
    assert not node.spray_complete_pub.messages
    assert not node.spray_result_pub.messages


def test_manual_release_cleanup_and_fault_persist_failures_do_not_escape(tmp_path):
    node = build_node(tmp_path)
    node.transaction_kind = SprayController.TRANSACTION_MANUAL
    node.state = SprayController.STATE_WAIT_RELEASE_ACK
    node.press_may_be_active = True
    node.release_confirmed = False

    def fail_cleanup() -> None:
        raise OSError("read-only filesystem")

    def fail_persist(*, state, reason=None) -> None:
        raise OSError(f"cannot persist {state}: {reason}")

    node._clear_journal = fail_cleanup
    node._write_journal = fail_persist

    node._release_succeeded()

    assert node.release_confirmed and not node.press_may_be_active
    assert node.state == SprayController.STATE_FAULT
    assert node.fault_latched
    assert node.fault_reason.startswith("MANUAL_JOURNAL_CLEANUP_FAILED:")
    assert not node.spray_complete_pub.messages
    assert not node.spray_result_pub.messages


def test_restarted_manual_recovery_persist_failure_does_not_escape(tmp_path):
    node = build_node(tmp_path)
    node.transaction_kind = SprayController.TRANSACTION_MANUAL
    node.state = SprayController.STATE_WAIT_RELEASE_ACK
    node.press_may_be_active = True
    node.release_confirmed = False
    node.failure_after_release = "NODE_RESTART_DURING_UNCERTAIN_SPRAY"
    node.journal = {
        "version": SprayController.PROTOCOL_VERSION,
        "state": SprayController.JOURNAL_RELEASE_UNCONFIRMED,
        "transaction_kind": SprayController.TRANSACTION_MANUAL,
    }

    def fail_persist(*, state, reason=None) -> None:
        raise OSError(f"read-only filesystem: {state}: {reason}")

    node._write_journal = fail_persist

    node._release_succeeded()

    assert node.release_confirmed and not node.press_may_be_active
    assert node.state == SprayController.STATE_FAULT
    assert node.fault_latched
    assert node.fault_reason == "NODE_RESTART_DURING_UNCERTAIN_SPRAY"
    assert node.transaction_kind == SprayController.TRANSACTION_MANUAL
    assert node.journal["state"] == SprayController.JOURNAL_RELEASE_UNCONFIRMED
    assert not node.spray_complete_pub.messages
    assert not node.spray_result_pub.messages


def test_manual_deadline_initiates_release_before_hard_watchdog(tmp_path):
    node = build_node(tmp_path)
    node.transaction_kind = SprayController.TRANSACTION_MANUAL
    node.state = SprayController.STATE_SPRAYING
    node.manual_requested = True
    node.press_may_be_active = True
    node.press_command_sent_at = time.monotonic() - 3.1
    node.spray_started_at = node.press_command_sent_at
    node.manual_deadline_at = time.monotonic() - 0.1
    node.release_confirmed = False

    node._control_loop()

    assert node.state == SprayController.STATE_WAIT_RELEASE_ACK
    assert not node.manual_requested
    assert node.failure_after_release is None
    assert len(node.command_client.requests) == 1


def test_hard_watchdog_and_estop_abort_manual_spray_safely(tmp_path):
    node = build_node(tmp_path)
    node.transaction_kind = SprayController.TRANSACTION_MANUAL
    node.state = SprayController.STATE_SPRAYING
    node.manual_requested = True
    node.press_may_be_active = True
    node.press_command_sent_at = time.monotonic() - 5.1
    node.spray_started_at = node.press_command_sent_at
    node.manual_deadline_at = time.monotonic() + 1.0
    node.release_confirmed = False

    node._control_loop()
    assert node.failure_after_release == "HARD_PRESS_WATCHDOG_EXPIRED"
    assert node.state == SprayController.STATE_WAIT_RELEASE_ACK

    other = build_node(tmp_path / "other")
    other.transaction_kind = SprayController.TRANSACTION_MANUAL
    other.state = SprayController.STATE_SPRAYING
    other.press_may_be_active = True
    other.command_future = None
    other._emergency_stop_callback(_request(True))
    assert other.failure_after_release == "EMERGENCY_STOP_DURING_SPRAY"
    assert other.state == SprayController.STATE_WAIT_RELEASE_ACK

    release_future = other.command_future
    release_future._done = True
    release_future._result = SimpleNamespace(success=True, result=0)
    other._control_loop()
    assert other.state == SprayController.STATE_FAULT
    assert other.fault_latched
    assert not other.spray_complete_pub.messages
    assert not other.spray_result_pub.messages


def test_mission_only_signal_changes_do_not_abort_manual_spray(tmp_path):
    node = build_node(tmp_path)
    node.transaction_kind = SprayController.TRANSACTION_MANUAL
    node.state = SprayController.STATE_SPRAYING
    node.press_may_be_active = True
    node.mission_enable = True
    node.marking_active = True

    node._mission_enable_callback(_request(False))
    node._marking_active_callback(_request(False))
    status = SimpleNamespace(
        data=json.dumps(
            {
                "state": "READY",
                "marking_active": False,
                "raw_gnss_spray_gate": {"status": "FAIL"},
            }
        )
    )
    node._mission_status_callback(status)

    assert node.state == SprayController.STATE_SPRAYING
    assert node.failure_after_release is None


def test_status_adds_manual_fields(tmp_path):
    node = build_node(tmp_path)
    node.transaction_kind = SprayController.TRANSACTION_MANUAL
    node.manual_requested = True
    node.press_may_be_active = True
    node.manual_deadline_at = time.monotonic() + 2.0
    node.release_confirmed = False

    node._publish_status()
    payload = json.loads(node.spray_status_pub.messages[-1].data)

    assert payload["enabled"] is True
    assert payload["transaction_kind"] == "MANUAL"
    assert payload["manual_active"] is True
    assert payload["manual_requested"] is True
    assert 0.0 < payload["manual_deadline_remaining_sec"] <= 2.0
    assert payload["press_may_be_active"] is True
    assert payload["release_confirmed"] is False


def test_manual_timeout_must_be_strictly_below_hard_timeout(tmp_path):
    node = build_node(tmp_path)
    node.manual_override_timeout_sec = node.hard_press_timeout_sec

    with pytest.raises(ValueError, match="strictly less"):
        node._validate_parameters()


def _duration_msg(request_id: str, value) -> SimpleNamespace:
    return SimpleNamespace(
        data=json.dumps({"request_id": request_id, "spray_duration_sec": value})
    )


def test_duration_config_is_accepted_and_reported_when_idle(tmp_path):
    node = build_node(tmp_path)

    node._spray_duration_config_callback(_duration_msg("req-1", 0.35))

    assert node.spray_duration_sec == pytest.approx(0.35)
    assert node._last_config_request_id == "req-1"
    assert node._last_config_result == "ACCEPTED"
    payload = json.loads(node.spray_status_pub.messages[-1].data)
    assert payload["spray_duration_sec"] == pytest.approx(0.35)
    assert payload["config_request_id"] == "req-1"
    assert payload["config_result"] == "ACCEPTED"


@pytest.mark.parametrize(
    "busy",
    ["mission_enable", "marking_active", "press_may_be_active", "spraying"],
)
def test_duration_config_is_rejected_unless_idle(tmp_path, busy):
    node = build_node(tmp_path)
    if busy == "spraying":
        node.state = SprayController.STATE_SPRAYING
    else:
        setattr(node, busy, True)

    node._spray_duration_config_callback(_duration_msg("req-2", 0.35))

    assert node.spray_duration_sec == pytest.approx(0.2)
    assert node._last_config_result == "REJECTED"
    assert node._last_config_reason == "SPRAY_CONTROLLER_NOT_IDLE"


@pytest.mark.parametrize("value", [0.0, -0.1, 3.01, "abc", None])
def test_duration_config_rejects_out_of_range_or_invalid(tmp_path, value):
    node = build_node(tmp_path)

    node._spray_duration_config_callback(_duration_msg("req-3", value))

    assert node.spray_duration_sec == pytest.approx(0.2)
    assert node._last_config_result == "REJECTED"
    assert node._last_config_reason in {
        "SPRAY_DURATION_OUT_OF_RANGE",
        "INVALID_VALUE",
    }


def test_duration_config_accepts_the_manual_override_limit(tmp_path):
    node = build_node(tmp_path)

    node._spray_duration_config_callback(
        _duration_msg("req-4", node.manual_override_timeout_sec)
    )

    assert node._last_config_result == "ACCEPTED"
    assert node.spray_duration_sec == pytest.approx(node.manual_override_timeout_sec)


def test_mission_hold_uses_the_configured_duration(tmp_path):
    node = build_node(tmp_path)
    node._spray_duration_config_callback(_duration_msg("req-5", 0.35))
    assert node._last_config_result == "ACCEPTED"

    node._spray_gate_ok = lambda _now: True
    node.transaction_kind = SprayController.TRANSACTION_MISSION
    node.state = SprayController.STATE_SPRAYING
    node.press_may_be_active = True
    started = time.monotonic()
    node.press_command_sent_at = started - 0.30
    node.spray_started_at = started - 0.25

    node._control_loop()
    assert node.state == SprayController.STATE_SPRAYING
    assert node.command_client.requests == []

    node.spray_started_at = started - 0.40
    node._control_loop()
    assert node.state == SprayController.STATE_WAIT_RELEASE_ACK
    assert len(node.command_client.requests) == 1


def test_accepted_duration_is_saved_to_the_settings_file(tmp_path):
    node = build_node(tmp_path)

    node._spray_duration_config_callback(_duration_msg("req-6", 0.45))

    saved = json.loads(node.spray_settings_path.read_text())
    assert saved["spray_duration_sec"] == pytest.approx(0.45)
    assert node.spray_duration_source == "saved"
    payload = json.loads(node.spray_status_pub.messages[-1].data)
    assert payload["spray_duration_source"] == "saved"


def test_rejected_duration_is_not_saved(tmp_path):
    node = build_node(tmp_path)
    node.mission_enable = True

    node._spray_duration_config_callback(_duration_msg("req-7", 0.45))

    assert not node.spray_settings_path.exists()
    assert node.spray_duration_source == "launch"


def test_saved_duration_is_applied_on_restart(tmp_path):
    first = build_node(tmp_path)
    first._spray_duration_config_callback(_duration_msg("req-8", 0.45))

    restarted = build_node(tmp_path)
    assert restarted.spray_duration_sec == pytest.approx(0.2)
    restarted._apply_saved_settings()

    assert restarted.spray_duration_sec == pytest.approx(0.45)
    assert restarted.spray_duration_source == "saved"


@pytest.mark.parametrize(
    "content",
    ["not json", "{}", '{"spray_duration_sec": 0}', '{"spray_duration_sec": 9.0}'],
)
def test_bad_settings_file_keeps_launch_value(tmp_path, content):
    node = build_node(tmp_path)
    node.spray_settings_path.write_text(content)

    node._apply_saved_settings()

    assert node.spray_duration_sec == pytest.approx(0.2)
    assert node.spray_duration_source == "launch"


def test_missing_settings_file_keeps_launch_value(tmp_path):
    node = build_node(tmp_path)

    node._apply_saved_settings()

    assert node.spray_duration_sec == pytest.approx(0.2)
    assert node.spray_duration_source == "launch"


def test_duration_is_rejected_and_not_applied_when_save_fails(tmp_path):
    node = build_node(tmp_path)
    blocker = tmp_path / "not_a_dir"
    blocker.write_text("")
    node.spray_settings_path = blocker / "spray_settings.json"

    node._spray_duration_config_callback(_duration_msg("req-9", 0.45))

    assert node.spray_duration_sec == pytest.approx(0.2)
    assert node._last_config_result == "REJECTED"
    assert node._last_config_reason == "SPRAY_SETTINGS_SAVE_FAILED"
