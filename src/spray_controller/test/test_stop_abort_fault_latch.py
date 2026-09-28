"""A spray cut short by a mission STOP must not latch a fault.

Field case 2026-09-26 (run f573fb38, P0001): the operator pressed STOP 0.4 s
into a spray. /marking_active, /mission_enable and /emergency_stop dropped
together, PX4 acknowledged the RELEASE, and the controller still latched
MARKING_HOLD_LOST_DURING_SPRAY into its persistent journal. Every later
mission then asked for sprays that were silently refused.
"""

from __future__ import annotations

import importlib
import json
import sys
import types
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


def _install_ros_stubs() -> None:
    class _Anything:
        def __init__(self, *args, **kwargs):
            pass

        def __getattr__(self, name):
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
    module("rclpy.qos", DurabilityPolicy=_Anything(), HistoryPolicy=_Anything(),
           QoSProfile=_Anything, ReliabilityPolicy=_Anything())
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
    def warn(self, *_a, **_k) -> None:
        pass

    def error(self, *_a, **_k) -> None:
        pass


def build(tmp_path, *, emergency_stop: bool, mission_enable: bool, reason: str):
    """A controller whose RELEASE for an aborted spray has just been ACKed."""
    node = object.__new__(SprayController)
    node.journal_path = tmp_path / "journal.json"
    node.journal = None
    node.press_value = -1.0
    node.release_value = 0.090909
    node.spray_duration_sec = 0.2
    node.transaction_run_id = "run-1"
    node.transaction_point_id = "P0001"
    node.transaction_point_index = 0
    node.transaction_kind = SprayController.TRANSACTION_MISSION
    node.manual_requested = False
    node.manual_deadline_at = None
    node.transaction_timing = {}
    node.result_sequence = 0
    node.last_result = None
    node.press_may_be_active = True
    node.release_confirmed = False
    node.fault_latched = False
    node.fault_reason = None
    node.state = SprayController.STATE_WAIT_RELEASE_ACK
    node.failure_after_release = reason
    node.preserve_fault_after_release = False
    node.emergency_stop = emergency_stop
    node.mission_enable = mission_enable
    node.spray_result_pub = _Publisher()
    node.spray_active_pub = _Publisher()
    node.get_logger = lambda: _Logger()
    return node


def journal_state(node) -> str:
    return json.loads(node.journal_path.read_text(encoding="utf-8"))["state"]


@pytest.mark.parametrize(
    "reason",
    sorted(SprayController.STOP_ABORT_REASONS),
)
def test_operator_stop_mid_spray_does_not_latch(tmp_path, reason):
    node = build(tmp_path, emergency_stop=True, mission_enable=False, reason=reason)
    SprayController._release_succeeded(node)

    assert not node.fault_latched and node.fault_reason is None
    assert node.state == SprayController.STATE_WAIT_MARKING_CLEAR
    assert node.release_confirmed and not node.press_may_be_active
    # The point is still reported as not sprayed ...
    assert node.last_result["result"] == "FAILED"
    assert node.last_result["reason"] == reason
    # ... and the journal cannot re-latch the fault on the next start.
    assert journal_state(node) == SprayController.JOURNAL_ABORTED_BY_STOP
    assert journal_state(node) != SprayController.JOURNAL_FAILED_SAFE


def test_mission_disabled_without_estop_counts_as_stop(tmp_path):
    node = build(
        tmp_path,
        emergency_stop=False,
        mission_enable=False,
        reason="MARKING_HOLD_LOST_DURING_SPRAY",
    )
    SprayController._release_succeeded(node)
    assert not node.fault_latched


def test_hold_lost_while_mission_still_running_still_latches(tmp_path):
    node = build(
        tmp_path,
        emergency_stop=False,
        mission_enable=True,
        reason="MARKING_HOLD_LOST_DURING_SPRAY",
    )
    SprayController._release_succeeded(node)

    assert node.fault_latched
    assert node.fault_reason == "MARKING_HOLD_LOST_DURING_SPRAY"
    assert node.state == SprayController.STATE_FAULT
    assert journal_state(node) == SprayController.JOURNAL_FAILED_SAFE


def test_other_failures_still_latch_even_when_stopped(tmp_path):
    node = build(
        tmp_path,
        emergency_stop=True,
        mission_enable=False,
        reason="HARD_PRESS_WATCHDOG_EXPIRED",
    )
    SprayController._release_succeeded(node)
    assert node.fault_latched
    assert journal_state(node) == SprayController.JOURNAL_FAILED_SAFE


def test_aborted_by_stop_journal_does_not_fault_on_restart():
    # Startup only latches on FAILED_SAFE and only recovers the three
    # uncertain press states; the new state must be neither.
    assert SprayController.JOURNAL_ABORTED_BY_STOP not in {
        SprayController.JOURNAL_FAILED_SAFE,
        SprayController.JOURNAL_PRESS_COMMAND_SENT,
        SprayController.JOURNAL_PRESSED,
        SprayController.JOURNAL_RELEASE_UNCONFIRMED,
    }
