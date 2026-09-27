"""Behavioral tests for START/RESUME/NEXT/STOP command sequencing.

These drive the real MissionManager service methods on an instance built
without ``__init__`` (no ROS graph). Collaborators that talk to PX4, ROS
topics or the RTK gate are replaced with recorders so each test can prove
what was (and was not) called, in which order, and whether motion was ever
enabled. When ROS is not installed the ROS imports are stubbed; the logic
under test does not touch them.
"""

from __future__ import annotations

import importlib
import sys
import threading
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

        def __getattr__(self, name):
            return _Anything()

        def __call__(self, *args, **kwargs):
            return _Anything()

    def module(name: str, **attrs):
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        sys.modules[name] = mod
        return mod

    class _Msg(_Anything):
        pass

    class _Srv:
        Request = _Msg
        Response = _Msg

    module("rclpy", ok=lambda: True, init=lambda *a, **k: None)
    module("rclpy.callback_groups", MutuallyExclusiveCallbackGroup=_Anything,
           ReentrantCallbackGroup=_Anything)
    module("rclpy.executors", MultiThreadedExecutor=_Anything)
    module("rclpy.node", Node=type("Node", (), {}))
    module("rclpy.qos", DurabilityPolicy=_Anything(), HistoryPolicy=_Anything(),
           QoSProfile=_Anything, ReliabilityPolicy=_Anything())
    module("geometry_msgs")
    module("geometry_msgs.msg", PoseStamped=_Msg)
    module("mavros_msgs")
    module("mavros_msgs.msg", GPSRAW=_Msg, PositionTarget=_Msg, State=_Msg)
    module("mavros_msgs.srv", CommandBool=_Srv, CommandLong=_Srv)
    module("nav_msgs")
    module("nav_msgs.msg", Odometry=_Msg, Path=_Msg)
    module("rcl_interfaces")
    module("rcl_interfaces.msg", SetParametersResult=_Msg)
    module("std_msgs")
    module("std_msgs.msg", Bool=_Msg, Float32=_Msg, Int32MultiArray=_Msg,
           String=_Msg, UInt8MultiArray=_Msg)
    module("mission_manager_interfaces")
    module("mission_manager_interfaces.srv", ReleaseEmergencyStop=_Srv)
    module("std_srvs")
    module("std_srvs.srv", Trigger=_Srv)


try:
    import rclpy  # noqa: F401
except ImportError:
    _install_ros_stubs()

node_module = importlib.import_module("mission_manager.mission_manager_node")
MissionManager = node_module.MissionManager


class Response:
    def __init__(self) -> None:
        self.success = False
        self.message = ""


class PreparedPath:
    ready = True


def build_manager(
    *,
    state: str,
    mode: str = "OFFBOARD",
    armed: bool = True,
    connected: bool = True,
    fcu_age_sec: float = 0.1,
    execution_mode: str = "AUTO",
) -> tuple[MissionManager, list]:
    manager = object.__new__(MissionManager)
    events: list = []

    manager._lock = threading.RLock()
    manager._state = state
    manager._execution_mode = execution_mode
    manager._safety_generation = 7
    manager._emergency_stop = False
    manager._mission_enable = False
    manager._fcu_connected = connected
    manager._px4_mode = mode
    manager._px4_armed = armed
    manager._last_fcu_rx_monotonic = time.monotonic() - fcu_age_sec
    manager._pause_reason = "OPERATOR" if state == "PAUSED" else None
    manager._resume_available = state == "PAUSED"
    manager._last_error = None
    manager._last_message = ""
    manager._resume_stage = "IDLE"
    manager._start_stage = "IDLE"
    manager._start_failed_stage = None
    manager._start_debug = {}
    manager._stop_stage = "IDLE"
    manager._navigation_path = [object(), object()]
    manager._pending_prepared_path = PreparedPath()
    manager._mission_waypoints = [object(), object()]
    manager._spray_success_keys = set()
    manager._spray_failure_keys = set()
    manager._command_acks = {"start": 0, "resume": 0, "next_point": 0, "stop": 0}
    # Healthy spray controller unless a test says otherwise.
    manager.spray_required = True
    manager.spray_status_timeout_sec = 2.0
    manager._spray_controller_ready = True
    manager._spray_controller_state = "IDLE"
    manager._spray_fault_reason = None
    manager._spray_status_last_rx_monotonic = time.monotonic()
    manager._spray_status_timestamp_unix_ns = None

    def record(name):
        def recorder(*args, **kwargs):
            events.append(name)
        return recorder

    manager._require_motion_health = lambda **kw: events.append("health")
    manager._disable_motion_preserve_estop = record("disable_motion")
    manager._request_px4_mode = lambda m: events.append(f"set_mode:{m}")
    manager._request_arm = lambda value: events.append(f"arm:{value}")
    manager._wait_for_zero_setpoint_stream = record("zero_stream_live")
    manager._publish_safety = record("publish_safety")
    manager._publish_status = lambda force=False: events.append(
        ("status", manager._state, manager._mission_enable, manager._stop_stage)
    )
    manager._publish_goal = record("publish_goal")
    manager._reset_arrival_state = record("reset_arrival")
    manager._publish_marking_active = record("marking_off")
    manager._next_pending_marking_number = lambda start: 1
    manager._reset_execution_progress = record("reset_progress")
    manager._publish_mission_complete = record("mission_complete")
    manager._publish_runtime_path = record("runtime_path")

    real_enable = MissionManager._enable_motion

    def enable():
        real_enable(manager)
        events.append("enable_motion")

    manager._enable_motion = enable
    return manager, events


@pytest.fixture
def sleeps(monkeypatch):
    calls: list[float] = []
    monkeypatch.setattr(node_module.time, "sleep", lambda s: calls.append(s))
    return calls


def px4_calls(events):
    return [e for e in events if isinstance(e, str) and e.startswith(("set_mode", "arm"))]


# ---------------------------------------------------------------- RESUME


def test_resume_fast_path_no_sleep_no_px4_call(sleeps):
    manager, events = build_manager(state="PAUSED")
    response = MissionManager._resume_service(manager, None, Response())
    assert response.success, response.message
    assert sleeps == []
    assert px4_calls(events) == []
    assert manager._state == "RUNNING" and manager._mission_enable


def test_resume_motion_disabled_until_final_locked_validation(sleeps):
    manager, events = build_manager(state="PAUSED")
    MissionManager._resume_service(manager, None, Response())
    disable = events.index("disable_motion")
    final_health = len(events) - 1 - events[::-1].index("health")
    enable = events.index("enable_motion")
    assert disable < final_health < enable
    assert events.count("health") == 2


@pytest.mark.parametrize(
    "mode,armed,connected,age",
    [
        ("POSCTL", True, True, 0.1),     # not OFFBOARD
        ("OFFBOARD", False, True, 0.1),  # disarmed
        ("OFFBOARD", True, False, 0.1),  # FCU disconnected
        ("OFFBOARD", True, True, 10.0),  # stale MAVROS state
    ],
)
def test_resume_degraded_path_keeps_settle_and_reacquisition(sleeps, mode, armed, connected, age):
    manager, events = build_manager(
        state="PAUSED", mode=mode, armed=armed, connected=connected, fcu_age_sec=age
    )
    MissionManager._resume_service(manager, None, Response())
    assert sleeps == []  # observed-stream wait, no fixed settle
    assert events.index("zero_stream_live") < events.index("set_mode:OFFBOARD")
    if not armed:
        assert "arm:True" in events
        assert events.index("arm:True") < events.index("enable_motion")


def test_resume_rtk_float_cannot_enable_motion(sleeps):
    manager, events = build_manager(state="PAUSED")
    calls = {"n": 0}

    def health(**kw):
        calls["n"] += 1
        if calls["n"] == 2:
            raise RuntimeError("RTK FIXED required: fix_type=5")

    manager._require_motion_health = health
    response = MissionManager._resume_service(manager, None, Response())
    assert not response.success and "RTK" in response.message
    assert "enable_motion" not in events
    assert manager._state == "PAUSED" and not manager._mission_enable


def test_resume_estop_asserted_mid_command_invalidates(sleeps):
    manager, events = build_manager(state="PAUSED")

    def health(**kw):
        if "health" not in events:
            events.append("health")
            manager._emergency_stop = True
            manager._safety_generation += 1
            return
        events.append("health")

    manager._require_motion_health = health
    response = MissionManager._resume_service(manager, None, Response())
    assert not response.success
    assert "enable_motion" not in events and not manager._mission_enable


def test_resume_safety_generation_race_cannot_enable_motion(sleeps):
    manager, events = build_manager(state="PAUSED")
    original = manager._disable_motion_preserve_estop

    def disable_then_race():
        original()
        # A newer hard stop that has already been released again: E-stop is
        # clear, but the generation moved. RESUME must still be invalid.
        manager._safety_generation += 2

    manager._disable_motion_preserve_estop = disable_then_race
    response = MissionManager._resume_service(manager, None, Response())
    assert not response.success and "newer hard-stop" in response.message
    assert "enable_motion" not in events


def test_resume_fast_path_rejects_px4_leaving_offboard_before_commit(sleeps):
    manager, events = build_manager(state="PAUSED")
    real_ready = MissionManager._px4_offboard_armed_fresh_locked
    checks = {"n": 0}

    def ready():
        # First check selects the fast path; PX4 then drops OFFBOARD before
        # the locked FINAL_CHECK re-evaluates it.
        checks["n"] += 1
        if checks["n"] == 2:
            manager._px4_mode = "POSCTL"
        return real_ready(manager)

    manager._px4_offboard_armed_fresh_locked = ready
    response = MissionManager._resume_service(manager, None, Response())
    assert not response.success and "left OFFBOARD" in response.message
    assert "enable_motion" not in events and px4_calls(events) == []


def test_double_resume_second_is_rejected(sleeps):
    manager, events = build_manager(state="PAUSED")
    assert MissionManager._resume_service(manager, None, Response()).success
    second = MissionManager._resume_service(manager, None, Response())
    assert not second.success and second.message == "Mission is not paused"


# ------------------------------------------------------------------ NEXT


def test_next_fast_path_no_sleep_no_px4_call(sleeps):
    manager, events = build_manager(state="WAITING_FOR_NEXT", execution_mode="MANUAL")
    response = MissionManager._next_point_service(manager, None, Response())
    assert response.success, response.message
    assert sleeps == [] and px4_calls(events) == []
    assert manager._state == "RUNNING" and manager._mission_enable
    assert events.index("disable_motion") < events.index("enable_motion")


def test_next_degraded_path_keeps_settle(sleeps):
    manager, events = build_manager(
        state="WAITING_FOR_NEXT", execution_mode="MANUAL", armed=False
    )
    assert MissionManager._next_point_service(manager, None, Response()).success
    assert sleeps == []
    assert events.index("zero_stream_live") < events.index("set_mode:OFFBOARD")
    assert events.index("arm:True") < events.index("enable_motion")


def test_next_only_in_manual_waiting_for_next(sleeps):
    manager, events = build_manager(state="WAITING_FOR_NEXT", execution_mode="AUTO")
    assert not MissionManager._next_point_service(manager, None, Response()).success
    manager, events = build_manager(state="RUNNING", execution_mode="MANUAL")
    response = MissionManager._next_point_service(manager, None, Response())
    assert not response.success and "not waiting for NEXT" in response.message
    assert "enable_motion" not in events


def test_next_rtk_float_and_estop_cannot_enable_motion(sleeps):
    manager, events = build_manager(state="WAITING_FOR_NEXT", execution_mode="MANUAL")
    calls = {"n": 0}

    def health(**kw):
        calls["n"] += 1
        if calls["n"] == 2:
            raise RuntimeError("RTK FIXED required: fix_type=5")

    manager._require_motion_health = health
    assert not MissionManager._next_point_service(manager, None, Response()).success
    assert manager._state == "WAITING_FOR_NEXT" and "enable_motion" not in events

    manager, events = build_manager(state="WAITING_FOR_NEXT", execution_mode="MANUAL")

    def health_estop(**kw):
        manager._emergency_stop = True
        manager._safety_generation += 1
        manager._require_motion_health = lambda **k: None

    manager._require_motion_health = health_estop
    assert not MissionManager._next_point_service(manager, None, Response()).success
    assert "enable_motion" not in events


# ----------------------------------------------------------------- START


def build_start_manager():
    manager, events = build_manager(state="READY", mode="POSCTL", armed=False)
    manager._assert_emergency_stop = lambda: (
        events.append("assert_estop"),
        setattr(manager, "_emergency_stop", True),
        setattr(manager, "_safety_generation", manager._safety_generation + 1),
    )
    manager._best_effort_px4_disarm_only = lambda: (events.append("disarm"), (True, []))[1]
    return manager, events


def test_start_waits_on_observed_zero_stream_not_a_fixed_sleep(sleeps):
    manager, events = build_start_manager()
    response = MissionManager._start_service(manager, None, Response())
    assert response.success, response.message
    assert sleeps == []
    assert (
        events.index("disable_motion")
        < events.index("zero_stream_live")
        < events.index("set_mode:OFFBOARD")
        < events.index("arm:True")
        < events.index("enable_motion")
    )


def test_start_fails_before_offboard_when_zero_stream_not_live(sleeps):
    manager, events = build_start_manager()

    def not_live():
        raise RuntimeError("Zero setpoint stream not live within 1.0s")

    manager._wait_for_zero_setpoint_stream = not_live
    response = MissionManager._start_service(manager, None, Response())
    assert not response.success and "Zero setpoint stream" in response.message
    assert px4_calls(events) == [] and "enable_motion" not in events


def test_start_offboard_rejection_never_arms(sleeps):
    manager, events = build_start_manager()

    def reject(mode):
        events.append(f"set_mode:{mode}")
        raise RuntimeError("PX4 rejected mode OFFBOARD")

    manager._request_px4_mode = reject
    response = MissionManager._start_service(manager, None, Response())
    assert not response.success
    assert "arm:True" not in events and "enable_motion" not in events


def test_start_arm_rejection_never_enables_motion(sleeps):
    manager, events = build_start_manager()

    def reject(value):
        events.append(f"arm:{value}")
        raise RuntimeError("PX4 rejected arm")

    manager._request_arm = reject
    response = MissionManager._start_service(manager, None, Response())
    assert not response.success and "enable_motion" not in events
    assert manager._state == "READY"


def test_start_estop_during_offboard_request_prevents_arm(sleeps):
    manager, events = build_start_manager()

    def offboard_then_estop(mode):
        events.append(f"set_mode:{mode}")
        manager._emergency_stop = True
        manager._safety_generation += 1

    manager._request_px4_mode = offboard_then_estop
    response = MissionManager._start_service(manager, None, Response())
    assert not response.success and "before ARM" in response.message
    assert "arm:True" not in events and "enable_motion" not in events


def test_start_estop_during_arm_request_invalidates(sleeps):
    manager, events = build_start_manager()

    def arm_then_estop(value):
        events.append(f"arm:{value}")
        manager._emergency_stop = True
        manager._safety_generation += 1

    manager._request_arm = arm_then_estop
    response = MissionManager._start_service(manager, None, Response())
    assert not response.success and "enable_motion" not in events


def test_start_estop_during_settle_invalidates(sleeps):
    manager, events = build_start_manager()

    def settle():
        manager._emergency_stop = True
        manager._safety_generation += 1

    manager._wait_for_zero_setpoint_stream = settle
    response = MissionManager._start_service(manager, None, Response())
    assert not response.success
    assert "arm:True" not in events and "enable_motion" not in events


# ------------------------------------------------------------------ STOP


def build_stop_manager(disarm_confirmed: bool):
    manager, events = build_manager(state="RUNNING")
    manager._mission_enable = True
    manager._reset_point_timers = lambda: events.append("reset_timers")
    manager._emit_system_event = lambda *a, **k: events.append("system_event")
    manager._emit_mission_terminated = lambda **k: events.append("terminated")
    manager._clear_after_stop = lambda **k: events.append("clear_runtime")

    def disarm():
        events.append(("disarm", manager._stop_stage, manager._emergency_stop, manager._mission_enable))
        return disarm_confirmed, [] if disarm_confirmed else ["no confirmation"]

    manager._best_effort_px4_disarm_only = disarm
    manager._assert_emergency_stop = lambda: MissionManager._assert_emergency_stop(manager)
    return manager, events


def test_stop_asserts_hard_stop_before_disarm_and_reports_progress(sleeps):
    manager, events = build_stop_manager(disarm_confirmed=True)
    terminated, _ = MissionManager._execute_stop_cleanup(manager, completed=False, source="OPERATOR")
    assert terminated
    statuses = [e for e in events if isinstance(e, tuple) and e[0] == "status"]
    stages = [s[3] for s in statuses]
    assert stages[:2] == ["HARD_STOP_ASSERTED", "DISARMING"]
    # Motion was already disabled in the very first STOP status.
    assert statuses[0][2] is False
    disarm = next(e for e in events if isinstance(e, tuple) and e[0] == "disarm")
    assert disarm == ("disarm", "DISARMING", True, False)
    assert manager._stop_stage == "DISARM_CONFIRMED"
    assert events.index("clear_runtime") > events.index(disarm)


def test_stop_disarm_failure_keeps_hard_stop_and_runtime(sleeps):
    manager, events = build_stop_manager(disarm_confirmed=False)
    terminated, _ = MissionManager._execute_stop_cleanup(manager, completed=False, source="OPERATOR")
    assert not terminated
    assert manager._stop_stage == "DISARM_FAILED"
    assert manager._state == "ERROR"
    assert manager._emergency_stop is True and manager._mission_enable is False
    assert "clear_runtime" not in events and "terminated" not in events


# ------------------------------------------------------------ ACK WRAPPER


def test_command_ack_only_on_success_and_published_before_return():
    manager, events = build_manager(state="PAUSED")

    def ok(request, response):
        response.success = True
        return response

    def fail(request, response):
        response.success = False
        return response

    MissionManager._acknowledged_trigger(manager, "resume", fail)(None, Response())
    assert manager._command_acks["resume"] == 0 and events == []
    MissionManager._acknowledged_trigger(manager, "resume", ok)(None, Response())
    assert manager._command_acks["resume"] == 1
    assert events and events[-1][0] == "status"


# ------------------------------------------------ spray readiness at START/RESUME


def latch_spray_fault(manager):
    manager._spray_controller_ready = False
    manager._spray_controller_state = "FAULT"
    manager._spray_fault_reason = "MARKING_HOLD_LOST_DURING_SPRAY"


def test_start_refused_while_spray_fault_latched(sleeps):
    manager, events = build_start_manager()
    latch_spray_fault(manager)
    response = MissionManager._start_service(manager, None, Response())
    assert not response.success
    assert "MARKING_HOLD_LOST_DURING_SPRAY" in response.message
    assert "/spray/reset_fault" in response.message
    assert px4_calls(events) == [] and "enable_motion" not in events
    assert manager._state == "READY"


def test_start_refused_when_spray_status_stale(sleeps):
    manager, events = build_start_manager()
    manager._spray_status_last_rx_monotonic = time.monotonic() - 10.0
    response = MissionManager._start_service(manager, None, Response())
    assert not response.success and "not received" in response.message
    assert px4_calls(events) == [] and "enable_motion" not in events


def test_start_not_gated_on_spray_when_spray_not_required(sleeps):
    manager, _events = build_start_manager()
    latch_spray_fault(manager)
    manager.spray_required = False
    response = MissionManager._start_service(manager, None, Response())
    assert response.success, response.message


def test_resume_refused_while_spray_fault_latched(sleeps):
    manager, events = build_manager(state="PAUSED")
    latch_spray_fault(manager)
    response = MissionManager._resume_service(manager, None, Response())
    assert not response.success
    assert "MARKING_HOLD_LOST_DURING_SPRAY" in response.message
    assert manager._state == "PAUSED" and "enable_motion" not in events


# ------------------------------------------- OFFBOARD must be PX4-ACK confirmed


def mode_request_manager(*, success: bool, result: int = 0):
    manager = object.__new__(MissionManager)
    manager._command_client = object()
    calls: list = []

    def call(client, request, label):
        calls.append((client, request, label))
        return SimpleNamespace(success=success, result=result)

    manager._call_service = call
    return manager, calls


def test_offboard_uses_acked_do_set_mode_not_fire_and_forget_set_mode():
    manager, calls = mode_request_manager(success=True)
    MissionManager._request_px4_mode(manager, "OFFBOARD")
    [(client, request, _label)] = calls
    assert client is manager._command_client
    assert request.command == 176  # MAV_CMD_DO_SET_MODE
    assert request.param1 == 1.0  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    assert request.param2 == 6.0  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
    assert request.param3 == 0.0 and request.confirmation == 0
    assert request.broadcast is False
    source = Path(node_module.__file__).read_text(encoding="utf-8")
    assert 'SetMode, "/mavros/set_mode"' not in source


@pytest.mark.parametrize("result", [1, 2, 4])  # TEMP_REJECTED, DENIED, FAILED
def test_offboard_not_accepted_raises(result):
    manager, _calls = mode_request_manager(success=False, result=result)
    with pytest.raises(RuntimeError, match="did not accept mode OFFBOARD"):
        MissionManager._request_px4_mode(manager, "OFFBOARD")


def test_only_offboard_is_ever_requested():
    manager, calls = mode_request_manager(success=True)
    with pytest.raises(RuntimeError):
        MissionManager._request_px4_mode(manager, "MANUAL")
    assert calls == []


def test_start_requests_offboard_even_when_heartbeat_already_says_offboard(sleeps):
    # After an automatic STOP PX4 is left disarmed in OFFBOARD; that ~1 s old
    # HEARTBEAT must not let START skip the ACK-confirmed request and ARM.
    manager, events = build_start_manager()
    manager._px4_mode = "OFFBOARD"
    response = MissionManager._start_service(manager, None, Response())
    assert response.success, response.message
    assert events.index("set_mode:OFFBOARD") < events.index("arm:True")


def test_degraded_resume_requests_offboard_even_when_heartbeat_says_offboard(sleeps):
    manager, events = build_manager(state="PAUSED", mode="OFFBOARD", armed=False)
    response = MissionManager._resume_service(manager, None, Response())
    assert response.success, response.message
    assert events.index("set_mode:OFFBOARD") < events.index("arm:True")


# --------------------------------------------- observed zero setpoint stream


class StreamClock:
    def __init__(self) -> None:
        self.t = 1000.0

    def monotonic(self) -> float:
        return self.t


def stream_manager(monkeypatch):
    clock = StreamClock()
    monkeypatch.setattr(node_module.time, "monotonic", clock.monotonic)
    manager = object.__new__(MissionManager)
    manager._setpoint_stream_cond = threading.Condition()
    manager._last_setpoint_rx_monotonic = None
    manager._zero_setpoint_stream_since = None
    return manager, clock


def setpoint(vx=0.0, vy=0.0, vz=0.0):
    return SimpleNamespace(velocity=SimpleNamespace(x=vx, y=vy, z=vz))


def feed_stream(manager, clock, seconds, *, period=0.025, **velocity):
    for _ in range(int(round(seconds / period))):
        clock.t += period
        MissionManager._setpoint_stream_callback(manager, setpoint(**velocity))


def live(manager, clock):
    return MissionManager._zero_setpoint_stream_live(manager, clock.t)


def test_zero_stream_live_after_continuous_window(monkeypatch):
    manager, clock = stream_manager(monkeypatch)
    feed_stream(manager, clock, 0.25)
    assert not live(manager, clock)
    feed_stream(manager, clock, 0.10)
    assert live(manager, clock)


def test_nonzero_setpoint_restarts_the_window(monkeypatch):
    manager, clock = stream_manager(monkeypatch)
    feed_stream(manager, clock, 0.5)
    feed_stream(manager, clock, 0.025, vx=0.3)
    assert not live(manager, clock)
    feed_stream(manager, clock, 0.25)
    assert not live(manager, clock)
    feed_stream(manager, clock, 0.10)
    assert live(manager, clock)


def test_gap_restarts_the_window(monkeypatch):
    manager, clock = stream_manager(monkeypatch)
    feed_stream(manager, clock, 0.5)
    clock.t += MissionManager.ZERO_STREAM_MAX_GAP_SEC + 0.01
    feed_stream(manager, clock, 0.1)
    assert not live(manager, clock)


def test_measured_worst_gap_does_not_break_a_live_stream_for_long(monkeypatch):
    # 2026-09-26 bags: p99 gap 67-76 ms stays live throughout.
    manager, clock = stream_manager(monkeypatch)
    feed_stream(manager, clock, 0.5, period=0.075)
    assert live(manager, clock)


def test_stale_stream_is_not_live(monkeypatch):
    manager, clock = stream_manager(monkeypatch)
    feed_stream(manager, clock, 0.5)
    clock.t += MissionManager.ZERO_STREAM_FRESH_SEC + 0.01
    assert not live(manager, clock)


def test_wait_returns_immediately_when_already_live(monkeypatch):
    manager, clock = stream_manager(monkeypatch)
    feed_stream(manager, clock, 0.5)
    MissionManager._wait_for_zero_setpoint_stream(manager)


def test_wait_times_out_without_a_stream(monkeypatch):
    manager, _clock = stream_manager(monkeypatch)
    monkeypatch.setattr(node_module.time, "monotonic", time.perf_counter)
    manager.ZERO_STREAM_TIMEOUT_SEC = 0.05
    with pytest.raises(RuntimeError, match="last setpoint never"):
        MissionManager._wait_for_zero_setpoint_stream(manager)
