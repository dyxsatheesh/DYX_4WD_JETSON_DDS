"""Runtime RTK policy (operator-approved 2026-09-26, changed 2026-10-03).

FLOAT stops the rover only once it has lasted float_confirm_sec (default 1 s),
and auto-resumes once FIXED holds fixed_hold_sec (default 0.5 s); DGPS / 3D /
lower fixes, stale GPS status, or FLOAT lasting 120 s need a manual Resume;
a spray press already in progress finishes before the FLOAT pause. Both times
are operator settings saved on the rover.

The mechanism tests pin the pre-2026-10-03 values (no confirmation window,
3 s hold) through LEGACY so their timings stay meaningful; the new defaults
and the settings path have their own tests below.
"""

from __future__ import annotations

import json
import sys
import threading
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from mission_manager.rtk_runtime_policy import (  # noqa: E402
    FloatPauseAction,
    RtkRunningAction,
    RtkRuntimeConfig,
    RtkRuntimePolicy,
)

# Reuse the lifecycle harness's ROS stubs and module import.
import test_lifecycle_command_behavior as harness  # noqa: E402

node_module = harness.node_module
MissionManager = harness.MissionManager


# ----------------------------------------------------------- pure policy
LEGACY = dict(fixed_hold_sec=3.0, float_confirm_sec=0.0)


def policy(**overrides):
    return RtkRuntimePolicy(RtkRuntimeConfig(**{**LEGACY, **overrides}))


def test_config_validation():
    with pytest.raises(ValueError):
        RtkRuntimeConfig(fixed_hold_sec=0.0)
    with pytest.raises(ValueError):
        RtkRuntimeConfig(fixed_hold_sec=5.0, float_manual_after_sec=4.0)
    with pytest.raises(ValueError):
        RtkRuntimeConfig(float_confirm_sec=-0.1)
    with pytest.raises(ValueError):
        RtkRuntimeConfig(float_confirm_sec=float("nan"))


def test_new_defaults_are_one_second_confirm_and_half_second_hold():
    config = RtkRuntimeConfig()
    assert (config.float_confirm_sec, config.fixed_hold_sec) == (1.0, 0.5)
    assert MissionManager.RTK_FLOAT_CONFIRM_SEC == 1.0
    assert MissionManager.RTK_FLOAT_FIXED_HOLD_SEC == 0.5


def test_float_blip_shorter_than_the_window_never_pauses():
    p = RtkRuntimePolicy(RtkRuntimeConfig())
    assert p.running(0.0, 5, 0.1, False) is RtkRunningAction.CONFIRMING_FLOAT
    assert p.running(0.95, 5, 0.1, False) is RtkRunningAction.CONFIRMING_FLOAT
    assert p.float_confirming_sec(0.95) == pytest.approx(0.95)
    assert p.running(0.99, 6, 0.1, False) is RtkRunningAction.CONTINUE
    assert p.float_confirming_sec(0.99) == 0.0
    # A new FLOAT starts a fresh window: the earlier 0.95 s does not count.
    assert p.running(1.5, 5, 0.1, False) is RtkRunningAction.CONFIRMING_FLOAT
    assert p.running(2.4, 5, 0.1, False) is RtkRunningAction.CONFIRMING_FLOAT
    assert p.running(2.5, 5, 0.1, False) is RtkRunningAction.PAUSE_FLOAT


def test_float_lasting_the_window_pauses_and_half_second_fixed_resumes():
    p = RtkRuntimePolicy(RtkRuntimeConfig())
    assert p.running(0.0, 5, 0.1, False) is RtkRunningAction.CONFIRMING_FLOAT
    assert p.running(1.0, 5, 0.1, False) is RtkRunningAction.PAUSE_FLOAT
    assert p.float_paused(1.1, 6, 0.1) is FloatPauseAction.HOLD
    assert p.float_paused(1.59, 6, 0.1) is FloatPauseAction.HOLD
    assert p.float_paused(1.6, 6, 0.1) is FloatPauseAction.AUTO_RESUME


@pytest.mark.parametrize("worse", [4, 3, 0])
def test_worse_than_float_still_pauses_at_once_even_inside_the_window(worse):
    p = RtkRuntimePolicy(RtkRuntimeConfig())
    assert p.running(0.0, 5, 0.1, False) is RtkRunningAction.CONFIRMING_FLOAT
    assert p.running(0.2, worse, 0.1, False) is RtkRunningAction.PAUSE_LOST
    assert p.float_confirming_sec(0.2) == 0.0


def test_stale_gps_inside_the_window_pauses_as_lost():
    p = RtkRuntimePolicy(RtkRuntimeConfig())
    p.running(0.0, 5, 0.1, False)
    assert p.running(0.5, 5, 11.0, False) is RtkRunningAction.PAUSE_LOST


def test_spray_in_progress_is_deferred_only_after_confirmation():
    p = RtkRuntimePolicy(RtkRuntimeConfig())
    assert p.running(0.0, 5, 0.1, True) is RtkRunningAction.CONFIRMING_FLOAT
    assert p.running(1.0, 5, 0.1, True) is (
        RtkRunningAction.DEFER_FLOAT_UNTIL_SPRAY_DONE
    )
    assert p.running(1.2, 5, 0.1, False) is RtkRunningAction.PAUSE_FLOAT


def test_zero_window_restores_stop_on_first_float():
    p = RtkRuntimePolicy(RtkRuntimeConfig(float_confirm_sec=0.0))
    assert p.running(0.0, 5, 0.1, False) is RtkRunningAction.PAUSE_FLOAT


@pytest.mark.parametrize(
    "fix,age,spray,expected",
    [
        (6, 0.1, False, RtkRunningAction.CONTINUE),
        (6, 0.1, True, RtkRunningAction.CONTINUE),
        (5, 0.1, False, RtkRunningAction.PAUSE_FLOAT),
        (5, 0.1, True, RtkRunningAction.DEFER_FLOAT_UNTIL_SPRAY_DONE),
        (4, 0.1, False, RtkRunningAction.PAUSE_LOST),   # DGPS
        (3, 0.1, False, RtkRunningAction.PAUSE_LOST),   # 3D
        (4, 0.1, True, RtkRunningAction.PAUSE_LOST),    # never deferred
        (0, 0.1, False, RtkRunningAction.PAUSE_LOST),
        (8, 0.1, False, RtkRunningAction.PAUSE_LOST),   # PPP is not FIXED
        (6, 11.0, False, RtkRunningAction.PAUSE_LOST),  # stale
        (5, float("inf"), False, RtkRunningAction.PAUSE_LOST),
    ],
)
def test_running_decisions(fix, age, spray, expected):
    assert policy().running(0.0, fix, age, spray) is expected


def test_fixed_must_hold_three_seconds_and_float_restarts_the_hold():
    p = policy()
    assert p.running(0.0, 5, 0.1, False) is RtkRunningAction.PAUSE_FLOAT
    assert p.float_paused(1.0, 6, 0.1) is FloatPauseAction.HOLD
    assert p.float_paused(3.9, 6, 0.1) is FloatPauseAction.HOLD
    assert p.float_paused(4.0, 5, 0.1) is FloatPauseAction.HOLD   # flicker
    assert p.fixed_held_sec(4.0) == 0.0
    assert p.float_paused(5.0, 6, 0.1) is FloatPauseAction.HOLD
    assert p.float_paused(7.9, 6, 0.1) is FloatPauseAction.HOLD
    assert p.float_paused(8.0, 6, 0.1) is FloatPauseAction.AUTO_RESUME


@pytest.mark.parametrize("fix,age", [(4, 0.1), (3, 0.1), (2, 0.1), (0, 0.1), (6, 11.0)])
def test_float_pause_escalates_on_worse_fix_or_stale(fix, age):
    p = policy()
    p.running(0.0, 5, 0.1, False)
    assert p.float_paused(1.0, fix, age) is FloatPauseAction.ESCALATE_LOST


def test_float_pause_times_out_to_manual_after_120_s():
    p = policy()
    p.running(0.0, 5, 0.1, False)
    assert p.float_paused(119.9, 5, 0.1) is FloatPauseAction.HOLD
    assert p.float_paused(120.0, 5, 0.1) is FloatPauseAction.ESCALATE_TIMEOUT


def test_fixed_hold_completing_at_the_timeout_still_resumes():
    p = policy()
    p.running(0.0, 5, 0.1, False)
    p.float_paused(117.0, 6, 0.1)
    assert p.float_paused(120.0, 6, 0.1) is FloatPauseAction.AUTO_RESUME


# ------------------------------------------------------------ node harness
class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def clock(monkeypatch):
    c = Clock()
    monkeypatch.setattr(node_module.time, "monotonic", c)
    return c


class Logger:
    def __init__(self, sink):
        self.sink = sink

    def warn(self, message):
        self.sink.append(("log", message))

    error = info = warn


def build(clock, *, state="RUNNING", fix=6, mode="OFFBOARD", armed=True, config=None):
    manager, events = harness.build_manager(state=state, mode=mode, armed=armed)
    manager._rtk_policy = RtkRuntimePolicy(
        RtkRuntimeConfig(
            float_manual_after_sec=MissionManager.RTK_FLOAT_MANUAL_AFTER_SEC,
            gps_stale_sec=MissionManager.GPS_FIX_STALE_SEC,
            **(LEGACY if config is None else config),
        )
    )
    manager._rtk_float_warning_active = False
    manager._rtk_float_confirm_logged = False
    manager._gps_fix_type = fix
    manager._last_gps_fix_rx_monotonic = clock.t
    manager._last_fcu_rx_monotonic = clock.t
    manager._spray_request_started = None
    manager._rtk_healthy = True
    manager._last_rtk_health_rx_monotonic = clock.t
    manager._last_rtk_age_rx_monotonic = clock.t
    manager._rtk_correction_age_sec = 1.0
    manager._pause_reason = None if state == "RUNNING" else manager._pause_reason
    manager._resume_available = False if state == "RUNNING" else manager._resume_available
    manager._emit_system_event = lambda name, msg: events.append(("event", name, msg))
    manager.get_logger = lambda: Logger(events)
    return manager, events


def fix(manager, clock, fix_type, advance=0.0):
    clock.t += advance
    manager._gps_fix_type = fix_type
    manager._last_gps_fix_rx_monotonic = clock.t
    manager._last_fcu_rx_monotonic = clock.t


def tick(manager):
    MissionManager._monitor_runtime_rtk(manager)
    MissionManager._monitor_pause_recovery(manager)


def event_names(events):
    return [e[1] for e in events if isinstance(e, tuple) and e[0] == "event"]


def test_node_float_stops_the_rover_immediately(clock):
    m, events = build(clock)
    fix(m, clock, 5)
    tick(m)
    assert (m._state, m._pause_reason, m._resume_available) == ("PAUSED", "RTK_FLOAT", False)
    assert "disable_motion" in events and "reset_arrival" in events
    assert not m._mission_enable
    assert event_names(events) == ["RTK_FLOAT"]


def test_node_auto_resumes_after_three_seconds_of_fixed_without_px4_calls(clock):
    m, events = build(clock)
    fix(m, clock, 5)
    tick(m)
    fix(m, clock, 6, advance=0.5)
    tick(m)
    fix(m, clock, 6, advance=2.9)       # 2.9 s held
    tick(m)
    assert m._state == "PAUSED"
    fix(m, clock, 6, advance=0.1)       # 3.0 s held
    tick(m)
    assert (m._state, m._pause_reason) == ("RUNNING", None)
    assert m._mission_enable and "enable_motion" in events
    assert harness.px4_calls(events) == []
    assert event_names(events)[-1] == "RTK_RECOVERED"


def test_node_flicker_restarts_the_hold(clock):
    m, _ = build(clock)
    fix(m, clock, 5)
    tick(m)
    fix(m, clock, 6, advance=1.0)
    tick(m)
    fix(m, clock, 6, advance=2.5)
    tick(m)
    fix(m, clock, 5, advance=0.2)       # back to FLOAT before 3 s
    tick(m)
    fix(m, clock, 6, advance=0.2)
    tick(m)
    fix(m, clock, 6, advance=2.9)
    tick(m)
    assert m._state == "PAUSED"
    fix(m, clock, 6, advance=0.1)
    tick(m)
    assert m._state == "RUNNING"


@pytest.mark.parametrize("worse", [4, 3, 2, 0])
def test_node_worse_fix_while_running_needs_manual_resume(clock, worse):
    m, events = build(clock)
    fix(m, clock, worse)
    tick(m)
    assert (m._state, m._pause_reason) == ("PAUSED", "RTK_LOST")
    assert event_names(events) == ["RTK_PAUSED"]
    fix(m, clock, 6, advance=10.0)
    tick(m)
    assert m._state == "PAUSED"          # never auto-resumes
    assert "enable_motion" not in events


@pytest.mark.parametrize("worse", [4, 3])
def test_node_float_pause_escalates_to_manual_on_dgps_or_3d(clock, worse):
    m, events = build(clock)
    fix(m, clock, 5)
    tick(m)
    fix(m, clock, worse, advance=1.0)
    tick(m)
    assert (m._state, m._pause_reason) == ("PAUSED", "RTK_LOST")
    fix(m, clock, 6, advance=1.0)
    tick(m)
    fix(m, clock, 6, advance=10.0)
    tick(m)
    assert m._state == "PAUSED"
    assert "enable_motion" not in events
    # FIXED back: the existing manual path only OFFERS Resume.
    assert m._resume_available is True
    assert event_names(events) == ["RTK_FLOAT", "RTK_PAUSED", "RTK_RECOVERED"]
    assert "ready to resume" in [e for e in events if isinstance(e, tuple)
                                 and e[0] == "event"][-1][2]


def test_node_stale_gps_while_float_paused_escalates(clock):
    m, _ = build(clock)
    fix(m, clock, 5)
    tick(m)
    clock.t += 11.0                     # no GPSRAW for 11 s
    m._last_fcu_rx_monotonic = clock.t
    tick(m)
    assert m._pause_reason == "RTK_LOST"


def test_node_float_timeout_needs_manual_resume(clock):
    m, events = build(clock)
    fix(m, clock, 5)
    tick(m)
    fix(m, clock, 5, advance=120.0)
    tick(m)
    assert (m._state, m._pause_reason) == ("PAUSED", "RTK_FLOAT_TIMEOUT")
    fix(m, clock, 6, advance=1.0)
    tick(m)
    fix(m, clock, 6, advance=5.0)
    tick(m)
    assert m._state == "PAUSED" and "enable_motion" not in events


@pytest.mark.parametrize("mode,armed", [("MANUAL", True), ("OFFBOARD", False)])
def test_node_auto_resume_refused_if_px4_left_offboard(clock, mode, armed):
    m, events = build(clock)
    fix(m, clock, 5)
    tick(m)
    m._px4_mode, m._px4_armed = mode, armed
    fix(m, clock, 6, advance=1.0)
    tick(m)
    fix(m, clock, 6, advance=3.0)
    tick(m)
    assert (m._state, m._pause_reason) == ("PAUSED", "RTK_FLOAT_MANUAL")
    assert harness.px4_calls(events) == []
    assert "enable_motion" not in events


def test_node_auto_resume_refused_under_estop(clock):
    m, events = build(clock)
    fix(m, clock, 5)
    tick(m)
    m._emergency_stop = True
    fix(m, clock, 6, advance=1.0)
    tick(m)
    fix(m, clock, 6, advance=3.0)
    tick(m)
    assert (m._state, m._pause_reason) == ("PAUSED", "ESTOP")
    assert "enable_motion" not in events


def test_node_float_during_spray_finishes_the_press_then_pauses(clock):
    m, events = build(clock)
    m._spray_request_started = clock.t
    fix(m, clock, 5)
    tick(m)
    assert m._state == "RUNNING"         # press continues
    assert "disable_motion" not in events
    fix(m, clock, 5, advance=0.5)
    tick(m)
    assert m._state == "RUNNING"
    m._spray_request_started = None      # point completed
    fix(m, clock, 5, advance=0.05)
    tick(m)
    assert (m._state, m._pause_reason) == ("PAUSED", "RTK_FLOAT")


def test_node_float_clearing_during_spray_never_pauses(clock):
    m, events = build(clock)
    m._spray_request_started = clock.t
    fix(m, clock, 5)
    tick(m)
    fix(m, clock, 6, advance=0.3)
    tick(m)
    m._spray_request_started = None
    fix(m, clock, 6, advance=0.1)
    tick(m)
    assert m._state == "RUNNING" and "disable_motion" not in events


def test_node_dgps_during_spray_is_not_deferred(clock):
    m, _ = build(clock)
    m._spray_request_started = clock.t
    fix(m, clock, 4)
    tick(m)
    assert (m._state, m._pause_reason) == ("PAUSED", "RTK_LOST")


def test_node_float_pause_never_advertises_manual_resume(clock):
    m, _ = build(clock)
    fix(m, clock, 5)
    tick(m)
    fix(m, clock, 6, advance=1.0)
    tick(m)
    assert m._resume_available is False


# ------------------------------------------------- node: new defaults
DEFAULTS = dict(
    float_confirm_sec=MissionManager.RTK_FLOAT_CONFIRM_SEC,
    fixed_hold_sec=MissionManager.RTK_FLOAT_FIXED_HOLD_SEC,
)


def test_node_float_blip_keeps_the_mission_running(clock):
    m, events = build(clock, config=DEFAULTS)
    fix(m, clock, 5)
    tick(m)
    fix(m, clock, 5, advance=0.9)
    tick(m)
    assert m._state == "RUNNING" and m._pause_reason is None
    assert "disable_motion" not in events and event_names(events) == []
    fix(m, clock, 6, advance=0.05)
    tick(m)
    fix(m, clock, 5, advance=0.5)        # new blip, fresh window
    tick(m)
    fix(m, clock, 5, advance=0.9)
    tick(m)
    assert m._state == "RUNNING"


def test_node_float_for_one_second_pauses_and_resumes_after_half_second(clock):
    m, events = build(clock, config=DEFAULTS)
    fix(m, clock, 5)
    tick(m)
    fix(m, clock, 5, advance=1.0)
    tick(m)
    assert (m._state, m._pause_reason) == ("PAUSED", "RTK_FLOAT")
    assert event_names(events) == ["RTK_FLOAT"]
    fix(m, clock, 6, advance=0.1)
    tick(m)
    fix(m, clock, 6, advance=0.45)
    tick(m)
    assert m._state == "PAUSED"
    fix(m, clock, 6, advance=0.05)       # 0.5 s held
    tick(m)
    assert (m._state, m._pause_reason) == ("RUNNING", None)
    assert harness.px4_calls(events) == []


def test_node_dgps_inside_the_window_still_needs_manual_resume(clock):
    m, _ = build(clock, config=DEFAULTS)
    fix(m, clock, 5)
    tick(m)
    fix(m, clock, 4, advance=0.3)
    tick(m)
    assert (m._state, m._pause_reason) == ("PAUSED", "RTK_LOST")


# ------------------------------------------------- node: saved settings
def settings_manager(clock, tmp_path):
    m, events = build(clock, config=DEFAULTS)
    m.rtk_policy_settings_path = str(tmp_path / "rtk_policy.json")
    m._rtk_policy_source = "default"
    m._rtk_policy_config_request_id = ""
    m._rtk_policy_config_result = ""
    m._rtk_policy_config_reason = ""
    return m, events


class Msg:
    def __init__(self, payload):
        self.data = payload if isinstance(payload, str) else json.dumps(payload)


def send(m, payload):
    MissionManager._rtk_policy_config_callback(m, Msg(payload))
    return MissionManager._rtk_policy_status(m)


def test_settings_are_validated_saved_and_applied_live(clock, tmp_path):
    m, _ = settings_manager(clock, tmp_path)
    status = send(m, {"request_id": "a", "float_confirm_sec": 2.0, "fixed_hold_sec": 1.5})
    assert (status["config_request_id"], status["config_result"]) == ("a", "ACCEPTED")
    assert (status["float_confirm_sec"], status["fixed_hold_sec"]) == (2.0, 1.5)
    assert status["source"] == "saved"
    saved = json.loads((tmp_path / "rtk_policy.json").read_text())
    assert (saved["float_confirm_sec"], saved["fixed_hold_sec"]) == (2.0, 1.5)
    # A partial update keeps the other value.
    status = send(m, {"request_id": "b", "fixed_hold_sec": 0.5})
    assert (status["float_confirm_sec"], status["fixed_hold_sec"]) == (2.0, 0.5)


@pytest.mark.parametrize(
    "payload",
    [
        {"float_confirm_sec": 5.1},
        {"float_confirm_sec": -0.1},
        {"fixed_hold_sec": 0.4},
        {"fixed_hold_sec": 10.5},
        {"fixed_hold_sec": True},
        {"fixed_hold_sec": "1"},
        {"fixed_hold_sec": float("nan")},
        {"other": 1},
        {},
    ],
)
def test_out_of_range_settings_are_rejected_and_nothing_changes(clock, tmp_path, payload):
    m, _ = settings_manager(clock, tmp_path)
    status = send(m, {"request_id": "x", **payload})
    assert (status["config_request_id"], status["config_result"]) == ("x", "REJECTED")
    assert (status["float_confirm_sec"], status["fixed_hold_sec"]) == (1.0, 0.5)
    assert not (tmp_path / "rtk_policy.json").exists()


def test_range_ends_are_accepted(clock, tmp_path):
    m, _ = settings_manager(clock, tmp_path)
    low = send(m, {"float_confirm_sec": 0.0, "fixed_hold_sec": 10.0})
    high = send(m, {"float_confirm_sec": 5.0, "fixed_hold_sec": 0.5})
    assert low["config_result"] == high["config_result"] == "ACCEPTED"


def test_saved_settings_load_at_start_and_bad_file_keeps_defaults(clock, tmp_path):
    m, _ = settings_manager(clock, tmp_path)
    (tmp_path / "rtk_policy.json").write_text(
        json.dumps({"version": 1, "float_confirm_sec": 3.0, "fixed_hold_sec": 2.0})
    )
    MissionManager._load_saved_rtk_policy(m)
    config = m._rtk_policy.config
    assert (config.float_confirm_sec, config.fixed_hold_sec) == (3.0, 2.0)
    assert m._rtk_policy_source == "saved"

    m2, _ = settings_manager(clock, tmp_path)
    (tmp_path / "rtk_policy.json").write_text(
        json.dumps({"float_confirm_sec": 99, "fixed_hold_sec": 2.0})
    )
    MissionManager._load_saved_rtk_policy(m2)
    config = m2._rtk_policy.config
    assert (config.float_confirm_sec, config.fixed_hold_sec) == (1.0, 0.5)
    assert m2._rtk_policy_source == "default"


def test_new_settings_apply_to_a_float_pause_in_progress(clock, tmp_path):
    m, _ = settings_manager(clock, tmp_path)
    fix(m, clock, 5)
    tick(m)
    fix(m, clock, 5, advance=1.0)
    tick(m)
    assert m._pause_reason == "RTK_FLOAT"
    send(m, {"fixed_hold_sec": 2.0})
    fix(m, clock, 6, advance=0.1)
    tick(m)
    fix(m, clock, 6, advance=1.0)
    tick(m)
    assert m._state == "PAUSED"
    fix(m, clock, 6, advance=1.0)        # 2.0 s held
    tick(m)
    assert m._state == "RUNNING"


def test_node_float_window_restarts_after_the_mission_leaves_running(clock):
    m, _ = build(clock, config=DEFAULTS)
    fix(m, clock, 5)
    tick(m)                              # window starts
    m._state = "PAUSED"                  # e.g. operator pause
    fix(m, clock, 5, advance=0.5)
    tick(m)
    m._state = "RUNNING"                 # resumed (on FIXED in real life)
    fix(m, clock, 5, advance=2.0)
    tick(m)
    assert m._state == "RUNNING"         # fresh window, not 2.5 s old
    fix(m, clock, 5, advance=1.0)
    tick(m)
    assert (m._state, m._pause_reason) == ("PAUSED", "RTK_FLOAT")
