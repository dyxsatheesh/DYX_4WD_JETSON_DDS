"""LASER marking tool and saved mission defaults (MANUAL + LASER).

LASER: at each verified point the laser (AUX6) is turned ON by
spray_controller and held; mission_manager completes the point, publishes
laser_hold and waits for NEXT. NEXT clears the hold and re-enables motion only
after spray_controller reports the laser OFF. LASER is refused with AUTO.
"""

from __future__ import annotations

import json
import time
from types import SimpleNamespace

import pytest

from test_lifecycle_command_behavior import (
    MissionManager,
    Response,
    build_manager,
    build_start_manager,
    node_module,
)
from test_marking_certified_fast_path import (
    RUN_ID,
    build,
    certificate_payload,
    clock,  # noqa: F401  (pytest fixture)
    deliver,
    result_payload,
    spray_success,
)


# ------------------------------------------------------------ completion


def complete_first_point(m) -> None:
    deliver(m, result=result_payload(m), certificate=certificate_payload(m))
    m._control_loop()
    spray_success(m, "P0001")
    m._control_loop()


def test_laser_point_completes_and_holds_laser_until_next(clock):
    m, events = build(clock, execution="MANUAL")
    m._marking_tool = "LASER"

    complete_first_point(m)

    assert m._point_status[0] == "COMPLETED"
    assert m._state == "WAITING_FOR_NEXT"
    assert m._laser_hold == (RUN_ID, "P0001", "STEADY")
    assert "laser ON" in m._last_message
    # The hold is set after motion/marking authority was removed.
    assert "disable_motion" in events


def test_laser_final_point_completes_mission_without_next(clock):
    m, events = build(clock, execution="MANUAL")
    m._marking_tool = "LASER"
    m._point_status = ["COMPLETED", "COMPLETED", "PENDING"]
    m._current_path_index = 2
    from test_marking_certified_fast_path import GOALS

    m._x, m._y = GOALS[2]
    deliver(m, result=result_payload(m), certificate=certificate_payload(m))
    m._control_loop()
    spray_success(m, "P0003")
    m._control_loop()

    assert m._point_status[2] == "COMPLETED"
    # The harness stubs _complete_mission(); the real one asserts the E-stop,
    # which clears any hold and turns the laser OFF.
    assert "mission_complete" in events
    assert m._state != "WAITING_FOR_NEXT"
    assert m._laser_hold is None


def test_laser_failure_does_not_hold(clock):
    m, events = build(clock, execution="MANUAL")
    m._marking_tool = "LASER"
    deliver(m, result=result_payload(m), certificate=certificate_payload(m))
    m._control_loop()
    m._spray_result_callback(
        SimpleNamespace(
            data=json.dumps(
                {
                    "mission_run_id": RUN_ID,
                    "point_id": "P0001",
                    "result": "FAILED",
                    "reason": "PRESS_REJECTED_RESULT_4",
                }
            )
        )
    )
    m._control_loop()

    assert m._laser_hold is None
    assert m._state == "WAITING_FOR_NEXT"


def test_spray_tool_manual_completion_is_unchanged(clock):
    m, events = build(clock, execution="MANUAL")

    complete_first_point(m)

    assert m._laser_hold is None
    assert m._state == "WAITING_FOR_NEXT"
    assert m._last_message.endswith("waiting for NEXT")


# ------------------------------------------------------------ NEXT


def laser_waiting_manager():
    manager, events = build_manager(state="WAITING_FOR_NEXT", execution_mode="MANUAL")
    manager._marking_tool = "LASER"
    manager._laser_hold = ("run-1", "P0001", "STEADY")
    manager._spray_laser_on = True
    manager._spray_status_last_rx_monotonic = time.monotonic() - 0.5
    manager._complete_mission = lambda: events.append("complete_mission")
    return manager, events


@pytest.fixture
def laser_turns_off(monkeypatch):
    """First poll in the NEXT wait: spray_controller reports laser OFF."""

    def fake_sleep(_seconds):
        manager = fake_sleep.manager
        with manager._lock:
            manager._spray_laser_on = False
            manager._spray_status_last_rx_monotonic = time.monotonic() + 1.0
        fake_sleep.events.append("laser_off_confirmed")

    monkeypatch.setattr(node_module.time, "sleep", fake_sleep)
    return fake_sleep


def test_next_turns_laser_off_before_enabling_motion(laser_turns_off):
    manager, events = laser_waiting_manager()
    laser_turns_off.manager, laser_turns_off.events = manager, events

    response = MissionManager._next_point_service(manager, None, Response())

    assert response.success, response.message
    assert manager._laser_hold is None
    assert manager._state == "RUNNING" and manager._mission_enable
    assert events.index("laser_off_confirmed") < events.index("enable_motion")


def test_next_blocked_when_laser_off_not_confirmed(monkeypatch):
    monkeypatch.setattr(node_module.time, "sleep", lambda s: None)
    manager, events = laser_waiting_manager()
    manager.laser_off_confirm_timeout_sec = 0.05

    response = MissionManager._next_point_service(manager, None, Response())

    assert not response.success
    assert "laser OFF not confirmed" in response.message
    assert manager._state == "WAITING_FOR_NEXT"
    assert "enable_motion" not in events
    assert manager._mission_enable is False


def test_stale_status_from_before_next_does_not_count(monkeypatch):
    # Laser already reported OFF, but only by a status from before NEXT.
    monkeypatch.setattr(node_module.time, "sleep", lambda s: None)
    manager, events = laser_waiting_manager()
    manager._spray_laser_on = False
    manager.laser_off_confirm_timeout_sec = 0.05

    response = MissionManager._next_point_service(manager, None, Response())

    assert not response.success
    assert "enable_motion" not in events


def test_next_at_last_laser_point_turns_off_then_completes(laser_turns_off):
    manager, events = laser_waiting_manager()
    manager._next_pending_marking_number = lambda start: None
    laser_turns_off.manager, laser_turns_off.events = manager, events

    response = MissionManager._next_point_service(manager, None, Response())

    assert response.success
    assert "Laser OFF; mission completed" in response.message
    assert events.index("laser_off_confirmed") < events.index("complete_mission")
    assert "enable_motion" not in events


# ------------------------------------------------------------ START / modes


def test_start_refuses_laser_with_auto(monkeypatch):
    monkeypatch.setattr(node_module.time, "sleep", lambda s: None)
    manager, events = build_start_manager()
    manager._marking_tool = "LASER"
    manager._execution_mode = "AUTO"

    response = MissionManager._start_service(manager, None, Response())

    assert not response.success
    assert "LASER marking requires MANUAL" in response.message
    assert not any(str(e).startswith(("set_mode", "arm")) for e in events)


def test_start_allows_laser_with_manual(monkeypatch):
    monkeypatch.setattr(node_module.time, "sleep", lambda s: None)
    manager, events = build_start_manager()
    manager._marking_tool = "LASER"
    manager._execution_mode = "MANUAL"

    response = MissionManager._start_service(manager, None, Response())

    assert response.success, response.message


def test_marking_tool_changes_only_when_not_active():
    manager, events = build_manager(state="READY")
    manager.get_logger = lambda: SimpleNamespace(warn=lambda msg: None)
    manager._marking_tool_callback(SimpleNamespace(data="spray"))
    assert manager._marking_tool == "SPRAY"

    manager._state = "WAITING_FOR_NEXT"
    manager._marking_tool_callback(SimpleNamespace(data="LASER"))
    assert manager._marking_tool == "SPRAY"

    manager._state = "READY"
    manager._marking_tool_callback(SimpleNamespace(data="PAINT"))
    assert manager._marking_tool == "SPRAY"


def defaults_manager(tmp_path):
    manager, events = build_manager(state="READY", execution_mode="MANUAL")
    manager._marking_tool = "LASER"
    manager.mission_defaults_path = str(tmp_path / "defaults.json")
    manager.get_logger = lambda: SimpleNamespace(error=lambda msg: None)
    return manager, events


def test_save_defaults_persists_and_loads_on_restart(tmp_path):
    manager, _ = defaults_manager(tmp_path)
    manager._execution_mode = "AUTO"
    manager._marking_tool = "SPRAY"

    response = MissionManager._save_defaults_service(manager, None, Response())

    assert response.success, response.message
    assert manager._default_execution_mode == "AUTO"
    assert manager._defaults_source == "saved"

    restarted, _ = defaults_manager(tmp_path)
    restarted._default_execution_mode = "MANUAL"
    restarted._default_marking_tool = "LASER"
    restarted._load_saved_defaults()
    assert restarted._default_execution_mode == "AUTO"
    assert restarted._default_marking_tool == "SPRAY"
    assert restarted._defaults_source == "saved"


def test_save_defaults_refuses_laser_with_auto(tmp_path):
    manager, _ = defaults_manager(tmp_path)
    manager._execution_mode = "AUTO"

    response = MissionManager._save_defaults_service(manager, None, Response())

    assert not response.success
    assert not (tmp_path / "defaults.json").exists()


@pytest.mark.parametrize(
    "content",
    ["not json", "{}", '{"execution_mode": "AUTO", "marking_tool": "PAINT"}'],
)
def test_bad_defaults_file_keeps_launch_defaults(tmp_path, content):
    manager, _ = defaults_manager(tmp_path)
    (tmp_path / "defaults.json").write_text(content)

    manager._load_saved_defaults()

    assert manager._default_execution_mode == "MANUAL"
    assert manager._default_marking_tool == "LASER"
    assert manager._defaults_source == "launch"


def test_factory_defaults_are_manual_and_laser():
    assert MissionManager.EXECUTION_MODES[0] == "MANUAL"
    assert MissionManager.MARKING_TOOLS[0] == "LASER"
    source = (
        __import__("pathlib").Path(node_module.__file__).read_text(encoding="utf-8")
    )
    assert 'declare_parameter("default_execution_mode", "MANUAL")' in source
    assert 'declare_parameter("default_marking_tool", "LASER")' in source


# ------------------------------------------------------------ BLINK on a measured miss

import test_raw_gnss_spray_gate as gated  # noqa: E402


def laser_gated(clock, *, point=0):
    m, events, _ = gated.build_gated(clock, execution="MANUAL")
    m._resolve_accuracy_failure = m._real_resolve
    m._store_point_result = lambda **kw: dict(kw)
    m._marking_tool = "LASER"
    if point:
        from test_marking_certified_fast_path import GOALS

        m._point_status = ["FAILED"] * point + ["PENDING"] * (len(GOALS) - point)
        m._current_path_index = point
        m._x, m._y = GOALS[point]
    deliver(m, result=result_payload(m), certificate=certificate_payload(m))
    m._control_loop()
    return m, events


def test_measured_miss_blinks_the_laser_until_next(clock):
    m, events = laser_gated(clock)

    gated.feed(m, clock, 1.1, north_m=0.045)

    assert m._point_status[0] == "FAILED"
    assert m._state == "WAITING_FOR_NEXT"
    assert m._laser_hold == (RUN_ID, "P0001", "BLINK")
    assert "laser BLINKING" in m._last_message
    assert not ("marking_active", True) in events


def test_unmeasurable_stop_keeps_the_laser_off(clock):
    m, events = laser_gated(clock)

    gated.feed(m, clock, 3.2, fix_type=5)  # RTK FLOAT: no trustworthy measurement

    assert m._point_status[0] == "FAILED"
    assert m._laser_hold is None
    assert m._state == "WAITING_FOR_NEXT"


def test_measured_miss_at_final_point_completes_mission_without_next(clock):
    m, events = laser_gated(clock, point=2)
    target = gated.TARGETS[2]

    # gated.feed() keeps delivering certificates for the active point; after
    # the LAST point is decided there is none, so stop feeding right there.
    for i in range(40):
        clock.t += 0.05
        if i % 2 == 0:
            m._gnss_history.append(gated.fix(clock.t, target, 0.0, 0.050))
        deliver(m, certificate=certificate_payload(m))
        m._control_loop()
        if m._point_status[2] not in {"PENDING", "ACTIVE"}:
            break

    assert m._point_status[2] == "FAILED"
    assert "mission_complete" in events
    assert m._state != "WAITING_FOR_NEXT"
    assert m._laser_hold is None


def test_spray_tool_measured_miss_is_unchanged(clock):
    m, events = laser_gated(clock)
    m._marking_tool = "SPRAY"

    gated.feed(m, clock, 1.1, north_m=0.045)

    assert m._laser_hold is None
    assert m._last_message.endswith("waiting for NEXT")
