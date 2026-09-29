"""Forward-obstacle stop-and-wait in mission_manager.

Danger (fresh, healthy, enabled) pauses a RUNNING mission with reason
OBSTACLE, manual resume only, and blocks START/RESUME/NEXT through the motion
health gate. A stale/unhealthy sensor is a warning only: it never pauses.
"""

from __future__ import annotations

import json
import time
from types import SimpleNamespace

from test_marking_certified_fast_path import build, clock  # noqa: F401
from test_lifecycle_command_behavior import MissionManager


def status(**overrides):
    payload = {
        "schema": "dyx4wd/obstacle@1",
        "enabled": True,
        "healthy": True,
        "zone": "danger",
        "distance_mm": 1200,
        "danger_mm": 1500,
        "warning_mm": 3000,
    }
    payload.update(overrides)
    return SimpleNamespace(data=json.dumps(payload))


def running_manager(clock):
    m, events = build(clock, execution="MANUAL")
    m._obstacle_status = {}
    m._obstacle_status_rx_monotonic = None
    m._obstacle_sensor_warned = False
    return m, events


def test_danger_pauses_running_mission_immediately(clock):
    m, events = running_manager(clock)

    m._obstacle_status_callback(status())

    assert m._state == "PAUSED"
    assert m._pause_reason == "OBSTACLE"
    assert m._resume_available is False
    assert "disable_motion" in events
    assert ("system", "OBSTACLE_PAUSED") in events
    assert "120 cm" in m._last_message


def test_warning_and_clear_do_not_pause(clock):
    m, events = running_manager(clock)
    m._obstacle_status_callback(status(zone="warning", distance_mm=2500))
    m._obstacle_status_callback(status(zone="clear", distance_mm=None))
    assert m._state == "RUNNING"


def test_disabled_sensor_never_pauses(clock):
    m, _ = running_manager(clock)
    m._obstacle_status_callback(status(enabled=False))
    assert m._state == "RUNNING"


def test_unhealthy_sensor_is_warning_only_and_warned_once(clock):
    m, events = running_manager(clock)
    m._obstacle_status_callback(status(healthy=False, zone="unknown", distance_mm=None))
    m._obstacle_status_callback(status(healthy=False, zone="unknown", distance_mm=None))
    assert m._state == "RUNNING"
    assert events.count(("system", "OBSTACLE_SENSOR_LOST")) == 1


def test_stale_danger_status_is_ignored(clock):
    m, _ = running_manager(clock)
    m._obstacle_status = json.loads(status().data)
    m._obstacle_status_rx_monotonic = time.monotonic() - 5.0
    m._monitor_runtime_obstacle()
    assert m._state == "RUNNING"


def test_not_running_does_not_pause(clock):
    m, _ = running_manager(clock)
    m._state = "WAITING_FOR_NEXT"
    m._obstacle_status_callback(status())
    assert m._state == "WAITING_FOR_NEXT"


def gate_manager(obstacle):
    m = object.__new__(MissionManager)
    m._navigation_path = [object()]
    m._pending_prepared_path = SimpleNamespace(ready=True)
    m._state = "READY"
    m._emergency_stop = False
    m._rtk_motion_ok = lambda: (True, "fixed")
    m._obstacle_status = json.loads(status(**obstacle).data)
    m._obstacle_status_rx_monotonic = time.monotonic()
    return m


def test_motion_gate_blocks_start_resume_next_while_danger():
    ok, reason = gate_manager({})._motion_health_status(require_ready_state=True)
    assert not ok and "Obstacle ahead at 120 cm" in reason


def test_motion_gate_ignores_clear_disabled_or_dead_sensor():
    for obstacle in (
        {"zone": "clear", "distance_mm": None},
        {"zone": "warning", "distance_mm": 2500},
        {"enabled": False},
        {"healthy": False, "zone": "unknown"},
    ):
        ok, _ = gate_manager(obstacle)._motion_health_status(require_ready_state=True)
        assert ok, obstacle


def test_resume_offered_only_after_path_clears(clock):
    m, events = running_manager(clock)
    m._obstacle_status_callback(status())
    m._navigation_path = [object()]
    m._pending_prepared_path = SimpleNamespace(ready=True)
    m._rtk_motion_ok = lambda: (True, "fixed")
    recover = MissionManager._monitor_pause_recovery  # harness stubs it out

    recover(m)
    assert m._resume_available is False

    m._obstacle_status_callback(status(zone="clear", distance_mm=None))
    recover(m)
    assert m._resume_available is True
    assert m._state == "PAUSED"  # never auto-resumes
    assert ("system", "OBSTACLE_CLEARED") in events
