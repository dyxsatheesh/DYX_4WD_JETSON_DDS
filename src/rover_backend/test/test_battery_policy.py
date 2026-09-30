from pathlib import Path

import pytest

from rover_backend.battery_policy import BATTERY_CRITICAL_PERCENT
from rover_backend.battery_policy import BatteryLockoutFilter
from rover_backend.battery_policy import BATTERY_CRITICAL_VOLTAGE_V
from rover_backend.battery_policy import BATTERY_FULL_VOLTAGE_V
from rover_backend.battery_policy import battery_mission_locked
from rover_backend.battery_policy import battery_percentage_from_voltage
from rover_backend.battery_policy import classify_battery_percentage
from rover_backend.battery_policy import normalize_battery_percentage


def test_voltage_maps_to_smooth_whole_percentage():
    assert BATTERY_FULL_VOLTAGE_V == 26.50
    assert BATTERY_CRITICAL_VOLTAGE_V == 23.80

    assert battery_percentage_from_voltage(26.50) == 100
    assert battery_percentage_from_voltage(26.00) == 83
    assert battery_percentage_from_voltage(25.50) == 67
    assert battery_percentage_from_voltage(25.00) == 50
    assert battery_percentage_from_voltage(24.50) == 33
    assert battery_percentage_from_voltage(24.00) == 17
    assert battery_percentage_from_voltage(23.80) == 10


def test_whole_percentage_preserves_exact_voltage_endpoints():
    assert battery_percentage_from_voltage(27.20) == 100
    assert battery_percentage_from_voltage(26.49) == 99
    assert battery_percentage_from_voltage(23.81) == 11
    assert battery_percentage_from_voltage(23.80) == 10
    assert battery_percentage_from_voltage(23.79) == 10
    assert battery_percentage_from_voltage(23.50) == 10
    assert battery_percentage_from_voltage(20.00) == 10
    assert battery_percentage_from_voltage(None) is None
    assert battery_percentage_from_voltage(float("nan")) is None
    assert battery_percentage_from_voltage(0.0) is None


def test_battery_percentage_display_thresholds():
    assert classify_battery_percentage(100.0) == "GOOD"
    assert classify_battery_percentage(30.0) == "GOOD"
    assert classify_battery_percentage(29.0) == "LOW"
    assert classify_battery_percentage(11.0) == "LOW"
    assert classify_battery_percentage(10.0) == "CRITICAL"
    assert classify_battery_percentage(0.0) == "CRITICAL"


def test_critical_battery_locks_mission_at_ten_percent():
    assert BATTERY_CRITICAL_PERCENT == 10
    assert battery_mission_locked(10.1) is False
    assert battery_mission_locked(10.0) is True
    assert battery_mission_locked(5.0) is True
    assert battery_mission_locked(0.0) is True
    assert battery_mission_locked(None) is False


def test_legacy_percentage_normalization_remains_available():
    assert normalize_battery_percentage(1.0) == 100.0
    assert normalize_battery_percentage(0.30) == 30.0
    assert normalize_battery_percentage(0.10) == 10.0
    assert normalize_battery_percentage(42.0) == 42.0
    assert normalize_battery_percentage(-1.0) is None
    assert normalize_battery_percentage(101.0) is None


# ---------------------------------------------------------------- lockout filter


def _feed(lockout, samples, *, start=0.0, step=3.0):
    """Feed (voltage) samples ~3 s apart (the measured /mavros/battery rate)."""
    t = start
    states = []
    for voltage in samples:
        states.append(lockout.update(voltage, t))
        t += step
    return states, t


def test_single_sag_sample_does_not_lock():
    lockout = BatteryLockoutFilter()
    states, _ = _feed(lockout, [23.5, 25.8, 23.6, 25.9])
    assert states == [False, False, False, False]


def test_sustained_critical_voltage_locks_after_confirmation():
    lockout = BatteryLockoutFilter()
    states, _ = _feed(lockout, [23.7, 23.7, 23.7, 23.7])
    # 3 samples span 6 s < 8 s; the 4th (9 s) confirms.
    assert states == [False, False, False, True]


def test_confirmation_needs_both_samples_and_time():
    lockout = BatteryLockoutFilter()
    states, _ = _feed(lockout, [23.7] * 5, step=0.5)      # fast samples, 2 s
    assert not any(states)
    states, _ = _feed(lockout, [23.7], start=8.5)
    assert states == [True]


def test_invalid_sample_breaks_the_streak_and_keeps_the_lock():
    lockout = BatteryLockoutFilter()
    states, _ = _feed(lockout, [23.7, 23.7, None, 23.7, 23.7])
    assert not any(states)
    lockout.locked = True
    assert lockout.update(float("nan"), 100.0) is True


def test_unlock_needs_recovered_voltage_confirmed():
    lockout = BatteryLockoutFilter()
    _feed(lockout, [23.7] * 4)
    assert lockout.locked
    # 24.0 V is above critical but inside the hysteresis band: stays locked.
    states, _ = _feed(lockout, [24.0, 24.1, 24.0], start=20.0)
    assert states == [True, True, True]
    # One recovered sample is not enough; two in a row unlock.
    states, _ = _feed(lockout, [26.2, 24.0, 26.2, 26.3], start=40.0)
    assert states == [True, True, True, False]


def test_filter_rejects_inverted_thresholds():
    with pytest.raises(ValueError):
        BatteryLockoutFilter(critical_voltage_v=24.0, unlock_voltage_v=23.9)


def test_critical_battery_pauses_instead_of_stopping():
    source = (
        Path(__file__).resolve().parents[1] / "rover_backend" / "ros_bridge.py"
    ).read_text()
    worker = source[
        source.index("def _critical_battery_stop_worker"):
        source.index("def _schedule_critical_battery_stop")
    ]
    assert "ros_bridge.pause_mission()" in worker
    assert "stop_mission()" not in worker
    schedule = source[
        source.index("def _schedule_critical_battery_stop"):
        source.index("def _battery_callback")
    ]
    assert 'if mission_state != "RUNNING":' in schedule
    callback = source[
        source.index("def _battery_callback"):
        source.index("def _rtk_health_callback")
    ]
    assert "self._battery_lockout.update(" in callback
