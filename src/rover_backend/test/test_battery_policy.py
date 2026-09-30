from rover_backend.battery_policy import BATTERY_CRITICAL_PERCENT
from rover_backend.battery_policy import battery_mission_locked
from rover_backend.battery_policy import classify_battery_percentage
from rover_backend.battery_policy import normalize_battery_percentage


def test_battery_percentage_display_thresholds():
    assert classify_battery_percentage(100.0) == "GOOD"
    assert classify_battery_percentage(30.0) == "GOOD"
    assert classify_battery_percentage(29.0) == "LOW"
    assert classify_battery_percentage(11.0) == "LOW"
    assert classify_battery_percentage(10.0) == "CRITICAL"
    assert classify_battery_percentage(0.0) == "CRITICAL"


def test_critical_battery_locks_mission_at_ten_percent():
    assert BATTERY_CRITICAL_PERCENT == 10.0
    assert battery_mission_locked(10.1) is False
    assert battery_mission_locked(10.0) is True
    assert battery_mission_locked(5.0) is True
    assert battery_mission_locked(0.0) is True
    assert battery_mission_locked(None) is False


def test_mavros_percentage_normalization():
    assert normalize_battery_percentage(1.0) == 100.0
    assert normalize_battery_percentage(0.30) == 30.0
    assert normalize_battery_percentage(0.10) == 10.0
    assert normalize_battery_percentage(42.0) == 42.0
    assert normalize_battery_percentage(-1.0) is None
    assert normalize_battery_percentage(101.0) is None
