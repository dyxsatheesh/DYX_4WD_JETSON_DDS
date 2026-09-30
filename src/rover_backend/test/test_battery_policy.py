from rover_backend.battery_policy import BATTERY_CRITICAL_PERCENT
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
