from pathlib import Path

from rover_backend.battery_policy import BATTERY_CRITICAL_DEBOUNCE_SEC
from rover_backend.battery_policy import BATTERY_CRITICAL_PERCENT
from rover_backend.battery_policy import BATTERY_CRITICAL_VOLTAGE_V
from rover_backend.battery_policy import BATTERY_FULL_VOLTAGE_V
from rover_backend.battery_policy import BATTERY_PRECRITICAL_PERCENT
from rover_backend.battery_policy import BatteryCriticalDebounce
from rover_backend.battery_policy import battery_mission_locked
from rover_backend.battery_policy import battery_percentage_from_voltage
from rover_backend.battery_policy import classify_battery_percentage
from rover_backend.battery_policy import debounced_battery_percentage
from rover_backend.battery_policy import normalize_battery_percentage


def test_voltage_maps_to_smooth_whole_percentage():
    assert BATTERY_FULL_VOLTAGE_V == 26.50
    assert BATTERY_CRITICAL_VOLTAGE_V == 22.00

    assert battery_percentage_from_voltage(26.50) == 100
    assert battery_percentage_from_voltage(26.00) == 90
    assert battery_percentage_from_voltage(25.50) == 80
    assert battery_percentage_from_voltage(25.00) == 70
    assert battery_percentage_from_voltage(24.50) == 60
    assert battery_percentage_from_voltage(24.00) == 50
    assert battery_percentage_from_voltage(23.00) == 30
    assert battery_percentage_from_voltage(22.50) == 20
    assert battery_percentage_from_voltage(22.00) == 10


def test_exact_five_second_critical_confirmation():
    assert BATTERY_CRITICAL_DEBOUNCE_SEC == 20.0
    debounce = BatteryCriticalDebounce()

    # 2 Hz stream: 0.5-second samples across the five-second window.
    for step in range(40):
        assert debounce.observe(22.00, step * 0.5) is False

    assert debounce.observe(22.00, 20.0) is True


def test_pending_critical_stays_eleven_percent_low():
    raw = battery_percentage_from_voltage(22.00)
    assert raw == 10
    assert BATTERY_PRECRITICAL_PERCENT == 11
    assert (
        debounced_battery_percentage(
            raw,
            critical_confirmed=False,
        )
        == 11
    )
    assert classify_battery_percentage(11) == "LOW"


def test_recovery_above_22_cancels_pending_or_confirmed():
    debounce = BatteryCriticalDebounce()

    debounce.observe(21.90, 0.0)
    debounce.observe(21.90, 0.5)
    assert debounce.pending is True

    assert debounce.observe(22.01, 2.0) is False
    assert debounce.pending is False

    for step in range(41):
        debounce.observe(21.90, 10.0 + step * 0.5)

    assert debounce.confirmed is True

    assert debounce.observe(22.01, 30.1) is False
    assert debounce.confirmed is False


def test_stale_low_stream_does_not_complete_twenty_seconds():
    debounce = BatteryCriticalDebounce()

    debounce.observe(21.90, 0.0)
    debounce.observe(21.90, 0.5)

    assert debounce.confirm_if_due(20.0) is False
    assert debounce.pending is False


def test_whole_percentage_preserves_exact_voltage_endpoints():
    assert battery_percentage_from_voltage(27.20) == 100
    assert battery_percentage_from_voltage(26.49) == 99

    assert battery_percentage_from_voltage(22.01) == 11
    assert battery_percentage_from_voltage(22.00) == 10
    assert battery_percentage_from_voltage(21.99) == 10
    assert battery_percentage_from_voltage(21.50) == 10
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


def test_critical_battery_uses_dedicated_battery_pause():
    source = (
        Path(__file__).resolve().parents[1] / "rover_backend" / "ros_bridge.py"
    ).read_text()
    worker = source[
        source.index("def _critical_battery_stop_worker") : source.index(
            "def _schedule_critical_battery_stop"
        )
    ]
    assert "ros_bridge.pause_battery_mission()" in worker
    assert "stop_mission()" not in worker
