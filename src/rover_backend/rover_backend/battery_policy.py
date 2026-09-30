"""Battery percentage policy shared by telemetry and mission safety."""

from __future__ import annotations

import math

BATTERY_LOW_PERCENT = 30.0
BATTERY_CRITICAL_PERCENT = 10.0


def normalize_battery_percentage(
    raw_percentage: float | None,
) -> float | None:
    """Normalize MAVROS BatteryState percentage to 0..100 percent."""

    if raw_percentage is None or not math.isfinite(raw_percentage):
        return None

    if 0.0 <= raw_percentage <= 1.0:
        percentage = raw_percentage * 100.0
    elif 1.0 < raw_percentage <= 100.0:
        percentage = raw_percentage
    else:
        return None

    return round(max(0.0, min(100.0, percentage)), 1)


def classify_battery_percentage(
    percentage: float | None,
) -> str:
    """Return the backend battery percentage status."""

    if percentage is None or not math.isfinite(percentage):
        return "UNKNOWN"

    if percentage <= BATTERY_CRITICAL_PERCENT:
        return "CRITICAL"

    if percentage < BATTERY_LOW_PERCENT:
        return "LOW"

    return "GOOD"


def battery_mission_locked(
    percentage: float | None,
) -> bool:
    """Return True when battery percentage forbids mission operation."""

    return (
        percentage is not None
        and math.isfinite(percentage)
        and percentage <= BATTERY_CRITICAL_PERCENT
    )
