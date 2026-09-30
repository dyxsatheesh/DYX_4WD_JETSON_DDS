"""Battery percentage policy shared by telemetry and mission safety."""

from __future__ import annotations

import math

BATTERY_FULL_VOLTAGE_V = 26.50
BATTERY_CRITICAL_VOLTAGE_V = 23.80

BATTERY_FULL_PERCENT = 100
BATTERY_LOW_PERCENT = 30
BATTERY_CRITICAL_PERCENT = 10


def battery_percentage_from_voltage(
    voltage_v: float | None,
) -> int | None:
    """Convert measured whole-pack voltage to a smooth whole percentage.

    Production GCS contract:
      26.50 V and above -> 100%
      23.80 V and below -> 10%
      Between them      -> one continuous linear curve, displayed as whole %

    There is no voltage lookup table. The continuous voltage result is rounded
    to the nearest whole percentage only at the final presentation boundary.

    Endpoint guards are deliberate:
      - voltage below 26.50 V cannot round up to 100%
      - voltage above 23.80 V cannot round down to 10%

    This keeps the 10% mission lockout tied exactly to 23.80 V and below.
    """

    if voltage_v is None or not math.isfinite(voltage_v) or voltage_v <= 0.0:
        return None

    if voltage_v >= BATTERY_FULL_VOLTAGE_V:
        return BATTERY_FULL_PERCENT

    if voltage_v <= BATTERY_CRITICAL_VOLTAGE_V:
        return BATTERY_CRITICAL_PERCENT

    voltage_span = BATTERY_FULL_VOLTAGE_V - BATTERY_CRITICAL_VOLTAGE_V
    percentage_span = BATTERY_FULL_PERCENT - BATTERY_CRITICAL_PERCENT

    continuous_percentage = BATTERY_CRITICAL_PERCENT + (
        (voltage_v - BATTERY_CRITICAL_VOLTAGE_V)
        / voltage_span
        * percentage_span
    )

    whole_percentage = int(round(continuous_percentage))

    # Preserve the exact voltage endpoints despite whole-number rounding.
    return max(
        BATTERY_CRITICAL_PERCENT + 1,
        min(BATTERY_FULL_PERCENT - 1, whole_percentage),
    )


def normalize_battery_percentage(
    raw_percentage: float | None,
) -> float | None:
    """Normalize a percentage value for compatibility with offline/test data.

    Live rover telemetry does not use this function as its percentage source;
    the live callback uses battery_percentage_from_voltage().
    """

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
