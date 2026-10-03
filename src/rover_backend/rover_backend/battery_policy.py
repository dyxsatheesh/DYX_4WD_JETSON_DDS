"""Battery percentage policy shared by telemetry and mission safety."""

from __future__ import annotations

import math

BATTERY_FULL_VOLTAGE_V = 26.50
BATTERY_CRITICAL_VOLTAGE_V = 22.00

BATTERY_FULL_PERCENT = 100
BATTERY_LOW_PERCENT = 30
BATTERY_PRECRITICAL_PERCENT = 11
BATTERY_CRITICAL_PERCENT = 10

BATTERY_CRITICAL_DEBOUNCE_SEC = 30.0
# BATTERY_STATUS is requested at 2 Hz. A 1.5 s freshness limit tolerates a
# small scheduling delay while preventing stale telemetry from completing
# the 30-second critical window.
BATTERY_CRITICAL_SAMPLE_FRESH_SEC = 1.5


def battery_percentage_from_voltage(
    voltage_v: float | None,
) -> int | None:
    """Convert measured whole-pack voltage to a smooth whole percentage."""

    if voltage_v is None or not math.isfinite(voltage_v) or voltage_v <= 0.0:
        return None

    if voltage_v >= BATTERY_FULL_VOLTAGE_V:
        return BATTERY_FULL_PERCENT

    if voltage_v <= BATTERY_CRITICAL_VOLTAGE_V:
        return BATTERY_CRITICAL_PERCENT

    voltage_span = BATTERY_FULL_VOLTAGE_V - BATTERY_CRITICAL_VOLTAGE_V
    percentage_span = BATTERY_FULL_PERCENT - BATTERY_CRITICAL_PERCENT

    continuous_percentage = BATTERY_CRITICAL_PERCENT + (
        (voltage_v - BATTERY_CRITICAL_VOLTAGE_V) / voltage_span * percentage_span
    )
    whole_percentage = int(round(continuous_percentage))

    return max(
        BATTERY_CRITICAL_PERCENT + 1,
        min(BATTERY_FULL_PERCENT - 1, whole_percentage),
    )


def debounced_battery_percentage(
    raw_percentage: int | None,
    *,
    critical_confirmed: bool,
) -> int | None:
    """Show 11%/LOW while <=22.00 V is still inside the five-second window."""

    if raw_percentage is None:
        return None

    if raw_percentage <= BATTERY_CRITICAL_PERCENT and not critical_confirmed:
        return BATTERY_PRECRITICAL_PERCENT

    return raw_percentage


def normalize_battery_percentage(
    raw_percentage: float | None,
) -> float | None:
    """Normalize a percentage value for compatibility with offline/test data."""

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


class BatteryCriticalDebounce:
    """Confirm <=22.00 V continuously for exactly 30 seconds.

    A valid voltage above 22.00 V cancels both a pending and a confirmed
    critical condition. Invalid/stale telemetry cannot create a critical lock.
    """

    def __init__(
        self,
        *,
        critical_voltage_v: float = BATTERY_CRITICAL_VOLTAGE_V,
        debounce_sec: float = BATTERY_CRITICAL_DEBOUNCE_SEC,
        sample_fresh_sec: float = BATTERY_CRITICAL_SAMPLE_FRESH_SEC,
    ) -> None:
        if debounce_sec <= 0.0 or sample_fresh_sec <= 0.0:
            raise ValueError("battery debounce timing must be > 0")

        self.critical_voltage_v = float(critical_voltage_v)
        self.debounce_sec = float(debounce_sec)
        self.sample_fresh_sec = float(sample_fresh_sec)

        self.confirmed = False
        self.low_since_sec: float | None = None
        self.last_low_sample_sec: float | None = None
        self.low_sample_count = 0

    @property
    def pending(self) -> bool:
        return self.low_since_sec is not None and not self.confirmed

    def reset(self) -> None:
        self.confirmed = False
        self.low_since_sec = None
        self.last_low_sample_sec = None
        self.low_sample_count = 0

    def reset_pending(self) -> None:
        self.low_since_sec = None
        self.last_low_sample_sec = None
        self.low_sample_count = 0

    def remaining_sec(self, now_sec: float) -> float | None:
        if self.low_since_sec is None or self.confirmed:
            return None

        return max(
            0.0,
            self.debounce_sec - (float(now_sec) - self.low_since_sec),
        )

    def observe(self, voltage_v: float | None, now_sec: float) -> bool:
        if not math.isfinite(now_sec):
            return self.confirmed

        if voltage_v is None or not math.isfinite(voltage_v) or voltage_v <= 0.0:
            if not self.confirmed:
                self.reset_pending()
            return self.confirmed

        voltage = float(voltage_v)
        now = float(now_sec)

        if voltage > self.critical_voltage_v:
            self.reset()
            return False

        if self.confirmed:
            self.last_low_sample_sec = now
            return True

        if self.low_since_sec is None:
            self.low_since_sec = now

        self.last_low_sample_sec = now
        self.low_sample_count += 1
        return self.confirm_if_due(now)

    def confirm_if_due(self, now_sec: float) -> bool:
        if self.confirmed:
            return True

        if (
            self.low_since_sec is None
            or self.last_low_sample_sec is None
            or self.low_sample_count < 2
            or not math.isfinite(now_sec)
        ):
            return False

        now = float(now_sec)

        if now - self.low_since_sec < self.debounce_sec:
            return False

        if now - self.last_low_sample_sec > self.sample_fresh_sec:
            self.reset_pending()
            return False

        self.confirmed = True
        return True
