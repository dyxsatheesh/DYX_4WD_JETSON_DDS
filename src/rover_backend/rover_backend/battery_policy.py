"""Battery percentage policy shared by telemetry and mission safety."""

from __future__ import annotations

import math

BATTERY_FULL_VOLTAGE_V = 26.50
BATTERY_CRITICAL_VOLTAGE_V = 23.80

BATTERY_FULL_PERCENT = 100
BATTERY_LOW_PERCENT = 30
BATTERY_CRITICAL_PERCENT = 10

# Lockout is confirmed, not single-sample: driving load sags the pack, and one
# low sample must not pause a mission. /mavros/battery measured ~0.3 Hz on
# 2026-09-30, so 3 samples spanning >= 8 s is roughly 9-10 s of evidence.
BATTERY_LOCKOUT_CONFIRM_SAMPLES = 3
BATTERY_LOCKOUT_CONFIRM_SEC = 8.0
# Unlock needs a clearly recovered pack (hysteresis above 23.80 V), confirmed.
BATTERY_UNLOCK_VOLTAGE_V = 24.20
BATTERY_UNLOCK_CONFIRM_SAMPLES = 2


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


class BatteryLockoutFilter:
    """Confirmed critical-battery lockout with unlock hysteresis (pure).

    Locks after BATTERY_LOCKOUT_CONFIRM_SAMPLES consecutive samples at or
    below BATTERY_CRITICAL_VOLTAGE_V spanning BATTERY_LOCKOUT_CONFIRM_SEC.
    Unlocks after BATTERY_UNLOCK_CONFIRM_SAMPLES consecutive samples at or
    above BATTERY_UNLOCK_VOLTAGE_V. An invalid sample breaks either streak and
    never changes the lock.
    """

    def __init__(
        self,
        *,
        critical_voltage_v: float = BATTERY_CRITICAL_VOLTAGE_V,
        unlock_voltage_v: float = BATTERY_UNLOCK_VOLTAGE_V,
        confirm_samples: int = BATTERY_LOCKOUT_CONFIRM_SAMPLES,
        confirm_sec: float = BATTERY_LOCKOUT_CONFIRM_SEC,
        unlock_samples: int = BATTERY_UNLOCK_CONFIRM_SAMPLES,
    ) -> None:
        if not unlock_voltage_v > critical_voltage_v:
            raise ValueError("unlock voltage must be above the critical voltage")
        if confirm_samples < 1 or unlock_samples < 1 or confirm_sec < 0.0:
            raise ValueError("confirmation counts must be >= 1 and time >= 0")
        self.critical_voltage_v = float(critical_voltage_v)
        self.unlock_voltage_v = float(unlock_voltage_v)
        self.confirm_samples = int(confirm_samples)
        self.confirm_sec = float(confirm_sec)
        self.unlock_samples = int(unlock_samples)
        self.locked = False
        self._low_count = 0
        self._low_since: float | None = None
        self._high_count = 0

    def _reset_streaks(self) -> None:
        self._low_count = 0
        self._low_since = None
        self._high_count = 0

    def update(self, voltage_v: float | None, now_sec: float) -> bool:
        """Consume one sample; return the (possibly new) lock state."""

        if (
            voltage_v is None
            or not math.isfinite(voltage_v)
            or voltage_v <= 0.0
            or not math.isfinite(now_sec)
        ):
            self._reset_streaks()
            return self.locked

        if not self.locked:
            self._high_count = 0
            if voltage_v <= self.critical_voltage_v:
                if self._low_since is None:
                    self._low_since = now_sec
                self._low_count += 1
                if (
                    self._low_count >= self.confirm_samples
                    and now_sec - self._low_since >= self.confirm_sec
                ):
                    self.locked = True
                    self._reset_streaks()
            else:
                self._low_count = 0
                self._low_since = None
            return self.locked

        self._low_count = 0
        self._low_since = None
        if voltage_v >= self.unlock_voltage_v:
            self._high_count += 1
            if self._high_count >= self.unlock_samples:
                self.locked = False
                self._reset_streaks()
        else:
            self._high_count = 0
        return self.locked
