"""Obstacle zone tracking: debounce, exit hysteresis, sensor health.

Zones (configurable, applied immediately, unlike NRP_ROS whose zone settings
were stored but never used):

- ``danger``  distance <= danger_mm            -> mission_manager pauses
- ``warning`` danger_mm < distance <= warning_mm -> announcement only
- ``clear``   farther, or no echo in range
- ``unknown`` sensor unhealthy (no usable reading recently)

A zone becomes confirmed after ``debounce_count`` consecutive usable
readings in it, except that a reading at or inside ``immediate_danger_mm``
(or the blind zone) confirms *danger* at once: close in, the ~200 ms a
three-reading debounce costs matters more than a nuisance pause. Leaving
*danger* additionally requires the distance to be beyond
``danger_mm + hysteresis_mm`` so an object at the boundary cannot make the
state flicker.

Interference frames carry no distance and do not count as the sensor being
alive: a sensor that reports only interference is blind and goes ``unknown``
after ``health_timeout_sec``. Pure logic; the node supplies readings and the
clock.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from obstacle_monitor.a21_protocol import KIND_BLIND_ZONE, KIND_NO_ECHO, Reading

ZONE_CLEAR = "clear"
ZONE_WARNING = "warning"
ZONE_DANGER = "danger"
ZONE_UNKNOWN = "unknown"

ZONE_RANK = {ZONE_CLEAR: 0, ZONE_WARNING: 1, ZONE_DANGER: 2}


@dataclass(frozen=True)
class ZoneConfig:
    danger_mm: int = 1500
    warning_mm: int = 3000
    debounce_count: int = 3
    hysteresis_mm: int = 200
    health_timeout_sec: float = 0.5
    immediate_danger_mm: int = 800

    # A21 measuring range is roughly 0.25-7.5 m depending on the variant.
    MIN_DANGER_MM = 300
    MAX_WARNING_MM = 7500

    def validate(self) -> None:
        if not self.MIN_DANGER_MM <= self.danger_mm < self.warning_mm <= self.MAX_WARNING_MM:
            raise ValueError(
                f"need {self.MIN_DANGER_MM} <= danger_mm < warning_mm <= "
                f"{self.MAX_WARNING_MM} (got danger={self.danger_mm}, "
                f"warning={self.warning_mm})"
            )
        if self.debounce_count < 1:
            raise ValueError("debounce_count must be >= 1")
        if self.hysteresis_mm < 0:
            raise ValueError("hysteresis_mm must be >= 0")
        if self.health_timeout_sec <= 0:
            raise ValueError("health_timeout_sec must be > 0")
        if self.immediate_danger_mm < 0:
            raise ValueError("immediate_danger_mm must be >= 0")


class ZoneTracker:
    def __init__(self, config: ZoneConfig) -> None:
        config.validate()
        self.config = config
        self.confirmed_zone = ZONE_CLEAR
        self.distance_mm: Optional[int] = None
        self.last_reading_at: Optional[float] = None
        self.readings = 0
        self.interference = 0
        self._candidate = ZONE_CLEAR
        self._candidate_count = 0

    def reconfigure(self, config: ZoneConfig) -> None:
        config.validate()
        self.config = config
        self._candidate_count = 0

    def raw_zone(self, reading: Reading) -> str:
        if reading.kind == KIND_NO_ECHO:
            return ZONE_CLEAR
        if reading.kind == KIND_BLIND_ZONE:
            return ZONE_DANGER  # inside the blind zone: something is very close
        distance = reading.distance_mm or 0
        if self.confirmed_zone == ZONE_DANGER and distance <= (
            self.config.danger_mm + self.config.hysteresis_mm
        ):
            return ZONE_DANGER
        if distance <= self.config.danger_mm:
            return ZONE_DANGER
        if distance <= self.config.warning_mm:
            return ZONE_WARNING
        return ZONE_CLEAR

    def update(self, reading: Reading, now: float) -> bool:
        """Apply one reading. Returns True when the confirmed zone changed."""
        if not reading.usable:
            # Interference: no distance, so no evidence about the path. Only
            # usable readings keep the sensor healthy.
            self.interference += 1
            return False
        self.last_reading_at = now
        self.readings += 1
        self.distance_mm = reading.distance_mm
        zone = self.raw_zone(reading)
        if zone == self._candidate:
            self._candidate_count += 1
        else:
            self._candidate = zone
            self._candidate_count = 1
        needed = 1 if self._immediate_danger(reading) else self.config.debounce_count
        if self._candidate_count >= needed and zone != self.confirmed_zone:
            self.confirmed_zone = zone
            return True
        return False

    def _immediate_danger(self, reading: Reading) -> bool:
        if reading.kind == KIND_BLIND_ZONE:
            return True
        limit = min(self.config.immediate_danger_mm, self.config.danger_mm)
        return reading.distance_mm is not None and reading.distance_mm <= limit

    def healthy(self, now: float) -> bool:
        return (
            self.last_reading_at is not None
            and now - self.last_reading_at <= self.config.health_timeout_sec
        )

    def zone(self, now: float) -> str:
        return self.confirmed_zone if self.healthy(now) else ZONE_UNKNOWN
