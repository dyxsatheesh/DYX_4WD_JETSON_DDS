"""DYP A21-series (A2111BU) ultrasonic UART frame parser.

Frame: 4 bytes ``[0xFF, high, low, checksum]`` with
``checksum = (0xFF + high + low) & 0xFF`` and distance = ``high << 8 | low``
in millimetres. Same protocol as the 3WD NRP_ROS ``obstacle_monitor.py``.

Special values (DYP A21 datasheet / NRP_ROS ``ultrasonic_watch.py``):

- ``0xFFFE``: interference, no usable distance;
- ``0xFFFD`` and above: no echo, i.e. nothing within range (a *clear* path);
- ``0``: object inside the blind zone, i.e. *very* close.

No-echo and blind-zone frames are real measurements; an interference frame
is not, and does not prove the sensor can see (see zones.py). Pure and
I/O-free.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

HEADER = 0xFF
FRAME_LEN = 4
INTERFERENCE = 0xFFFE
NO_ECHO_MIN = 0xFFFD

KIND_DISTANCE = "distance"
KIND_NO_ECHO = "no_echo"
KIND_BLIND_ZONE = "blind_zone"
KIND_INTERFERENCE = "interference"


@dataclass(frozen=True)
class Reading:
    kind: str
    distance_mm: Optional[int]

    @property
    def usable(self) -> bool:
        """True when the reading says something about the path ahead."""
        return self.kind != KIND_INTERFERENCE


def classify(raw_mm: int) -> Reading:
    if raw_mm == INTERFERENCE:
        return Reading(KIND_INTERFERENCE, None)
    if raw_mm >= NO_ECHO_MIN:
        return Reading(KIND_NO_ECHO, None)
    if raw_mm == 0:
        return Reading(KIND_BLIND_ZONE, 0)
    return Reading(KIND_DISTANCE, raw_mm)


class FrameParser:
    """Incremental parser: feed raw serial bytes, get complete readings."""

    MAX_BUFFER = 256

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.bad_checksums = 0

    def feed(self, data: bytes) -> list[Reading]:
        self._buffer.extend(data)
        if len(self._buffer) > self.MAX_BUFFER:
            del self._buffer[: len(self._buffer) - self.MAX_BUFFER]

        readings: list[Reading] = []
        while len(self._buffer) >= FRAME_LEN:
            if self._buffer[0] != HEADER:
                del self._buffer[0]
                continue
            high, low, checksum = self._buffer[1], self._buffer[2], self._buffer[3]
            if (HEADER + high + low) & 0xFF != checksum:
                self.bad_checksums += 1
                del self._buffer[0]
                continue
            readings.append(classify((high << 8) | low))
            del self._buffer[:FRAME_LEN]
        return readings
