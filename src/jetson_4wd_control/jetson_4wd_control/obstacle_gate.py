"""Second, independent obstacle layer for cmd_vel_bridge (pure logic).

mission_manager pauses the mission on a forward obstacle; this gate makes the
bridge hold zero velocity on the same reading even if mission_manager is late
or wedged. Same contract as mission_manager: only a fresh, healthy, enabled
DANGER reading blocks. A stale or unhealthy sensor never blocks (sensor
failure is warning-only, operator decision 2026-09-29).
"""

from __future__ import annotations

from typing import Any, Mapping

OBSTACLE_STATUS_STALE_SEC = 1.0


def obstacle_blocks_motion(
    status: Mapping[str, Any],
    age_sec: float,
    stale_sec: float = OBSTACLE_STATUS_STALE_SEC,
) -> bool:
    return (
        bool(status.get("enabled"))
        and bool(status.get("healthy"))
        and status.get("zone") == "danger"
        and age_sec <= stale_sec
    )
