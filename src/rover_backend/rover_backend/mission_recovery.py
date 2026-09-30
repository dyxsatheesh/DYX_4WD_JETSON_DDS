"""Durable power-cycle mission recovery checkpoint validation."""

from __future__ import annotations

import json
import logging

from dataclasses import dataclass
from typing import Any

from rover_backend.config import settings
from rover_backend.state import rover_state


LOGGER = logging.getLogger(__name__)

RECOVERABLE_MISSION_STATES = frozenset(
    {"RUNNING", "PAUSED", "WAITING_FOR_NEXT"}
)
TERMINAL_POINT_STATES = frozenset({"COMPLETED", "SKIPPED", "FAILED"})
VALID_POINT_STATES = frozenset(
    {"PENDING", "ACTIVE", "COMPLETED", "SKIPPED", "FAILED"}
)


@dataclass(frozen=True, slots=True)
class PowerCycleRecoveryCandidate:
    mission_id: str
    checksum_sha256: str
    saved_at: str | None
    previous_state: str
    mission_run_id: str
    current_point_id: str
    current_point_number: int
    runtime: dict[str, Any]


def validate_power_cycle_recovery_payload(
    payload: Any,
    metadata: dict[str, Any],
) -> PowerCycleRecoveryCandidate | None:
    """Return a recovery candidate only for the same unfinished mission."""

    if not isinstance(payload, dict):
        return None

    if int(payload.get("schema_version") or 0) != 1:
        return None

    mission_id = str(payload.get("mission_id") or "").strip()
    metadata_mission_id = str(metadata.get("mission_id") or "").strip()
    if not mission_id or mission_id != metadata_mission_id:
        return None

    checksum = str(payload.get("checksum_sha256") or "").strip()
    metadata_checksum = str(metadata.get("checksum_sha256") or "").strip()
    if not checksum or checksum != metadata_checksum:
        return None

    runtime = payload.get("runtime")
    if not isinstance(runtime, dict):
        return None

    previous_state = str(runtime.get("state") or "").strip().upper()
    if previous_state not in RECOVERABLE_MISSION_STATES:
        return None

    mission_run_id = str(runtime.get("mission_run_id") or "").strip()
    if not mission_run_id:
        return None

    try:
        total_points = int(metadata.get("total_points") or 0)
    except (TypeError, ValueError):
        return None

    point_status = runtime.get("point_status")
    if (
        total_points <= 0
        or not isinstance(point_status, list)
        or len(point_status) != total_points
    ):
        return None

    # Optional evidence arrays must either be absent or exactly match the
    # mission point count. Never silently truncate/repair corrupt recovery.
    for field_name in ("point_accuracy_snapshots", "point_results"):
        value = runtime.get(field_name)
        if value is not None and (
            not isinstance(value, list)
            or len(value) != total_points
        ):
            return None

    normalized: list[str] = []
    for value in point_status:
        status = str(value or "PENDING").strip().upper()
        if status not in VALID_POINT_STATES:
            return None
        normalized.append(status)

    unresolved = next(
        (
            number
            for number, status in enumerate(normalized)
            if status not in TERMINAL_POINT_STATES
        ),
        None,
    )
    if unresolved is None:
        return None

    return PowerCycleRecoveryCandidate(
        mission_id=mission_id,
        checksum_sha256=checksum,
        saved_at=(
            str(payload.get("saved_at"))
            if payload.get("saved_at") is not None
            else None
        ),
        previous_state=previous_state,
        mission_run_id=mission_run_id,
        current_point_id=f"P{unresolved + 1:04d}",
        current_point_number=unresolved + 1,
        runtime=dict(runtime),
    )


def load_power_cycle_recovery(
    metadata: dict[str, Any] | None,
) -> PowerCycleRecoveryCandidate | None:
    """Read the durable recovery checkpoint before ROS can publish new state."""

    if metadata is None or not settings.mission_recovery_file.is_file():
        return None

    try:
        payload = json.loads(
            settings.mission_recovery_file.read_text(encoding="utf-8")
        )
    except (OSError, json.JSONDecodeError):
        LOGGER.exception(
            "Unable to read mission power-cycle recovery checkpoint"
        )
        return None

    candidate = validate_power_cycle_recovery_payload(
        payload,
        metadata,
    )
    if candidate is None:
        LOGGER.warning(
            "Ignoring stale or invalid mission recovery checkpoint: %s",
            settings.mission_recovery_file,
        )
    return candidate


def mark_recovery_detected(
    candidate: PowerCycleRecoveryCandidate,
) -> None:
    """Expose a safe, non-moving recovery state to REST/realtime clients."""

    rover_state.update(
        "mission",
        recovery_required=True,
        recovery_available=False,
        recovery_state="DETECTED",
        recovery_reason="POWER_CYCLE",
        recovery_previous_state=candidate.previous_state,
        recovery_current_point_id=candidate.current_point_id,
        recovery_current_point_number=candidate.current_point_number,
        recovery_saved_at=candidate.saved_at,
        recovery_error=None,
        message=(
            "Unfinished mission found after power cycle; "
            "rebuilding trajectory before Resume is offered"
        ),
    )
