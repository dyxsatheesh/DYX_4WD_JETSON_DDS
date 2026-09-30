"""Pure validation/planning for power-cycle mission runtime recovery."""

from __future__ import annotations

import copy

from dataclasses import dataclass
from typing import Any


TERMINAL_POINT_STATES = frozenset({"COMPLETED", "SKIPPED", "FAILED"})
VALID_POINT_STATES = frozenset(
    {"PENDING", "ACTIVE", "COMPLETED", "SKIPPED", "FAILED"}
)
VALID_EXECUTION_MODES = frozenset({"AUTO", "MANUAL"})
VALID_MARKING_TOOLS = frozenset({"LASER", "SPRAY"})


@dataclass(frozen=True, slots=True)
class RecoveryPlan:
    mission_run_id: str
    mission_started_unix_ns: int | None
    execution_mode: str
    marking_tool: str
    point_status: tuple[str, ...]
    point_accuracy_snapshots: tuple[Any, ...]
    point_results: tuple[Any, ...]
    last_point_result: dict[str, Any] | None
    unresolved_marking_number: int
    current_path_index: int


def _validated_optional_list(
    value: Any,
    expected_length: int,
    *,
    field_name: str,
) -> list[Any]:
    if value is None:
        return [None] * expected_length
    if not isinstance(value, list) or len(value) != expected_length:
        raise ValueError(
            f"Saved {field_name} count does not match the prepared mission"
        )
    return copy.deepcopy(value)


def build_recovery_plan(
    runtime: dict[str, Any],
    *,
    point_count: int,
    marking_path_index_by_number: list[int],
    semantic_path_indices: list[int],
) -> RecoveryPlan:
    """Validate saved runtime and map it onto a freshly rebuilt path.

    Terminal point results are retained. Any in-progress point is deliberately
    converted back to PENDING so timers, spray state and old controller
    certificates can never survive a power cycle.
    """

    if not isinstance(runtime, dict):
        raise ValueError("Saved mission runtime must be an object")

    mission_run_id = str(runtime.get("mission_run_id") or "").strip()
    if not mission_run_id:
        raise ValueError("Saved mission runtime has no mission_run_id")

    if point_count <= 0:
        raise ValueError("Prepared mission has no marking points")

    raw_status = runtime.get("point_status")
    if not isinstance(raw_status, list) or len(raw_status) != point_count:
        raise ValueError(
            "Saved point_status count does not match the prepared mission"
        )

    normalized_status: list[str] = []
    for raw in raw_status:
        status = str(raw or "PENDING").strip().upper()
        if status not in VALID_POINT_STATES:
            raise ValueError(f"Unsupported saved point state: {status}")
        normalized_status.append(
            status if status in TERMINAL_POINT_STATES else "PENDING"
        )

    unresolved = next(
        (
            index
            for index, status in enumerate(normalized_status)
            if status not in TERMINAL_POINT_STATES
        ),
        None,
    )
    if unresolved is None:
        raise ValueError("Saved mission has no unresolved marking point")

    if len(marking_path_index_by_number) != point_count:
        raise ValueError("Prepared marking-path mapping is incomplete")

    target_path_index = int(marking_path_index_by_number[unresolved])
    if target_path_index < 0:
        raise ValueError("Unresolved point is missing from the prepared path")

    # Restart the transition to the unresolved marking point. When the rebuilt
    # path has a dummy alignment semantic goal before that point, select it
    # instead of skipping directly to the marking point.
    lower_bound = 0
    if unresolved > 0:
        previous_path_index = int(
            marking_path_index_by_number[unresolved - 1]
        )
        lower_bound = previous_path_index + 1

    semantic_candidates = [
        int(path_index)
        for path_index in semantic_path_indices
        if lower_bound <= int(path_index) <= target_path_index
    ]
    current_path_index = (
        semantic_candidates[0]
        if semantic_candidates
        else target_path_index
    )

    accuracy = _validated_optional_list(
        runtime.get("point_accuracy_snapshots"),
        point_count,
        field_name="point_accuracy_snapshots",
    )
    results = _validated_optional_list(
        runtime.get("point_results"),
        point_count,
        field_name="point_results",
    )

    # Never restore partial evidence for an unresolved point.
    for number, status in enumerate(normalized_status):
        if status not in TERMINAL_POINT_STATES:
            accuracy[number] = None
            results[number] = None

    execution_mode = str(
        runtime.get("execution_mode") or "MANUAL"
    ).strip().upper()
    if execution_mode not in VALID_EXECUTION_MODES:
        raise ValueError("Saved execution_mode is invalid")

    marking_tool = str(
        runtime.get("marking_tool") or "LASER"
    ).strip().upper()
    if marking_tool not in VALID_MARKING_TOOLS:
        raise ValueError("Saved marking_tool is invalid")

    started_raw = runtime.get("mission_started_unix_ns")
    mission_started_unix_ns: int | None = None
    if started_raw is not None:
        try:
            candidate = int(started_raw)
        except (TypeError, ValueError) as error:
            raise ValueError(
                "Saved mission_started_unix_ns is invalid"
            ) from error
        if candidate > 0:
            mission_started_unix_ns = candidate

    # Never trust a cached last_point_result from an interrupted ACTIVE
    # transaction. Rebuild it only from a retained terminal point result.
    last_point_result = None
    for number in range(point_count - 1, -1, -1):
        if (
            normalized_status[number] in TERMINAL_POINT_STATES
            and isinstance(results[number], dict)
        ):
            last_point_result = copy.deepcopy(results[number])
            break

    return RecoveryPlan(
        mission_run_id=mission_run_id,
        mission_started_unix_ns=mission_started_unix_ns,
        execution_mode=execution_mode,
        marking_tool=marking_tool,
        point_status=tuple(normalized_status),
        point_accuracy_snapshots=tuple(accuracy),
        point_results=tuple(results),
        last_point_result=last_point_result,
        unresolved_marking_number=unresolved,
        current_path_index=current_path_index,
    )
