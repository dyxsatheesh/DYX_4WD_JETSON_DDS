"""Pure mission-topology compiler for the DYX 4WD marking rover.

No ROS, MAVROS, RTK, odometry, current rover pose, or live PX4 state
is required here.

For GPS missions, surveyed P1 is used only as a deterministic temporary
projection origin for static topology classification. Final runtime path
coordinates are still produced later in the actual PX4 local frame.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from trajectory_generator.localization_frame import GeographicOrigin
from trajectory_generator.localization_frame import project_geodetic_to_px4_enu


Point = tuple[float, float]


@dataclass(frozen=True)
class SegmentTopology:
    index: int
    distance_m: float
    use_dummy: bool
    incoming_direction: Point | None
    transfer_angle_deg: float | None
    reversal_angle_deg: float | None
    # The rule would have added an extension here, but the operator skipped
    # this row end.
    skipped_by_operator: bool = False


@dataclass(frozen=True)
class MissionTopology:
    metric_points: tuple[Point, ...]
    segments: tuple[SegmentTopology, ...]


def _finite_point(
    point: Sequence[float],
    *,
    label: str,
) -> Point:
    if len(point) != 2:
        raise ValueError(f"{label} must contain exactly two values")

    first = float(point[0])
    second = float(point[1])

    if not (math.isfinite(first) and math.isfinite(second)):
        raise ValueError(f"{label} must contain finite values")

    return first, second


def _distance(first: Point, second: Point) -> float:
    return math.hypot(
        second[0] - first[0],
        second[1] - first[1],
    )


def _angle_between_vectors_deg(
    first: Point,
    second: Point,
) -> float:
    first_length = math.hypot(first[0], first[1])
    second_length = math.hypot(second[0], second[1])

    if first_length <= 0.0 or second_length <= 0.0:
        raise ValueError(
            "Cannot calculate extension angle from zero vector"
        )

    cosine = (
        first[0] * second[0]
        + first[1] * second[1]
    ) / (
        first_length * second_length
    )

    cosine = max(-1.0, min(1.0, cosine))

    return math.degrees(math.acos(cosine))


def _evaluate_row_transition(
    *,
    marking_points: Sequence[Point],
    index: int,
    row_transfer_min_angle_deg: float,
    row_transfer_max_angle_deg: float,
    row_reversal_min_angle_deg: float,
) -> tuple[
    bool,
    Point | None,
    float | None,
    float | None,
]:
    following_index = index + 2

    if following_index >= len(marking_points):
        return False, None, None, None

    current = marking_points[index]
    next_marking = marking_points[index + 1]
    following = marking_points[following_index]

    transfer = (
        next_marking[0] - current[0],
        next_marking[1] - current[1],
    )

    outgoing = (
        following[0] - next_marking[0],
        following[1] - next_marking[1],
    )

    if index > 0:
        previous = marking_points[index - 1]
        incoming = (
            current[0] - previous[0],
            current[1] - previous[1],
        )
    else:
        incoming = (
            -outgoing[0],
            -outgoing[1],
        )

    transfer_angle = _angle_between_vectors_deg(
        incoming,
        transfer,
    )

    reversal_angle = _angle_between_vectors_deg(
        incoming,
        outgoing,
    )

    use_dummy = (
        row_transfer_min_angle_deg
        <= transfer_angle
        <= row_transfer_max_angle_deg
        and reversal_angle
        >= row_reversal_min_angle_deg
    )

    return (
        use_dummy,
        incoming,
        transfer_angle,
        reversal_angle,
    )


def compile_metric_topology(
    marking_points: Sequence[Sequence[float]],
    *,
    extension_mode: str,
    row_transition_threshold_m: float | None,
    minimum_segment_length_m: float,
    row_transfer_min_angle_deg: float,
    row_transfer_max_angle_deg: float,
    row_reversal_min_angle_deg: float,
    skip_row_end_indices: frozenset[int] = frozenset(),
) -> MissionTopology:
    points = tuple(
        _finite_point(
            point,
            label=f"marking point {index + 1}",
        )
        for index, point in enumerate(marking_points)
    )

    if len(points) < 2:
        raise ValueError(
            "Mission requires at least two marking points"
        )

    mode = str(extension_mode).strip().upper()

    if mode not in {"ENABLE", "DISABLE"}:
        raise ValueError(
            f"Unsupported extension_mode: {extension_mode!r}"
        )

    segments: list[SegmentTopology] = []

    for index in range(len(points) - 1):
        current = points[index]
        next_marking = points[index + 1]

        transition_distance = _distance(
            current,
            next_marking,
        )

        if (
            not math.isfinite(transition_distance)
            or transition_distance
            < minimum_segment_length_m
        ):
            raise ValueError(
                "Consecutive marking points "
                f"{index + 1} and "
                f"{index + 2} are too close"
            )

        candidate = (
            mode == "ENABLE"
            and row_transition_threshold_m is not None
            and transition_distance
            < row_transition_threshold_m
        )

        use_dummy = False
        incoming_direction = None
        transfer_angle = None
        reversal_angle = None

        if candidate:
            (
                use_dummy,
                incoming_direction,
                transfer_angle,
                reversal_angle,
            ) = _evaluate_row_transition(
                marking_points=points,
                index=index,
                row_transfer_min_angle_deg=(
                    row_transfer_min_angle_deg
                ),
                row_transfer_max_angle_deg=(
                    row_transfer_max_angle_deg
                ),
                row_reversal_min_angle_deg=(
                    row_reversal_min_angle_deg
                ),
            )

        skipped_by_operator = use_dummy and index in skip_row_end_indices
        if skipped_by_operator:
            use_dummy = False

        segments.append(
            SegmentTopology(
                index=index,
                distance_m=transition_distance,
                use_dummy=use_dummy,
                incoming_direction=incoming_direction,
                transfer_angle_deg=transfer_angle,
                reversal_angle_deg=reversal_angle,
                skipped_by_operator=skipped_by_operator,
            )
        )

    return MissionTopology(
        metric_points=points,
        segments=tuple(segments),
    )


# Row-end decision codes reported to the operator (Upgrade Extension phase 2).
DECISION_ADDED = "ADDED"
DECISION_SKIPPED_GAP = "SKIPPED_GAP"
DECISION_SKIPPED_OPERATOR = "SKIPPED_OPERATOR"
DECISION_DISABLED = "DISABLED"

# Warn (never block) when an extension leg passes this close to another
# marking point (operator decision 2026-10-04).
CLEARANCE_WARNING_M = 0.30


@dataclass(frozen=True)
class ClearanceWarning:
    """Another marking point lies near an extension leg."""

    point_index: int
    distance_m: float


@dataclass(frozen=True)
class RowEndDecision:
    """Why one marking-point pair did or did not get an extension point.

    ``from_index`` is the zero-based index of the row end Pn; the pair is
    Pn -> Pn+1. ``point`` is the extension point in the topology's metric
    frame, only for ADDED.
    """

    from_index: int
    gap_m: float
    transfer_angle_deg: float
    reversal_angle_deg: float
    decision: str
    point: Point | None
    warnings: tuple[ClearanceWarning, ...] = ()


def _distance_to_segment(point: Point, start: Point, end: Point) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= 0.0:
        return _distance(point, start)
    t = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared
    t = max(0.0, min(1.0, t))
    return _distance(point, (start[0] + t * dx, start[1] + t * dy))


class _PointGrid:
    """Uniform grid over the marking points for near-segment queries."""

    def __init__(self, points: Sequence[Point], cell_m: float) -> None:
        self._cell = cell_m
        self._points = points
        self._cells: dict[tuple[int, int], list[int]] = {}
        for index, point in enumerate(points):
            self._cells.setdefault(self._key(point), []).append(index)

    def _key(self, point: Point) -> tuple[int, int]:
        return (math.floor(point[0] / self._cell), math.floor(point[1] / self._cell))

    def near_segment(
        self, start: Point, end: Point, radius_m: float
    ) -> dict[int, float]:
        low = self._key((min(start[0], end[0]) - radius_m, min(start[1], end[1]) - radius_m))
        high = self._key((max(start[0], end[0]) + radius_m, max(start[1], end[1]) + radius_m))
        found: dict[int, float] = {}
        for cell_x in range(low[0], high[0] + 1):
            for cell_y in range(low[1], high[1] + 1):
                for index in self._cells.get((cell_x, cell_y), ()):
                    distance = _distance_to_segment(self._points[index], start, end)
                    if distance < radius_m:
                        found[index] = distance
        return found


def _clearance_warnings(
    grid: _PointGrid,
    *,
    row_end_index: int,
    extension_point: Point,
    points: Sequence[Point],
    clearance_m: float,
) -> tuple[ClearanceWarning, ...]:
    """Marking points (other than the row end and the next row start) near
    the legs row end -> extension point -> next row start."""

    own = {row_end_index, row_end_index + 1}
    nearest: dict[int, float] = {}
    for start, end in (
        (points[row_end_index], extension_point),
        (extension_point, points[row_end_index + 1]),
    ):
        for index, distance in grid.near_segment(start, end, clearance_m).items():
            if index in own:
                continue
            nearest[index] = min(distance, nearest.get(index, distance))
    return tuple(
        ClearanceWarning(point_index=index, distance_m=distance)
        for index, distance in sorted(nearest.items())
    )


def unused_skip_indices(
    topology: MissionTopology,
    skip_row_end_indices: frozenset[int],
) -> tuple[int, ...]:
    """Skipped row ends that no longer match a row end that would extend
    (for example after the trigger distance changed). They have no effect."""

    used = {segment.index for segment in topology.segments if segment.skipped_by_operator}
    return tuple(sorted(skip_row_end_indices - used))


def row_end_decisions(
    topology: MissionTopology,
    *,
    extension_mode: str,
    row_transition_threshold_m: float,
    dummy_point_distance_m: float | None,
    row_transfer_min_angle_deg: float,
    row_transfer_max_angle_deg: float,
    row_reversal_min_angle_deg: float,
    clearance_m: float = CLEARANCE_WARNING_M,
) -> tuple[RowEndDecision, ...]:
    """Explain the extension rule for every pair worth explaining.

    Reported: every row end (sideways hop into a row that runs back), as
    ADDED, SKIPPED_OPERATOR (would extend, operator turned it off),
    SKIPPED_GAP (hop not shorter than the trigger) or DISABLED. ADDED
    entries carry clearance warnings for marking points within
    ``clearance_m`` of either extension leg.
    Everything else is omitted, so the list is about one entry per row.

    Two deliberate omissions. The first pair is reported only when ADDED:
    the rule judges it against a synthetic incoming direction (the reverse
    of the next hop), so a first row that is merely long would read as a
    skipped row end. And short hops that are not row ends are not reported:
    the first hop into every new row is also a 90 deg turn from the hop
    before it, so they cannot be told apart from a real near-miss.

    ADDED is taken from the compiled topology itself, so this report cannot
    disagree with the path.
    """

    mode = str(extension_mode).strip().upper()
    points = topology.metric_points
    decisions: list[RowEndDecision] = []
    grid = _PointGrid(points, cell_m=max(1.0, clearance_m))

    for segment in topology.segments:
        (
            row_end,
            incoming,
            transfer_angle,
            reversal_angle,
        ) = _evaluate_row_transition(
            marking_points=points,
            index=segment.index,
            row_transfer_min_angle_deg=row_transfer_min_angle_deg,
            row_transfer_max_angle_deg=row_transfer_max_angle_deg,
            row_reversal_min_angle_deg=row_reversal_min_angle_deg,
        )
        if transfer_angle is None or reversal_angle is None:
            continue  # last pair: no following row to judge

        point = None
        warnings: tuple[ClearanceWarning, ...] = ()

        if segment.use_dummy:
            decision = DECISION_ADDED
            assert incoming is not None and dummy_point_distance_m is not None
            length = math.hypot(incoming[0], incoming[1])
            start = points[segment.index]
            point = (
                start[0] + incoming[0] / length * dummy_point_distance_m,
                start[1] + incoming[1] / length * dummy_point_distance_m,
            )
            warnings = _clearance_warnings(
                grid,
                row_end_index=segment.index,
                extension_point=point,
                points=points,
                clearance_m=clearance_m,
            )
        elif segment.skipped_by_operator:
            decision = DECISION_SKIPPED_OPERATOR
        elif not row_end or segment.index == 0:
            continue
        elif mode != "ENABLE":
            decision = DECISION_DISABLED
        else:
            decision = DECISION_SKIPPED_GAP

        decisions.append(
            RowEndDecision(
                from_index=segment.index,
                gap_m=segment.distance_m,
                transfer_angle_deg=transfer_angle,
                reversal_angle_deg=reversal_angle,
                decision=decision,
                point=point,
                warnings=warnings,
            )
        )

    return tuple(decisions)


def gps_points_to_survey_local(
    gps_points: Sequence[Sequence[float]],
) -> tuple[Point, ...]:
    points = tuple(
        _finite_point(
            point,
            label=f"GPS marking point {index + 1}",
        )
        for index, point in enumerate(gps_points)
    )

    if len(points) < 2:
        raise ValueError(
            "Mission requires at least two marking points"
        )

    anchor_latitude, anchor_longitude = points[0]

    origin = GeographicOrigin(
        latitude_deg=anchor_latitude,
        longitude_deg=anchor_longitude,
    )

    metric_points: list[Point] = []

    for latitude, longitude in points:
        projected = project_geodetic_to_px4_enu(
            origin,
            latitude,
            longitude,
        )

        metric_points.append(
            (
                projected.east_m,
                projected.north_m,
            )
        )

    return tuple(metric_points)


def compile_source_topology(
    *,
    coordinate_mode: str,
    raw_marking_points: Sequence[Sequence[float]],
    extension_mode: str,
    row_transition_threshold_m: float | None,
    minimum_segment_length_m: float,
    row_transfer_min_angle_deg: float,
    row_transfer_max_angle_deg: float,
    row_reversal_min_angle_deg: float,
    skip_row_end_indices: frozenset[int] = frozenset(),
) -> MissionTopology:
    mode = str(coordinate_mode).strip().lower()

    if mode == "gps":
        metric_points = gps_points_to_survey_local(
            raw_marking_points
        )

    elif mode == "local":
        metric_points = tuple(
            _finite_point(
                point,
                label=f"local marking point {index + 1}",
            )
            for index, point
            in enumerate(raw_marking_points)
        )

    else:
        raise ValueError(
            f"Unsupported coordinate mode: {coordinate_mode!r}"
        )

    return compile_metric_topology(
        metric_points,
        extension_mode=extension_mode,
        row_transition_threshold_m=(
            row_transition_threshold_m
        ),
        minimum_segment_length_m=(
            minimum_segment_length_m
        ),
        row_transfer_min_angle_deg=(
            row_transfer_min_angle_deg
        ),
        row_transfer_max_angle_deg=(
            row_transfer_max_angle_deg
        ),
        row_reversal_min_angle_deg=(
            row_reversal_min_angle_deg
        ),
        skip_row_end_indices=skip_row_end_indices,
    )
