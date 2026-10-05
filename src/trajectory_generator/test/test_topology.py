"""Regression tests for the pure mission-topology compiler."""

import pytest

from trajectory_generator.topology import compile_metric_topology
from trajectory_generator.topology import compile_source_topology


COMMON = {
    "row_transition_threshold_m": 3.0,
    "minimum_segment_length_m": 0.001,
    "row_transfer_min_angle_deg": 45.0,
    "row_transfer_max_angle_deg": 135.0,
    "row_reversal_min_angle_deg": 135.0,
}


def test_short_straight_segments_do_not_create_dummy():
    topology = compile_metric_topology(
        [
            (0.0, 0.0),
            (1.0, 0.0),
            (2.0, 0.0),
        ],
        extension_mode="ENABLE",
        **COMMON,
    )

    assert [
        segment.use_dummy
        for segment in topology.segments
    ] == [
        False,
        False,
    ]


def test_true_serpentine_transition_creates_dummy_decision():
    topology = compile_metric_topology(
        [
            (0.0, 0.0),
            (0.0, 1.0),
            (1.0, 1.0),
            (1.0, 0.0),
        ],
        extension_mode="ENABLE",
        **COMMON,
    )

    first = topology.segments[0]

    assert first.use_dummy is True
    assert first.incoming_direction is not None
    assert first.transfer_angle_deg == pytest.approx(90.0)
    assert first.reversal_angle_deg == pytest.approx(180.0)


def test_extension_disable_never_creates_dummy():
    topology = compile_metric_topology(
        [
            (0.0, 0.0),
            (0.0, 1.0),
            (1.0, 1.0),
            (1.0, 0.0),
        ],
        extension_mode="DISABLE",
        **COMMON,
    )

    assert not any(
        segment.use_dummy
        for segment in topology.segments
    )


def test_duplicate_consecutive_marking_is_rejected():
    with pytest.raises(
        ValueError,
        match="too close",
    ):
        compile_metric_topology(
            [
                (0.0, 0.0),
                (0.0, 0.0),
                (1.0, 0.0),
            ],
            extension_mode="ENABLE",
            **COMMON,
        )


def test_metric_point_order_is_preserved_exactly():
    source = [
        (4.0, -2.0),
        (7.0, -2.0),
        (7.0, 1.0),
        (3.0, 1.0),
    ]

    topology = compile_metric_topology(
        source,
        extension_mode="ENABLE",
        **COMMON,
    )

    assert topology.metric_points == tuple(source)


def test_same_input_produces_identical_topology():
    source = [
        (0.0, 0.0),
        (0.0, 1.0),
        (1.0, 1.0),
        (1.0, 0.0),
    ]

    first = compile_metric_topology(
        source,
        extension_mode="ENABLE",
        **COMMON,
    )

    second = compile_metric_topology(
        source,
        extension_mode="ENABLE",
        **COMMON,
    )

    assert first == second


def test_gps_source_topology_compiles_without_gp_origin_or_live_state():
    gps_points = [
        (13.0000000, 80.0000000),
        (13.0000000, 80.0000200),
        (12.9999900, 80.0000200),
        (12.9999900, 80.0000000),
    ]

    topology = compile_source_topology(
        coordinate_mode="gps",
        raw_marking_points=gps_points,
        extension_mode="ENABLE",
        **COMMON,
    )

    assert len(topology.metric_points) == len(gps_points)
    assert len(topology.segments) == len(gps_points) - 1

    assert topology.metric_points[0][0] == pytest.approx(
        0.0,
        abs=1e-12,
    )

    assert topology.metric_points[0][1] == pytest.approx(
        0.0,
        abs=1e-12,
    )


def test_gps_source_topology_is_repeatable():
    gps_points = [
        (13.18937434, 80.22219259),
        (13.18936988, 80.22223429),
        (13.18936523, 80.22227606),
        (13.18936129, 80.22231909),
    ]

    first = compile_source_topology(
        coordinate_mode="gps",
        raw_marking_points=gps_points,
        extension_mode="ENABLE",
        **COMMON,
    )

    second = compile_source_topology(
        coordinate_mode="gps",
        raw_marking_points=gps_points,
        extension_mode="ENABLE",
        **COMMON,
    )

    assert first == second


def test_gap_exactly_at_threshold_does_not_create_dummy():
    topology = compile_metric_topology(
        [
            (0.0, 0.0),
            (0.0, 3.0),
            (1.0, 3.0),
            (1.0, 0.0),
        ],
        extension_mode="ENABLE",
        **COMMON,
    )

    assert topology.segments[0].distance_m == pytest.approx(3.0)
    assert topology.segments[0].use_dummy is False


def test_minimum_segment_length_exact_boundary_is_accepted():
    topology = compile_metric_topology(
        [
            (0.0, 0.0),
            (0.001, 0.0),
        ],
        extension_mode="DISABLE",
        **COMMON,
    )

    assert topology.segments[0].distance_m == pytest.approx(0.001)


def test_gps_serpentine_source_topology_decisions():
    gps_points = [
        (13.0000000, 80.0000000),
        (13.0000000, 80.0000200),
        (12.9999900, 80.0000200),
        (12.9999900, 80.0000000),
    ]

    topology = compile_source_topology(
        coordinate_mode="gps",
        raw_marking_points=gps_points,
        extension_mode="ENABLE",
        **COMMON,
    )

    assert [
        segment.use_dummy
        for segment in topology.segments
    ] == [
        True,
        True,
        False,
    ]

    # Longitude increase must be East in the temporary survey-local frame.
    assert topology.metric_points[1][0] > 0.0

    # P1 -> P2 changes longitude only, so North displacement stays tiny.
    assert abs(topology.metric_points[1][1]) < 0.01

    # P2 -> P3 moves south.
    assert (
        topology.metric_points[2][1]
        < topology.metric_points[1][1]
    )


def test_gap_just_below_threshold_creates_dummy_when_angles_qualify():
    topology = compile_metric_topology(
        [
            (0.0, 0.0),
            (0.0, 2.999),
            (1.0, 2.999),
            (1.0, 0.0),
        ],
        extension_mode="ENABLE",
        **COMMON,
    )

    assert topology.segments[0].distance_m == pytest.approx(2.999)
    assert topology.segments[0].use_dummy is True


def test_segment_just_below_minimum_is_rejected():
    with pytest.raises(
        ValueError,
        match="too close",
    ):
        compile_metric_topology(
            [
                (0.0, 0.0),
                (0.000999, 0.0),
            ],
            extension_mode="DISABLE",
            **COMMON,
        )


def test_local_source_topology_preserves_metric_coordinates():
    points = [
        (10.0, -4.0),
        (12.0, -4.0),
        (12.0, -2.0),
    ]

    topology = compile_source_topology(
        coordinate_mode="local",
        raw_marking_points=points,
        extension_mode="DISABLE",
        **COMMON,
    )

    assert topology.metric_points == tuple(points)


def test_unsupported_source_coordinate_mode_is_rejected():
    with pytest.raises(
        ValueError,
        match="Unsupported coordinate mode",
    ):
        compile_source_topology(
            coordinate_mode="unknown",
            raw_marking_points=[
                (0.0, 0.0),
                (1.0, 0.0),
            ],
            extension_mode="DISABLE",
            **COMMON,
        )


# ── Row-end decisions (Upgrade Extension phase 2) ─────────────────────────

from trajectory_generator.topology import row_end_decisions  # noqa: E402

_ANGLES = {
    "row_transfer_min_angle_deg": 45.0,
    "row_transfer_max_angle_deg": 135.0,
    "row_reversal_min_angle_deg": 135.0,
}


def _decisions(points, *, mode="ENABLE", trigger=3.0, distance=4.0):
    topology = compile_metric_topology(
        points,
        extension_mode=mode,
        row_transition_threshold_m=trigger,
        minimum_segment_length_m=0.001,
        **_ANGLES,
    )
    return topology, row_end_decisions(
        topology,
        extension_mode=mode,
        row_transition_threshold_m=trigger,
        dummy_point_distance_m=distance if mode == "ENABLE" else None,
        **_ANGLES,
    )


# Three rows 1 m apart, marks every 1 m along each 3 m row (serpentine).
_THREE_ROWS = [
    (0.0, 0.0), (0.0, 1.0), (0.0, 2.0), (0.0, 3.0),
    (1.0, 3.0), (1.0, 2.0), (1.0, 1.0), (1.0, 0.0),
    (2.0, 0.0), (2.0, 1.0), (2.0, 2.0), (2.0, 3.0),
]


def test_row_end_decisions_report_one_entry_per_row_end():
    topology, decisions = _decisions(_THREE_ROWS)

    assert [(d.from_index, d.decision) for d in decisions] == [(3, "ADDED"), (7, "ADDED")]
    # ADDED matches the compiled path exactly.
    added = {d.from_index for d in decisions if d.decision == "ADDED"}
    assert added == {s.index for s in topology.segments if s.use_dummy}


def test_added_point_is_distance_past_the_row_end_along_the_row():
    _, decisions = _decisions(_THREE_ROWS, distance=4.0)

    assert decisions[0].point == pytest.approx((0.0, 7.0))   # P4 (0,3) + 4 m north
    assert decisions[1].point == pytest.approx((1.0, -4.0))  # P8 (1,0) + 4 m south


def test_row_end_over_the_trigger_is_skipped_for_gap():
    _, decisions = _decisions(_THREE_ROWS, trigger=0.5)

    assert [d.decision for d in decisions] == ["SKIPPED_GAP", "SKIPPED_GAP"]
    assert all(d.point is None for d in decisions)


def test_disabled_reports_row_ends_without_points():
    _, decisions = _decisions(_THREE_ROWS, mode="DISABLE")

    assert [d.decision for d in decisions] == ["DISABLED", "DISABLED"]
    assert all(d.point is None for d in decisions)


def test_first_pair_is_reported_only_when_added():
    # P1->P2 is a 3 m row; the rule judges it against a synthetic incoming
    # direction, so it must not read as a skipped row end.
    points = [(0.0, 0.0), (0.0, 3.0), (1.0, 3.0), (1.0, 0.0)]
    _, decisions = _decisions(points)

    assert [(d.from_index, d.decision) for d in decisions] == [(1, "ADDED")]


def test_hops_along_a_row_are_not_reported():
    points = [(0.0, float(i)) for i in range(10)]
    _, decisions = _decisions(points)

    assert decisions == ()


def test_operator_skip_turns_an_added_row_end_into_skipped_operator():
    topology = compile_metric_topology(
        _THREE_ROWS,
        extension_mode="ENABLE",
        row_transition_threshold_m=3.0,
        minimum_segment_length_m=0.001,
        skip_row_end_indices=frozenset({3}),
        **_ANGLES,
    )
    decisions = row_end_decisions(
        topology,
        extension_mode="ENABLE",
        row_transition_threshold_m=3.0,
        dummy_point_distance_m=4.0,
        **_ANGLES,
    )

    assert [(d.from_index, d.decision) for d in decisions] == [
        (3, "SKIPPED_OPERATOR"),
        (7, "ADDED"),
    ]
    assert [s.index for s in topology.segments if s.use_dummy] == [7]


def test_unused_skips_are_reported():
    from trajectory_generator.topology import unused_skip_indices

    topology = compile_metric_topology(
        _THREE_ROWS,
        extension_mode="ENABLE",
        row_transition_threshold_m=3.0,
        minimum_segment_length_m=0.001,
        skip_row_end_indices=frozenset({3, 5}),
        **_ANGLES,
    )
    assert unused_skip_indices(topology, frozenset({3, 5})) == (5,)


def test_clearance_warning_for_a_mark_near_the_extension_leg():
    # A stray mark 0.2 m beside the run-out past P4 (0,3) -> X (0,7).
    points = list(_THREE_ROWS) + [(0.2, 5.0)]
    _, decisions = _decisions(points, distance=4.0)

    first = decisions[0]
    assert first.decision == "ADDED"
    assert [(w.point_index, round(w.distance_m, 3)) for w in first.warnings] == [(12, 0.2)]
    # The second row end's legs are clear.
    assert decisions[1].warnings == ()


def test_row_end_and_next_row_start_never_warn():
    _, decisions = _decisions(_THREE_ROWS, distance=4.0)
    assert all(d.warnings == () for d in decisions)
