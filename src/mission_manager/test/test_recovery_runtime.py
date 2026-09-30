from mission_manager.recovery_runtime import build_recovery_plan


def _runtime(status):
    count = len(status)
    return {
        "mission_run_id": "run-123",
        "mission_started_unix_ns": 123456789,
        "execution_mode": "AUTO",
        "marking_tool": "SPRAY",
        "point_status": status,
        "point_accuracy_snapshots": [
            {"point": number + 1} for number in range(count)
        ],
        "point_results": [
            {"point": number + 1} for number in range(count)
        ],
        "last_point_result": {"point": 2},
    }


def test_restore_uses_first_unresolved_point_and_keeps_terminal_progress():
    plan = build_recovery_plan(
        _runtime(["COMPLETED", "COMPLETED", "ACTIVE", "PENDING"]),
        point_count=4,
        marking_path_index_by_number=[5, 10, 20, 30],
        semantic_path_indices=[5, 10, 15, 20, 30],
    )

    assert plan.point_status == (
        "COMPLETED",
        "COMPLETED",
        "PENDING",
        "PENDING",
    )
    assert plan.unresolved_marking_number == 2
    # 15 is the dummy/alignment semantic goal before P3 at path index 20.
    assert plan.current_path_index == 15
    assert plan.point_results[0] == {"point": 1}
    assert plan.point_results[1] == {"point": 2}
    assert plan.point_results[2] is None
    assert plan.point_accuracy_snapshots[2] is None


def test_completed_current_point_resumes_from_next_unresolved_point():
    plan = build_recovery_plan(
        _runtime(["COMPLETED", "COMPLETED", "COMPLETED", "PENDING"]),
        point_count=4,
        marking_path_index_by_number=[5, 10, 20, 30],
        semantic_path_indices=[5, 10, 20, 25, 30],
    )

    assert plan.unresolved_marking_number == 3
    assert plan.current_path_index == 25


def test_terminal_mission_is_not_recoverable():
    try:
        build_recovery_plan(
            _runtime(["COMPLETED", "SKIPPED", "FAILED"]),
            point_count=3,
            marking_path_index_by_number=[5, 10, 15],
            semantic_path_indices=[5, 10, 15],
        )
    except ValueError as error:
        assert "no unresolved" in str(error).lower()
    else:
        raise AssertionError("terminal mission unexpectedly recovered")
