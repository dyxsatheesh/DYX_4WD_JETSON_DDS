from rover_backend.mission_recovery import (
    validate_power_cycle_recovery_payload,
)


def _metadata():
    return {
        "mission_id": "mission-1",
        "checksum_sha256": "abc123",
        "total_points": 4,
    }


def _payload(state="RUNNING"):
    return {
        "schema_version": 1,
        "saved_at": "2026-09-30T10:00:00+00:00",
        "mission_id": "mission-1",
        "checksum_sha256": "abc123",
        "runtime": {
            "state": state,
            "mission_run_id": "run-1",
            "point_status": [
                "COMPLETED",
                "COMPLETED",
                "ACTIVE",
                "PENDING",
            ],
        },
    }


def test_running_mission_recovers_from_first_unresolved_point():
    candidate = validate_power_cycle_recovery_payload(
        _payload(),
        _metadata(),
    )
    assert candidate is not None
    assert candidate.previous_state == "RUNNING"
    assert candidate.current_point_id == "P0003"
    assert candidate.current_point_number == 3


def test_paused_and_waiting_missions_are_recoverable():
    assert validate_power_cycle_recovery_payload(
        _payload("PAUSED"),
        _metadata(),
    ) is not None
    assert validate_power_cycle_recovery_payload(
        _payload("WAITING_FOR_NEXT"),
        _metadata(),
    ) is not None


def test_ready_or_completed_runtime_is_not_recoverable():
    assert validate_power_cycle_recovery_payload(
        _payload("READY"),
        _metadata(),
    ) is None

    payload = _payload("PAUSED")
    payload["runtime"]["point_status"] = [
        "COMPLETED",
        "SKIPPED",
        "FAILED",
        "COMPLETED",
    ]
    assert validate_power_cycle_recovery_payload(
        payload,
        _metadata(),
    ) is None


def test_mission_identity_and_checksum_must_match():
    wrong_id = _payload()
    wrong_id["mission_id"] = "other"
    assert validate_power_cycle_recovery_payload(
        wrong_id,
        _metadata(),
    ) is None

    wrong_checksum = _payload()
    wrong_checksum["checksum_sha256"] = "different"
    assert validate_power_cycle_recovery_payload(
        wrong_checksum,
        _metadata(),
    ) is None
