"""The recovery checkpoint keeps per-point evidence after it left status."""

from rover_backend.ros_bridge import RoverBackendRosNode


def _node(history):
    node = object.__new__(RoverBackendRosNode)
    node._mission_point_history = history
    return node


HISTORY = {
    "schema": "dyx4wd/point_history@1",
    "mission_run_id": "run-1",
    "point_history_version": 3,
    "point_results": [{"point_id": "P0001"}, None],
    "point_accuracy_snapshots": [{"overall_accuracy_mm": 9.0}, None],
    "point_survey_snapshots": [None, None],
    "last_termination_report": None,
}


def test_history_merged_for_same_mission_run():
    status = {"mission_run_id": "run-1", "state": "RUNNING", "point_status": ["COMPLETED", "PENDING"]}
    merged = RoverBackendRosNode._with_point_history(_node(HISTORY), status)
    assert merged["point_results"] == HISTORY["point_results"]
    assert merged["point_accuracy_snapshots"] == HISTORY["point_accuracy_snapshots"]
    assert merged["state"] == "RUNNING"
    assert "point_results" not in status  # caller's dict untouched


def test_history_from_another_run_is_never_merged():
    status = {"mission_run_id": "run-2", "state": "RUNNING"}
    merged = RoverBackendRosNode._with_point_history(_node(HISTORY), status)
    assert "point_results" not in merged


def test_no_history_leaves_payload_unchanged():
    status = {"mission_run_id": "run-1"}
    assert RoverBackendRosNode._with_point_history(_node(None), status) is status
