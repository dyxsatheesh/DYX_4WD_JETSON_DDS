"""Per-point evidence leaves the 4.5 Hz status and is published on change.

2026-09-30: /mission_manager/status averaged ~97 KB on a 15-point mission
(92% point_results / point_accuracy_snapshots / point_survey_snapshots) and
was 65% of a 576 MB bag.
"""

import ast
import json
from pathlib import Path
from types import SimpleNamespace

from test_lifecycle_command_behavior import MissionManager

NODE_SOURCE = (
    Path(__file__).resolve().parents[1] / "mission_manager" / "mission_manager_node.py"
).read_text(encoding="utf-8")

HEAVY = (
    "point_results",
    "point_accuracy_snapshots",
    "point_survey_snapshots",
    "last_termination_report",
)


def _manager():
    sent = []
    m = object.__new__(MissionManager)
    m.point_history_pub = SimpleNamespace(publish=lambda msg: sent.append(json.loads(msg.data)))
    m._mission_run_id = "run-1"
    m._point_accuracy_snapshots = [None, None]
    m._point_results = [None, None]
    m._last_termination_report = None
    return m, sent


def test_history_published_once_then_only_on_change():
    m, sent = _manager()
    MissionManager._publish_point_history_if_changed(m)
    MissionManager._publish_point_history_if_changed(m)
    assert len(sent) == 1 and sent[0]["point_history_version"] == 1

    m._point_results[0] = {"point_id": "P0001", "point_outcome": "COMPLETED"}
    m._point_accuracy_snapshots[0] = {"survey": {"radial_error_mm": 12.0}}
    MissionManager._publish_point_history_if_changed(m)
    MissionManager._publish_point_history_if_changed(m)
    assert len(sent) == 2
    assert sent[1]["point_history_version"] == 2
    assert sent[1]["mission_run_id"] == "run-1"
    assert sent[1]["point_survey_snapshots"] == [{"radial_error_mm": 12.0}, None]


def test_in_place_mutation_is_detected():
    m, sent = _manager()
    MissionManager._publish_point_history_if_changed(m)
    m._point_results[1] = {"point_id": "P0002"}  # same list object, mutated
    MissionManager._publish_point_history_if_changed(m)
    assert len(sent) == 2


def test_status_payload_no_longer_carries_heavy_history():
    tree = ast.parse(NODE_SOURCE)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "MissionManager")
    method = next(
        n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "_status_payload"
    )
    keys = {
        k.value
        for node in ast.walk(method)
        if isinstance(node, ast.Dict)
        for k in node.keys
        if isinstance(k, ast.Constant)
    }
    assert "point_history_version" in keys
    assert not keys & set(HEAVY)
