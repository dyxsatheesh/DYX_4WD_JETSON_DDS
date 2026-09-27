"""/rpp/debug keeps live geometry while motion is held (monitor only).

Pause (mission disabled), E-stop and the marking hold return from
control_loop before geometry is computed, so /rpp/debug went
available=false and the tablet's RPP panels froze for the whole hold.
"""

import ast
import math
from pathlib import Path
import textwrap

import pytest

SOURCE_PATH = (
    Path(__file__).resolve().parents[1] / "rpp_controller" / "rpp_controller_node.py"
)
SOURCE = SOURCE_PATH.read_text(encoding="utf-8")
TREE = ast.parse(SOURCE)
NODE_CLASS = next(
    n for n in TREE.body
    if isinstance(n, ast.ClassDef)
    and any(isinstance(m, ast.FunctionDef) and m.name == "control_loop" for m in n.body)
)
METHODS = (
    "_record_rpp_debug_hold_geometry",
    "_record_rpp_debug_geometry",
    "_finite_or_none",
    "ground_xtrack",
    "normalize_angle",
    "along_track_remaining",
)


def _extract():
    parts = []
    for node in NODE_CLASS.body:
        if isinstance(node, ast.FunctionDef) and node.name in METHODS:
            start = node.decorator_list[0].lineno if node.decorator_list else node.lineno
            lines = SOURCE.splitlines()[start - 1:node.end_lineno]
            parts.append(textwrap.dedent("\n".join(lines)))
    body = "\n\n".join(parts)
    namespace = {"math": math}
    exec("class Extracted:\n" + textwrap.indent(body, "    "), namespace)
    return namespace["Extracted"]


Extracted = _extract()


class Rpp(Extracted):
    def __init__(self):
        self._rpp_debug_pending = {"available": False, "control_mode": "CONTROL_CYCLE"}
        self._rpp_hold_geometry_reference = None
        self.segment_goal_number = 2
        self.geometry_installed_signature = "sig-A"
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_fresh = True
        self.last_odom_time = object()
        self.odom_timeout_sec = 0.5

    def is_fresh(self, _stamp, _timeout):
        return self.odom_fresh


def reference(rpp, goal=(10.0, 0.0), bearing=0.0):
    rpp._rpp_hold_geometry_reference = (
        goal[0], goal[1], bearing, rpp.segment_goal_number, rpp.geometry_installed_signature,
    )


def test_paused_rover_reports_live_geometry_against_the_last_goal():
    rpp = Rpp()
    reference(rpp, goal=(10.0, 0.0), bearing=0.0)  # driving east to (10, 0)
    rpp.current_x, rpp.current_y, rpp.current_yaw = 7.0, 0.02, math.radians(5.0)
    rpp._record_rpp_debug_hold_geometry("HOLD_MISSION_DISABLED")
    d = rpp._rpp_debug_pending
    assert d["available"] is True and d["geometry_valid"] is True
    assert d["control_mode"] == "HOLD_MISSION_DISABLED"
    assert d["distance_to_goal_m"] == pytest.approx(math.hypot(3.0, 0.02))
    assert d["along_remaining_mm"] == pytest.approx(3000.0)
    # 20 mm north of an eastbound line = LEFT; ground convention flips sign.
    assert d["cross_track_side"] == "LEFT"
    assert d["cross_track_error_mm"] == pytest.approx(-20.0)
    assert d["heading_error_deg"] == pytest.approx(-5.0)


def test_live_pose_changes_are_reflected_every_cycle():
    rpp = Rpp()
    reference(rpp)
    rpp.current_x = 9.0
    rpp._record_rpp_debug_hold_geometry("HOLD_MARKING")
    first = rpp._rpp_debug_pending["distance_to_goal_m"]
    rpp._rpp_debug_pending = {"available": False, "control_mode": "CONTROL_CYCLE"}
    rpp.current_x = 9.5
    rpp._record_rpp_debug_hold_geometry("HOLD_MARKING")
    assert rpp._rpp_debug_pending["distance_to_goal_m"] < first


@pytest.mark.parametrize(
    "change",
    [
        lambda r: setattr(r, "segment_goal_number", 3),          # goal advanced
        lambda r: setattr(r, "geometry_installed_signature", "B"),  # new path
        lambda r: setattr(r, "odom_fresh", False),               # stale pose
        lambda r: setattr(r, "current_yaw", None),               # no pose
        lambda r: setattr(r, "_rpp_hold_geometry_reference", None),  # never ran
    ],
)
def test_no_geometry_when_it_would_be_wrong(change):
    rpp = Rpp()
    reference(rpp)
    change(rpp)
    rpp._record_rpp_debug_hold_geometry("HOLD_ESTOP")
    d = rpp._rpp_debug_pending
    assert d["available"] is False and "distance_to_goal_m" not in d
    # The hold reason is still reported.
    assert d["control_mode"] == "HOLD_ESTOP"


def test_hold_geometry_is_monitor_only():
    method = next(
        n for n in NODE_CLASS.body
        if isinstance(n, ast.FunctionDef) and n.name == "_record_rpp_debug_hold_geometry"
    )
    body = ast.get_source_segment(SOURCE, method)
    for forbidden in ("publish", "_reset", "latch", "self.target_", "cmd"):
        assert forbidden not in body.split('"""', 2)[2], forbidden
    assigned = {
        target.attr
        for node in ast.walk(method)
        if isinstance(node, (ast.Assign, ast.AugAssign))
        for target in (node.targets if isinstance(node, ast.Assign) else [node.target])
        if isinstance(target, ast.Attribute)
    }
    assert assigned == set()  # writes only into the pending debug dict


def test_every_hold_exit_reports_hold_geometry_and_active_cycle_stores_reference():
    loop = next(
        n for n in NODE_CLASS.body
        if isinstance(n, ast.FunctionDef) and n.name == "control_loop"
    )
    text = ast.get_source_segment(SOURCE, loop)
    for mode in ("HOLD_ESTOP", "HOLD_MISSION_DISABLED", "HOLD_MARKING", "HOLD_MARKING_MISSED"):
        assert f'self._record_rpp_debug_hold_geometry("{mode}")' in text
    record = text.index("self._record_rpp_debug_geometry(")
    assert text.index("self._rpp_hold_geometry_reference = (", record) > record
