"""/restore returns once the restored mission is START-ready.

The app's rerun path (MissionReportScreen.restoreRetainedMissionForStart)
skips its REST status poll + 400 ms sleep when the /restore response already
reports loaded and trajectory_ready. These tests pin the backend side of that:
a bounded, in-process wait on the same gates /load and /start use.
"""

from __future__ import annotations

import ast
import asyncio
from pathlib import Path
import threading

import pytest

SOURCE = Path(__file__).resolve().parents[1] / "rover_backend"


def production_function(filename, name, namespace, *, node_method=False):
    tree = ast.parse((SOURCE / filename).read_text())
    body = tree.body
    if node_method:
        body = next(
            n for n in tree.body
            if isinstance(n, ast.ClassDef) and n.name == "RoverBackendRosNode"
        ).body
    function = next(
        n for n in body
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)) and n.name == name
    )
    function.decorator_list = []
    exec(
        compile(ast.Module(body=[function], type_ignores=[]), str(SOURCE / filename), "exec"),
        namespace,
    )
    return namespace[name]


class State:
    def __init__(self, **mission):
        self.mission = {"mission_id": "m1", "loaded": True, "trajectory_ready": False, **mission}

    def section(self, _name):
        return dict(self.mission)


class Bridge:
    def __init__(self, sequence):
        # Each call to preparation_readiness() consumes one (ready, error).
        self.sequence = list(sequence)
        self.calls = 0

    def preparation_readiness(self):
        self.calls += 1
        if len(self.sequence) > 1:
            return self.sequence.pop(0)
        return self.sequence[0]


def waiter(state, bridge, *, flip_trajectory_ready_after=None):
    namespace = {"asyncio": asyncio, "ros_bridge": bridge}

    def mission_state():
        if (
            flip_trajectory_ready_after is not None
            and bridge.calls >= flip_trajectory_ready_after
        ):
            state.mission["trajectory_ready"] = True
        return state.section("mission")

    namespace["_mission_state"] = mission_state
    namespace["PREPARE_READY_WAIT_SEC"] = 3.0
    namespace["PREPARE_READY_POLL_SEC"] = 0.001
    return production_function("mission_routes.py", "_await_prepared_for_start", namespace)


def test_returns_true_once_manager_and_trajectory_are_ready():
    state = State()
    bridge = Bridge([(False, None), (False, None), (True, None)])
    wait = waiter(state, bridge, flip_trajectory_ready_after=3)
    assert asyncio.run(wait()) is True
    assert bridge.calls == 3


def test_trajectory_ready_alone_is_not_enough():
    # /start needs Mission Manager READY, not just trajectory_ready.
    state = State(trajectory_ready=True)
    bridge = Bridge([(False, None)])
    wait = waiter(state, bridge)
    assert asyncio.run(wait(timeout_sec=0.02)) is False


def test_error_returns_false_without_waiting_out_the_timeout():
    state = State()
    bridge = Bridge([(False, "Trajectory preparation failed")])
    wait = waiter(state, bridge)
    assert asyncio.run(wait(timeout_sec=5.0)) is False
    assert bridge.calls == 1


def test_bridge_not_running_returns_false():
    class Down:
        def preparation_readiness(self):
            raise RuntimeError("ROS bridge is not running")

    wait = waiter(State(), Down())
    assert asyncio.run(wait()) is False


def test_timeout_returns_false_and_leaves_the_old_response():
    wait = waiter(State(), Bridge([(False, None)]))
    assert asyncio.run(wait(timeout_sec=0.02)) is False


def test_restore_and_archived_start_both_wait_before_responding():
    source = (SOURCE / "mission_routes.py").read_text()
    tree = ast.parse(source)

    def body_of(name):
        node = next(
            n for n in tree.body
            if isinstance(n, ast.AsyncFunctionDef) and n.name == name
        )
        return ast.get_source_segment(source, node)

    restore = body_of("restore_mission")
    assert restore.index("ros_bridge.prepare_trajectory") < restore.index(
        "await _await_prepared_for_start()"
    ) < restore.index('"operation": "restore"')

    start = body_of("start_mission")
    assert start.index("ros_bridge.prepare_trajectory") < start.index(
        "await _await_prepared_for_start()"
    ) < start.index('if mission.get("trajectory_ready") is not True:')


@pytest.mark.parametrize(
    "trajectory_ready,manager_ready,preparing,expected",
    [
        (True, True, False, True),
        (True, False, False, False),
        (False, True, False, False),
        (True, True, True, False),
    ],
)
def test_node_readiness_requires_all_three(trajectory_ready, manager_ready, preparing, expected):
    readiness = production_function(
        "ros_bridge.py", "preparation_readiness", {}, node_method=True
    )

    class Node:
        _runtime_lock = threading.Lock()
        _trajectory_error = None
        _mission_manager_error = None

    node = Node()
    node._trajectory_ready = trajectory_ready
    node._mission_manager_ready = manager_ready
    node._preparation_in_progress = preparing
    assert readiness(node) == (expected, None)


def test_node_readiness_reports_errors():
    readiness = production_function(
        "ros_bridge.py", "preparation_readiness", {}, node_method=True
    )

    class Node:
        _runtime_lock = threading.Lock()
        _trajectory_error = None
        _mission_manager_error = "Mission manager error"
        _trajectory_ready = False
        _mission_manager_ready = False
        _preparation_in_progress = True

    assert readiness(Node()) == (False, "Mission manager error")
