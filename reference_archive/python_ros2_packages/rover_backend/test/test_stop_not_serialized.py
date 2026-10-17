"""STOP must not queue behind an in-flight START/RESUME/NEXT/restore.

E-stop already bypasses the backend's mission-mutation lock and
RosBridgeRuntime._operation_lock; STOP now does the same. Mission Manager
serializes STOP itself and invalidates the overlapped command by safety
generation.
"""

from __future__ import annotations

import ast
from pathlib import Path
import threading

SOURCE = Path(__file__).resolve().parents[1] / "rover_backend"


def method(filename, class_name, name):
    tree = ast.parse((SOURCE / filename).read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    function = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    namespace: dict = {"Any": object}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(SOURCE / filename), "exec"), namespace)
    return namespace[name]


def route_source(name):
    source = (SOURCE / "mission_routes.py").read_text()
    node = next(
        n for n in ast.parse(source).body
        if isinstance(n, ast.AsyncFunctionDef) and n.name == name
    )
    return ast.get_source_segment(source, node)


def test_stop_route_does_not_take_the_mission_mutation_lock():
    assert "_serialize_mission_mutation" not in route_source("stop_mission")
    # Ordinary mutations keep it.
    for serialized in ("start_mission", "pause_mission", "resume_mission", "restore_mission"):
        assert "_serialize_mission_mutation" in route_source(serialized)


class Runtime:
    def __init__(self):
        self._operation_lock = threading.RLock()
        self._safety_command_lock = threading.RLock()
        self._safety_generation = 3
        self.stopped = threading.Event()
        runtime = self

        class Node:
            def stop_mission(self):
                runtime.stopped.set()
                return {"state": "EMPTY"}

        self.node = Node()


def test_stop_completes_while_another_operation_holds_the_operation_lock():
    stop = method("ros_bridge.py", "RosBridgeRuntime", "stop_mission")
    runtime = Runtime()
    holding = threading.Event()
    release = threading.Event()

    def long_start():
        with runtime._operation_lock:
            holding.set()
            release.wait(5.0)

    starter = threading.Thread(target=long_start)
    starter.start()
    assert holding.wait(1.0)
    try:
        worker = threading.Thread(target=lambda: stop(runtime))
        worker.start()
        assert runtime.stopped.wait(1.0), "STOP queued behind the operation lock"
        worker.join(1.0)
    finally:
        release.set()
        starter.join(1.0)


def test_stop_bumps_the_backend_safety_generation():
    # A RELEASE captured before this STOP compares generations and is rejected.
    stop = method("ros_bridge.py", "RosBridgeRuntime", "stop_mission")
    runtime = Runtime()
    stop(runtime)
    assert runtime._safety_generation == 4
