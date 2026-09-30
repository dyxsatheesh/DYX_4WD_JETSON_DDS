"""Event-loop lag probe and transport reporting (DYX_REALTIME_METRICS).

realtime.py imports ROS through system_routes, so -- like
test_realtime_transport_contract -- the functions are compiled from source.
"""

import ast
import asyncio
import logging
import time
from pathlib import Path

from rover_backend.realtime_contract import RealtimeMetrics

SOURCE_PATH = Path(__file__).resolve().parents[1] / "rover_backend" / "realtime.py"
SOURCE = SOURCE_PATH.read_text(encoding="utf-8")


def _load(names, **namespace):
    tree = ast.parse(SOURCE)
    wanted = [
        node
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name in names
    ]
    assert {node.name for node in wanted} == set(names)
    namespace.setdefault("asyncio", asyncio)
    exec(compile(ast.Module(body=wanted, type_ignores=[]), str(SOURCE_PATH), "exec"), namespace)
    return namespace


def test_probe_records_a_blocked_loop(caplog):
    metrics = RealtimeMetrics(enabled=True, window_sec=3600.0)
    ns = _load(
        ["_loop_lag_probe"],
        realtime_metrics=metrics,
        LOGGER=logging.getLogger("test.realtime"),
        LOOP_LAG_PROBE_PERIOD_SEC=0.1,
        LOOP_LAG_WARN_MS=250.0,
    )

    async def scenario():
        stop = asyncio.Event()
        task = asyncio.create_task(ns["_loop_lag_probe"](stop))
        await asyncio.sleep(0.05)
        time.sleep(0.4)  # block the loop the way a slow sync payload build would
        await asyncio.sleep(0.25)
        stop.set()
        await asyncio.wait_for(task, 1.0)

    with caplog.at_level(logging.WARNING, logger="test.realtime"):
        asyncio.run(scenario())

    lag = metrics._timings["event_loop_lag"]
    assert lag.calls >= 2
    assert lag.max_ms >= 250.0
    assert "event loop stalled" in caplog.text


def test_transport_helper_never_raises():
    class _Sio:
        def transport(self, sid):
            if sid == "ws":
                return "websocket"
            raise KeyError(sid)

    ns = _load(["_socket_transport"], sio=_Sio())
    assert ns["_socket_transport"]("ws") == "websocket"
    assert ns["_socket_transport"]("gone") == "unknown"


def test_probe_only_started_when_metrics_enabled():
    start = SOURCE[SOURCE.index("async def start_realtime"):]
    start = start[: start.index("async def stop_realtime")]
    assert "if realtime_metrics.enabled:" in start
    assert "_loop_lag_probe(_stop_event)" in start
