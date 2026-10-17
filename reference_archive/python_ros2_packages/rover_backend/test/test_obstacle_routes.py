"""/api/obstacle routes (loaded without ROS, like test_spray_routes)."""

from __future__ import annotations

import ast
from pathlib import Path
from typing import Any

import pytest
from fastapi import APIRouter, Depends, FastAPI, HTTPException
from fastapi.testclient import TestClient
from pydantic import BaseModel
from starlette.concurrency import run_in_threadpool

SOURCE = Path(__file__).resolve().parents[1] / "rover_backend" / "obstacle_routes.py"


class FakeBridge:
    def __init__(self) -> None:
        self.running = True
        self.status: dict[str, Any] = {"enabled": False, "zone": "clear", "healthy": True}
        self.changes: list[dict[str, Any]] = []
        self.error: RuntimeError | None = None

    def get_obstacle_status(self):
        return dict(self.status)

    def set_obstacle_config(self, changes):
        self.changes.append(changes)
        if self.error:
            raise self.error
        self.status.update(changes)
        return dict(self.status)


def _unauthenticated():
    raise HTTPException(status_code=401, detail="Authentication required")


def load(bridge):
    tree = ast.parse(SOURCE.read_text())
    tree.body = [n for n in tree.body if not isinstance(n, (ast.Import, ast.ImportFrom))]
    namespace = {
        "__name__": "obstacle_routes_under_test",
        "Any": Any, "APIRouter": APIRouter, "Depends": Depends,
        "HTTPException": HTTPException, "BaseModel": BaseModel,
        "run_in_threadpool": run_in_threadpool,
        "AuthenticatedSession": object, "require_auth": _unauthenticated,
        "ros_bridge": bridge,
    }
    exec(compile(tree, str(SOURCE), "exec"), namespace)  # noqa: S102
    return namespace


@pytest.fixture
def api():
    bridge = FakeBridge()
    routes = load(bridge)
    app = FastAPI()
    app.include_router(routes["obstacle_router"])
    app.dependency_overrides[_unauthenticated] = lambda: object()
    with TestClient(app) as client:
        yield client, bridge


def test_requires_login():
    routes = load(FakeBridge())
    app = FastAPI()
    app.include_router(routes["obstacle_router"])
    with TestClient(app) as client:
        assert client.get("/api/obstacle/status").status_code == 401
        assert client.post("/api/obstacle/config", json={"enabled": True}).status_code == 401


def test_status(api):
    client, _ = api
    body = client.get("/api/obstacle/status").json()
    assert body["success"] is True and body["status"]["zone"] == "clear"


def test_config_sends_only_given_fields(api):
    client, bridge = api
    body = client.post("/api/obstacle/config", json={"enabled": True, "danger_mm": 1200}).json()
    assert bridge.changes == [{"enabled": True, "danger_mm": 1200}]
    assert body["status"]["enabled"] is True


def test_config_empty_and_rejected(api):
    client, bridge = api
    assert client.post("/api/obstacle/config", json={}).status_code == 422
    bridge.error = RuntimeError("Obstacle configuration rejected: need 300 <= danger_mm < warning_mm")
    response = client.post("/api/obstacle/config", json={"danger_mm": 5000})
    assert response.status_code == 409
    assert "rejected" in response.json()["detail"]["message"]
