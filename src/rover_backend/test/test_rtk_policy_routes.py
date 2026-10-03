"""/api/rtk/policy routes (loaded without ROS, like test_obstacle_routes)."""

from __future__ import annotations

import ast
from pathlib import Path
from typing import Any

import pytest
from fastapi import APIRouter, Depends, FastAPI, HTTPException
from fastapi.testclient import TestClient
from pydantic import BaseModel, Field
from starlette.concurrency import run_in_threadpool

SOURCE = Path(__file__).resolve().parents[1] / "rover_backend" / "rtk_policy_routes.py"


class FakeBridge:
    def __init__(self) -> None:
        self.running = True
        self.status: dict[str, Any] = {
            "float_confirm_sec": 1.0,
            "fixed_hold_sec": 0.5,
            "source": "default",
        }
        self.changes: list[dict[str, Any]] = []
        self.error: RuntimeError | None = None

    def get_rtk_policy_status(self):
        return dict(self.status)

    def set_rtk_policy_config(self, changes):
        self.changes.append(changes)
        if self.error:
            raise self.error
        self.status.update(changes, source="saved")
        return dict(self.status)


def _unauthenticated():
    raise HTTPException(status_code=401, detail="Authentication required")


def load(bridge):
    tree = ast.parse(SOURCE.read_text())
    tree.body = [n for n in tree.body if not isinstance(n, (ast.Import, ast.ImportFrom))]
    namespace = {
        "__name__": "rtk_policy_routes_under_test",
        "Any": Any, "APIRouter": APIRouter, "Depends": Depends,
        "HTTPException": HTTPException, "BaseModel": BaseModel, "Field": Field,
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
    app.include_router(routes["rtk_policy_router"])
    app.dependency_overrides[_unauthenticated] = lambda: object()
    with TestClient(app) as client:
        yield client, bridge


def test_requires_login():
    routes = load(FakeBridge())
    app = FastAPI()
    app.include_router(routes["rtk_policy_router"])
    with TestClient(app) as client:
        assert client.get("/api/rtk/policy").status_code == 401
        assert client.post("/api/rtk/policy", json={"fixed_hold_sec": 1.0}).status_code == 401


def test_status(api):
    client, _ = api
    body = client.get("/api/rtk/policy").json()
    assert body["success"] is True and body["available"] is True
    assert (body["policy"]["float_confirm_sec"], body["policy"]["fixed_hold_sec"]) == (1.0, 0.5)


def test_config_sends_only_given_fields(api):
    client, bridge = api
    body = client.post("/api/rtk/policy", json={"float_confirm_sec": 2.0}).json()
    assert bridge.changes == [{"float_confirm_sec": 2.0}]
    assert body["policy"]["float_confirm_sec"] == 2.0 and body["policy"]["source"] == "saved"


@pytest.mark.parametrize(
    "payload",
    [
        {},
        {"float_confirm_sec": -0.1},
        {"float_confirm_sec": 5.01},
        {"fixed_hold_sec": 0.49},
        {"fixed_hold_sec": 10.01},
        {"fixed_hold_sec": "fast"},
    ],
)
def test_out_of_range_is_rejected_before_reaching_the_rover(api, payload):
    client, bridge = api
    assert client.post("/api/rtk/policy", json=payload).status_code == 422
    assert bridge.changes == []


def test_range_ends_are_accepted(api):
    client, bridge = api
    for payload in ({"float_confirm_sec": 0.0, "fixed_hold_sec": 10.0},
                    {"float_confirm_sec": 5.0, "fixed_hold_sec": 0.5}):
        assert client.post("/api/rtk/policy", json=payload).status_code == 200
    assert len(bridge.changes) == 2


def test_rover_rejection_is_409(api):
    client, bridge = api
    bridge.error = RuntimeError("RTK pause settings rejected: fixed_hold_sec must be ...")
    response = client.post("/api/rtk/policy", json={"fixed_hold_sec": 1.0})
    assert response.status_code == 409
    assert "rejected" in response.json()["detail"]["message"]
