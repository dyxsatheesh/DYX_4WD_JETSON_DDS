"""Safety and compatibility contracts for the spray REST API."""

from __future__ import annotations

import ast
import logging
import math
import threading
import time
from pathlib import Path
from typing import Any

import pytest
from fastapi import APIRouter, Depends, FastAPI, HTTPException
from fastapi.testclient import TestClient
from pydantic import BaseModel
from starlette.concurrency import run_in_threadpool

PACKAGE_DIR = Path(__file__).resolve().parents[1] / "rover_backend"


class RosServiceOutcomeUnknownError(RuntimeError):
    retry_safe = False


class RetrySafeOutcomeUnknownError(RosServiceOutcomeUnknownError):
    retry_safe = True


class FakeState:
    def __init__(self) -> None:
        self.mission = {"state": "EMPTY"}

    def section(self, section_name: str) -> dict[str, Any]:
        assert section_name == "mission"
        return dict(self.mission)

    def update(self, section_name: str, **values: Any) -> None:
        assert section_name == "mission"
        self.mission.update(values)


def _load_spray_routes(bridge: Any, state: FakeState) -> dict[str, Any]:
    """Execute the production route module without importing ROS on macOS."""

    source = PACKAGE_DIR / "spray_routes.py"
    tree = ast.parse(source.read_text())
    tree.body = [
        node
        for node in tree.body
        if not isinstance(node, (ast.Import, ast.ImportFrom))
    ]
    namespace = {
        "__name__": "test_spray_routes_production",
        "logging": logging,
        "math": math,
        "threading": threading,
        "Any": Any,
        "APIRouter": APIRouter,
        "Depends": Depends,
        "HTTPException": HTTPException,
        "BaseModel": BaseModel,
        "run_in_threadpool": run_in_threadpool,
        "AuthenticatedSession": object,
        "require_auth": _unauthenticated,
        "RosServiceOutcomeUnknownError": RosServiceOutcomeUnknownError,
        "ros_bridge": bridge,
        "rover_state": state,
    }
    exec(compile(tree, str(source), "exec"), namespace)  # noqa: S102
    return namespace


def _unauthenticated() -> None:
    raise HTTPException(status_code=401, detail="Authentication required")


def authenticated_session() -> object:
    return object()


class FakeSprayBridge:
    def __init__(self) -> None:
        self.running = True
        self.calls: list[tuple[str, bool]] = []
        self.manual_error: RuntimeError | None = None
        self.enabled_error: RuntimeError | None = None
        self.config: dict[str, Any] = {
            "available": True,
            "press_pwm_us": 2000,
            "release_pwm_us": 900,
            "spraying": False,
        }

    def get_spray_config(self) -> dict[str, Any]:
        return dict(self.config)

    def set_spray_config(
        self,
        *,
        press_pwm_us: float,
        release_pwm_us: float,
    ) -> dict[str, Any]:
        self.config.update(
            press_pwm_us=int(press_pwm_us),
            release_pwm_us=int(release_pwm_us),
        )
        return dict(self.config)

    def set_manual_spray(self, enabled: bool) -> dict[str, Any]:
        self.calls.append(("manual", enabled))
        if self.manual_error is not None:
            raise self.manual_error
        return {
            "accepted": True,
            "requested": "ON" if enabled else "OFF",
            "message": "accepted",
            "config": dict(self.config),
        }

    def set_spray_enabled(self, enabled: bool) -> dict[str, Any]:
        self.calls.append(("enabled", enabled))
        if self.enabled_error is not None:
            raise self.enabled_error
        return {
            "accepted": True,
            "enabled": enabled,
            "message": "accepted",
            "config": dict(self.config),
        }


@pytest.fixture
def spray_api(monkeypatch):
    bridge = FakeSprayBridge()
    state = FakeState()
    routes = _load_spray_routes(bridge, state)

    app = FastAPI()
    app.include_router(routes["spray_router"])
    app.dependency_overrides[routes["require_auth"]] = authenticated_session

    with TestClient(app) as client:
        yield client, bridge, state

    with routes["_spray_operation_lock"]:
        routes["_cancel_manual_test_timer_locked"]()


def test_all_spray_routes_require_auth() -> None:
    routes = _load_spray_routes(FakeSprayBridge(), FakeState())
    app = FastAPI()
    app.include_router(routes["spray_router"])

    with TestClient(app) as client:
        for method, path, body in (
            ("get", "/api/spray/status", None),
            ("get", "/api/spray/config", None),
            ("post", "/api/spray/config", {"press_pwm_us": 2000, "release_pwm_us": 900}),
            ("post", "/api/spray/on", None),
            ("post", "/api/spray/off", None),
            ("post", "/api/spray/test", {"on": True}),
            ("post", "/api/spray/enable", None),
            ("post", "/api/spray/disable", None),
        ):
            response = client.request(method, path, json=body)
            assert response.status_code == 401, path


def test_status_and_config_keep_existing_success_config_wrapper(spray_api) -> None:
    client, _, _ = spray_api

    for path in ("/api/spray/status", "/api/spray/config"):
        response = client.get(path)
        assert response.status_code == 200
        assert response.json() == {
            "success": True,
            "config": {
                "available": True,
                "press_pwm_us": 2000,
                "release_pwm_us": 900,
                "spraying": False,
            },
        }

    response = client.post(
        "/api/spray/config",
        json={"press_pwm_us": 1900, "release_pwm_us": 1000},
    )
    assert response.status_code == 200
    assert response.json()["success"] is True
    assert response.json()["config"]["press_pwm_us"] == 1900


@pytest.mark.parametrize("state_name", ["RUNNING", "PAUSED", "WAITING_FOR_NEXT"])
@pytest.mark.parametrize(
    ("path", "body"),
    [
        ("/api/spray/on", None),
        ("/api/spray/test", {"on": True, "duration_s": 0.2}),
    ],
)
def test_manual_on_is_blocked_during_active_mission(
    spray_api,
    state_name: str,
    path: str,
    body: dict[str, Any] | None,
) -> None:
    client, bridge, state = spray_api
    state.update("mission", state=state_name)

    response = client.post(path, json=body)

    assert response.status_code == 409
    assert response.json()["detail"]["mission_state"] == state_name
    assert bridge.calls == []


def test_on_reports_accepted_request_not_confirmed_physical_state(spray_api) -> None:
    client, bridge, _ = spray_api

    response = client.post("/api/spray/on")

    assert response.status_code == 200
    payload = response.json()
    assert payload["accepted"] is True
    assert payload["requested"] == "ON"
    assert payload["physical_state_confirmed"] is False
    assert bridge.calls == [("manual", True)]


def test_service_rejection_is_reported_honestly(spray_api) -> None:
    client, bridge, _ = spray_api
    bridge.manual_error = RuntimeError("controller rejected: DISARMED")

    response = client.post("/api/spray/on")

    assert response.status_code == 409
    detail = response.json()["detail"]
    assert detail["success"] is False
    assert detail["outcome"] == "FAILED"
    assert "DISARMED" in detail["message"]


def test_unknown_on_outcome_is_not_marked_retry_safe(spray_api) -> None:
    client, bridge, _ = spray_api
    bridge.manual_error = RosServiceOutcomeUnknownError(
        "manual PRESS outcome is unknown"
    )

    response = client.post("/api/spray/on")

    assert response.status_code == 504
    detail = response.json()["detail"]
    assert detail["outcome"] == "UNKNOWN"
    assert detail["retry_safe"] is False


def test_unknown_off_outcome_is_honest_and_retry_safe(spray_api) -> None:
    client, bridge, _ = spray_api
    bridge.manual_error = RetrySafeOutcomeUnknownError(
        "manual RELEASE outcome is unknown"
    )

    response = client.post("/api/spray/off")

    assert response.status_code == 504
    detail = response.json()["detail"]
    assert detail["outcome"] == "UNKNOWN"
    assert detail["retry_safe"] is True


@pytest.mark.parametrize("path", ["/api/spray/enable", "/api/spray/disable"])
def test_unknown_enable_gate_outcomes_are_retry_safe(
    spray_api,
    path: str,
) -> None:
    client, bridge, _ = spray_api
    bridge.enabled_error = RetrySafeOutcomeUnknownError(
        "spray enable gate outcome is unknown"
    )

    response = client.post(path)

    assert response.status_code == 504
    detail = response.json()["detail"]
    assert detail["outcome"] == "UNKNOWN"
    assert detail["retry_safe"] is True

    if path.endswith("/disable"):
        assert bridge.calls == [("manual", False), ("enabled", False)]
    else:
        assert bridge.calls == [("enabled", True)]


@pytest.mark.parametrize("duration", [0.0, -0.1, float("inf"), float("nan")])
def test_test_duration_must_be_finite_and_positive(spray_api, duration: float) -> None:
    client, bridge, _ = spray_api

    response = client.post(
        "/api/spray/test",
        content=(
            '{"on":true,"duration_s":NaN}'
            if math.isnan(duration)
            else (
                '{"on":true,"duration_s":Infinity}'
                if duration == float("inf")
                else f'{{"on":true,"duration_s":{duration}}}'
            )
        ),
        headers={"content-type": "application/json"},
    )

    assert response.status_code == 422
    assert bridge.calls == []


def test_test_defaults_caps_and_automatically_requests_off(spray_api) -> None:
    client, bridge, _ = spray_api

    default_response = client.post("/api/spray/test", json={"on": True})
    assert default_response.status_code == 200
    assert default_response.json()["duration_s"] == pytest.approx(0.2)

    capped_response = client.post(
        "/api/spray/test",
        json={"on": True, "duration_s": 10.0},
    )
    assert capped_response.status_code == 200
    assert capped_response.json()["duration_s"] == pytest.approx(3.0)
    assert capped_response.json()["duration_capped"] is True

    short_response = client.post(
        "/api/spray/test",
        json={"on": True, "duration_s": 0.02},
    )
    assert short_response.status_code == 200

    deadline = time.monotonic() + 1.0
    while ("manual", False) not in bridge.calls and time.monotonic() < deadline:
        time.sleep(0.01)

    assert bridge.calls[-2:] == [("manual", True), ("manual", False)]


def test_explicit_off_cancels_pending_timer(spray_api) -> None:
    client, bridge, _ = spray_api
    response = client.post(
        "/api/spray/test",
        json={"on": True, "duration_s": 0.05},
    )
    assert response.status_code == 200

    response = client.post("/api/spray/off")
    assert response.status_code == 200
    time.sleep(0.10)

    assert bridge.calls == [("manual", True), ("manual", False)]


def test_off_cancels_timer_even_when_ros_bridge_is_unavailable(spray_api) -> None:
    client, bridge, _ = spray_api
    response = client.post(
        "/api/spray/test",
        json={"on": True, "duration_s": 0.05},
    )
    assert response.status_code == 200

    bridge.running = False
    response = client.post("/api/spray/off")
    assert response.status_code == 503
    time.sleep(0.10)

    assert bridge.calls == [("manual", True)]


def test_disable_releases_before_closing_master_gate(spray_api) -> None:
    client, bridge, _ = spray_api

    response = client.post("/api/spray/disable")

    assert response.status_code == 200
    assert bridge.calls == [("manual", False), ("enabled", False)]


def test_disable_does_not_close_gate_when_release_fails(spray_api) -> None:
    client, bridge, _ = spray_api
    bridge.manual_error = RuntimeError("release acknowledgement timed out")

    response = client.post("/api/spray/disable")

    assert response.status_code == 409
    assert bridge.calls == [("manual", False)]
    assert response.json()["detail"]["outcome"] == "FAILED"


def test_bridge_status_mapping_is_additive() -> None:
    source = PACKAGE_DIR / "ros_bridge.py"
    tree = ast.parse(source.read_text())
    source_class = next(
        item
        for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "RoverBackendRosNode"
    )
    method_names = {
        "_spray_actuator_value_to_pwm",
        "get_spray_config",
    }
    class_body = [
        item
        for item in source_class.body
        if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef))
        and item.name in method_names
    ]
    extracted_class = ast.ClassDef(
        name="ExtractedSprayNode",
        bases=[],
        keywords=[],
        body=class_body,
        decorator_list=[],
    )
    extracted_module = ast.Module(body=[extracted_class], type_ignores=[])
    ast.fix_missing_locations(extracted_module)
    namespace = {
        "Any": Any,
        "math": math,
        "_finite_float": lambda value: (
            float(value)
            if value is not None and math.isfinite(float(value))
            else None
        ),
    }
    exec(  # noqa: S102
        compile(extracted_module, str(source), "exec"),
        namespace,
    )
    node_class = namespace["ExtractedSprayNode"]
    node_class.SPRAY_PWM_MIN_US = 900.0
    node_class.SPRAY_PWM_MAX_US = 2000.0
    node_class._monotonic_age = staticmethod(
        lambda timestamp: time.monotonic() - timestamp
    )

    node = object.__new__(node_class)
    node._runtime_lock = threading.RLock()
    node._spray_status_monotonic = time.monotonic()
    node._spray_status = {
        "press_value": 1.0,
        "release_value": -1.0,
        "spraying": True,
        "enabled": True,
        "transaction_kind": "MANUAL",
        "manual_active": True,
        "manual_requested": True,
        "manual_deadline_remaining_sec": 0.15,
        "press_may_be_active": True,
        "release_confirmed": False,
        "mavros_connected": True,
        "px4_armed": True,
        "px4_mode": "MANUAL",
    }

    config = node_class.get_spray_config(node)

    assert config["press_pwm_us"] == 2000
    assert config["release_pwm_us"] == 900
    assert config["enabled"] is True
    assert config["transaction_kind"] == "MANUAL"
    assert config["manual_active"] is True
    assert config["manual_deadline_remaining_sec"] == pytest.approx(0.15)
    assert config["press_may_be_active"] is True
    assert config["release_confirmed"] is False
    assert config["mavros_connected"] is True
    assert config["px4_connected"] is True
    assert config["px4_armed"] is True
    assert config["px4_mode"] == "MANUAL"


def test_bridge_enable_service_timeout_is_declared_idempotent() -> None:
    source = PACKAGE_DIR / "ros_bridge.py"
    tree = ast.parse(source.read_text())
    source_class = next(
        item
        for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "RoverBackendRosNode"
    )
    method = next(
        item
        for item in source_class.body
        if isinstance(item, ast.FunctionDef)
        and item.name == "set_spray_enabled"
    )
    call = next(
        item
        for item in ast.walk(method)
        if isinstance(item, ast.Call)
        and isinstance(item.func, ast.Attribute)
        and item.func.attr == "_call_service"
    )
    keywords = {keyword.arg: keyword.value for keyword in call.keywords}

    assert isinstance(keywords["timeout_outcome_unknown"], ast.Constant)
    assert keywords["timeout_outcome_unknown"].value is True
    assert isinstance(keywords["timeout_retry_safe"], ast.Constant)
    assert keywords["timeout_retry_safe"].value is True
