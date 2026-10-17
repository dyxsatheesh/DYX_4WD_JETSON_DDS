"""Spray-servo REST API for the DYX 4WD Rover Backend.

This module exposes the runtime AUX5 spray-servo configuration to the
frontend.

The frontend works in PWM microseconds:

    1200 us -> -1.0 actuator command   (PWM_AUX_MIN5)
    1600 us ->  0.0 actuator command
    2000 us -> +1.0 actuator command   (PWM_AUX_MAX5)

The conversion and ROS acknowledgement are handled by ros_bridge.py.

This module does not command PX4 directly and does not modify mission logic.
"""

from __future__ import annotations

import logging
import math
import threading
from typing import Any

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from starlette.concurrency import run_in_threadpool

from rover_backend.auth import AuthenticatedSession, require_auth
from rover_backend.ros_bridge import RosServiceOutcomeUnknownError, ros_bridge
from rover_backend.state import rover_state

spray_router = APIRouter(
    prefix="/api/spray",
    tags=["spray"],
)

LOGGER = logging.getLogger(__name__)

ACTIVE_MISSION_STATES = {
    "RUNNING",
    "PAUSED",
    "WAITING_FOR_NEXT",
}
DEFAULT_TEST_DURATION_SEC = 0.20
MAX_TEST_DURATION_SEC = 3.0

_spray_operation_lock = threading.RLock()
_manual_test_timer: threading.Timer | None = None
_manual_timer_generation = 0


class SprayConfigRequest(BaseModel):
    """Frontend spray-servo PWM configuration."""

    press_pwm_us: float
    release_pwm_us: float


class LaserConfigRequest(BaseModel):
    """Laser (AUX6) ON/OFF PWM configuration."""

    on_pwm_us: float
    off_pwm_us: float


class SprayDurationRequest(BaseModel):
    """Mission spray hold time (servo ON interval) configuration."""

    spray_duration_sec: float


class SprayTestRequest(BaseModel):
    """Bounded manual spray test request."""

    on: bool
    duration_s: float | None = None


def _require_ros_bridge() -> None:
    if not ros_bridge.running:
        raise HTTPException(
            status_code=503,
            detail={
                "success": False,
                "message": ("The rover ROS bridge is not running."),
            },
        )


def _validate_pwm(
    value: float,
    *,
    field_name: str,
) -> float:
    try:
        pwm = float(value)
    except (TypeError, ValueError) as error:
        raise HTTPException(
            status_code=422,
            detail={
                "success": False,
                "message": (f"{field_name} must be a number."),
            },
        ) from error

    if pwm < 1200.0 or pwm > 2000.0:
        raise HTTPException(
            status_code=422,
            detail={
                "success": False,
                "message": (
                    f"{field_name} must be between " "1200 and 2000 microseconds."
                ),
            },
        )

    return pwm


async def _read_spray_config() -> dict[str, Any]:
    _require_ros_bridge()

    try:
        config = await run_in_threadpool(ros_bridge.get_spray_config)
    except RuntimeError as error:
        raise HTTPException(
            status_code=409,
            detail={
                "success": False,
                "message": str(error),
            },
        ) from error

    return config


def _require_manual_spray_allowed() -> None:
    mission = rover_state.section("mission")
    state_name = str(mission.get("state", "EMPTY")).strip().upper()

    if state_name in ACTIVE_MISSION_STATES:
        raise HTTPException(
            status_code=409,
            detail={
                "success": False,
                "operation": "manual_spray_on",
                "message": (
                    "Manual spray is unavailable while the mission is "
                    f"{state_name.lower()}. Stop the mission first."
                ),
                "mission_state": state_name,
            },
        )


def _control_error(
    *,
    operation: str,
    error: RuntimeError,
) -> HTTPException:
    outcome_unknown = isinstance(error, RosServiceOutcomeUnknownError)
    retry_safe = bool(
        getattr(
            error,
            "retry_safe",
            not outcome_unknown,
        )
    )
    return HTTPException(
        status_code=504 if outcome_unknown else 409,
        detail={
            "success": False,
            "operation": operation,
            "message": str(error),
            "outcome": "UNKNOWN" if outcome_unknown else "FAILED",
            "retry_safe": retry_safe,
        },
    )


def _cancel_manual_test_timer_locked() -> None:
    global _manual_test_timer
    global _manual_timer_generation

    _manual_timer_generation += 1
    timer = _manual_test_timer
    _manual_test_timer = None

    if timer is not None:
        timer.cancel()


def _cancel_manual_test_timer() -> None:
    with _spray_operation_lock:
        _cancel_manual_test_timer_locked()


def _automatic_manual_spray_off(generation: int) -> None:
    global _manual_test_timer

    with _spray_operation_lock:
        if generation != _manual_timer_generation:
            return

        _manual_test_timer = None

        try:
            ros_bridge.set_manual_spray(False)
        except RuntimeError:
            # The controller's independent hard watchdog remains authoritative.
            # This failure must still be visible in logs; an HTTP request no
            # longer exists to receive it.
            LOGGER.exception("Automatic manual-spray OFF request failed")


def _manual_spray_on() -> dict[str, Any]:
    with _spray_operation_lock:
        _cancel_manual_test_timer_locked()
        return ros_bridge.set_manual_spray(True)


def _manual_spray_off() -> dict[str, Any]:
    with _spray_operation_lock:
        _cancel_manual_test_timer_locked()
        return ros_bridge.set_manual_spray(False)


def _manual_spray_test(duration_sec: float) -> dict[str, Any]:
    global _manual_test_timer

    with _spray_operation_lock:
        _cancel_manual_test_timer_locked()
        result = ros_bridge.set_manual_spray(True)
        generation = _manual_timer_generation
        timer = threading.Timer(
            duration_sec,
            _automatic_manual_spray_off,
            args=(generation,),
        )
        timer.daemon = True
        _manual_test_timer = timer
        timer.start()
        return result


def _set_spray_enabled(enabled: bool) -> dict[str, Any]:
    with _spray_operation_lock:
        if not enabled:
            _cancel_manual_test_timer_locked()
            # RELEASE must be accepted before the master gate is closed. A
            # failed/uncertain OFF must never be described as safely disabled.
            ros_bridge.set_manual_spray(False)

        return ros_bridge.set_spray_enabled(enabled)


async def _run_control(
    operation: str,
    callback: Any,
) -> dict[str, Any]:
    _require_ros_bridge()

    try:
        result = await run_in_threadpool(callback)
    except RuntimeError as error:
        raise _control_error(
            operation=operation,
            error=error,
        ) from error

    return {
        "success": True,
        "operation": operation,
        **result,
    }


@spray_router.get("/status")
async def spray_status(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Return current spray-controller status/configuration."""

    config = await _read_spray_config()

    return {
        "success": True,
        "config": config,
    }


@spray_router.get("/config")
async def get_spray_config(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Return the currently active spray PWM configuration."""

    config = await _read_spray_config()

    return {
        "success": True,
        "config": config,
    }


@spray_router.post("/config")
async def set_spray_config(
    request: SprayConfigRequest,
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Update AUX5 press/release PWM values safely."""

    _require_ros_bridge()

    press_pwm_us = _validate_pwm(
        request.press_pwm_us,
        field_name="press_pwm_us",
    )

    release_pwm_us = _validate_pwm(
        request.release_pwm_us,
        field_name="release_pwm_us",
    )

    try:
        config = await run_in_threadpool(
            lambda: ros_bridge.set_spray_config(
                press_pwm_us=press_pwm_us,
                release_pwm_us=release_pwm_us,
            )
        )

    except RuntimeError as error:
        raise HTTPException(
            status_code=409,
            detail={
                "success": False,
                "message": str(error),
            },
        ) from error

    return {
        "success": True,
        "message": ("Spray PWM configuration updated successfully."),
        "config": config,
    }


@spray_router.post("/laser-config")
async def set_laser_config(
    request: LaserConfigRequest,
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Set and save the laser ON/OFF PWM (1000-2000 us, AUX6)."""

    _require_ros_bridge()

    for name, value in (
        ("on_pwm_us", request.on_pwm_us),
        ("off_pwm_us", request.off_pwm_us),
    ):
        if not math.isfinite(value) or not 1000.0 <= value <= 2000.0:
            raise HTTPException(
                status_code=422,
                detail={
                    "success": False,
                    "message": f"{name} must be between 1000 and 2000 microseconds.",
                },
            )

    if request.on_pwm_us == request.off_pwm_us:
        raise HTTPException(
            status_code=422,
            detail={
                "success": False,
                "message": "on_pwm_us and off_pwm_us must differ.",
            },
        )

    try:
        config = await run_in_threadpool(
            lambda: ros_bridge.set_laser_config(
                on_pwm_us=request.on_pwm_us,
                off_pwm_us=request.off_pwm_us,
            )
        )

    except RuntimeError as error:
        raise HTTPException(
            status_code=409,
            detail={
                "success": False,
                "message": str(error),
            },
        ) from error

    return {
        "success": True,
        "message": "Laser PWM configuration saved.",
        "config": config,
    }


@spray_router.post("/duration")
async def set_spray_duration(
    request: SprayDurationRequest,
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Set the mission spray hold time used for every marking point.

    Tune it first with POST /test (duration_s), then save the chosen value
    here. The controller rejects the change while it is not idle or while a
    mission is enabled, and bounds it by its manual override timeout.
    """

    _require_ros_bridge()

    spray_duration_sec = request.spray_duration_sec

    if (
        not math.isfinite(spray_duration_sec)
        or spray_duration_sec <= 0.0
        or spray_duration_sec > MAX_TEST_DURATION_SEC
    ):
        raise HTTPException(
            status_code=422,
            detail={
                "success": False,
                "message": (
                    "spray_duration_sec must be greater than zero and at most "
                    f"{MAX_TEST_DURATION_SEC:g} seconds."
                ),
            },
        )

    try:
        config = await run_in_threadpool(
            lambda: ros_bridge.set_spray_duration(
                spray_duration_sec=spray_duration_sec,
            )
        )

    except RuntimeError as error:
        raise HTTPException(
            status_code=409,
            detail={
                "success": False,
                "message": str(error),
            },
        ) from error

    return {
        "success": True,
        "message": "Spray hold time updated successfully.",
        "config": config,
    }


@spray_router.post("/on")
async def spray_on(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Request bounded manual spray activation.

    Acceptance confirms only that the controller accepted the request; the
    physical actuator state remains available through GET /status.
    """

    _require_manual_spray_allowed()
    response = await _run_control("manual_spray_on", _manual_spray_on)
    response["physical_state_confirmed"] = False
    return response


@spray_router.post("/off")
async def spray_off(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Cancel timers and request the idempotent manual RELEASE path."""

    _cancel_manual_test_timer()
    return await _run_control("manual_spray_off", _manual_spray_off)


@spray_router.post("/test")
async def spray_test(
    request: SprayTestRequest,
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Run or cancel a server-timed manual spray test."""

    if not request.on:
        _cancel_manual_test_timer()
        return await _run_control("manual_spray_test_off", _manual_spray_off)

    _require_manual_spray_allowed()

    requested_duration_sec = (
        DEFAULT_TEST_DURATION_SEC
        if request.duration_s is None
        else request.duration_s
    )

    if not math.isfinite(requested_duration_sec) or requested_duration_sec <= 0.0:
        raise HTTPException(
            status_code=422,
            detail={
                "success": False,
                "message": "duration_s must be a finite number greater than zero.",
            },
        )

    duration_sec = min(requested_duration_sec, MAX_TEST_DURATION_SEC)
    response = await _run_control(
        "manual_spray_test_on",
        lambda: _manual_spray_test(duration_sec),
    )
    response.update(
        {
            "duration_s": duration_sec,
            "duration_capped": duration_sec != requested_duration_sec,
            "physical_state_confirmed": False,
        }
    )
    return response


@spray_router.post("/enable")
async def spray_enable(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Open the spray controller's runtime master gate."""

    return await _run_control(
        "spray_enable",
        lambda: _set_spray_enabled(True),
    )


@spray_router.post("/disable")
async def spray_disable(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Request RELEASE, then close the runtime master gate."""

    _cancel_manual_test_timer()
    return await _run_control(
        "spray_disable",
        lambda: _set_spray_enabled(False),
    )
