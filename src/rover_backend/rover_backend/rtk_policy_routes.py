"""RTK FLOAT pause settings API (operator-approved 2026-10-03).

mission_manager pauses a RUNNING mission only after RTK FLOAT has lasted
``float_confirm_sec`` and auto-resumes once RTK FIXED has held for
``fixed_hold_sec``. These routes read and change those two values;
mission_manager validates, saves and applies them. All routes require login.
"""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, Field
from starlette.concurrency import run_in_threadpool

from rover_backend.auth import AuthenticatedSession, require_auth
from rover_backend.ros_bridge import ros_bridge


rtk_policy_router = APIRouter(
    prefix="/api/rtk/policy",
    tags=["rtk"],
)


class RtkPolicyConfigRequest(BaseModel):
    float_confirm_sec: float | None = Field(default=None, ge=0.0, le=5.0)
    fixed_hold_sec: float | None = Field(default=None, ge=0.5, le=10.0)


def _require_ros_bridge() -> None:
    if not ros_bridge.running:
        raise HTTPException(
            status_code=503,
            detail={"success": False, "message": "The rover ROS bridge is not running."},
        )


@rtk_policy_router.get("")
async def rtk_policy_status(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    _require_ros_bridge()
    status = ros_bridge.get_rtk_policy_status()
    return {
        "success": True,
        "available": bool(status),
        "policy": status,
    }


@rtk_policy_router.post("")
async def rtk_policy_config(
    request: RtkPolicyConfigRequest,
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Change the RTK FLOAT pause timing (saved on the rover, applied at once).

    float_confirm_sec: FLOAT must last this long before the mission pauses
    (0 to 5 s, default 1.0; 0 pauses on the first FLOAT sample).
    fixed_hold_sec: FIXED must hold this long before a FLOAT pause
    auto-resumes (0.5 to 10 s, default 0.5). A field left out is unchanged.
    """
    _require_ros_bridge()
    changes = request.model_dump(exclude_none=True)
    if not changes:
        raise HTTPException(
            status_code=422,
            detail={"success": False, "message": "No settings to change."},
        )
    try:
        status = await run_in_threadpool(ros_bridge.set_rtk_policy_config, changes)
    except RuntimeError as error:
        raise HTTPException(
            status_code=409,
            detail={"success": False, "message": str(error)},
        ) from error
    return {
        "success": True,
        "message": "RTK pause settings saved.",
        "policy": status,
    }
