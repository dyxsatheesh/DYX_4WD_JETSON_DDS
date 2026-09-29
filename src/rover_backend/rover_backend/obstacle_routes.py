"""Forward obstacle sensor API (stop and wait).

The obstacle_monitor node measures and classifies; mission_manager pauses a
RUNNING mission on danger (manual resume only) and cmd_vel_bridge holds zero
velocity. These routes only read the state and change the saved settings
(obstacle_monitor validates and persists them). All routes require login.
"""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from starlette.concurrency import run_in_threadpool

from rover_backend.auth import AuthenticatedSession, require_auth
from rover_backend.ros_bridge import ros_bridge

obstacle_router = APIRouter(
    prefix="/api/obstacle",
    tags=["obstacle"],
)


class ObstacleConfigRequest(BaseModel):
    enabled: bool | None = None
    danger_mm: int | None = None
    warning_mm: int | None = None
    port: str | None = None
    baud: int | None = None


def _require_ros_bridge() -> None:
    if not ros_bridge.running:
        raise HTTPException(
            status_code=503,
            detail={"success": False, "message": "The rover ROS bridge is not running."},
        )


@obstacle_router.get("/status")
async def obstacle_status(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    _require_ros_bridge()
    status = ros_bridge.get_obstacle_status()
    return {
        "success": True,
        "available": bool(status),
        "status": status,
    }


@obstacle_router.post("/config")
async def obstacle_config(
    request: ObstacleConfigRequest,
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Enable/disable detection or change zones / serial port (saved on the rover).

    danger_mm: the rover pauses when an object is this close (default 1500).
    warning_mm: announcement only (default 3000). Needs 300 <= danger < warning.
    """

    _require_ros_bridge()
    changes = request.model_dump(exclude_none=True)
    if not changes:
        raise HTTPException(
            status_code=422,
            detail={"success": False, "message": "No settings to change."},
        )
    try:
        status = await run_in_threadpool(ros_bridge.set_obstacle_config, changes)
    except RuntimeError as error:
        raise HTTPException(
            status_code=409,
            detail={"success": False, "message": str(error)},
        ) from error
    return {
        "success": True,
        "message": "Obstacle settings saved.",
        "status": status,
    }
