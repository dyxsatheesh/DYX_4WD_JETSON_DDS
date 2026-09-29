"""Voice (TTS) REST API.

Keeps the NRP_ROS ``/api/tts/*`` contract the tablet app already implements
(getTTSStatus / controlTTS / testTTS / setTTSLanguage / get|setTTSGender in
DYX_GCS_Frontend ``useRoverTelemetry.ts``), backed by ``voice.voice_service``.
All routes require login. Settings persist on the Jetson.
"""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from starlette.concurrency import run_in_threadpool

from rover_backend.auth import AuthenticatedSession, require_auth
from rover_backend.voice import voice_announcer, voice_service
from rover_backend.voice_phrases import LANGUAGES

voice_router = APIRouter(
    prefix="/api/tts",
    tags=["voice"],
)


class VoiceControlRequest(BaseModel):
    enabled: bool


class VoiceTestRequest(BaseModel):
    message: str | None = None


class VoiceLanguageRequest(BaseModel):
    language: str


class VoiceGenderRequest(BaseModel):
    gender: str


class VoiceDeviceRequest(BaseModel):
    audio_device: str


def _invalid(message: str) -> HTTPException:
    return HTTPException(
        status_code=422,
        detail={"success": False, "message": message},
    )


@voice_router.get("/status")
async def voice_status(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    status = await run_in_threadpool(voice_service.status)
    return {"success": True, **status}


@voice_router.post("/control")
async def voice_control(
    request: VoiceControlRequest,
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Enable or disable announcements (saved on the rover)."""

    settings = await run_in_threadpool(voice_service.set_enabled, request.enabled)
    action = "enabled" if settings.enabled else "disabled"
    return {
        "success": True,
        "enabled": settings.enabled,
        "message": f"Voice output {action}",
    }


@voice_router.post("/test")
async def voice_test(
    request: VoiceTestRequest | None = None,
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Speak a test phrase (or a custom message) on the rover speaker."""

    if not voice_service.settings().enabled:
        raise HTTPException(
            status_code=409,
            detail={"success": False, "message": "Voice output is disabled."},
        )
    message = (request.message or "").strip() if request else ""
    if len(message) > 200:
        raise _invalid("message must be at most 200 characters.")
    if message:
        voice_service.speak_text(message)
    else:
        voice_service.say("voice_test")
    return {"success": True, "message": "Voice test queued"}


@voice_router.get("/languages")
async def voice_languages(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    return {
        "success": True,
        "languages": [{"code": code, "name": name} for code, name in LANGUAGES.items()],
    }


@voice_router.post("/language")
async def voice_set_language(
    request: VoiceLanguageRequest,
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    try:
        settings = await run_in_threadpool(voice_service.set_language, request.language)
    except ValueError as error:
        raise _invalid(str(error)) from error
    # Render the fixed phrases for the new language in the background.
    voice_service.precache(voice_announcer.static_phrases())
    return {
        "success": True,
        "language": settings.language,
        "message": f"Language set to {LANGUAGES[settings.language]}",
    }


@voice_router.get("/gender")
async def voice_get_gender(
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    return {"success": True, "gender": voice_service.settings().gender}


@voice_router.post("/gender")
async def voice_set_gender(
    request: VoiceGenderRequest,
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Male/female applies to the cloud voices (Tamil, and fallbacks).

    The offline Piper voices (English, Hindi) are single-voice models.
    """

    try:
        settings = await run_in_threadpool(voice_service.set_gender, request.gender)
    except ValueError as error:
        raise _invalid(str(error)) from error
    return {"success": True, "gender": settings.gender, "message": "Voice gender updated"}


@voice_router.post("/device")
async def voice_set_device(
    request: VoiceDeviceRequest,
    _session: AuthenticatedSession = Depends(require_auth),
) -> dict[str, Any]:
    """Select the ALSA playback device (e.g. plughw:CARD=Device,DEV=0)."""

    try:
        settings = await run_in_threadpool(
            voice_service.set_audio_device, request.audio_device
        )
    except ValueError as error:
        raise _invalid(str(error)) from error
    return {"success": True, "audio_device": settings.audio_device}
