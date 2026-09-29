"""Voice announcements: phrases, service (engines/cache/playback), event rules, API."""

from __future__ import annotations

import json
import re
import threading
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from rover_backend import voice_phrases as phrases
from rover_backend.voice import VoiceService
from rover_backend.voice_events import VoiceAnnouncer


# ============================================================ phrases


def test_every_phrase_exists_in_every_language_with_the_same_placeholders():
    for key, variants in phrases.PHRASES.items():
        assert set(variants) == set(phrases.LANGUAGES), key
        fields = {code: set(re.findall(r"{(\w+)}", text)) for code, text in variants.items()}
        assert len({frozenset(v) for v in fields.values()}) == 1, key


def test_render_and_unknown_language_falls_back_to_english():
    assert phrases.render("heading_to_target", "en", n=3) == "Heading to target 3."
    assert "3" in phrases.render("target_done", "ta", n=3)
    assert phrases.render("target_done", "fr", n=2) == "Target 2 done."


@pytest.mark.parametrize("hour,key", [(6, "good_morning"), (13, "good_afternoon"), (20, "good_evening")])
def test_greeting_by_hour(hour, key):
    assert phrases.greeting_key(hour) == key


# ============================================================ service


class FakeProcesses:
    """Stands in for piper / edge-tts / ffmpeg / aplay / nice."""

    def __init__(self, *, fail: set[str] | None = None) -> None:
        self.calls: list[list[str]] = []
        self.fail = fail or set()

    def __call__(self, command, **kwargs):
        args = list(command)
        if args[0].endswith("nice"):
            args = args[3:]
        self.calls.append(args)
        tool = Path(args[0]).name
        if tool in self.fail:
            return SimpleNamespace(returncode=1, stderr=f"{tool} broke")
        if tool == "piper":
            Path(args[args.index("-f") + 1]).write_bytes(b"R" * 500)
        elif tool == "edge-tts":
            Path(args[args.index("--write-media") + 1]).write_bytes(b"M" * 500)
        elif tool == "ffmpeg":
            Path(args[-1]).write_bytes(b"W" * 500)
        return SimpleNamespace(returncode=0, stderr="")

    def tools(self) -> list[str]:
        return [Path(call[0]).name for call in self.calls]


def make_service(tmp_path, *, available=("piper", "edge-tts", "ffmpeg", "aplay", "nice"), fail=None, models=("en", "hi")):
    model_dir = tmp_path / "models"
    model_dir.mkdir(exist_ok=True)
    names = {"en": "en_US-ryan-medium.onnx", "hi": "hi_IN-rohan-medium.onnx"}
    for code in models:
        (model_dir / names[code]).write_bytes(b"model")
    processes = FakeProcesses(fail=fail)
    clock = {"t": 100.0}
    service = VoiceService(
        settings_path=tmp_path / "voice.json",
        cache_dir=tmp_path / "cache",
        piper_model_dir=model_dir,
        runner=processes,
        which=lambda name: f"/usr/bin/{name}" if name in available else None,
        clock=lambda: clock["t"],
    )
    return service, processes, clock


def drain(service) -> list[bool]:
    played = []
    while not service._queue.empty():
        played.append(service.handle(service._queue.get_nowait()))
    return played


def test_english_uses_piper_then_plays_on_the_alsa_device(tmp_path):
    service, processes, _ = make_service(tmp_path)
    service.set_audio_device("plughw:CARD=Device,DEV=0")
    service.flush()

    service.say("heading_to_target", n=4)
    assert drain(service) == [True]

    piper, aplay = processes.calls
    assert Path(piper[0]).name == "piper" and piper[piper.index("-m") + 1].endswith("en_US-ryan-medium.onnx")
    assert aplay[:4] == ["/usr/bin/aplay", "-q", "-D", "plughw:CARD=Device,DEV=0"]


def test_second_time_plays_from_cache_without_synthesis(tmp_path):
    service, processes, clock = make_service(tmp_path)
    service.say("target_done", n=1)
    drain(service)
    clock["t"] += 10
    service.say("target_done", n=1)
    drain(service)

    assert processes.tools() == ["piper", "aplay", "aplay"]


def test_synthesis_runs_at_lowest_cpu_priority(tmp_path):
    service, _, _ = make_service(tmp_path)
    seen = []
    service._run_process = lambda command, **kw: (seen.append(list(command)), FakeProcesses()(command, **kw))[1]
    service.say("mission_started")
    drain(service)
    assert seen[0][:3] == ["/usr/bin/nice", "-n", "19"]


def test_tamil_uses_edge_tts_converted_to_wav(tmp_path):
    service, processes, _ = make_service(tmp_path)
    service.set_language("ta")
    service.flush()

    service.say("target_done", n=2)
    assert drain(service) == [True]

    assert processes.tools() == ["edge-tts", "ffmpeg", "aplay"]
    edge = processes.calls[0]
    assert edge[edge.index("--voice") + 1] == "ta-IN-ValluvarNeural"
    assert "2" in edge[edge.index("--text") + 1]


def test_female_voice_for_cloud_engine(tmp_path):
    service, processes, _ = make_service(tmp_path)
    service.set_language("ta")
    service.set_gender("female")
    service.flush()
    service.say("mission_completed")
    drain(service)
    assert "ta-IN-PallaviNeural" in processes.calls[0]


def test_tamil_offline_falls_back_to_english_piper(tmp_path):
    service, processes, _ = make_service(tmp_path, fail={"edge-tts"})
    service.set_language("ta")
    service.flush()

    service.say("target_done", n=5)
    assert drain(service) == [True]

    assert processes.tools() == ["edge-tts", "piper", "aplay"]


def test_nothing_available_reports_error_and_never_raises(tmp_path):
    service, processes, _ = make_service(tmp_path, available=("aplay",))
    service.say("mission_started")
    assert drain(service) == [False]
    assert "no speech engine" in service.last_error


def test_disabled_speaks_nothing(tmp_path):
    service, processes, _ = make_service(tmp_path)
    service.set_enabled(False)
    service.say("mission_started")
    assert drain(service) == []
    assert processes.calls == []


def test_same_phrase_within_three_seconds_is_spoken_once(tmp_path):
    service, processes, clock = make_service(tmp_path)
    service.say("mission_paused")
    service.say("mission_paused")
    assert drain(service) == [True, False]
    clock["t"] += 3.5
    service.say("mission_paused")
    assert drain(service) == [True]


def test_priority_phrase_jumps_the_queue(tmp_path):
    service, _, _ = make_service(tmp_path)
    service.say("heading_to_target", n=1)
    service.say("target_done", n=1)
    service.say("emergency_stop", priority=True)
    assert [u.fallback_text for u in list(service._queue.queue)] == ["Emergency stop."]


def test_settings_persist_across_restart_and_reject_bad_values(tmp_path):
    service, _, _ = make_service(tmp_path)
    service.set_language("hi")
    service.set_gender("female")
    service.set_enabled(False)
    service.set_audio_device("plughw:CARD=USB,DEV=0")

    reloaded, _, _ = make_service(tmp_path)
    settings = reloaded.settings()
    assert (settings.language, settings.gender, settings.enabled, settings.audio_device) == (
        "hi", "female", False, "plughw:CARD=USB,DEV=0",
    )
    with pytest.raises(ValueError):
        service.set_language("fr")
    with pytest.raises(ValueError):
        service.set_gender("robot")


def test_corrupt_settings_file_uses_defaults(tmp_path):
    (tmp_path / "voice.json").write_text("{not json")
    service, _, _ = make_service(tmp_path)
    assert service.settings().enabled is True
    assert service.settings().language == "en"


def test_precache_renders_in_background(tmp_path):
    service, processes, _ = make_service(tmp_path)
    service.precache([("heading_to_target", {"n": n}) for n in (1, 2)]).join(timeout=5)
    assert processes.tools() == ["piper", "piper"]
    service.say("heading_to_target", n=2)
    drain(service)
    assert processes.tools() == ["piper", "piper", "aplay"]


def test_status_reports_engines_and_tools(tmp_path):
    service, _, _ = make_service(tmp_path, models=("en",))
    status = service.status()
    assert status["engines"] == {"en": "piper", "ta": "edge-tts", "hi": "edge-tts"}
    assert status["tools"]["aplay"] is True
    assert status["bluetooth_warmup_enabled"] is False


# ============================================================ event rules


class Recorder:
    def __init__(self) -> None:
        self.spoken: list[tuple[str, dict[str, Any], bool]] = []

    def __call__(self, key, *, priority=False, **params):
        self.spoken.append((key, params, priority))

    def keys(self) -> list[str]:
        return [key for key, _, _ in self.spoken]


def announcer(**kw):
    recorder = Recorder()
    clock = {"t": 1000.0}
    return VoiceAnnouncer(recorder, clock=lambda: clock["t"], hour=lambda: 9, **kw), recorder, clock


def status(state, index=None, **extra):
    return {"state": state, "current_point_index": index, "mission_run_id": "run-1", **extra}


def test_backend_start_learns_state_silently():
    a, r, _ = announcer()
    a.on_mission_status(status("RUNNING", 2))
    assert r.spoken == []


def test_start_announces_started_then_heading_to_first_target():
    a, r, _ = announcer()
    a.on_mission_status(status("READY"))
    a.on_mission_status(status("RUNNING", 0))
    assert r.spoken == [("mission_started", {}, False), ("heading_to_target", {"n": 1}, False)]


def test_auto_next_point_announces_heading():
    a, r, _ = announcer()
    a.on_mission_status(status("RUNNING", 0))
    a.on_mission_status(status("RUNNING", 0))
    a.on_mission_status(status("RUNNING", 1))
    assert r.spoken == [("heading_to_target", {"n": 2}, False)]


def test_manual_next_announces_heading_only():
    a, r, _ = announcer()
    a.on_mission_status(status("WAITING_FOR_NEXT", 1))
    a.on_mission_status(status("RUNNING", 1))
    assert r.keys() == ["heading_to_target"]


@pytest.mark.parametrize(
    "reason,key",
    [
        ("ESTOP", "emergency_stop"),
        ("RTK_FLOAT", "paused_rtk_float"),
        ("RTK_LOST", "paused_rtk_lost"),
        ("PX4_CONTROL_LOST", "paused_vehicle_control"),
        ("ODOM_STALE", "paused_position"),
        ("OPERATOR", "mission_paused"),
    ],
)
def test_pause_reason_is_spoken_with_priority(reason, key):
    a, r, _ = announcer()
    a.on_mission_status(status("RUNNING", 0))
    a.on_mission_status(status("PAUSED", 0, pause_reason=reason))
    assert r.spoken == [(key, {}, True)]


def test_resume_says_resumed_then_heading():
    a, r, _ = announcer()
    a.on_mission_status(status("PAUSED", 3))
    a.on_mission_status(status("RUNNING", 3))
    assert r.keys() == ["mission_resumed", "heading_to_target"]


def test_completion_and_stop():
    a, r, _ = announcer()
    a.on_mission_status(status("RUNNING", 0))
    a.on_mission_status(status("COMPLETED"))
    a.on_mission_status(status("EMPTY"))  # auto cleanup after completion: silent
    assert r.keys() == ["mission_completed"]

    a2, r2, _ = announcer()
    a2.on_mission_status(status("WAITING_FOR_NEXT", 1))
    a2.on_mission_status(status("EMPTY"))
    assert r2.keys() == ["mission_stopped"]


def test_ready_precaches_per_tool():
    items = []
    a, r, _ = announcer(precache=items.extend)
    a.on_mission_status(status("EMPTY"))
    a.on_mission_status(status("READY", total_points=2, marking_tool="LASER"))
    assert r.keys() == ["mission_ready"]
    assert ("target_laser_mark", {"n": 2}) in items
    assert ("target_done", {"n": 1}) not in items
    assert len(items) == 6


def point_event(event, index, *, tool="SPRAY", reason=None):
    return {
        "event": event,
        "mission_run_id": "run-1",
        "point_id": f"P{index + 1:04d}",
        "point_index": index,
        "reason": reason,
        "spray": {"tool": tool},
    }


@pytest.mark.parametrize(
    "event,tool,reason,key",
    [
        ("COMPLETED", "SPRAY", None, "target_done"),
        ("COMPLETED", "LASER", None, "target_laser_mark"),
        ("ACCURACY_FAILED", "LASER", "RAW_GNSS_RADIAL_EXCEEDS_TOLERANCE (raw=45mm)", "target_laser_do_not_mark"),
        ("ACCURACY_FAILED", "LASER", "RAW_GNSS_UNAVAILABLE:GNSS_NOT_RTK_FIXED", "target_failed"),
        ("ACCURACY_FAILED", "SPRAY", "RAW_GNSS_RADIAL_EXCEEDS_TOLERANCE", "target_failed"),
        ("SKIPPED", "SPRAY", "OPERATOR_SKIP", "target_skipped"),
    ],
)
def test_point_events(event, tool, reason, key):
    a, r, _ = announcer()
    a.on_point_event(point_event(event, 2, tool=tool, reason=reason))
    assert r.spoken == [(key, {"n": 3}, False)]


def test_point_event_spoken_once_and_accuracy_achieved_silent():
    a, r, _ = announcer()
    a.on_point_event(point_event("ACCURACY_ACHIEVED", 0))
    a.on_point_event(point_event("COMPLETED", 0))
    a.on_point_event(point_event("COMPLETED", 0))
    assert r.keys() == ["target_done"]


def test_rtk_transitions_with_cooldown_and_quiet_during_mission():
    a, r, clock = announcer()
    a.on_gps_fix_type(3)          # GPS only at boot: nothing to report
    a.on_gps_fix_type(6)
    assert r.keys() == ["rtk_fixed"]
    clock["t"] += 5
    a.on_gps_fix_type(5)          # inside the 12 s cooldown: held back
    assert r.keys() == ["rtk_fixed"]
    clock["t"] += 10
    a.on_gps_fix_type(5)
    assert r.keys() == ["rtk_fixed", "rtk_float"]

    a.on_mission_status(status("RUNNING", 0))
    clock["t"] += 20
    a.on_gps_fix_type(0)          # mission pause phrase covers it
    assert r.keys() == ["rtk_fixed", "rtk_float"]


def test_battery_edges_and_rearm():
    a, r, _ = announcer()
    for pct in (60, 20, 19, 10, 9, 30, 18):
        a.on_battery_percent(pct)
    assert r.spoken == [
        ("battery_low", {"pct": 20}, False),
        ("battery_critical", {"pct": 10}, True),
        ("battery_low", {"pct": 18}, False),
    ]


def test_vehicle_link():
    a, r, _ = announcer()
    a.on_vehicle_connected(True)
    assert r.keys() == ["startup", "good_morning"]
    a.on_vehicle_connected(False)           # not in a mission: silent
    a.on_vehicle_connected(True)
    assert r.keys() == ["startup", "good_morning"]
    a.on_mission_status(status("RUNNING", 0))
    a.on_vehicle_connected(False)
    a.on_vehicle_connected(True)
    assert r.keys()[-2:] == ["connection_lost", "connection_restored"]
    assert r.spoken[-2][2] is True


def test_handlers_never_raise():
    def broken(*a, **k):
        raise RuntimeError("speaker on fire")

    a = VoiceAnnouncer(broken)
    a.on_vehicle_connected(True)
    a.on_mission_status({"state": "READY"})
    a.on_point_event(point_event("COMPLETED", 0))
    a.on_battery_percent(5)
    a.on_gps_fix_type(6)


# ============================================================ API


def _voice_routes():
    """Import voice_routes; stub login only where argon2 is not installed.

    The real rover_backend.auth needs argon2 (present on the Jetson, not on
    every dev machine). The stub keeps the same 401-without-login contract.
    """
    import importlib
    import sys
    import types

    try:
        import argon2  # noqa: F401
    except ImportError:
        if "rover_backend.auth" not in sys.modules:
            from fastapi import HTTPException

            stub = types.ModuleType("rover_backend.auth")

            def require_auth():
                raise HTTPException(status_code=401, detail="Authentication required")

            stub.require_auth = require_auth
            stub.AuthenticatedSession = object
            sys.modules["rover_backend.auth"] = stub
    return importlib.import_module("rover_backend.voice_routes")


@pytest.fixture
def voice_api(tmp_path, monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    voice_routes = _voice_routes()

    service, processes, _ = make_service(tmp_path)
    monkeypatch.setattr(voice_routes, "voice_service", service)
    app = FastAPI()
    app.include_router(voice_routes.voice_router)
    app.dependency_overrides[voice_routes.require_auth] = lambda: object()
    with TestClient(app) as client:
        yield client, service


def test_api_requires_login(tmp_path):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    voice_routes = _voice_routes()

    app = FastAPI()
    app.include_router(voice_routes.voice_router)
    with TestClient(app) as client:
        assert client.get("/api/tts/status").status_code == 401


def test_api_status_matches_frontend_contract(voice_api):
    client, _ = voice_api
    body = client.get("/api/tts/status").json()
    for field in ("success", "enabled", "engine", "language"):
        assert field in body


def test_api_control_language_gender_test(voice_api):
    client, service = voice_api
    assert client.post("/api/tts/control", json={"enabled": False}).json()["enabled"] is False
    assert client.post("/api/tts/test", json={}).status_code == 409
    client.post("/api/tts/control", json={"enabled": True})

    body = client.post("/api/tts/language", json={"language": "ta"}).json()
    assert body["language"] == "ta" and service.settings().language == "ta"
    assert client.post("/api/tts/language", json={"language": "fr"}).status_code == 422

    assert client.post("/api/tts/gender", json={"gender": "female"}).json()["gender"] == "female"
    assert client.get("/api/tts/gender").json()["gender"] == "female"
    assert client.post("/api/tts/gender", json={"gender": "x"}).status_code == 422

    assert client.post("/api/tts/test", json={"message": "hello rover"}).json()["success"] is True
    assert [code for code in (c["code"] for c in client.get("/api/tts/languages").json()["languages"])] == ["en", "ta", "hi"]
    assert client.post("/api/tts/device", json={"audio_device": "plughw:CARD=USB,DEV=0"}).json()["audio_device"] == "plughw:CARD=USB,DEV=0"


# ============================================================ obstacle voice


def obstacle(**overrides):
    payload = {"enabled": True, "healthy": True, "zone": "clear", "distance_mm": None}
    payload.update(overrides)
    return payload


def test_obstacle_warning_spoken_on_entering_warning_while_running():
    a, r, clock = announcer()
    a.on_mission_status(status("RUNNING", 0))
    a.on_obstacle_status(obstacle(zone="warning", distance_mm=2460))
    a.on_obstacle_status(obstacle(zone="warning", distance_mm=2300))  # still in zone
    assert r.spoken == [("obstacle_ahead", {"cm": 250}, False)]

    a.on_obstacle_status(obstacle(zone="clear"))
    clock["t"] += 3
    a.on_obstacle_status(obstacle(zone="warning", distance_mm=2000))  # inside cooldown
    assert len(r.spoken) == 1
    clock["t"] += 10
    a.on_obstacle_status(obstacle(zone="clear"))
    a.on_obstacle_status(obstacle(zone="warning", distance_mm=2000))
    assert r.keys() == ["obstacle_ahead", "obstacle_ahead"]


def test_obstacle_warning_silent_when_not_running_or_disabled():
    a, r, _ = announcer()
    a.on_mission_status(status("WAITING_FOR_NEXT", 1))
    a.on_obstacle_status(obstacle(zone="warning", distance_mm=2000))
    a.on_mission_status(status("RUNNING", 1))
    r.spoken.clear()
    a.on_obstacle_status(obstacle(enabled=False, zone="warning", distance_mm=2000))
    assert r.spoken == []


def test_obstacle_pause_reason_spoken_with_priority():
    a, r, _ = announcer()
    a.on_mission_status(status("RUNNING", 0))
    a.on_mission_status(status("PAUSED", 0, pause_reason="OBSTACLE"))
    assert r.spoken == [("obstacle_paused", {}, True)]


def test_obstacle_sensor_lost_once_during_mission():
    a, r, _ = announcer()
    a.on_obstacle_status(obstacle(healthy=False, zone="unknown"))   # idle: silent
    a.on_mission_status(status("RUNNING", 0))
    a.on_obstacle_status(obstacle(healthy=False, zone="unknown"))
    a.on_obstacle_status(obstacle(healthy=False, zone="unknown"))
    a.on_obstacle_status(obstacle())
    a.on_obstacle_status(obstacle(healthy=False, zone="unknown"))
    assert r.keys() == ["obstacle_sensor_lost", "obstacle_sensor_lost"]
