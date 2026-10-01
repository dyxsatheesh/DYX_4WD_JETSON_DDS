"""The auto recorder must already be recording when START is pressed.

`ros2 bag record` needs 0.4-2.6 s to subscribe; START now takes well under a
second. On the 2026-09-26 bags the recorder was started by START itself and
missed START and first motion entirely. It now arms as soon as START may
follow, keeps the session when START is observed, and discards it otherwise.
"""

import importlib.util
import json
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
_SPEC = importlib.util.spec_from_file_location(
    "bag_autorecord", REPO_ROOT / "scripts" / "bag_autorecord.py"
)
bag = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(bag)


class FakeRecorder:
    def __init__(self):
        self.session_open = False
        self.started = False
        self.arm_t = 0.0
        self.calls = []

    def start(self, mission):
        self.calls.append(("start", mission.get("state")))
        self.session_open = True
        self.started = False
        self.arm_t = bag.time.time()

    def mark_started(self, mission):
        self.calls.append(("mark_started", mission.get("state")))
        self.started = True

    def stop(self, reason, *, discard=False):
        self.calls.append(("discard" if discard else "stop", reason))
        self.session_open = False
        self.started = False


def mission(state, **extra):
    return {"state": state, "start_stage": extra.pop("start_stage", "IDLE"), **extra}


@pytest.mark.parametrize("state", ["LOADED", "PREPARING", "READY", "COMPLETED"])
def test_arms_before_start_whenever_start_may_follow(state):
    recorder = FakeRecorder()
    bag.step_session(recorder, mission(state))
    assert recorder.calls == [("start", state)]
    assert recorder.session_open and not recorder.started


def test_arms_after_archive_for_the_one_tap_rerun():
    recorder = FakeRecorder()
    bag.step_session(recorder, mission("EMPTY", terminal_cleanup_status="ARCHIVED"))
    assert recorder.calls == [("start", "EMPTY")]


def test_idle_empty_rover_does_not_record():
    recorder = FakeRecorder()
    bag.step_session(recorder, mission("EMPTY"))
    bag.step_session(recorder, mission("ERROR"))
    assert recorder.calls == [] and not recorder.session_open


def test_armed_session_is_kept_from_the_first_start_stage():
    recorder = FakeRecorder()
    bag.step_session(recorder, mission("READY"))
    bag.step_session(recorder, mission("READY", start_stage="ZERO_SETPOINT_SETTLE"))
    bag.step_session(recorder, mission("RUNNING", start_stage="RUNNING"))
    assert recorder.calls == [("start", "READY"), ("mark_started", "READY")]
    assert recorder.started


def test_completion_finalises_then_rearms_for_a_rerun():
    recorder = FakeRecorder()
    bag.step_session(recorder, mission("READY"))
    bag.step_session(recorder, mission("RUNNING"))
    bag.step_session(recorder, mission("COMPLETED"))  # kept bundle finalised
    bag.step_session(recorder, mission("COMPLETED"))  # fresh armed session
    assert recorder.calls == [
        ("start", "READY"),
        ("mark_started", "RUNNING"),
        ("stop", "mission COMPLETED"),
        ("start", "COMPLETED"),
    ]
    assert recorder.session_open and not recorder.started


def test_armed_session_without_start_is_discarded_when_mission_goes_away():
    recorder = FakeRecorder()
    bag.step_session(recorder, mission("READY"))
    bag.step_session(recorder, mission("EMPTY"))
    assert recorder.calls[-1][0] == "discard"
    assert not recorder.session_open


def test_armed_session_is_discarded_after_the_arm_window(monkeypatch):
    recorder = FakeRecorder()
    bag.step_session(recorder, mission("READY"))
    monkeypatch.setattr(bag.time, "time", lambda: recorder.arm_t + bag.ARM_MAX_S + 1)
    bag.step_session(recorder, mission("READY"))
    assert recorder.calls[-1] == ("discard", "arm window expired without START")
    # The next poll re-arms.
    bag.step_session(recorder, mission("READY"))
    assert recorder.calls[-1] == ("start", "READY")


def test_a_started_session_is_never_discarded():
    recorder = FakeRecorder()
    bag.step_session(recorder, mission("READY"))
    bag.step_session(recorder, mission("RUNNING"))
    bag.step_session(recorder, mission("EMPTY"))
    assert ("discard" not in {c[0] for c in recorder.calls})
    assert recorder.calls[-1] == ("stop", "mission EMPTY")


def test_reconcile_removes_armed_leftovers_and_keeps_started_ones(tmp_path, monkeypatch):
    monkeypatch.setattr(bag, "log", lambda _msg: None)

    def bundle(name, status):
        path = tmp_path / name
        path.mkdir()
        (path / bag.MANIFEST_NAME).write_text(
            json.dumps({"outcome": {"status": status, "recorder_end": None}})
        )
        return path

    armed = bundle("armed_20260927_100000", "ARMED")
    recording = bundle("recording_20260927_100100", "RECORDING")
    bag.reconcile_incomplete(str(tmp_path))
    assert not armed.exists()
    assert recording.exists()
    assert json.loads((recording / bag.MANIFEST_NAME).read_text())["outcome"]["status"] == "INCOMPLETE"


def test_rearm_in_the_same_second_never_reuses_the_kept_bundle(tmp_path, monkeypatch):
    class Proc:
        pid = 1

        def poll(self):
            return None

    monkeypatch.setattr(bag, "BAGS_DIR", str(tmp_path))
    monkeypatch.setattr(bag, "log", lambda _msg: None)
    monkeypatch.setattr(bag, "_preflight_free_space", lambda _d: True)
    monkeypatch.setattr(bag, "_environment", lambda: {})
    monkeypatch.setattr(bag, "_enforce_retention", lambda *a, **k: None)
    monkeypatch.setattr(bag.subprocess, "Popen", lambda *a, **k: Proc())
    monkeypatch.setattr(bag.time, "sleep", lambda _s: None)
    frozen = bag._now_utc()
    monkeypatch.setattr(bag, "_now_utc", lambda: frozen)

    first = bag.Recorder()
    first.start({"state": "READY", "mission_id": "m1"})
    second = bag.Recorder()
    second.start({"state": "COMPLETED", "mission_id": "m1"})
    assert first.bundle_dir != second.bundle_dir
    assert Path(second.bundle_dir).name == Path(first.bundle_dir).name + "_2"
