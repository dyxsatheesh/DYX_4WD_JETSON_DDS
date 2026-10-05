"""Upgrade Extension phase 1: versioned settings, in-place update, route guards.

docs/UPGRADE_EXTENSION_PLAN.md sections 4, 5.1 and 7 (tests 1-6).
"""

from __future__ import annotations

import ast
import asyncio
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi import HTTPException

from rover_backend import extension_settings as ext
from rover_backend import mission_store as mission_store_module

SOURCE = Path(__file__).resolve().parents[1] / "rover_backend"
CSV = b"latitude,longitude\n13.1,80.2\n13.2,80.3\n"


@pytest.fixture(autouse=True)
def _defaults_store(tmp_path, monkeypatch):
    """Never read or write the real ~/.ros defaults file."""

    store = ext.ExtensionDefaultsStore(tmp_path / "extension_defaults.json")
    monkeypatch.setattr(ext, "extension_defaults_store", store)
    return store


# ── extension_settings ─────────────────────────────────────────────────────


def test_operator_values_are_kept_inside_bounds():
    settings = ext.from_request(
        enabled=True,
        distance_m=1.0,
        trigger_distance_m=5.0,
        source="operator",
        revision=2,
    )
    assert settings == ext.ExtensionSettings(True, 1.0, 5.0, "OPERATOR", 2)


@pytest.mark.parametrize(
    ("distance", "trigger"),
    [(0.99, 3.0), (20.01, 3.0), (4.0, 0.99), (4.0, 5.01), (float("nan"), 3.0), (True, 3.0)],
)
def test_operator_values_outside_bounds_are_refused(distance, trigger):
    with pytest.raises(ext.ExtensionSettingsError):
        ext.from_request(
            enabled=True,
            distance_m=distance,
            trigger_distance_m=trigger,
            source="OPERATOR",
            revision=1,
        )


def test_unmarked_request_gets_defaults():
    settings = ext.from_request(
        enabled=True, distance_m=3.5, trigger_distance_m=2.0, source=None, revision=1
    )
    assert settings == ext.ExtensionSettings(True, 4.0, 3.0, "DEFAULT", 1)


def test_disabled_keeps_nothing():
    settings = ext.from_request(
        enabled=False, distance_m=6.0, trigger_distance_m=2.0, source="OPERATOR", revision=4
    )
    assert settings == ext.ExtensionSettings(False, None, None, None, 4)
    fields = settings.metadata_fields()
    assert fields["extension_mode"] == "DISABLE"
    assert fields["dummy_point_distance_m"] is None
    assert fields["dummy_point_distance_source"] is None


def test_legacy_metadata_converts_at_revision_zero():
    old_tablet = ext.from_metadata(
        {"extension_mode": "ENABLE", "dummy_point_distance_m": 3.5}
    )
    marked = ext.from_metadata(
        {
            "extension_mode": "ENABLE",
            "dummy_point_distance_m": 6.0,
            "dummy_point_distance_source": "OPERATOR",
        }
    )
    assert old_tablet == ext.ExtensionSettings(True, 4.0, 3.0, "DEFAULT", 0)
    assert marked == ext.ExtensionSettings(True, 6.0, 3.0, "OPERATOR", 0)


def test_metadata_fields_keep_legacy_fields_in_sync():
    fields = ext.ExtensionSettings(True, 6.0, 2.5, "OPERATOR", 3).metadata_fields()
    assert fields["extension"]["revision"] == 3
    assert fields["extension_mode"] == "ENABLE"
    assert fields["dummy_point_distance_m"] == 6.0
    assert fields["row_transition_threshold_m"] == 2.5
    assert ext.from_metadata(fields) == ext.ExtensionSettings(True, 6.0, 2.5, "OPERATOR", 3)


# ── MissionStore.update_extension ──────────────────────────────────────────


def _store(tmp_path, monkeypatch):
    runtime = tmp_path / "runtime"
    settings = SimpleNamespace(
        maximum_upload_bytes=5 * 1024 * 1024,
        maximum_marking_points=10_000,
        mission_runtime_file=runtime / "mission_runtime.json",
        mission_recovery_file=runtime / "mission_recovery.json",
    )
    monkeypatch.setattr(mission_store_module, "settings", settings)
    return mission_store_module.MissionStore(
        tmp_path / "mission.csv",
        runtime / "mission_metadata.json",
        tmp_path / "archive",
    )


def _upload(store, **extension):
    return store.save(
        raw_bytes=CSV,
        filename="field.csv",
        extension_mode="ENABLE",
        dummy_point_distance_m=extension.get("distance", 4.0),
        dummy_point_distance_source="OPERATOR",
        trigger_distance_m=extension.get("trigger", 3.0),
    )


def _update(store, revision, **values):
    return store.update_extension(
        enabled=values.get("enabled", True),
        distance_m=values.get("distance"),
        trigger_distance_m=values.get("trigger"),
        source="OPERATOR",
        expected_revision=revision,
        skip_row_ends=values.get("skips"),
    )


def test_upload_writes_revision_one(tmp_path, monkeypatch):
    store = _store(tmp_path, monkeypatch)
    uploaded = _upload(store, distance=6.0, trigger=2.5)
    assert uploaded["extension"]["revision"] == 1
    assert uploaded["extension"]["trigger_distance_m"] == 2.5
    assert uploaded["row_transition_threshold_m"] == 2.5


def test_update_keeps_mission_and_bumps_revision(tmp_path, monkeypatch):
    store = _store(tmp_path, monkeypatch)
    uploaded = _upload(store)
    csv_before = store.mission_file.read_bytes()

    updated = _update(store, 1, distance=6.5, trigger=2.0)

    assert updated["mission_id"] == uploaded["mission_id"]
    assert updated["checksum_sha256"] == uploaded["checksum_sha256"]
    assert store.mission_file.read_bytes() == csv_before
    assert updated["extension"]["revision"] == 2
    assert updated["dummy_point_distance_m"] == 6.5
    stored = json.loads(store.metadata_file.read_text())
    assert stored["extension"]["distance_m"] == 6.5
    assert store.load_metadata()["extension"]["revision"] == 2


def test_stale_revision_is_refused_and_nothing_written(tmp_path, monkeypatch):
    store = _store(tmp_path, monkeypatch)
    _upload(store)
    _update(store, 1, distance=5.0)
    before = store.metadata_file.read_bytes()

    with pytest.raises(mission_store_module.MissionRevisionConflict) as error:
        _update(store, 1, distance=9.0)

    assert error.value.current_revision == 2
    assert store.metadata_file.read_bytes() == before


def test_out_of_bounds_update_writes_nothing(tmp_path, monkeypatch):
    store = _store(tmp_path, monkeypatch)
    _upload(store)
    before = store.metadata_file.read_bytes()

    with pytest.raises(mission_store_module.MissionValidationError):
        _update(store, 1, distance=0.5)

    assert store.metadata_file.read_bytes() == before


def test_disable_leaves_no_extension_values(tmp_path, monkeypatch):
    store = _store(tmp_path, monkeypatch)
    _upload(store, distance=6.0, trigger=2.5)

    _update(store, 1, enabled=False)
    stored = json.loads(store.metadata_file.read_text())

    assert stored["extension"]["enabled"] is False
    assert stored["extension"]["distance_m"] is None
    assert stored["extension"]["trigger_distance_m"] is None
    assert stored["extension"]["source"] is None
    assert stored["extension_mode"] == "DISABLE"
    assert stored["dummy_point_distance_m"] is None
    assert stored["dummy_point_distance_source"] is None


def test_returning_to_earlier_values_restores_identical_settings(tmp_path, monkeypatch):
    store = _store(tmp_path, monkeypatch)
    first = _upload(store, distance=4.0, trigger=3.0)
    _update(store, 1, distance=6.0, trigger=2.0)
    back = _update(store, 2, distance=4.0, trigger=3.0)

    def geometry(metadata):
        value = dict(metadata["extension"])
        value.pop("revision")
        return value

    assert geometry(back) == geometry(first)


def test_update_without_mission_is_refused(tmp_path, monkeypatch):
    store = _store(tmp_path, monkeypatch)
    with pytest.raises(mission_store_module.MissionValidationError):
        _update(store, 0, distance=4.0)


def test_legacy_stored_metadata_reads_as_revision_zero(tmp_path, monkeypatch):
    store = _store(tmp_path, monkeypatch)
    _upload(store)
    stored = json.loads(store.metadata_file.read_text())
    for key in ("extension", "dummy_point_distance_source"):
        stored.pop(key)
    stored["dummy_point_distance_m"] = 3.5
    store.metadata_file.write_text(json.dumps(stored))

    loaded = store.load_metadata()

    assert loaded["extension"] == {
        "schema": ext.SCHEMA,
        "enabled": True,
        "distance_m": 4.0,
        "trigger_distance_m": 3.0,
        "source": "DEFAULT",
        "revision": 0,
        "skip_row_ends": [],
    }
    # The legacy mission can then be edited from revision 0.
    assert _update(store, 0, distance=5.0)["extension"]["revision"] == 1


# ── PUT /api/mission/extension ─────────────────────────────────────────────


def _route(namespace):
    tree = ast.parse((SOURCE / "mission_routes.py").read_text())
    wanted = {"update_mission_extension", "_extension_error"}
    functions = [
        node
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name in wanted
    ]
    for node in functions:
        node.decorator_list = []
    exec(compile(ast.Module(body=functions, type_ignores=[]), "mission_routes.py", "exec"), namespace)
    return namespace["update_mission_extension"]


def _namespace(tmp_path, *, active=False, recovery=False, update_error=None, prepare_error=None):
    calls = []
    mission = {"recovery_required": recovery, "accepted_for_start": True}

    def require_not_active(*, operation, include_preparing=True):
        if active:
            raise HTTPException(status_code=409, detail="Stop the mission first.")

    async def run_in_threadpool(function, *args, **kwargs):
        return function(*args, **kwargs)

    def update_extension(**kwargs):
        calls.append(("update", kwargs))
        if update_error is not None:
            raise update_error
        return {"mission_id": "m1", "extension": {"revision": kwargs["expected_revision"] + 1}}

    def prepare():
        calls.append(("prepare",))
        if prepare_error is not None:
            raise prepare_error

    recovery_file = tmp_path / "mission_recovery.json"
    return calls, mission, {
        "Any": object,
        "Depends": lambda dependency: None,
        "require_auth": None,
        "_serialize_mission_mutation": None,
        "AuthenticatedSession": object,
        "ExtensionUpdateRequest": object,
        "HTTPException": HTTPException,
        "run_in_threadpool": run_in_threadpool,
        "_require_not_active": require_not_active,
        "_require_ros_bridge": lambda: None,
        "_mission_state": lambda: mission,
        "settings": SimpleNamespace(mission_recovery_file=recovery_file),
        "mission_store": SimpleNamespace(
            load_metadata=lambda: {"mission_id": "m1"},
            update_extension=update_extension,
        ),
        "MissionRevisionConflict": mission_store_module.MissionRevisionConflict,
        "MissionValidationError": mission_store_module.MissionValidationError,
        "extension_settings": ext,
        "trajectory_snapshot": SimpleNamespace(
            invalidate=lambda reason: calls.append(("invalidate", reason))
        ),
        "rover_state": SimpleNamespace(
            update=lambda section, **values: mission.update(values),
            set_mission_state=lambda *args, **kwargs: None,
        ),
        "ros_bridge": SimpleNamespace(prepare_trajectory=prepare),
    }


def _request(**values):
    return SimpleNamespace(
        enabled=values.get("enabled", True),
        distance_m=values.get("distance_m", 5.0),
        trigger_distance_m=values.get("trigger_distance_m", 3.0),
        skip_row_ends=values.get("skip_row_ends"),
        expected_revision=values.get("expected_revision", 1),
    )


def _call(route, request):
    return asyncio.run(route(request, _session=None, _mutation=None))


def test_put_success_clears_path_unstages_and_prepares(tmp_path):
    calls, mission, namespace = _namespace(tmp_path)
    response = _call(_route(namespace), _request())

    assert response["extension"]["revision"] == 2
    assert [call[0] for call in calls] == ["update", "invalidate", "prepare"]
    assert calls[0][1]["source"] == "OPERATOR"
    assert ("invalidate", "extension_changed") in calls
    assert mission["accepted_for_start"] is False


@pytest.mark.parametrize(
    ("kwargs", "code"),
    [({"active": True}, "MISSION_ACTIVE"), ({"recovery": True}, "RECOVERY_PENDING")],
)
def test_put_refused_while_active_or_recovering(tmp_path, kwargs, code):
    calls, _mission, namespace = _namespace(tmp_path, **kwargs)
    with pytest.raises(HTTPException) as error:
        _call(_route(namespace), _request())
    assert error.value.status_code == 409
    assert error.value.detail["code"] == code
    assert calls == []


def test_put_refused_when_recovery_checkpoint_file_exists(tmp_path):
    calls, _mission, namespace = _namespace(tmp_path)
    namespace["settings"].mission_recovery_file.write_text("{}")
    with pytest.raises(HTTPException) as error:
        _call(_route(namespace), _request())
    assert error.value.detail["code"] == "RECOVERY_PENDING"
    assert calls == []


def test_put_revision_conflict_reports_current_revision(tmp_path):
    calls, _mission, namespace = _namespace(
        tmp_path, update_error=mission_store_module.MissionRevisionConflict(4)
    )
    with pytest.raises(HTTPException) as error:
        _call(_route(namespace), _request(expected_revision=3))
    assert error.value.status_code == 409
    assert error.value.detail["code"] == "REVISION_CONFLICT"
    assert error.value.detail["current_revision"] == 4
    assert [call[0] for call in calls] == ["update"]


def test_put_invalid_values_are_422_and_path_kept(tmp_path):
    calls, _mission, namespace = _namespace(
        tmp_path,
        update_error=mission_store_module.MissionValidationError("Extension distance must be between"),
    )
    with pytest.raises(HTTPException) as error:
        _call(_route(namespace), _request(distance_m=0.5))
    assert error.value.status_code == 422
    assert error.value.detail["code"] == "INVALID_EXTENSION"
    assert [call[0] for call in calls] == ["update"]


def test_put_prepare_failure_reports_saved_settings(tmp_path):
    calls, _mission, namespace = _namespace(tmp_path, prepare_error=RuntimeError("no origin"))
    with pytest.raises(HTTPException) as error:
        _call(_route(namespace), _request())
    assert error.value.status_code == 422
    assert error.value.detail["code"] == "PREPARE_FAILED"
    assert error.value.detail["extension_saved"] is True
    assert [call[0] for call in calls] == ["update", "invalidate", "prepare"]


# ── Phase 2: decisions are only served for the stored revision ────────────


def _get_route(metadata, mission):
    tree = ast.parse((SOURCE / "mission_routes.py").read_text())
    wanted = {"get_mission_extension", "_extension_error"}
    functions = [
        node
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name in wanted
    ]
    for node in functions:
        node.decorator_list = []

    async def run_in_threadpool(function, *args, **kwargs):
        return function(*args, **kwargs)

    namespace = {
        "Any": object,
        "Depends": lambda dependency: None,
        "require_auth": None,
        "AuthenticatedSession": object,
        "HTTPException": HTTPException,
        "run_in_threadpool": run_in_threadpool,
        "mission_store": SimpleNamespace(load_metadata=lambda: metadata),
        "MissionValidationError": mission_store_module.MissionValidationError,
        "extension_settings": ext,
        "_mission_state": lambda: mission,
    }
    exec(compile(ast.Module(body=functions, type_ignores=[]), "mission_routes.py", "exec"), namespace)
    return namespace["get_mission_extension"]


_DECISION = {"from_point": "P0004", "decision": "ADDED", "revision": 3}


def test_get_returns_decisions_for_the_stored_revision():
    route = _get_route(
        {"mission_id": "m1", "extension": {"revision": 3}},
        {"mission_id": "m1", "extension_decisions_revision": 3, "extension_decisions": [_DECISION]},
    )
    response = asyncio.run(route(_session=None))
    assert response["decisions_current"] is True
    assert response["decisions"] == [_DECISION]
    assert response["bounds"]["trigger_distance_m"]["max"] == 5.0


def test_get_hides_decisions_from_an_older_revision():
    route = _get_route(
        {"mission_id": "m1", "extension": {"revision": 4}},
        {"mission_id": "m1", "extension_decisions_revision": 3, "extension_decisions": [_DECISION]},
    )
    response = asyncio.run(route(_session=None))
    assert response["decisions_current"] is False
    assert response["decisions"] == []


def test_get_without_mission():
    route = _get_route(None, {"mission_id": None})
    response = asyncio.run(route(_session=None))
    assert response["extension"] is None
    assert response["decisions"] == []


def test_settings_change_clears_previous_decisions(tmp_path, monkeypatch):
    from rover_backend.state import rover_state

    store = _store(tmp_path, monkeypatch)
    _upload(store)
    rover_state.update(
        "mission",
        extension_decisions=[_DECISION],
        extension_decisions_revision=1,
    )

    _update(store, 1, distance=6.0)

    mission = rover_state.section("mission")
    assert mission["extension_decisions"] == []
    assert mission["extension_decisions_revision"] is None
    assert mission["extension"]["revision"] == 2



# ── Phase 3: skipped row ends and saved defaults ──────────────────────────


def test_skip_row_ends_are_validated_sorted_and_deduplicated():
    settings = ext.from_request(
        enabled=True,
        source="OPERATOR",
        revision=1,
        skip_row_ends=["P0012", "P0004", "P0012"],
    )
    assert settings.skip_row_ends == ("P0004", "P0012")
    with pytest.raises(ext.ExtensionSettingsError):
        ext.from_request(enabled=True, source="OPERATOR", revision=1, skip_row_ends=["row 4"])
    with pytest.raises(ext.ExtensionSettingsError):
        ext.from_request(enabled=True, source="OPERATOR", revision=1, skip_row_ends="P0004")


def test_skips_are_dropped_when_disabled_or_unmarked():
    assert ext.from_request(
        enabled=False, source="OPERATOR", revision=1, skip_row_ends=["P0004"]
    ).skip_row_ends == ()
    assert ext.from_request(
        enabled=True, source=None, revision=1, skip_row_ends=["P0004"]
    ).skip_row_ends == ()


def test_update_stores_skips_and_upload_starts_without_them(tmp_path, monkeypatch):
    store = _store(tmp_path, monkeypatch)
    _upload(store)
    updated = _update(store, 1, distance=4.0, skips=["P0002"])
    assert updated["extension"]["skip_row_ends"] == ["P0002"]

    fresh = _upload(store)
    assert fresh["extension"]["skip_row_ends"] == []


def test_saved_defaults_fill_missing_operator_values_only(tmp_path, monkeypatch, _defaults_store):
    _defaults_store.save(ext.parse_defaults(6.0, 2.0))

    operator = ext.from_request(
        enabled=True, source="OPERATOR", revision=1, defaults=_defaults_store.load()[0]
    )
    unmarked = ext.from_request(
        enabled=True, source=None, revision=1, defaults=_defaults_store.load()[0]
    )
    assert (operator.distance_m, operator.trigger_distance_m) == (6.0, 2.0)
    # An old tablet's unmarked upload keeps the fixed defaults.
    assert (unmarked.distance_m, unmarked.trigger_distance_m) == (4.0, 3.0)


def test_defaults_store_round_trip_and_bad_file(_defaults_store):
    assert _defaults_store.load() == (ext.ExtensionDefaults(), "factory")
    _defaults_store.save(ext.parse_defaults(5.5, 2.5))
    assert _defaults_store.load() == (ext.ExtensionDefaults(5.5, 2.5), "saved")

    _defaults_store.path.write_text("{not json")
    assert _defaults_store.load() == (ext.ExtensionDefaults(), "factory")


def test_defaults_are_bounded_like_the_settings():
    with pytest.raises(ext.ExtensionSettingsError):
        ext.parse_defaults(0.5, 3.0)
    with pytest.raises(ext.ExtensionSettingsError):
        ext.parse_defaults(4.0, 6.0)


def test_put_forwards_skips(tmp_path):
    calls, _mission, namespace = _namespace(tmp_path)
    request = _request()
    request.skip_row_ends = ["P0004"]
    _call(_route(namespace), request)
    assert calls[0][1]["skip_row_ends"] == ["P0004"]


def test_get_returns_unused_skips_only_when_current():
    route = _get_route(
        {"mission_id": "m1", "extension": {"revision": 3}},
        {
            "mission_id": "m1",
            "extension_decisions_revision": 3,
            "extension_decisions": [_DECISION],
            "extension_unused_skips": ["P0009"],
        },
    )
    response = asyncio.run(route(_session=None))
    assert response["unused_skips"] == ["P0009"]
    assert response["defaults_source"] == "factory"
