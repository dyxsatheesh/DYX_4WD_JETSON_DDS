from __future__ import annotations

from types import SimpleNamespace

from rover_backend import mission_store as mission_store_module


def _store(tmp_path, monkeypatch):
    runtime = tmp_path / "runtime"
    settings = SimpleNamespace(
        maximum_upload_bytes=5 * 1024 * 1024,
        maximum_marking_points=10_000,
        default_dummy_point_distance_m=4.0,
        row_transition_threshold_m=3.0,
        mission_runtime_file=runtime / "mission_runtime.json",
        mission_recovery_file=runtime / "mission_recovery.json",
    )
    monkeypatch.setattr(mission_store_module, "settings", settings)
    return mission_store_module.MissionStore(
        tmp_path / "mission.csv",
        runtime / "mission_metadata.json",
        tmp_path / "archive",
    )


def test_completed_mission_is_archived_and_can_be_restored(tmp_path, monkeypatch):
    store = _store(tmp_path, monkeypatch)
    uploaded = store.save(
        raw_bytes=b"latitude,longitude\n13.1,80.2\n13.2,80.3\n",
        filename="field.csv",
        extension_mode="DISABLE",
    )

    archived = store.archive_active_mission()

    assert archived is not None
    assert archived["mission_id"] == uploaded["mission_id"]
    assert not store.mission_file.exists()
    assert store.list_archived_missions()[0]["mission_id"] == uploaded["mission_id"]

    restored = store.restore_archived_mission(uploaded["mission_id"])
    assert restored["mission_id"] == uploaded["mission_id"]
    assert store.load_metadata()["checksum_sha256"] == uploaded["checksum_sha256"]


def test_archive_rejects_path_traversal_ids(tmp_path, monkeypatch):
    store = _store(tmp_path, monkeypatch)

    try:
        store.restore_archived_mission("../mission")
    except mission_store_module.MissionValidationError as error:
        assert "not found" in str(error).lower() or "invalid" in str(error).lower()
    else:
        raise AssertionError("path traversal mission ID was accepted")


_CSV = b"latitude,longitude\n13.1,80.2\n13.2,80.3\n"


def test_operator_marked_extension_distance_is_kept(tmp_path, monkeypatch):
    store = _store(tmp_path, monkeypatch)
    uploaded = store.save(
        raw_bytes=_CSV,
        filename="field.csv",
        extension_mode="ENABLE",
        dummy_point_distance_m=6.5,
        dummy_point_distance_source="operator",
    )

    assert uploaded["dummy_point_distance_m"] == 6.5
    assert uploaded["dummy_point_distance_source"] == "OPERATOR"
    assert store.load_metadata()["dummy_point_distance_m"] == 6.5


def test_unmarked_extension_distance_uses_default(tmp_path, monkeypatch):
    # Earlier tablet builds always send 3.5 m without a source.
    store = _store(tmp_path, monkeypatch)
    uploaded = store.save(
        raw_bytes=_CSV,
        filename="field.csv",
        extension_mode="ENABLE",
        dummy_point_distance_m=3.5,
    )

    assert uploaded["dummy_point_distance_m"] == 4.0
    assert uploaded["dummy_point_distance_source"] == "DEFAULT"


def test_stored_legacy_metadata_reports_default_distance(tmp_path, monkeypatch):
    import json

    store = _store(tmp_path, monkeypatch)
    store.save(
        raw_bytes=_CSV,
        filename="field.csv",
        extension_mode="ENABLE",
    )
    stored = json.loads(store.metadata_file.read_text())
    stored["dummy_point_distance_m"] = 3.5
    stored.pop("dummy_point_distance_source")
    store.metadata_file.write_text(json.dumps(stored))

    loaded = store.load_metadata()

    assert loaded["dummy_point_distance_m"] == 4.0
    assert loaded["dummy_point_distance_source"] == "DEFAULT"


def test_disabled_extension_has_no_distance(tmp_path, monkeypatch):
    store = _store(tmp_path, monkeypatch)
    uploaded = store.save(
        raw_bytes=_CSV,
        filename="field.csv",
        extension_mode="DISABLE",
        dummy_point_distance_m=6.5,
        dummy_point_distance_source="OPERATOR",
    )

    assert uploaded["dummy_point_distance_m"] is None
    assert uploaded["dummy_point_distance_source"] is None
