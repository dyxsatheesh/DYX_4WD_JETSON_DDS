"""Row-extension settings: one versioned object per stored mission.

The trajectory generator adds a navigation-only point past a row end when the
next row is close and runs back. This module owns the operator-editable part
of that rule and nothing else; the geometry stays in trajectory_generator.

Stored in the mission metadata as ``extension`` (schema dyx4wd/extension@1).
The legacy flat fields (extension_mode, dummy_point_distance_m,
dummy_point_distance_source, row_transition_threshold_m) are written beside
it so an older trajectory_generator still reads the same mission.

Rules (docs/UPGRADE_EXTENSION_PLAN.md, operator decisions 2026-10-04):

- Disabled means absent: no distance, no trigger, no source.
- An operator value is honoured only with source OPERATOR. Tablet builds
  before the row-extension dialog sent a hard-coded 3.5 m without it, so an
  unmarked request gets the 4.0 m / 3.0 m defaults.
- Every accepted change carries a revision; derived state built from an older
  revision must be discarded.
- skip_row_ends lists row ends (marking point IDs, "P0006") where the
  operator turned the extension off. A new upload starts with none.

Saved defaults (ExtensionDefaultsStore) only pre-fill the tablet dialog and
fill a value an operator request leaves out. They never change an unmarked
request, which keeps the fixed 4.0 m / 3.0 m.
"""

from __future__ import annotations

import json
import logging
import math
import os
import re
import threading
from dataclasses import dataclass
from dataclasses import field
from pathlib import Path
from typing import Any

LOGGER = logging.getLogger(__name__)

SCHEMA = "dyx4wd/extension@1"

DISTANCE_MIN_M = 1.0
DISTANCE_MAX_M = 20.0
DISTANCE_WARN_M = 8.0
DEFAULT_DISTANCE_M = 4.0

TRIGGER_MIN_M = 1.0
TRIGGER_MAX_M = 5.0
DEFAULT_TRIGGER_M = 3.0

SOURCE_OPERATOR = "OPERATOR"
SOURCE_DEFAULT = "DEFAULT"

MODE_ENABLE = "ENABLE"
MODE_DISABLE = "DISABLE"

SKIP_ROW_ENDS_MAX = 1000
_POINT_ID = re.compile(r"^P\d{4,6}$")


class ExtensionSettingsError(ValueError):
    """The requested or stored extension settings are invalid."""


@dataclass(frozen=True, slots=True)
class ExtensionSettings:
    enabled: bool
    distance_m: float | None
    trigger_distance_m: float | None
    source: str | None
    revision: int
    skip_row_ends: tuple[str, ...] = field(default=())

    @property
    def extension_mode(self) -> str:
        return MODE_ENABLE if self.enabled else MODE_DISABLE

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema": SCHEMA,
            "enabled": self.enabled,
            "distance_m": self.distance_m,
            "trigger_distance_m": self.trigger_distance_m,
            "source": self.source,
            "revision": self.revision,
            "skip_row_ends": list(self.skip_row_ends),
        }

    def metadata_fields(self) -> dict[str, Any]:
        """The ``extension`` object plus the legacy flat fields it replaces."""

        return {
            "extension": self.as_dict(),
            "extension_mode": self.extension_mode,
            "dummy_point_distance_m": self.distance_m,
            "dummy_point_distance_source": self.source,
            # An older generator requires a positive threshold even when the
            # extension is off; it is unused in that case.
            "row_transition_threshold_m": (
                self.trigger_distance_m
                if self.trigger_distance_m is not None
                else DEFAULT_TRIGGER_M
            ),
        }


def bounds() -> dict[str, Any]:
    """Limits and defaults the tablet shows next to its inputs."""

    return {
        "distance_m": {
            "min": DISTANCE_MIN_M,
            "max": DISTANCE_MAX_M,
            "warn_above": DISTANCE_WARN_M,
            "default": DEFAULT_DISTANCE_M,
        },
        "trigger_distance_m": {
            "min": TRIGGER_MIN_M,
            "max": TRIGGER_MAX_M,
            "default": DEFAULT_TRIGGER_M,
        },
    }


def parse_mode(extension_mode: Any) -> bool:
    """Return True for ENABLE and False for DISABLE."""

    mode = str(extension_mode or "").strip().upper()
    if mode == MODE_ENABLE:
        return True
    if mode == MODE_DISABLE:
        return False
    raise ExtensionSettingsError("extension_mode must be ENABLE or DISABLE.")


def _length(
    value: Any,
    *,
    name: str,
    default: float,
    minimum: float,
    maximum: float,
) -> float:
    if value is None or value == "":
        return default
    if isinstance(value, bool):
        raise ExtensionSettingsError(f"{name} must be numeric.")
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ExtensionSettingsError(f"{name} must be numeric.") from error
    if not math.isfinite(number):
        raise ExtensionSettingsError(f"{name} must be finite.")
    if not minimum <= number <= maximum:
        raise ExtensionSettingsError(
            f"{name} must be between {minimum:.2f} m and {maximum:.2f} m."
        )
    return number


def _skip_row_ends(value: Any) -> tuple[str, ...]:
    if value is None:
        return ()
    if not isinstance(value, (list, tuple)):
        raise ExtensionSettingsError("skip_row_ends must be a list of point IDs.")
    if len(value) > SKIP_ROW_ENDS_MAX:
        raise ExtensionSettingsError("Too many skipped row ends.")
    points: set[str] = set()
    for item in value:
        if not isinstance(item, str) or not _POINT_ID.match(item.strip()):
            raise ExtensionSettingsError(
                "skip_row_ends must contain point IDs such as P0006."
            )
        points.add(item.strip())
    return tuple(sorted(points))


def _revision(value: Any) -> int:
    if isinstance(value, bool):
        raise ExtensionSettingsError("Extension revision is invalid.")
    try:
        revision = int(value)
    except (TypeError, ValueError) as error:
        raise ExtensionSettingsError("Extension revision is invalid.") from error
    if revision < 0:
        raise ExtensionSettingsError("Extension revision is invalid.")
    return revision


def from_request(
    *,
    enabled: bool,
    distance_m: Any = None,
    trigger_distance_m: Any = None,
    source: Any = None,
    revision: int,
    skip_row_ends: Any = None,
    defaults: "ExtensionDefaults | None" = None,
) -> ExtensionSettings:
    """Build settings from operator input, applying the source rule.

    ``defaults`` fills a value an operator request leaves out; an unmarked
    request always gets the fixed defaults and no skipped row ends.
    """

    revision = _revision(revision)
    if not enabled:
        return ExtensionSettings(False, None, None, None, revision)

    if str(source or "").strip().upper() != SOURCE_OPERATOR:
        return ExtensionSettings(
            True,
            DEFAULT_DISTANCE_M,
            DEFAULT_TRIGGER_M,
            SOURCE_DEFAULT,
            revision,
        )

    defaults = defaults or ExtensionDefaults()
    return ExtensionSettings(
        True,
        _length(
            distance_m,
            name="Extension distance",
            default=defaults.distance_m,
            minimum=DISTANCE_MIN_M,
            maximum=DISTANCE_MAX_M,
        ),
        _length(
            trigger_distance_m,
            name="Trigger distance",
            default=defaults.trigger_distance_m,
            minimum=TRIGGER_MIN_M,
            maximum=TRIGGER_MAX_M,
        ),
        SOURCE_OPERATOR,
        revision,
        _skip_row_ends(skip_row_ends),
    )


def from_metadata(metadata: dict[str, Any]) -> ExtensionSettings:
    """Read stored settings, converting metadata written before the object.

    Old metadata (no ``extension`` object) is revision 0. Its distance is kept
    only when marked OPERATOR; its trigger was always the fixed 3.0 m.
    """

    stored = metadata.get("extension")
    if stored is not None:
        if not isinstance(stored, dict):
            raise ExtensionSettingsError("Stored extension settings are invalid.")
        enabled = stored.get("enabled")
        if not isinstance(enabled, bool):
            raise ExtensionSettingsError("Stored extension settings are invalid.")
        return from_request(
            enabled=enabled,
            distance_m=stored.get("distance_m"),
            trigger_distance_m=stored.get("trigger_distance_m"),
            source=stored.get("source"),
            revision=stored.get("revision", 0),
            skip_row_ends=stored.get("skip_row_ends"),
        )

    return from_request(
        enabled=parse_mode(metadata.get("extension_mode")),
        distance_m=metadata.get("dummy_point_distance_m"),
        trigger_distance_m=None,
        source=metadata.get("dummy_point_distance_source"),
        revision=0,
    )


@dataclass(frozen=True, slots=True)
class ExtensionDefaults:
    """Operator-saved starting values for the row-extension dialog."""

    distance_m: float = DEFAULT_DISTANCE_M
    trigger_distance_m: float = DEFAULT_TRIGGER_M

    def as_dict(self) -> dict[str, float]:
        return {
            "distance_m": self.distance_m,
            "trigger_distance_m": self.trigger_distance_m,
        }


def parse_defaults(distance_m: Any, trigger_distance_m: Any) -> ExtensionDefaults:
    """Validate operator defaults against the same limits as the settings."""

    return ExtensionDefaults(
        distance_m=_length(
            distance_m,
            name="Extension distance",
            default=DEFAULT_DISTANCE_M,
            minimum=DISTANCE_MIN_M,
            maximum=DISTANCE_MAX_M,
        ),
        trigger_distance_m=_length(
            trigger_distance_m,
            name="Trigger distance",
            default=DEFAULT_TRIGGER_M,
            minimum=TRIGGER_MIN_M,
            maximum=TRIGGER_MAX_M,
        ),
    )


class ExtensionDefaultsStore:
    """Saved defaults in a small JSON file (atomic write).

    A missing or invalid file reads as the fixed defaults, so a bad file can
    never block an upload; it is logged and left for the next save.
    """

    def __init__(self, path: Path) -> None:
        self._path = path
        self._lock = threading.Lock()

    @property
    def path(self) -> Path:
        return self._path

    def load(self) -> tuple[ExtensionDefaults, str]:
        """Return (defaults, source) where source is "saved" or "factory"."""

        with self._lock:
            if not self._path.is_file():
                return ExtensionDefaults(), "factory"
            try:
                payload = json.loads(self._path.read_text(encoding="utf-8"))
                defaults = parse_defaults(
                    payload.get("distance_m"),
                    payload.get("trigger_distance_m"),
                )
            except (OSError, ValueError, AttributeError) as error:
                LOGGER.warning(
                    "Ignoring unreadable extension defaults %s: %s", self._path, error
                )
                return ExtensionDefaults(), "factory"
            return defaults, "saved"

    def save(self, defaults: ExtensionDefaults) -> None:
        with self._lock:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self._path.with_suffix(self._path.suffix + ".tmp")
            temporary.write_text(
                json.dumps(defaults.as_dict(), sort_keys=True), encoding="utf-8"
            )
            os.replace(temporary, self._path)


extension_defaults_store = ExtensionDefaultsStore(
    Path(
        os.environ.get(
            "DYX_EXTENSION_DEFAULTS_PATH",
            "~/.ros/dyx_extension_defaults.json",
        )
    ).expanduser()
)
