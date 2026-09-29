"""Voice announcements on the rover's USB speaker.

Ported from the 3WD NRP_ROS ``Backend/tts.py`` with its known problems fixed:

- one worker thread for the process lifetime (NRP_ROS re-imported the module
  on every settings change and leaked a worker each time);
- settings (enabled / language / gender / ALSA device) persist in a JSON file
  instead of process environment variables;
- every phrase is rendered once to a WAV cache and replayed from disk, so a
  cloud-generated Tamil phrase keeps working with no internet afterwards;
- playback goes straight to ALSA (``aplay -D <device>``), so no PulseAudio
  session is needed when the stack later runs under systemd.

Engines, in order: cached WAV -> Piper CLI (en, hi; offline) -> edge-tts
(Microsoft neural voices, needs internet; the only Tamil source) converted to
WAV with ffmpeg. A phrase that cannot be produced in the selected language is
spoken in English rather than dropped. Synthesis runs under ``nice -n 19`` so
it never competes with the control stack for CPU.

Nothing here may raise into a caller: speaking is best-effort and must never
block or break rover control.
"""

from __future__ import annotations

import hashlib
import json
import logging
import os
import queue
import shutil
import subprocess
import tempfile
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Callable, Iterable, Optional

from rover_backend import voice_phrases as phrases

LOGGER = logging.getLogger(__name__)

GENDERS = ("male", "female")

EDGE_VOICES: dict[str, dict[str, str]] = {
    "male": {
        "en": "en-US-RogerNeural",
        "ta": "ta-IN-ValluvarNeural",
        "hi": "hi-IN-MadhurNeural",
    },
    "female": {
        "en": "en-US-MichelleNeural",
        "ta": "ta-IN-PallaviNeural",
        "hi": "hi-IN-SwaraNeural",
    },
}

# Piper voices (rhasspy/piper-voices). Tamil has no upstream Piper voice.
PIPER_MODEL_FILES: dict[str, str] = {
    "en": "en_US-ryan-medium.onnx",
    "hi": "hi_IN-rohan-medium.onnx",
}


def _env_path(name: str, default: str) -> Path:
    return Path(os.path.expanduser(os.environ.get(name, default)))


@dataclass
class VoiceSettings:
    enabled: bool = True
    language: str = phrases.DEFAULT_LANGUAGE
    gender: str = "male"
    # ALSA playback device, e.g. "plughw:CARD=Device,DEV=0" for a USB
    # speaker. scripts/setup_voice.sh detects and stores it.
    audio_device: str = "default"


@dataclass(frozen=True)
class _Utterance:
    text: str
    language: str
    fallback_text: str


class VoiceService:
    MIN_REPEAT_INTERVAL_SEC = 3.0
    QUEUE_LIMIT = 50
    MIN_WAV_BYTES = 100

    def __init__(
        self,
        *,
        settings_path: Path,
        cache_dir: Path,
        piper_model_dir: Path,
        runner: Callable[..., Any] = subprocess.run,
        which: Callable[[str], Optional[str]] = shutil.which,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._settings_path = settings_path
        self._cache_dir = cache_dir
        self._piper_model_dir = piper_model_dir
        self._run_process = runner
        self._which = which
        self._clock = clock

        self._lock = threading.Lock()
        # Serializes synthesis (worker and pre-cache) so two threads never
        # write the same cache file or load two models at once.
        self._synth_lock = threading.Lock()
        self._settings = self._load_settings()
        self._queue: "queue.Queue[_Utterance]" = queue.Queue(maxsize=self.QUEUE_LIMIT)
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_text: Optional[str] = None
        self._last_spoken_at = 0.0
        self.last_error: Optional[str] = None
        self.spoken_count = 0

    # ------------------------------------------------------------ settings

    def settings(self) -> VoiceSettings:
        with self._lock:
            return VoiceSettings(**asdict(self._settings))

    def set_enabled(self, enabled: bool) -> VoiceSettings:
        self._update(enabled=bool(enabled))
        if enabled:
            self.say("voice_enabled")
        else:
            self.flush()
        return self.settings()

    def set_language(self, language: str) -> VoiceSettings:
        code = str(language or "").strip().lower()
        if code not in phrases.LANGUAGES:
            raise ValueError(f"Unsupported language: {language!r}")
        self._update(language=code)
        self.flush()
        self.say("language_set")
        return self.settings()

    def set_gender(self, gender: str) -> VoiceSettings:
        value = str(gender or "").strip().lower()
        if value not in GENDERS:
            raise ValueError("gender must be 'male' or 'female'")
        self._update(gender=value)
        self.say("voice_changed")
        return self.settings()

    def set_audio_device(self, device: str) -> VoiceSettings:
        value = str(device or "").strip()
        if not value:
            raise ValueError("audio_device must not be empty")
        self._update(audio_device=value)
        return self.settings()

    def _update(self, **changes: Any) -> None:
        with self._lock:
            candidate = VoiceSettings(**{**asdict(self._settings), **changes})
            self._save_settings(candidate)
            self._settings = candidate

    def _load_settings(self) -> VoiceSettings:
        try:
            if not self._settings_path.exists():
                return VoiceSettings()
            payload = json.loads(self._settings_path.read_text(encoding="utf-8"))
            defaults = VoiceSettings()
            gender = str(payload.get("gender", defaults.gender)).lower()
            device = str(payload.get("audio_device") or defaults.audio_device)
            return VoiceSettings(
                enabled=bool(payload.get("enabled", defaults.enabled)),
                language=phrases.normalize_language(payload.get("language")),
                gender=gender if gender in GENDERS else defaults.gender,
                audio_device=device,
            )
        except Exception:
            LOGGER.exception(
                "Ignoring unreadable voice settings %s", self._settings_path
            )
            return VoiceSettings()

    def _save_settings(self, settings: VoiceSettings) -> None:
        self._settings_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self._settings_path.with_suffix(self._settings_path.suffix + ".tmp")
        with temporary.open("w", encoding="utf-8") as handle:
            handle.write(json.dumps(asdict(settings), sort_keys=True))
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, self._settings_path)

    # ------------------------------------------------------------ speaking

    def say(self, key: str, *, priority: bool = False, **params: Any) -> None:
        """Queue phrase `key` in the selected language. Never raises."""
        try:
            settings = self.settings()
            if not settings.enabled:
                return
            self._enqueue(
                _Utterance(
                    text=phrases.render(key, settings.language, **params),
                    language=settings.language,
                    fallback_text=phrases.render(key, phrases.DEFAULT_LANGUAGE, **params),
                ),
                priority=priority,
            )
        except Exception:
            LOGGER.exception("Voice phrase %r could not be queued", key)

    def speak_text(self, text: str) -> None:
        """Queue free text (the /api/tts/test message) in the current language."""
        settings = self.settings()
        if not settings.enabled or not text.strip():
            return
        self._enqueue(
            _Utterance(text.strip(), settings.language, text.strip()),
            priority=False,
        )

    def flush(self) -> None:
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                return

    def _enqueue(self, utterance: _Utterance, *, priority: bool) -> None:
        if priority:
            # Safety phrases (E-stop, pause, link loss) jump ahead of any
            # queued routine announcements.
            self.flush()
        try:
            self._queue.put_nowait(utterance)
        except queue.Full:
            LOGGER.warning("Voice queue full; dropped %r", utterance.text)

    # ------------------------------------------------------------ lifecycle

    def start(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._run, name="voice-worker", daemon=True
            )
            self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)

    @property
    def running(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                utterance = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self.handle(utterance)
            except Exception as exc:  # never let the worker die
                self.last_error = f"{type(exc).__name__}: {exc}"
                LOGGER.exception("Voice playback failed")

    def handle(self, utterance: _Utterance) -> bool:
        """Synthesize (or reuse) and play one utterance. Returns True if played."""
        if not self.settings().enabled:
            return False
        now = self._clock()
        if (
            utterance.text == self._last_text
            and now - self._last_spoken_at < self.MIN_REPEAT_INTERVAL_SEC
        ):
            return False

        wav = self.audio_for(utterance.text, utterance.language)
        if wav is None and utterance.language != phrases.DEFAULT_LANGUAGE:
            wav = self.audio_for(utterance.fallback_text, phrases.DEFAULT_LANGUAGE)
        if wav is None:
            self.last_error = f"no speech engine produced audio for {utterance.text!r}"
            LOGGER.warning("Voice: %s", self.last_error)
            return False

        if not self._play(wav):
            return False
        self._last_text = utterance.text
        self._last_spoken_at = self._clock()
        self.spoken_count += 1
        return True

    # ------------------------------------------------------------ cache

    def cache_path(self, text: str, language: str) -> Path:
        gender = self.settings().gender
        digest = hashlib.sha1(f"{gender}|{text}".encode("utf-8")).hexdigest()[:20]
        return self._cache_dir / language / f"{digest}.wav"

    def audio_for(self, text: str, language: str) -> Optional[Path]:
        path = self.cache_path(text, language)
        if self._usable(path):
            return path
        with self._synth_lock:
            if self._usable(path):
                return path
            path.parent.mkdir(parents=True, exist_ok=True)
            if self._synthesize(text, language, path):
                return path
        return None

    def precache(self, items: Iterable[tuple[str, dict[str, Any]]]) -> threading.Thread:
        """Render phrases to the cache in the background (lowest priority)."""
        settings = self.settings()
        todo = [
            phrases.render(key, settings.language, **params) for key, params in items
        ]

        def work() -> None:
            generated = 0
            for text in todo:
                if self._stop_event.is_set():
                    return
                try:
                    if not self._usable(self.cache_path(text, settings.language)):
                        if self.audio_for(text, settings.language) is not None:
                            generated += 1
                except Exception:
                    LOGGER.exception("Voice pre-cache failed for %r", text)
            if generated:
                LOGGER.info("Voice: pre-cached %d phrases", generated)

        thread = threading.Thread(target=work, name="voice-precache", daemon=True)
        thread.start()
        return thread

    def _usable(self, path: Path) -> bool:
        try:
            return path.is_file() and path.stat().st_size > self.MIN_WAV_BYTES
        except OSError:
            return False

    # ------------------------------------------------------------ engines

    def _tool(self, name: str) -> Optional[str]:
        found = self._which(name)
        if found:
            return found
        # ros2 launch / systemd may not have ~/.local/bin on PATH.
        local = Path.home() / ".local" / "bin" / name
        return str(local) if local.is_file() and os.access(local, os.X_OK) else None

    def _piper_model(self, language: str) -> Optional[Path]:
        name = PIPER_MODEL_FILES.get(language)
        if name is None:
            return None
        model = self._piper_model_dir / name
        return model if model.is_file() else None

    def _synthesize(self, text: str, language: str, out: Path) -> bool:
        if self._synth_piper(text, language, out):
            return True
        return self._synth_edge(text, language, out)

    def _niced(self, command: list[str]) -> list[str]:
        nice = self._tool("nice")
        return [nice, "-n", "19", *command] if nice else command

    def _synth_piper(self, text: str, language: str, out: Path) -> bool:
        piper = self._tool("piper")
        model = self._piper_model(language)
        if piper is None or model is None:
            return False
        fd, temporary = tempfile.mkstemp(suffix=".wav", dir=str(out.parent))
        os.close(fd)
        try:
            result = self._run_process(
                self._niced([piper, "-m", str(model), "-f", temporary]),
                input=text,
                capture_output=True,
                text=True,
                timeout=60,
            )
            if result.returncode != 0 or not self._usable(Path(temporary)):
                self.last_error = f"piper failed: {str(result.stderr)[-200:]}"
                return False
            os.replace(temporary, out)
            return True
        except Exception as exc:
            self.last_error = f"piper error: {exc}"
            return False
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def _synth_edge(self, text: str, language: str, out: Path) -> bool:
        edge = self._tool("edge-tts")
        ffmpeg = self._tool("ffmpeg")
        if edge is None or ffmpeg is None:
            return False
        voice = EDGE_VOICES[self.settings().gender].get(language) or EDGE_VOICES["male"]["en"]
        fd, mp3 = tempfile.mkstemp(suffix=".mp3", dir=str(out.parent))
        os.close(fd)
        fd, wav = tempfile.mkstemp(suffix=".wav", dir=str(out.parent))
        os.close(fd)
        try:
            result = self._run_process(
                [edge, "--voice", voice, "--text", text, "--write-media", mp3],
                stdin=subprocess.DEVNULL,
                capture_output=True,
                text=True,
                timeout=30,
            )
            if result.returncode != 0 or os.path.getsize(mp3) == 0:
                self.last_error = f"edge-tts failed: {str(result.stderr)[-200:]}"
                return False
            result = self._run_process(
                self._niced(
                    # -nostdin: ffmpeg otherwise reads the parent's stdin.
                    [ffmpeg, "-nostdin", "-loglevel", "error", "-y", "-i", mp3,
                     "-ar", "22050", "-ac", "1", wav]
                ),
                stdin=subprocess.DEVNULL,
                capture_output=True,
                text=True,
                timeout=30,
            )
            if result.returncode != 0 or not self._usable(Path(wav)):
                self.last_error = f"ffmpeg failed: {str(result.stderr)[-200:]}"
                return False
            os.replace(wav, out)
            return True
        except Exception as exc:
            self.last_error = f"edge-tts error: {exc}"
            return False
        finally:
            for path in (mp3, wav):
                if os.path.exists(path):
                    os.unlink(path)

    def _play(self, wav: Path) -> bool:
        aplay = self._tool("aplay")
        if aplay is None:
            self.last_error = "aplay not installed (apt install alsa-utils)"
            return False
        device = self.settings().audio_device
        try:
            result = self._run_process(
                [aplay, "-q", "-D", device, str(wav)],
                stdin=subprocess.DEVNULL,
                capture_output=True,
                text=True,
                timeout=30,
            )
        except Exception as exc:
            self.last_error = f"aplay error: {exc}"
            return False
        if result.returncode != 0:
            self.last_error = f"aplay failed on {device}: {str(result.stderr)[-200:]}"
            return False
        return True

    # ------------------------------------------------------------ status

    def engine_for(self, language: str) -> str:
        if self._tool("piper") and self._piper_model(language):
            return "piper"
        if self._tool("edge-tts") and self._tool("ffmpeg"):
            return "edge-tts"
        return "unavailable"

    def status(self) -> dict[str, Any]:
        settings = self.settings()
        return {
            "enabled": settings.enabled,
            "language": settings.language,
            "gender": settings.gender,
            "audio_device": settings.audio_device,
            "engine": self.engine_for(settings.language),
            "engines": {code: self.engine_for(code) for code in phrases.LANGUAGES},
            "tools": {
                name: self._tool(name) is not None
                for name in ("aplay", "piper", "edge-tts", "ffmpeg")
            },
            "running": self.running,
            "queued": self._queue.qsize(),
            "spoken_count": self.spoken_count,
            "last_error": self.last_error,
            # NRP_ROS response field the tablet already reads; USB speaker
            # needs no Bluetooth warm-up.
            "bluetooth_warmup_enabled": False,
        }


voice_service = VoiceService(
    settings_path=_env_path("DYX_VOICE_SETTINGS_PATH", "~/.ros/dyx_voice_settings.json"),
    cache_dir=_env_path("DYX_VOICE_CACHE_DIR", "~/.ros/dyx_voice_cache"),
    piper_model_dir=_env_path("DYX_VOICE_PIPER_MODEL_DIR", "~/.local/share/piper/models"),
)


# Imported here (not at the top) only to keep voice_events free of any
# dependency on the service; there is no import cycle.
from rover_backend.voice_events import VoiceAnnouncer  # noqa: E402

voice_announcer = VoiceAnnouncer(voice_service.say, precache=voice_service.precache)


def start_voice() -> None:
    """Start the worker and render the fixed phrases for the current language."""
    voice_service.start()
    voice_service.precache(voice_announcer.static_phrases())
