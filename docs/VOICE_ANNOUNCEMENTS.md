# Voice announcements (USB speaker)

Added 2026-09-29. Ported from the 3WD NRP_ROS voice system (`Backend/tts.py`).
It is unit-tested only; **not yet run on the rover**.

## What is spoken (important events only)

| When | English phrase (also Tamil / Hindi) |
|---|---|
| Vehicle first connects | "Way to Mark ready." + "Good morning/afternoon/evening!" |
| Trajectory ready | "Mission ready." |
| START | "Mission started." then "Heading to target N." |
| Rover sets off for a point (AUTO next point, NEXT, resume) | "Heading to target N." |
| Point done, SPRAY | "Target N done." |
| Point done, LASER ≤ 30 mm | "Target N. Laser on. Mark here." |
| LASER, raw GNSS measured > 30 mm | "Target N. Outside tolerance. Do not mark." |
| Other failed point / skipped | "Target N failed." / "Target N skipped." |
| Pause | E-stop → "Emergency stop."; RTK float / lost, vehicle control lost, position lost → a matching reason; otherwise "Mission paused." |
| Resume / stop / complete / error | "Mission resumed." / "Mission stopped." / "Mission completed." / "Mission error." |
| RTK change (not while RUNNING/PAUSED; 12 s cooldown) | "RTK fixed." / "RTK float." / "RTK lost. GPS only." |
| Battery | ≤ 20 % "Battery low…", ≤ 10 % "Warning! Battery critical…" (once each; re-armed above 25 %) |
| Vehicle link lost during a mission / restored | "Warning! Vehicle connection lost." / "Vehicle connected." |

**Priority phrases skip the queue:**
- E-stop
- pauses
- link lost
- battery critical
- mission error

**Other rules:**
- The same phrase is not repeated within 3 s.
- A backend restart learns the current state silently.

## How it works

`rover_backend`:

| File | Role |
|---|---|
| `voice_phrases.py` | every phrase in en / ta / hi |
| `voice_events.py` | `VoiceAnnouncer`: when to speak, pure logic |
| `voice.py` | `VoiceService`: worker thread, cache, engines, playback, settings |
| `voice_routes.py` | `/api/tts/*` |

**Hooks in `ros_bridge.py`:**
- `_mission_status_callback`
- `_point_event_callback`
- `_gps_status_callback`
- `_battery_callback`
- `_mavros_state_callback`

The service starts in `main.startup_backend`. A failure there is logged and
never stops the backend.

**Engines, in order:**
1. **Cached WAV:** `~/.ros/dyx_voice_cache/<lang>/`.
2. **Piper CLI:** offline; `en_US-ryan-medium` and `hi_IN-rohan-medium` in
   `~/.local/share/piper/models`.
3. **edge-tts:** Microsoft neural voices over the rover's 4G, then `ffmpeg` to
   WAV. This is the **only Tamil source**; there is no Tamil Piper voice. Once a
   phrase is cached it plays offline.
4. If the selected language can't be produced, the phrase is spoken **in
   English** rather than dropped.

**Synthesis details:**
- Synthesis runs under `nice -n 19`, so it never competes with the control
  stack.
- When a mission becomes READY, "Heading to target N" and "Target N done" (or
  the laser phrases) are generated for every point in the background.

**Playback:** `aplay -q -D <audio_device>`, direct ALSA with no PulseAudio. This
works under systemd.

**Settings:** `~/.ros/dyx_voice_settings.json`, which survives restarts:
- `enabled`
- `language` (en / ta / hi)
- `gender` (male / female; applies to the edge-tts voices)
- `audio_device`

Environment overrides: `DYX_VOICE_SETTINGS_PATH`, `DYX_VOICE_CACHE_DIR`,
`DYX_VOICE_PIPER_MODEL_DIR`.

## One-time Jetson setup

```bash
ssh -t flash@192.168.3.101 "cd ~/rover_ws && ./scripts/setup_voice.sh"
```

Plug the USB speaker in first. The script asks for sudo only for apt and the
audio group. It:
- installs `alsa-utils` + `ffmpeg` and `piper-tts` + `edge-tts`;
- downloads the voices;
- detects the USB card and saves `plughw:CARD=<id>,DEV=0`;
- sets the volume and adds `flash` to `audio`;
- plays English and Tamil test phrases.

To force a device: `--device plughw:CARD=Device,DEV=0`. Then restart the
stack.

## API (login required; tablet Settings → Voice already uses it)

| Endpoint | Body | Notes |
|---|---|---|
| `GET /api/tts/status` | — | `enabled`, `language`, `gender`, `engine`, `engines` per language, `tools`, `audio_device`, `last_error`, `spoken_count` |
| `POST /api/tts/control` | `{"enabled": bool}` | saved |
| `POST /api/tts/test` | `{"message"?: str}` (≤ 200 chars) | 409 if voice is disabled |
| `GET /api/tts/languages` | — | en, ta, hi |
| `POST /api/tts/language` | `{"language": "en"\|"ta"\|"hi"}` | confirms in the new language; pre-caches its fixed phrases |
| `GET`/`POST /api/tts/gender` | `{"gender": "male"\|"female"}` | |
| `POST /api/tts/device` | `{"audio_device": "plughw:CARD=…,DEV=0"}` | ALSA device |

**Troubleshooting:** `GET /api/tts/status`. Look at `tools` (anything
missing), `engines` (`unavailable` for a language) and `last_error` (e.g.
`aplay failed on <device>`).

## Differences from NRP_ROS (deliberate)

- **One worker thread.** NRP_ROS reloaded the module on every settings change,
  leaking a worker thread each time.
- **Settings saved to a file**, not environment variables.
- **ALSA direct playback** to the USB speaker, instead of
  PulseAudio → Bluetooth.
- **Everything cached as WAV**, so Tamil works offline after first use.
- **No Piper HTTP server.** The phrase cache makes the always-on server
  unnecessary, so there is one fewer service.
- **Internal failure reasons are not read aloud.**
- **Tamil / Hindi wording:** existing phrases reuse the NRP_ROS translations.
  The LASER and pause-reason phrases are new and should be checked by a native
  speaker.
