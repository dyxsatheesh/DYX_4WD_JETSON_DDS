#!/bin/bash
# One-time Jetson setup for rover voice announcements on a USB speaker.
#
#   ./scripts/setup_voice.sh                         # auto-detect the USB speaker
#   ./scripts/setup_voice.sh --device plughw:CARD=Device,DEV=0
#
# Run as the rover user (flash), NOT with sudo: it asks for sudo only for
# apt and the audio group. Needs internet (packages, voice models).
#
# Installs:
#   - alsa-utils (aplay, amixer) and ffmpeg            (apt)
#   - piper-tts (offline English/Hindi) and edge-tts   in an ISOLATED venv
#     (~/.local/share/dyx-voice/venv), linked as ~/.local/bin/{piper,edge-tts}
#
#     Never `pip install --user` these: piper-tts pulls NumPy 2 + onnxruntime,
#     and a user-site NumPy 2 shadows the system NumPy 1.21 that ROS 2 Humble
#     needs (tf_transformations -> transforms3d dies, rpp_controller crash-
#     loops). Happened on 2026-09-29. This script also removes such a leak.
#   - Piper voices en_US-ryan-medium, hi_IN-rohan-medium
#     -> ~/.local/share/piper/models   (DYX_VOICE_PIPER_MODEL_DIR)
# Configures:
#   - the USB speaker as the ALSA device in ~/.ros/dyx_voice_settings.json
#   - speaker volume, and membership of the "audio" group
# Then plays a test phrase. Tamil uses edge-tts (cloud) the first time a
# phrase is needed; it is cached as WAV afterwards and works offline.

set -uo pipefail

DEVICE=""
if [[ "${1:-}" == "--device" && -n "${2:-}" ]]; then
    DEVICE="$2"
fi

if [[ $EUID -eq 0 ]]; then
    echo "Run as the rover user, not root: ./scripts/setup_voice.sh" >&2
    exit 1
fi

MODEL_DIR="${DYX_VOICE_PIPER_MODEL_DIR:-$HOME/.local/share/piper/models}"
SETTINGS="${DYX_VOICE_SETTINGS_PATH:-$HOME/.ros/dyx_voice_settings.json}"
HF="https://huggingface.co/rhasspy/piper-voices/resolve/main"
export PATH="$HOME/.local/bin:$PATH"

step() { echo; echo "== $*"; }
fail=0

step "1/6 System packages (alsa-utils, ffmpeg, python3-venv)"
sudo apt-get update -qq && sudo apt-get install -y -qq alsa-utils ffmpeg python3-venv || fail=1

step "2/6 Speech engines (piper-tts, edge-tts) in an isolated venv"
VENV="$HOME/.local/share/dyx-voice/venv"
# Undo any earlier --user install that would shadow ROS's NumPy.
python3 -m pip uninstall -y -q piper-tts edge-tts onnxruntime 2>/dev/null || true
if python3 -c "import numpy, sys; sys.exit(0 if numpy.__version__.startswith('2') and '.local' in numpy.__file__ else 1)" 2>/dev/null; then
    python3 -m pip uninstall -y -q numpy && echo "  removed user-site NumPy 2 (it breaks ROS)"
fi
python3 -m venv "$VENV" || fail=1
"$VENV/bin/pip" install --quiet --upgrade pip >/dev/null 2>&1 || true
"$VENV/bin/pip" install --quiet --upgrade piper-tts edge-tts || fail=1
mkdir -p "$HOME/.local/bin"
ln -sf "$VENV/bin/piper" "$HOME/.local/bin/piper"
ln -sf "$VENV/bin/edge-tts" "$HOME/.local/bin/edge-tts"
for tool in piper edge-tts; do
    printf "  %-9s %s\n" "$tool" "$(command -v $tool || echo MISSING)"
done
python3 -c "import numpy; print('  ROS python numpy:', numpy.__version__)"

step "3/6 Piper voices -> $MODEL_DIR"
mkdir -p "$MODEL_DIR"
fetch() {
    local rel="$1" name
    name="$(basename "$rel")"
    for ext in onnx onnx.json; do
        if [[ -s "$MODEL_DIR/$name.$ext" ]]; then
            echo "  have $name.$ext"
        else
            curl -fL --retry 3 -o "$MODEL_DIR/$name.$ext.part" "$HF/$rel.$ext" \
                && mv "$MODEL_DIR/$name.$ext.part" "$MODEL_DIR/$name.$ext" \
                && echo "  got  $name.$ext" || { echo "  FAILED $name.$ext"; fail=1; }
        fi
    done
}
fetch "en/en_US/ryan/medium/en_US-ryan-medium"
fetch "hi/hi_IN/rohan/medium/hi_IN-rohan-medium"

step "4/6 USB speaker"
aplay -l 2>/dev/null | grep -E "^card" | sed 's/^/  /'
if [[ -z "$DEVICE" ]]; then
    CARD_ID="$(aplay -l 2>/dev/null | grep -iE '^card [0-9]+: .*usb' \
        | head -1 | sed -E 's/^card [0-9]+: ([^ ]+) .*/\1/')"
    if [[ -n "$CARD_ID" ]]; then
        DEVICE="plughw:CARD=${CARD_ID},DEV=0"
    fi
fi
if [[ -z "$DEVICE" ]]; then
    echo "  No USB sound card found. Plug in the speaker, or pass --device <alsa-device>." >&2
    echo "  (Listed above; use plughw:CARD=<id>,DEV=0.)" >&2
    exit 1
fi
echo "  using $DEVICE"
CARD="$(sed -E 's/.*CARD=([^,]+).*/\1/' <<<"$DEVICE")"
for control in PCM Speaker Master Headphone; do
    amixer -q -c "$CARD" sset "$control" 90% unmute 2>/dev/null && echo "  volume: $control 90%"
done

python3 - "$SETTINGS" "$DEVICE" <<'PY'
import json, os, sys
path, device = sys.argv[1], sys.argv[2]
os.makedirs(os.path.dirname(path), exist_ok=True)
try:
    settings = json.load(open(path))
except Exception:
    settings = {}
settings["audio_device"] = device
settings.setdefault("enabled", True)
settings.setdefault("language", "en")
settings.setdefault("gender", "male")
tmp = path + ".tmp"
with open(tmp, "w") as handle:
    json.dump(settings, handle, sort_keys=True)
os.replace(tmp, path)
print(f"  saved {path}: {settings}")
PY

step "5/6 Audio group"
if id -nG | grep -qw audio; then
    echo "  $USER is in 'audio'"
else
    sudo usermod -aG audio "$USER" && echo "  added $USER to 'audio' (takes effect at next login / stack restart)"
fi

step "6/6 Test"
TMP="$(mktemp --suffix=.wav)"
if echo "Way to Mark voice test." | nice -n 19 piper -m "$MODEL_DIR/en_US-ryan-medium.onnx" -f "$TMP" >/dev/null 2>&1 \
    && aplay -q -D "$DEVICE" "$TMP"; then
    echo "  English (Piper): played"
else
    echo "  English (Piper): FAILED"; fail=1
fi
MP3="$(mktemp --suffix=.mp3)"
if edge-tts --voice ta-IN-ValluvarNeural --text "குரல் சோதனை" --write-media "$MP3" >/dev/null 2>&1 \
    && ffmpeg -nostdin -loglevel error -y -i "$MP3" -ar 22050 -ac 1 "$TMP" \
    && aplay -q -D "$DEVICE" "$TMP"; then
    echo "  Tamil (edge-tts, needs internet): played"
else
    echo "  Tamil (edge-tts): not available now (no internet?) - English is used until it is"
fi
rm -f "$TMP" "$MP3"

echo
if [[ $fail -eq 0 ]]; then
    echo "Voice setup complete. Restart the rover stack; the backend reads $SETTINGS."
else
    echo "Voice setup finished WITH ERRORS - see FAILED lines above." >&2
    exit 1
fi
