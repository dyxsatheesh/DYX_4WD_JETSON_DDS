#!/usr/bin/env bash
set -euo pipefail

# Point the RTK worker's direct-injection profile at a different serial
# receiver (device path + baud), then START RTK again.
#
# The worker is receiver-independent: device and baud live only in the saved
# RTK profile, so swapping receivers needs no code change or redeploy.
# Editing a direct-injection field forces the persisted desired_state to
# STOPPED (safety contract), hence the explicit START at the end.
#
# Usage (on the Jetson):
#   export DYX_ROVER_TOKEN=<X-Rover-Token from the app login>
#   scripts/set_rtk_direct_device.sh [PROFILE_ID] [DEVICE] [BAUD]
#
# Defaults: DEVICE/BAUD = CUAV C-RTK 2HP (UM982) COM2 over its CP2102N
# adapter, 115200. With PROFILE_ID omitted the profile list is printed and the
# script exits; pass the id to apply.

BACKEND_URL="${DYX_BACKEND_URL:-http://127.0.0.1:5001}"
ROVER_TOKEN="${DYX_ROVER_TOKEN:-}"
PROFILE_ID="${1:-}"
DEVICE="${2:-/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_0811b937708bee11a26005028acbdcd8-if00-port0}"
BAUD="${3:-115200}"

if [[ -z "${ROVER_TOKEN}" ]]; then
    echo "DYX_ROVER_TOKEN is required." >&2
    echo "Authenticate with rover_backend and export the returned token." >&2
    exit 2
fi

api() {
    # Token goes through curl config stdin, not argv.
    local method="$1" path="$2" body="${3:-}"
    {
        printf 'url = "%s%s"\n' "${BACKEND_URL%/}" "${path}"
        printf 'request = "%s"\n' "${method}"
        printf 'fail-with-body\nsilent\nshow-error\n'
        printf 'header = "X-Rover-Token: %s"\n' "${ROVER_TOKEN}"
        if [[ -n "${body}" ]]; then
            printf 'header = "Content-Type: application/json"\n'
            printf 'data = "%s"\n' "$(printf '%s' "${body}" | sed 's/\\/\\\\/g; s/"/\\"/g')"
        fi
    } | curl --config -
}

if [[ -z "${PROFILE_ID}" ]]; then
    echo "Profiles (pass the id as the first argument to apply):"
    api GET /api/rtk/profiles | python3 -c '
import json, sys
for p in json.load(sys.stdin)["profiles"]:
    print(p["id"], p["name"], p.get("direct_inject"),
          p.get("direct_serial_device"), p.get("direct_serial_baud"))'
    exit 0
fi

if [[ ! -e "${DEVICE}" ]]; then
    echo "WARNING: ${DEVICE} does not exist on this host." >&2
    echo "Profile will still be updated; the worker fails to open it until it appears." >&2
fi

api PATCH "/api/rtk/profiles/${PROFILE_ID}" \
    "{\"direct_serial_device\":\"${DEVICE}\",\"direct_serial_baud\":${BAUD}}"
printf '\n'
api POST /api/rtk/start
printf '\n'
