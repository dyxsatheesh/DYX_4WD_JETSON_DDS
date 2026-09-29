#!/bin/bash
# Install the rover stack as a boot service (see docs/ROVER_BOOT_SERVICE.md).
#
#   sudo ./scripts/install_rover_service.sh            # install + enable at boot
#   sudo ./scripts/install_rover_service.sh --now      # ... and start it now
#   sudo ./scripts/install_rover_service.sh --remove   # stop, disable, uninstall

set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
UNIT=dyx-rover.service
DEST=/etc/systemd/system/$UNIT

if [[ $EUID -ne 0 ]]; then
    echo "Run with sudo: sudo $0 $*" >&2
    exit 1
fi

if [[ "${1:-}" == "--remove" ]]; then
    systemctl disable --now "$UNIT" 2>/dev/null || true
    rm -f "$DEST"
    systemctl daemon-reload
    echo "$UNIT removed."
    exit 0
fi

chmod 0755 "$HERE/rover_launch.sh"
install -o root -g root -m 0644 "$HERE/../systemd/$UNIT" "$DEST"
systemctl daemon-reload
systemctl enable "$UNIT"
echo "Installed and enabled $DEST (starts at every boot)."

if [[ "${1:-}" == "--now" ]]; then
    systemctl restart "$UNIT"
    sleep 3
    systemctl --no-pager --lines=15 status "$UNIT" || true
else
    echo "Not started now. Start with: sudo systemctl start $UNIT"
fi
