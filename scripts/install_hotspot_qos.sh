#!/bin/bash
# Install and apply the DYX-4WD hotspot QoS (see scripts/network/99-dyx-hotspot-qos).
#
#   sudo ./scripts/install_hotspot_qos.sh            # install + apply now
#   sudo ./scripts/install_hotspot_qos.sh --remove   # uninstall + clear rules

set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
DISPATCHER=/etc/NetworkManager/dispatcher.d/99-dyx-hotspot-qos
MODPROBE_CONF=/etc/modprobe.d/dyx-wifi-regdomain.conf
CHAIN=DYX_HOTSPOT_QOS
WIFI_IF="${DYX_HOTSPOT_IFACE:-wlP1p1s0}"

if [[ $EUID -ne 0 ]]; then
    echo "Run with sudo: sudo $0 $*" >&2
    exit 1
fi

if [[ "${1:-}" == "--remove" ]]; then
    rm -f "$DISPATCHER" "$MODPROBE_CONF"
    iptables -t mangle -D FORWARD -j "$CHAIN" 2>/dev/null || true
    iptables -t mangle -F "$CHAIN" 2>/dev/null || true
    iptables -t mangle -X "$CHAIN" 2>/dev/null || true
    ip link set dev "$WIFI_IF" txqueuelen 1000 2>/dev/null || true
    echo "Hotspot QoS removed."
    exit 0
fi

install -o root -g root -m 0755 "$HERE/network/99-dyx-hotspot-qos" "$DISPATCHER"

# Regulatory domain at boot as well (cfg80211 is a module on this kernel), so
# the AP starts with India's 2.4 GHz power limit instead of the world default.
echo "options cfg80211 ieee80211_regdom=IN" > "$MODPROBE_CONF"

"$DISPATCHER" "$WIFI_IF" up

echo "Installed $DISPATCHER"
echo
echo "== rules"
iptables -t mangle -L "$CHAIN" -n -v
echo
echo "== Wi-Fi"
ip -d link show "$WIFI_IF" | grep -o "qlen [0-9]*"
iw reg get | sed -n 2p
iw dev "$WIFI_IF" info | grep -E "txpower|channel"
