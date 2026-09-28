#!/bin/bash
# Make the Jetson its own Wi-Fi access point so the tablet connects directly,
# with no router in between.
#
#   sudo ./scripts/setup_jetson_hotspot.sh            # create / update, start
#   sudo ./scripts/setup_jetson_hotspot.sh --remove   # delete the hotspot
#
# The Jetson keeps 192.168.3.101 on the hotspot, so the frontend
# (config.ts), backend.env and the MAVROS GCS target need no change. It hands
# out 192.168.3.x to clients (NetworkManager "shared" = dnsmasq DHCP + NAT).
# Internet from any other uplink (Ethernet, USB 4G dongle, phone USB tether)
# is shared to the clients and is what NTRIP/RTK needs.
#
# 2.4 GHz only: the regulatory domain is unset (country 00), which marks
# every 5 GHz channel no-IR, so an AP cannot start there. 2.4 GHz also has the
# longer outdoor range.
#
# The password is never stored in this repo: pass HOTSPOT_PSK in the
# environment or type it at the prompt. NetworkManager keeps it (root-only) in
# /etc/NetworkManager/system-connections/.

set -euo pipefail

CON_NAME="${HOTSPOT_CON_NAME:-DYX-4WD-hotspot}"
SSID="${HOTSPOT_SSID:-DYX-4WD}"
ADDRESS="${HOTSPOT_ADDRESS:-192.168.3.101/24}"
CHANNEL="${HOTSPOT_CHANNEL:-auto}"
IFACE="${HOTSPOT_IFACE:-$(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="wifi"{print $1; exit}')}"

if [[ $EUID -ne 0 ]]; then
    echo "Run with sudo: sudo $0 $*" >&2
    exit 1
fi

if [[ "${1:-}" == "--remove" ]]; then
    nmcli connection delete "$CON_NAME" 2>/dev/null \
        && echo "Removed hotspot connection $CON_NAME" \
        || echo "No connection named $CON_NAME"
    exit 0
fi

if [[ -z "$IFACE" ]]; then
    echo "No Wi-Fi device found (nmcli device)." >&2
    exit 1
fi

# Refuse if another interface is already on the hotspot subnet; two routes to
# 192.168.3.0/24 would make the backend unreachable from one side.
SUBNET_PREFIX="$(echo "${ADDRESS%/*}" | cut -d. -f1-3)."
if ip -4 -o addr show | grep -v " $IFACE " | grep -q "inet ${SUBNET_PREFIX}"; then
    echo "Another interface already uses ${SUBNET_PREFIX}0/24:" >&2
    ip -4 -br addr | grep "${SUBNET_PREFIX}" >&2
    exit 1
fi

PSK="${HOTSPOT_PSK:-}"
if [[ -z "$PSK" ]]; then
    read -rsp "Wi-Fi password for $SSID: " PSK; echo
    read -rsp "Repeat password: " PSK2; echo
    if [[ "$PSK" != "$PSK2" ]]; then
        echo "Passwords do not match." >&2
        exit 1
    fi
fi
if (( ${#PSK} < 8 || ${#PSK} > 63 )); then
    echo "WPA2 password must be 8-63 characters." >&2
    exit 1
fi

echo "Enabling Wi-Fi radio on $IFACE..."
rfkill unblock wifi
nmcli radio wifi on

for _ in $(seq 1 20); do
    state="$(nmcli -t -f DEVICE,STATE device | awk -F: -v d="$IFACE" '$1==d{print $2}')"
    [[ "$state" != "unavailable" ]] && break
    sleep 0.5
done
if [[ "$state" == "unavailable" ]]; then
    echo "$IFACE is still unavailable after enabling the radio." >&2
    exit 1
fi

if [[ "$CHANNEL" == "auto" ]]; then
    # Pick the least-crowded of the non-overlapping channels 1/6/11.
    scan="$(nmcli -t -f CHAN device wifi list ifname "$IFACE" --rescan yes 2>/dev/null || true)"
    CHANNEL=6
    best=999999
    for ch in 1 6 11; do
        count="$(grep -cx "$ch" <<<"$scan" || true)"
        echo "  channel $ch: $count networks"
        if (( count < best )); then
            best=$count
            CHANNEL=$ch
        fi
    done
fi
echo "Using 2.4 GHz channel $CHANNEL"

nmcli connection delete "$CON_NAME" >/dev/null 2>&1 || true

nmcli connection add \
    type wifi \
    ifname "$IFACE" \
    con-name "$CON_NAME" \
    autoconnect yes \
    connection.autoconnect-priority 100 \
    ssid "$SSID" \
    802-11-wireless.mode ap \
    802-11-wireless.band bg \
    802-11-wireless.channel "$CHANNEL" \
    802-11-wireless.powersave 2 \
    ipv4.method shared \
    ipv4.addresses "$ADDRESS" \
    ipv6.method disabled \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.proto rsn \
    wifi-sec.pairwise ccmp \
    wifi-sec.group ccmp \
    wifi-sec.psk "$PSK" \
    >/dev/null

nmcli connection up "$CON_NAME" >/dev/null

echo
echo "Hotspot up:"
iw dev "$IFACE" info | grep -E "ssid|type|channel"
ip -4 -br addr show "$IFACE"
echo
echo "Tablet: join Wi-Fi \"$SSID\"; backend at http://${ADDRESS%/*}:5001"
echo "Starts automatically on boot. Remove with: sudo $0 --remove"
