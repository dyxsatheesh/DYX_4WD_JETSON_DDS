# Jetson Wi-Fi hotspot — no router on the rover

Set up and verified on the DYX 4WD rover on 2026-09-28. Use this runbook to
provision the same network on any future rover Jetson.

## What this gives you

```
 Tablet / laptop ──Wi-Fi "DYX-4WD"──►  Jetson 192.168.3.101  ──USB 4G dongle──► internet
   (192.168.3.x by DHCP)                (backend :5001, SSH)     (NTRIP / RTK corrections,
                                                                  capped client internet)
```

- The Jetson **is** the access point. No router is needed on the rover.
- The Jetson keeps **`192.168.3.101`**, its old router-network address. The
  frontend (`DYX_GCS_Frontend/src/config.ts`), `backend.env`, and SSH habits
  (`ssh flash@192.168.3.101`) work unchanged.
- Clients get `192.168.3.x` by DHCP from NetworkManager's shared mode
  (dnsmasq), and share the Jetson's internet via NAT. They need it for
  Mapbox map tiles in the tablet app.
- Client internet is **capped** (see QoS below), so it can never stall the
  rover app or starve NTRIP.

## Provision a new rover (two sudo commands)

Both scripts are in the repo. `flash` has no passwordless sudo, so the
operator runs them.

```bash
# 1. Hotspot: asks for the Wi-Fi password (never stored in git)
ssh -t flash@<jetson-ip> "cd ~/rover_ws && sudo ./scripts/setup_jetson_hotspot.sh"

# 2. QoS: client-internet caps, short Wi-Fi queue, regulatory domain
ssh -t flash@<jetson-ip> "cd ~/rover_ws && sudo ./scripts/install_hotspot_qos.sh"
```

Run them from a wired (Ethernet) session if possible. Neither touches
Ethernet, so the session survives.

Undo:

```bash
sudo ./scripts/install_hotspot_qos.sh --remove
sudo ./scripts/setup_jetson_hotspot.sh --remove
```

### `scripts/setup_jetson_hotspot.sh`

| Setting | Value | Why |
|---|---|---|
| NM connection | `DYX-4WD-hotspot`, autoconnect, priority 100 | comes up on every boot |
| SSID | `DYX-4WD` (`HOTSPOT_SSID`) | |
| Security | WPA2-PSK, CCMP only | |
| Password | prompted, or `HOTSPOT_PSK` | never in the repo; NM stores it root-only |
| Band / channel | 2.4 GHz; least-busy of 1/6/11 (`HOTSPOT_CHANNEL`) | 5 GHz is no-IR with the regdomain unset, and 2.4 GHz has more outdoor range |
| Address | `192.168.3.101/24` (`HOTSPOT_ADDRESS`) | keeps every existing config valid |
| IPv4 | `shared` (DHCP + DNS + NAT) | |
| Wi-Fi power save | off | avoids latency spikes |

Before creating anything, the script:
- runs `rfkill unblock wifi` and `nmcli radio wifi on` (Wi-Fi was soft-blocked
  on this Jetson);
- refuses to continue if another interface already has an address on
  `192.168.3.0/24`.

### `scripts/install_hotspot_qos.sh`

This installs `scripts/network/99-dyx-hotspot-qos` as a NetworkManager
dispatcher at `/etc/NetworkManager/dispatcher.d/`. It re-applies on every
interface up, including the hotspot and a 4G dongle plugged in later.

| Setting | Value (env override) |
|---|---|
| All hotspot clients' **download** total | ≤ 200 KiB/s, burst 400 KiB (`DYX_HOTSPOT_DOWNLOAD_LIMIT`) |
| All hotspot clients' **upload** total | ≤ 64 KiB/s, burst 128 KiB (`DYX_HOTSPOT_UPLOAD_LIMIT`) |
| Wi-Fi `txqueuelen` | 1000 → 100 (`DYX_HOTSPOT_TXQUEUELEN`) |
| Regulatory domain | `IN` (`iw reg set` + `/etc/modprobe.d/dyx-wifi-regdomain.conf`) |

The rules live in the iptables chain `mangle/DYX_HOTSPOT_QOS`, hooked first
in `mangle FORWARD`. That runs before NetworkManager's own filter rules, so
NM can never bypass them.

**Only forwarded traffic is limited** (clients ↔ internet). Nothing to or
from the Jetson itself is limited: backend, websocket, SSH and NTRIP.

## Why it is built this way (measured 2026-09-28)

**Symptom.** With the stack running, the tablet app hung and looked
"down". The Wi-Fi link itself never dropped: no disassociations in the
journal after boot.

**Latency with the stack running, before the fix:**

| Path | Average | Worst |
|---|---|---|
| Mac → Jetson | 69 ms | **1459 ms** |
| Jetson → tablet | 83 ms | **785 ms** |

**Root cause.** In NM shared mode, hotspot clients use the Jetson's 4G.
Their downloads were forwarded onto Wi-Fi at up to **494 KiB/s**, against
432 KiB/s arriving on the 4G interface in the same window. That filled a
1000-packet `pfifo_fast` queue in front of the app's websocket, which
needed only ~31 KiB/s. The same downloads also compete with NTRIP on 4G.

**Ruled out:**
- **DDS multicast:** kernel `OutMcastOctets` was 3.5 KB/s.
- **TCP loss on the app:** zero retransmits on the `:5001` sockets.

**Why policing instead of shaping.** The Jetson kernel `5.15.185-tegra` has:
- no HTB, fq_codel, cake, TBF or prio qdisc;
- no `cls_u32` or `cls_fw`;
- no DSCP, TOS or CLASSIFY iptables target.

It does have `xt_hashlimit`. Dropping client traffic above a fixed rate
makes TCP back off, so the queue never fills. Use
`zcat /proc/config.gz | grep NET_SCH_` to check a new kernel.

**Result (same 10 MB download test from a hotspot client):**

| | Before | After |
|---|---|---|
| Client → Jetson ping, worst | 305 ms | **90 ms** |
| Pings over 100 ms | 3 of 100 | **0 of 100** |
| Jetson → tablet ping | — | 6.8 ms average, 26 ms worst |
| Client download | 2.5 MB/s | **216 KB/s** (capped) |

## Verify

```bash
nmcli -t -f NAME,DEVICE,AUTOCONNECT connection show --active   # DYX-4WD-hotspot on wlP1p1s0
iw dev wlP1p1s0 info                                           # type AP, channel, txpower
iw dev wlP1p1s0 station dump | grep -E "^Station|signal:"      # connected clients
sudo iptables -t mangle -L DYX_HOTSPOT_QOS -n -v               # QoS counters rising = working
journalctl -t dyx-hotspot-qos                                  # dispatcher applied on up
ping -c 100 -i 0.2 192.168.3.101                               # from a client
```

`sudo journalctl -b | grep dnsmasq-dhcp` shows DHCP leases with client
hostnames. The lease file is not readable as `flash`.

## Known limits and field checks

- **Transmit power stays at 15 dBm.** The regdomain changed to `IN`, but the
  vendor driver `rtl88x2ce` (built-in RTL8822CE) did not raise txpower at
  runtime. Re-check after a reboot. Clients close to the Jetson read −64 to
  −72 dBm, because the antennas are inside the case. If range is short at
  the far side of a site, fit external antennas.
- **2.4 GHz RC.** If the RC transmitter is 2.4 GHz, range-test it with the
  hotspot on. This rover does not disarm on RC or datalink loss
  (`NAV_RCL_ACT=1`, `NAV_DLL_ACT=0`).
- **RTK over 4G.** Confirm RTK reaches FIXED with the 4G dongle as the only
  uplink. Ethernet will be unplugged in the field.
- **MAVROS GCS target.** `rover.launch.py` sends MAVLink to
  `192.168.3.105:14550`. Nothing holds `.105` on the hotspot (ARP
  `INCOMPLETE`). If QGroundControl is used, reserve `.105` for that laptop's
  MAC in the NM dnsmasq config.
- **Android.** A tablet may warn "no internet" or switch away if the 4G is
  down. Choose "stay connected", and turn off auto-switch to a better network.
- **Map tiles.** Download the site's Mapbox offline pack in the app
  (`offlineMapService.ts`) before the field day. Then the map needs no
  internet, and the cap never matters.
- **Addresses.** With a cable in `enP8p1s0`, the Jetson is also reachable at
  its wired DHCP address (`192.168.1.37` on the bench). The Mosaic-H must be
  plugged in for the direct-USB RTCM path
  (`/dev/serial/by-id/usb-Septentrio_..._3804732-if04`).
