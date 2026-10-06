# RTK low satellite count — base + rover investigation (2026-10-03)

Fix-type numbering used throughout: **6 = RTK FIXED, 5 = RTK FLOAT, 4 = DGPS, 3 = 3D fix.**

## System

| Item | Value |
|---|---|
| Rover receiver | Septentrio mosaic-H, serial 3804732, FW 4.15.1 (GNSS 2025.03.4), dual antenna (master + aux 0.730 m) |
| Rover corrections path | Emlid caster `MP23960` → Jetson → mosaic-H USB2 (RTCM3 in). COM3 → PX4 at 10 Hz |
| Base receiver | u-blox ZED-F9P, FW HPG 1.32, on a Raspberry Pi (gnss_fastapi_backend "autoflow") |
| Base mode | FIXED on a saved position (13.0720463, 80.2619250, h −78.279 m ellipsoid, acc 0.925 m, surveyed 2026-06-22). **This is the long-term base.** |
| Base output | RTCM3 MSM4: 1005, 1074 (GPS), 1084 (GLO), 1094 (GAL), 1124 (BDS), 1230, 1 Hz, NTRIP 1.0 push |
| Baseline | ~5.8 m; base is 3.4 m higher than the rover |

## Symptom

- Rover tracks **34–36 satellites**. Without corrections (or in DGPS) it uses **16**.
- In RTK the solution uses only **6–7 satellites**. It sat at **5 (FLOAT)** for long periods today, sometimes at **4 (DGPS)**. Earlier today (12:11) it held **6 (FIXED) with 11 satellites**.
- Separately: FIXED drops when a person walks near the master antenna.

## What was ruled out (measured)

| Suspect | Evidence | Verdict |
|---|---|---|
| Rover receiver config | Live `lcf,Current` and `lcf,Boot` identical to the verified 12:25 backup | Not the cause |
| Rover signal strength | 30 s capture: most signals 38–47 dB-Hz on both antennas; **0 cycle slips** on every satellite above 20°; carrier phase valid on all but a few weak, low satellites | Not the cause (when nobody is near the antenna) |
| Correction transport | 60 s `DiffCorrIn` capture on the rover: all 6 message types at 1 Hz, **0 CRC errors, 0 invalid MSM cells**, max gap 1.3 s, latency 0.5–0.8 s; Jetson worker 581 B/s, 0 errors | Not the cause |
| Rover elevation mask (PVT 5°) | Lowest relevant satellite was 15° | Not the cause |

## Root cause: very few satellites are "common, dual-frequency and clean"

RTK used exactly the satellites that have **both frequencies at the base and at the rover with clean carrier phase**: G03, G04 (L1CA+L2C), R07, R11 (L1+L2), C09, C10 (B1I+B2I). Everything else fails for one of four reasons.

### 1. Base sends corrections only for the satellites its own solution uses, and that is capped at 32

From the F9P directly (UBX NAV-SAT and CFG-VALGET, read-only):

- `CFG-NAVSPG-INFIL_MAXSVS = 32` and **exactly 32 satellites were in use**.
- **E02 (73° elevation) and E08 (35°)** were tracked strongly at the base (C/N0 42–51, quality 7, healthy, ephemeris available) but **not used**, and they are exactly the Galileo satellites missing from the RTCM.
- The used set includes low, weak satellites (E13 15°, G31 14°, R09 12°, G02 11°, G07 13°). Those are the ones the rover receives weakly or without carrier.
- QZSS takes 2 of the 32 slots (J02, J07), but **the base sends no QZSS MSM at all** (no 1114), so those slots are wasted.
- Net effect on Galileo: only **E32** was both in the corrections and clean at the rover. The rover reported **no Galileo corrections available**, so Galileo contributed nothing.
- Caveat: "RTCM = used satellites only" is inferred from the correlation in two captures (every Galileo PRN in the RTCM was in the base's used set). It is not confirmed in u-blox documentation.

### 2. BeiDou is effectively single-frequency from this base

- The F9P provides B2I only for **C09 and C10** (BeiDou-2). Newer BeiDou-3 satellites (C19, C22, C36, C06, C07, C12) are B1I-only.
- The base does not track **C01, C02, C03, C08, C21, C26, C39** at all (C/N0 0). C01–C03 are BeiDou geostationary satellites, which this F9P doesn't track; why the others are missing is not established.
- So at most 2 BeiDou satellites can ever contribute dual-frequency RTK.

### 3. GPS G16 has no L2C

G16 is an older satellite without L2C. The rover tracks L2P(Y), which the F9P cannot provide. G16 is single-frequency in common.

### 4. Rover receives some sky sectors weakly

Satellites the base receives well are 13–20 dB weaker at the rover or have no carrier lock. Most are to the **south/southwest (azimuth ~170–280°)**, plus some low ones to the **northeast (~40–50°)**:

| Sat | Az / El | Base C/N0 (E1 / E5b) | Rover C/N0 |
|---|---|---|---|
| E30 | 201° / 42° | 44 / 50 | 30 / 30, E1 no carrier |
| E15 | 223° / 25° | 40 / 46 | 20 / 33 |
| E13 | 277° / 15° | 37 / 42 | no carrier |
| E03 | 51° / 43° | 44 / 49 | 31 / 26 |

That pattern fits a local obstruction near the rover antenna (structure, vehicle, person, or the rover body). The base is 3.4 m higher and does not see it. It is the same mechanism as the "person near the master antenna" FIXED→FLOAT drops. In earlier captures today, 18 master-antenna signals across all four constellations lost carrier lock simultaneously at 13:10:08 IST while the aux antenna kept lock.

## Incident found today: QGroundControl reconfigured the base

The F9P was connected to a Mac by USB to inspect it. QGroundControl was open and treats any u-blox as its own RTK base. At 13:48:16 IST it started a **survey-in** on the base (RAM: TMODE=1, 180 s / 2 m, measurement rate 200 ms, 1005 and 1230 off). The Pi still believed the base was FIXED. A completed survey would have moved the base reference by about 1 m.

Restored at 13:55 (verified on the F9P over USB): TMODE=2 FIXED on the saved position, rate 1000 ms, 1005 + MSM4 + 1230 on, NTRIP pushing at 582 B/s. The operator then reported the rover at 6 (FIXED); I did not re-measure it (the Mac was off the rover hotspot).

Other base findings:
- **F9P flash holds a stale FIXED position** at 11.1155809, 78.6535040 (~260 km away, MSM7, 38400 baud). The Pi re-applies the saved position (RAM+BBR) at boot, so this is harmless today. It would broadcast a wildly wrong base position if BBR is lost and autoflow fails.
- **Pi backend bug:** `POST /autoflow/start` within ~10 s of `POST /autoflow/stop` leaves the old streaming loop running with no NTRIP client. The new run never starts: the base sits in `WAITING_SERIAL` with RTCM off and no NTRIP push. A fix is prepared (see below).
- Pi root disk is 96% full (1.4 GB free); it hit 0 on 2026-09-09.

## How to fix the low satellite count (in order)

**Base side** (biggest, cheapest gain; none applied yet):
1. **Free the 32 navigation slots for useful satellites**, using one or both:
   - Disable QZSS in the base (`CFG-SIGNAL-QZSS_ENA=0`). It is not sent in RTCM anyway.
   - Raise the base elevation mask `CFG-NAVSPG-INFIL_MINELEV` from 10° to about 15°, which removes the 11–14° satellites the rover cannot use.

   Expected: E02, E08 and similar high, strong satellites enter the RTCM, so Galileo becomes usable (+2–3 dual-frequency satellites).
2. Check whether HPG 1.32 accepts `INFIL_MAXSVS` above 32.
3. Apply in RAM first, verify with a rover `DiffCorrIn` capture, then persist through the Pi backend's own config path (not QGC).
4. Clear or overwrite the stale flash TMODE position.

**Rover side:**
5. Check what is near the rover antenna to the south/southwest and northeast. Retest in open sky. If the RTK satellite count rises, obstruction is confirmed.
6. Antenna installation: ground plane, mounting height relative to the rover body and operator position.

**Do not:**
- relax rover RTK reliability thresholds or C/N0 masks,
- lower PX4 satellite requirements,
- use MSM7 to "add signals" (it adds resolution, not satellites or bands).

**Longer term:** a base receiver that tracks BeiDou-3 B2a and Galileo E5a and/or a rover band plan that matches it. BeiDou is limited to 2 dual-frequency satellites with the current pair.

## Evidence

- Rover captures and restore check: `rover_ws/receiver_backups/dci_gal_20261003_134053/` (also on the Jetson)
- Rover config backup reference: `receiver_backups/boot_save_20261003_122520.tar.gz`
- Base backend fix: branch `fix/autoflow-abort-restart-race` in `gnss_fastapi_backend` (commit `7625453`, not deployed); patch at `rover_ws/docs/patches/base-autoflow-abort-restart-race.patch`
