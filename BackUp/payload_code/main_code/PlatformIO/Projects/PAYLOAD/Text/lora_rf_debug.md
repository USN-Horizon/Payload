# LoRa signal-strength investigation & RF-path debug

**Status:** Firmware levers exhausted. Link decodes correctly but RSSI is ~50 dB
below expected — a hardware RF-path fault. This is the blocker for the 9 km
flight. Use this doc to drive the board-level fix.

---

## 1. Symptom

- Production firmware (`env:new`) transmits MAVLink over LoRa and the ground
  station **decodes the telemetry correctly** — nothing in software/protocol is
  wrong.
- But received signal strength is **~–64 to –76 dBm at 20–50 cm**.
- Free-space expectation at 0.2 m, 2.4 GHz, +13 dBm, decent antennas is
  **~–13 to –21 dBm**. So **~45–55 dB is missing** in the RF path.

## 2. Link budget — why this matters for 9 km

SX1280, SF11, BW 812.5 kHz, sensitivity ≈ –118 dBm.

| Distance | RSSI with current (broken) path | RSSI with a healthy path  |
|----------|----------------------------------|--------------------------|
| 0.2 m    | ~–70 (measured)                  | ~–17                     |
| ~100 m   | ~–125 → **link drops**           | ~–70                     |
| 9 km     | ~–150 → **dead**                 | ~–102 → ~16 dB margin    |

- **As-is the link dies around ~100 m.** The rocket goes ~9000 m, so it would
  lose telemetry in the first second of flight.
- **With the RF path fixed, 9 km is achievable** (~16 dB margin LOS), and the
  firmware range levers below add comfortable margin on top.

## 3. What was tested in firmware — ALL of it is ruled out

Standalone tests under `src/tests/` (env names in brackets). None of these
recovered the missing dB:

| Lever | Test | Result |
|-------|------|--------|
| DAC voltage 0→3300 mV | `test_lora_volt_sweep` | no change (≤~3 dB, within noise) |
| DAC address 0x47 vs 0x48 | `test_lora_dac_probe` | no change (both inert) |
| BOTH DACs at max together | `test_lora_dac_pair` | no change (flat ~–66 ±2 dBm across all 4 corners) |
| GPIO10 supply current 20→40 mA | `test_lora_maxpower` (E1 vs E2) | no change |
| TX power +13→–18 dBm | `test_lora_maxpower` | clamped: +13 ≈ +6 dBm |
| Antenna-switch CXT/CrX states | `test_lora_rfswitch_probe` | see below |

**Key tells that this is an RF-path / matching fault (not power level):**
- TX power +13 and +6 dBm give the **same** RSSI → the commanded power is not
  reaching the antenna (PA driving into a bad match / disconnected load).
- The DAC ("setting LoRa transmission power" in the board doc) has **no effect**
  — for an SX1280 (digital power control) it has nothing to act on; likely an
  unconnected/vestigial design element. Do not chase it.

## 4. Antenna switch — confirmed finding

IF15 "CXT" = GPIO11 and IF16 "CrX" = GPIO12 are **antenna TX/RX switch control
lines**, NOT chip DIOs (this is why DIO1 "never asserts on TxDone" — it is not
DIO1). Measured cleanly with `test_lora_rfswitch_burst` (burst-ID method, so each
state's RSSI is unambiguous even on an RSSI-only ground station):

| CXT (G11) | CrX (G12) | RSSI @ ~30 cm |
|-----------|-----------|---------------|
| float | float | ~–77 |
| HIGH  | LOW   | ~–73 |
| LOW   | HIGH  | **~–67 (best)** |
| HIGH  | HIGH  | ~–75 |
| LOW   | LOW   | ~–76 |

- `CXT=LOW, CrX=HIGH` is the best state, by **~6–10 dB** over the others. This is
  applied in production (`LoraTx::begin`, drives CXT=LOW/CrX=HIGH, DIO1=
  RADIOLIB_NC). (An earlier probe read the CXT=HIGH states as "dead" — that was a
  timing-correlation misread on an RSSI-only display; they are only ~6–10 dB
  weaker, not dead.)
- **KEY:** the switch only moves the signal ~10 dB total. ALL five states sit at
  ~–67 to –77, i.e. ~50 dB low. So the dominant ~50 dB loss is **common to every
  switch position** — it is in the part of the RF chain that does not change with
  the switch: **SX1280 RF output → antenna-side matching → connector → antenna.**
  The switch routing is NOT the main fault. Focus hardware debugging on that
  common path, not on the CXT/CrX control.

## 5. I2C bus oddities (board doc has wrong addresses)

From `test_i2c_scan`:
- Board doc says DAC at **0x72** — **0x72 NACKs (absent)**. Real DACs ACK at
  **0x47 and 0x48** (two DAC43401s likely). Firmware uses 0x48.
- **0x7E** — unknown device, unexplained. Worth identifying.
- 0x14 BMM350, 0x49 TMP1075, 0x76 BMP388 as expected.

Treat the board PDF's I2C addresses (and the RF notes) as unreliable — verify
against the actual schematic.

## 6. Hardware fix procedure (in priority order)

RF chain: **SX1280 TX pin → TX matching network → antenna switch → antenna
matching → connector → antenna.** The ~50 dB loss is somewhere in here.

**A. Get the two documents that answer it fastest (no tools):**
1. The **antenna-switch IC datasheet truth table** — read its part number off
   the board. It tells you which CXT/CrX combo connects ANT↔TX-port vs
   ANT↔RX-port. If the measured "best" state (CXT=LO/CrX=HI) is the RX path, the
   board has been radiating only switch leakage → the TX branch is the fault.
2. The **board schematic**, compared against the **Semtech SX1280 reference
   design** TX matching network — a wrong/missing L/C is a classic 20–40 dB loss.

**B. Localize with measurement (needs equipment):**
- `cw on` serial command keys a continuous carrier. With an **RF power meter or
  SDR at the antenna connector**, measure conducted power (expect ~+13 dBm). If
  it reads ~–37 dBm there, the loss is before the connector (switch/matching).
- **VNA**: measure return loss (S11) at the antenna port (mismatch = reflected
  power) and insertion loss across the switch and matching stages.

**C. Inspect + rework (no equipment, do regardless):**
- Under magnification, reflow every joint on: SX1280 RF pins, the antenna
  switch, all 0201/0402 matching L/C, and the antenna connector. One cold RF
  joint is exactly a ~50 dB sink.
- Verify matching components are populated and correct vs the reference design.
- Confirm the antenna connector is soldered (center + ground) on the right pad.

**D. Verify the switch is controlled and powered:**
- CXT/CrX actually reach the switch control pins; switch Vdd is powered; ESP
  3.3 V meets the switch's control thresholds.
- Design check: some SX1280 layouts drive the switch from the **chip's
  DIO2/RFSW** automatically, not from MCU GPIOs. Confirm against the schematic
  which scheme this board uses.

**Most likely culprit (refined by `test_lora_rfswitch_burst`):** the ~50 dB is
**common to all switch states** (the switch only moves ~10 dB), so the fault is
in the **switch-independent common path: SX1280 RF output → antenna-side matching
→ connector → antenna.** Measure conducted power right at the antenna connector
(`cw on` + power meter/SDR) and inspect the antenna-side matching + connector
FIRST; the CXT/CrX switch control is secondary.

## 7. Firmware range-margin levers (apply AFTER the hardware is fixed)

These improve link budget (receiver sensitivity), not transmit RSSI:
- **Bandwidth 812.5 → 203 kHz**: ~+10–12 dB. Biggest free win; telemetry is
  low-rate so the slower BW is fine.
- **SF11 → SF12**: ~+3 dB.
- **High-gain directional ground antenna** (Yagi/patch): +10–15 dBi, free on the
  flight side. Consider circular polarization for a tumbling rocket.

Together: ~+25 dB → a 9 km link with 40+ dB margin once the RF path is healthy.

## 8. Relevant files

- Tests: `src/tests/test_lora_rfswitch_probe.cpp`, `test_lora_maxpower.cpp`,
  `test_lora_dac_probe.cpp`, `test_lora_volt_sweep.cpp`, `test_lora_rx_rssi.cpp`,
  `test_i2c_scan.cpp`.
- Production radio: `src/New/LoraTx.cpp`, settings in `include/New/Config.hpp`.
- Switch state applied in production: `LoraTx::begin()` (CXT=LOW, CrX=HIGH).
