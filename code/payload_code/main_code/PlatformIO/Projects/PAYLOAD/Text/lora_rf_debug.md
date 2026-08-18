# LoRa signal-strength investigation & RF-path debug

**Status (updated 2026-07-06):** MAJOR REVISION. The DAC **does** drive TX power
(~25 dB: –72 → –47 dBm over 0→3300 mV) — the earlier "DAC is inert" conclusion
was a firmware bug: most tests (and production `Sensors.cpp`) wrote
GENERAL_CONFIG to register **0x09** instead of **0xD1**, so the DAC never left
power-down and the PA ran unbiased. Only `test_lora_volt_sweep` used 0xD1, which
is why it alone showed the effect. All 0x09 tests are now fixed. The PA-on switch
re-measurement then found the true TX path is **CXT=HIGH, CrX=LOW** (opposite of
the leakage-based "best") → **~–29 dBm @ ~30 cm** — the RF path is essentially
healthy; no board rework needed. **Both production fixes are applied
(2026-07-06): `Sensors.cpp` GENERAL_CONFIG 0x09→0xD1 and `LoraTx::begin()`
switch flipped to CXT=HIGH/CrX=LOW — verify with an end-to-end `env:new` range
check.** Remaining firmware levers for margin: §7 (BW/SF), plus the
`test_lora_maxpower` re-run (10 s gaps added).

**End-to-end result (env:new, 2026-07-06): –20 dBm on battery at close range —
matches the free-space estimate; the RF path is HEALTHY and this investigation
is closed.** Supply matters: on USB the same firmware reads ~–30 dBm, and
disconnecting one board of the stack brings USB back to ~–20. The PA output
tracks available supply current (USB + full stack starves the rail by ~10 dB).
Flight config (battery, full stack) delivers full power, so this is a
bench-measurement artifact — always judge RF numbers on battery. Rough 9 km
budget from –20 @ 0.3 m: ~–110 dBm at 9 km. **CORRECTED 2026-08-13:** sensitivity
is **–123 dBm** at SF11/BW812.5 per DS Table 6-1, not the –118 originally written
here, so the real margin was ~13 dB, not ~8. Since moving to SF12 + GS high
sensitivity mode it is **~18 dB**. See the rewritten §7 — the old "BW 203 kHz ≈
+12 dB" claim was wrong (it is +4 dB, and it costs the frequency budget).

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
*(Superseded: the correct figure is **–123 dBm**, DS Table 6-1. Flight profile is
now SF12 → –126, and –128 with GS high-sensitivity mode. Table below kept for the
relative comparison it was making; add ~8 dB to the "healthy path" column.)*

| Distance | RSSI with current (broken) path | RSSI with a healthy path  |
|----------|----------------------------------|--------------------------|
| 0.2 m    | ~–70 (measured)                  | ~–17                     |
| ~100 m   | ~–125 → **link drops**           | ~–70                     |
| 9 km     | ~–150 → **dead**                 | ~–102 → ~16 dB margin    |

- **As-is the link dies around ~100 m.** The rocket goes ~9000 m, so it would
  lose telemetry in the first second of flight.
- **With the RF path fixed, 9 km is achievable** (~16 dB margin LOS), and the
  firmware range levers below add comfortable margin on top.

## 3. Firmware levers — REVISED 2026-07-06

**The DAC works.** `test_lora_volt_sweep` (with the switch driven CXT=LOW/
CrX=HIGH): 0 → 3300 mV moves RSSI **–72 → –47 dBm (~25 dB)**. The board doc's
"DAC sets LoRa transmission power" is correct — it biases an external PA stage.

**Why every other test said "inert":** they wrote GENERAL_CONFIG to register
**0x09**; the real register is **0xD1** (see DAC43401 datasheet /
`test_dac_margin`). The DAC stayed in power-down, PA unbiased, so those tests
measured only leakage. Affected & now fixed (0x09 → 0xD1): `test_lora_dac_probe`,
`test_lora_dac_pair`, `test_lora_maxpower`, `test_lora_rfswitch_burst`,
`test_lora_beacon`, `test_pad_heartbeat`, and production `Sensors.cpp` (fixed
2026-07-06).

Re-measured with the PA actually on (new switch state CXT=HIGH/CrX=LOW):

| Lever | Test | PA-on result |
|-------|------|--------------|
| DAC 0x47 vs 0x48 | `test_lora_dac_pair` | **0x48 is the PA-bias DAC** (max → ~–27). 0x47 alone → nothing. Both maxed ≈ 0x48 alone (–28 vs –27). Ignore 0x47. |
| Switch truth table | `test_lora_rfswitch_burst` | resolved, see §4: CXT=HIGH/CrX=LOW → ~–29 |
| TX power +13→–18 dBm | `test_lora_maxpower` | **PA saturates**: +13 ≈ +6 (both –21); –6 and –18 both ~–41/–42. Radiated power is maxed at any setting ≥ +6; the usable "reduce power" knob sits between +6 and –6 (or use the DAC bias). |
| GPIO10 drive 20→40 mA | `test_lora_maxpower` | no effect (E1=E2=–21, PA-on) — supply-current-via-GPIO theory dead |

## 4. Antenna switch — RESOLVED 2026-07-06 (PA-on re-measurement)

With the DACs genuinely at full scale, `test_lora_rfswitch_burst` finds the true
TX path is **state 2: CXT=HIGH, CrX=LOW → ~–29 dBm @ ~30 cm** — the *opposite*
of the old "best". The old table below (kept for history) was measured with the
PA in power-down (§3 register bug), so it ranked switch states by leakage; the
real TX branch looked "weak" because the PA in it was off. All tests now drive
CXT=HIGH/CrX=LOW. –29 dBm at 30 cm is within ~10 dB of the free-space estimate —
the RF path is essentially healthy; the remainder is plausibly antenna gains/alignment.
Production `LoraTx::begin()` and `Sensors.cpp` are both fixed as of 2026-07-06.

Historical (PA off — leakage ranking, do not use):

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

**Most likely culprit — NOTE 2026-07-06: this paragraph predates the DAC/PA
discovery (§3). The missing amount is now ~30 dB at DAC max, and the switch
conclusion below must be re-verified with the fixed burst test first.** Original
text: the ~50 dB is
**common to all switch states** (the switch only moves ~10 dB), so the fault is
in the **switch-independent common path: SX1280 RF output → antenna-side matching
→ connector → antenna.** Measure conducted power right at the antenna connector
(`cw on` + power meter/SDR) and inspect the antenna-side matching + connector
FIRST; the CXT/CrX switch control is secondary.

## 7. Firmware range-margin levers — CORRECTED 2026-08-13 against DS Rev 3.3

**The old version of this section was wrong in two ways and is superseded.** It
claimed BW 812.5 → 203 kHz was "~+10–12 dB, biggest free win". The datasheet's
measured sensitivity table says **+4 dB**, and the change carries a serious
frequency-tolerance penalty. It also quoted sensitivity as "≈ –118 dBm", which is
~5 dB pessimistic. Corrected numbers below.

**DS Table 6-1 — LoRa receiver sensitivity, low power mode (dBm):**

| BW | SF11 | SF12 |
|----|------|------|
| 203.125 kHz | –127 | –130 |
| 406.25 | –125 | –128 |
| **812.5** | **–123** | **–126** |
| 1625 | –117 | –120 |

Note Table 6-1 is the *low power* table. High sensitivity mode (§4.2.1, Table 3-5)
adds ~2 dB on top of every cell.

**Levers, ranked by dB per unit of cost:**

| Lever | Gain | Cost | Status |
|-------|------|------|--------|
| GS `setHighSensitivityMode(true)` | **+2 dB** | 700 µA, nothing else | **APPLIED** |
| SF11 → SF12 | **+3 dB** | 2× time-on-air (222→445 ms/pkt) | **APPLIED** |
| 2400.0 → 2450 MHz | ? (unmeasured) | must re-measure | **APPLIED** |
| TX power 10 → 13 dBm | ~0 dB (PA saturated) | free | **APPLIED** |
| High-gain directional GS antenna | +10–15 dBi | hardware | not done |
| BW 812.5 → 203.125 | +4 dB | 4× ToA **+ ppm trap ↓** | **do not do yet** |

**Why 203.125 kHz is now the *last* resort, not the first:** DS Table 6-3, total
permissible reference drift **for the whole link**:

| LoRa BW | SF5–SF11 | SF12 |
|---------|----------|------|
| 800 kHz | ±80 ppm | **±50 ppm** ← we are here |
| 200 kHz | ±21 ppm | **±21 ppm** |

At BW 812.5/SF12 we have ±50 ppm to spend on two crystals plus temperature. At
203.125 that collapses to ±21 ppm total, i.e. ~±10 ppm per end — a standard
±10 ppm XTAL will exceed that over −40…+85 °C. Going to 203 kHz requires a TCXO
(DS §15.3 gives the AC-coupled 0.8 V clipped-sine circuit) or a crystal spec
verified over the full flight temperature range. 4 dB is not worth a link that
dies when the payload gets cold.

**Revised budget.** With sensitivity actually –126 dBm (SF12/BW812.5) and −128 with
GS high-sensitivity mode, versus the ~–110 dBm estimated at 9 km in the header:
**~18 dB margin**, not the ~8 dB previously stated. The link is in better shape
than this document used to claim. Remaining free upside is the directional GS
antenna.

## 8. Relevant files

- Tests: `src/tests/test_lora_rfswitch_probe.cpp`, `test_lora_maxpower.cpp`,
  `test_lora_dac_probe.cpp`, `test_lora_volt_sweep.cpp`, `test_lora_rx_rssi.cpp`,
  `test_i2c_scan.cpp`.
- Production radio: `src/New/LoraTx.cpp`, settings in `include/New/Config.hpp`.
- Switch state applied in production: `LoraTx::begin()` — **CXT=HIGH, CrX=LOW**
  (the §4 conclusion; this line previously still said LOW/HIGH). This also matches
  the ground station's own FEM convention (`fem_control.cpp`: TX ⇒ CTX=1, CRX=0),
  which is independent confirmation the 2026-07-06 flip was correct.
- Ground station: `ground_station/include/config.h` (profile), `src/main.cpp`
  (production), `src/main_scanner.cpp` (env:scanner — the path that actually
  receives, since DIO1 is not on Nano pin 5 and blocking `receive()` never returns).
