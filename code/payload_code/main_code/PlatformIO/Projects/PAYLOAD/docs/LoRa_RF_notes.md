# LoRa link notes (SX1280 + SKY65383 front end)

What is known about the payload's 2.4 GHz link, with the measurements
behind each point. Dates are when it was measured. The long investigation
log this replaces concluded on 2026-07-06 that the RF path is healthy; only
its conclusions are kept here.

## Radio profile (fixed by the ground station)

| Parameter | Value | Why |
|---|---|---|
| Frequency | 2400.0 MHz | adapter `config.h` in Avionics-main (the local `ground_station/include/config.h` says 2403, which is stale) |
| Bandwidth | 812.5 kHz | narrower would give ~+4 dB (DS Table 6-1) but the total frequency tolerance drops from ±50 ppm to ±21 ppm (DS Table 6-3): needs a TCXO, not worth it |
| Spreading factor | SF11 | sensitivity −123 dBm at BW 812.5 (low-power table), −125 with the GS's high-sensitivity mode |
| Coding rate | 4/8 | maximum FEC |
| Sync word | 0x12 (RadioLib default) | |
| CRC | on (`setCRC(2)`) | `setCRC(1)` is silently rejected by RadioLib; CRC is on by default anyway |
| Whitening | not a LoRa parameter on the SX1280 | |
| SX1280 output | +10 dBm | the external PA saturates above ~+6 dBm (2026-07-06: +6 and +13 radiate the same) |
| PA bias | DAC43401 at 0x48, 3300 mV | the actual power lever: 0 → 3300 mV moved RSSI −72 → −47 dBm (2026-07-06) |

Time on air per packet at this profile (12-symbol preamble, explicit header):
21 bytes ≈ 165 ms, 64 bytes ≈ 365 ms, 128 bytes ≈ 535 ms (measured 365 ms
for the 64-byte flight packets).

## Board facts that cost weeks to find

- **DAC43401 GENERAL_CONFIG is register 0xD1, not 0x09.** Writing 0x09 left
  the DAC in power-down and the PA unbiased; every "the DAC does nothing"
  result before 2026-07-06 came from that. Production and the remaining test
  use 0xD1. The DAC answers at 0x48 (a second one at 0x47 does nothing for
  the PA); the board document's 0x72 is wrong. TMP1075 is at 0x49, not 0x73.
- **GPIO 11 / 12 ("DIO1" / "DIO2" on the pin table) are the antenna-switch
  control lines CXT / CrX**, not the SX1280's DIOs. TX path = CXT HIGH,
  CrX LOW (2026-07-06: ~−29 dBm vs ~−47 dBm for the other state at 30 cm).
  Consequently DIO1 never signals TxDone; production polls the SX1280 IRQ
  register instead. BUSY is not a reliable TxDone indicator either.
- **RST must be held LOW until RadioLib takes over**, otherwise the
  half-powered SX1280 leaks onto MISO and the SD card fails to mount on
  ~75 % of boots (2026-07: 0/8 with RST floating, 8/8 with RST LOW).
- **Supply matters for bench numbers.** On USB with the full stack the
  radiated power reads ~10 dB lower than on battery (rail starved). Judge RF
  numbers on battery.
- **The GS front end saturates at bench distances.** At 2.5 m the adapter
  reports −38 … −60 dBm with SNR around −3 dB; those numbers say nothing
  about range. 2 km test (2026-07, old 20-byte packets): −84 dBm, SNR
  −15 dB, most packets lost, i.e. at the limit of that link.

## Ground-station constraints (Avionics-main, cannot be changed)

- Adapter (Nano 33 IoT): `radio->receive(received, 128); Serial.write(received, 128);`
  forwards **128 bytes per packet regardless of length**, leftover bytes
  included. Packets from the payload are therefore all exactly 64 bytes
  (zero padded); with varying lengths the Qt app re-decodes old frames as
  fresh data (`tools/gs_decode_sim.cpp` shows it).
- Nidhoggr `packetparser.cpp` decodes only SCALED_IMU (acc), SCALED_IMU2
  (gyro), SCALED_IMU3 (mag), SCALED_PRESSURE (pressure + temperature),
  COSMIC_RADIATION, VFR_HUD.groundspeed ("velocity"), HEARTBEAT, and
  NAMED_VALUE_INT only when named BARO_T. Its heartbeat indicator shows
  "Xs ago" after 10 s.
- The adapter's blocking `receive()` must be listening again before the next
  preamble (30 ms): the payload leaves 40 ms between the packets of a cycle.

## Thermal behaviour (2026-09-28, USB power, stack on a desk)

- The LoRa board idles at ~55 °C in the middle of the stack; at 48 % TX duty
  it settled at 64–66 °C after 5 min and at 64 % duty at 66–68 °C. All
  packets still decoded at those temperatures.
- Datasheet limits: SX1280 operating −40 … +85 °C, SKY65383 case +85 °C,
  both junction max 125 °C. The TMP1075 reads the board, the dies run hotter.
- The throttle in `Stack_with_LoRa/Config.hpp` (levels at 65/68/71/74/78 °C)
  acts as a thermostat; the old one-message-per-packet scheme ran at ~76 %
  duty and was reported to exceed 85 °C.
- One 50 s window during testing showed the link collapse to −88 dBm with a
  burst of CRC errors, unrelated to temperature (the hot 5-minute run was
  flawless). It looked like 2.4 GHz interference at the ground station; the
  hypothesis that the LoRa board's linear regulator (TPSPX3819, VIN → 3.3 V
  for the PA) overheats at high duty was not confirmed up to 68 °C and not
  tested beyond.

## Link budget (from the datasheet, SF11 / BW 812.5)

Sensitivity −123 dBm, −125 dBm with the GS high-sensitivity mode. Free-space
from −20 dBm at 0.3 m gives roughly −110 dBm at 9 km, i.e. ~15 dB margin on
paper; the 2 km test above shows the real link is weaker than that. A
directional ground-station antenna is the remaining cheap gain.

## Tools

- `env:test_lora_range`: production packet layout with a counter, for range
  tests without sensors or SD.
- `ground_station env:adapter_sim` + `tools/gs_capture.py` +
  `tools/gs_decode_sim.cpp`: receive like the real adapter, decode like the
  Qt app, RSSI/SNR per packet.
