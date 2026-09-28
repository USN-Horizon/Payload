# `Stack_with_LoRa` — the payload stack with the SX1280 radio

Flight firmware for the one stack whose LoRa board carries an SX1280. It
sends MAVLink telemetry over 2.4 GHz LoRa to the team's ground station and
mirrors a low-rate row log to the SD card and to the ESP32's own flash.

```bash
pio run -e Stack_with_LoRa -t upload
pio device monitor -e Stack_with_LoRa --dtr 1 --rts 1
```

After the upload press the reset button (or use the esptool watchdog reset
from [README.md](README.md)); the board otherwise stays in the bootloader.

## Life cycle

```
boot ─► launch gate (pad) ─► mission ─(30 min of flight)─► stopped
```

1. **Launch gate.** After init the firmware waits in a low-power loop until
   the accelerometer sees |a| > 1.5 g (`LAUNCH_THRESHOLD_MG`, 1 sample). While
   waiting it sends one beacon packet every 5 s (HEARTBEAT with
   `MAV_STATE_STANDBY` + the latest pressure) and keeps the SX1280 in retention
   sleep in between. The moment the gate trips, `flown=1` is written to NVS, so
   a brownout or watchdog reset mid-flight boots straight into mission mode
   instead of waiting on a pad that no longer exists. If the IMU failed to
   initialise the gate is skipped. `disarm` clears the flag (gate armed at
   next boot), `fly` sets it (bench use, no shaking needed).
2. **Mission.** Sensors are polled at 20 Hz. Every `TX_CYCLE_INTERVAL_MS`
   (1.5 s) one cycle runs: MAVLink frames are built and packed into
   fixed-length LoRa packets (as far as the thermal throttle allows) and one
   row is appended to SD and flash. Flight phase THRUSTING becomes COASTING
   once |a| has been under the launch threshold for 2 s.
3. **Stop.** Flight time counts from the gate and is saved to NVS once a
   minute, so a reset cannot restart the budget. After
   `FLIGHT_MAX_DURATION_S` (30 min) the payload sends a last heartbeat
   (`MAV_STATE_POWEROFF`, phase TOUCHDOWN), writes a last row, puts the radio
   to sleep, drops the PA bias to 0 V, switches the IMU and magnetometer off
   and stops logging. Serial still works. `LANDED_BEACON_INTERVAL_MS`
   (default 0) can keep one heartbeat every N ms afterwards.

## What goes on the air

The ground station cannot be changed, so the payload sends exactly the
messages and fields its parser reads (Avionics-main →
Ratatoskr-GS-Nidhoggr `src/packetparser.cpp`) and nothing else:

| GS value | MAVLink message / field read by the GS | Wire size |
|---|---|---|
| acceleration x/y/z | `SCALED_IMU` `xacc,yacc,zacc` (mg) | 22 B |
| rotation x/y/z | `SCALED_IMU2` `xgyro,ygyro,zgyro` (mrad/s) | 28 B |
| magnetometer x/y/z | `SCALED_IMU3` `xmag,ymag,zmag` (mgauss) | 34 B |
| pressure, temperature, altitude | `SCALED_PRESSURE` `press_abs` (hPa) + `temperature` (cdegC); the GS derives altitude | 26 B |
| radiation | `COSMIC_RADIATION` `radiation` = CRD1 + CRD2 counts in the cycle | 22 B |
| velocity | `VFR_HUD` `groundspeed` = **barometric vertical speed** (m/s, + up) | 20 B |
| heartbeat | `HEARTBEAT`; `custom_mode` = thermal level ∣ phase << 8, `system_status` = ACTIVE / CRITICAL / EMERGENCY / STANDBY / POWEROFF | 21 B |

The GS ignores `NAMED_VALUE_INT` (except `BARO_T`), `FLIGHT_STATES` and
`PAYLOAD_TEMPERATURE`, so those are not sent. LoRa-board temperature, PA
bias, both Geiger channels, flight phase, flight time and thermal level are
in the SD/flash rows instead. The IMU die temperature has no GS channel.

### Packets

The adapter firmware (Ratatoskr-GS-adapter-firmware `main.cpp`) does
`radio->receive(received, 128); Serial.write(received, 128);` — it forwards
**128 bytes for every packet, whatever its length**, and the Qt app feeds
every byte to the MAVLink parser. Two consequences shape the packet design
(`include/Stack_with_LoRa/Packer.hpp`):

1. Several MAVLink frames go into one LoRa packet: one message per packet
   spent ~75 % of the air time on preambles and MAVLink headers.
2. Every packet is padded to exactly `LORA_PACKET_LEN` (64) bytes. With
   varying lengths the bytes behind a short packet are leftovers of the
   previous longer one, and any complete frame in there is decoded *again*
   as fresh data. `tools/gs_decode_sim.cpp` demonstrates both cases.

One full cycle is three 64-byte packets with 40 ms gaps
(`LORA_PACKET_GAP_MS`, so the adapter's blocking `receive()` is listening
again before the next preamble):

```
[SCALED_IMU 22 + SCALED_IMU2 28]  [SCALED_IMU3 34 + SCALED_PRESSURE 26]  [COSMIC 22 + VFR_HUD 20 + HEARTBEAT 21]
```

≈ 0.96 s on air per 1.5 s cycle (64 % duty) when the board is cool; the old
one-message-per-packet scheme needed ≈ 1.9 s per cycle for less data. 64
bytes rather than 128 keeps a packet at ~116 symbols instead of ~240, which
matters at the range limit. Heartbeat every 5 s (the GS shows "Xs ago" only
after 10 s).

### Radio profile

2400 MHz, BW 812.5 kHz, SF11, CR 4/8, sync word 0x12, CRC on, +10 dBm from
the SX1280 into the external PA whose bias (DAC43401, 3300 mV) is the real
power lever. The profile is fixed by the adapter's `config.h`; see
[docs/LoRa_RF_notes.md](docs/LoRa_RF_notes.md) for the measurements behind
the antenna-switch state, the DAC register fix and why the bandwidth stays at
812.5 kHz. TX completion is detected by polling the SX1280 IRQ register:
GPIO 11/12 are the antenna-switch lines, not the chip's DIO1/DIO2, and BUSY
is not a reliable TxDone indicator.

## Thermal throttle

The SX1280 and the SKY65383 PA are rated for −40 … +85 °C (junction max
125 °C). The TMP1075 on the LoRa board reads the board next to the module,
which is cooler than the dies, so the firmware keeps that reading well below
85 °C by stretching the transmit interval. Logging is never throttled.

| Level | TMP1075 reading | Sensor frames | Frame set | PA bias (DAC) |
|---|---|---|---|---|
| L0 | < 65 °C | every cycle (1.5 s) | all | 3300 mV |
| L1 | ≥ 65 °C | every 1.5 cycles | all | 3300 mV |
| L2 | ≥ 68 °C | every 2 cycles | all | 3300 mV |
| L3 | ≥ 71 °C | every 3 cycles | essential only | 3300 mV |
| L4 | ≥ 74 °C | every 6 cycles | essential only | 2500 mV |
| L5 | ≥ 78 °C | none | heartbeat + pressure once a minute | 2500 mV |

Essential = HEARTBEAT, ACCEL, BARO pressure + temperature, COSMIC_RADIATION.
A level is left 2 °C below its entry temperature, so this is a thermostat:
on the desk (USB, no airflow) the stack idles at ~55 °C and settled at
66–68 °C while transmitting, i.e. around L1/L2; in cooler surroundings it
runs the full rate. The reading is taken every 2 s, also on the pad, where
the beacon interval stretches the same way. Table: `LORA_THERMAL_LEVELS` in
`Config.hpp`. Bench: `thermal 2` pins level 2, `thermal auto` releases it.

## Local logging

Every cycle appends one self-labelled row to `/logs/run_NNNNNN.csv` on the
SD card and to LittleFS (`NNNNNN` = run id from NVS, incremented at every
boot). Format `key = value|key = value|...`:

```
Run_ID|Now_Ms|Accsloration_mg(x,y,z)|Gyro_mrad_s(x,y,z)|Mag_mgauss(x,y,z)|Pressure_hPa|Temperature_cC|Geiger_1|Geiger_2|LoRa_Temp_cC|LoRa_Volt_mV|LoRa_Retries|LoRa_Drops|Storage_Fails|Flight_Phase|Flight_Time_s|LoRa_Thermal_Level|Vert_Speed_mps
```

LittleFS gets ~1.4 MB with the `default.csv` partition table (the quota in
`Config.hpp` is 1.5 MB, files are auto-wiped at 90 % full). For the full
~3.4 MB switch `board_build.partitions` to `default_16MB.csv` and raise
`FLASH_QUOTA_B`.

## Serial commands (115200 baud)

| Command | Effect |
|---|---|
| `status` | run id, SD/flash state, flight phase and time, thermal level and LoRa temperature, packet counters |
| `last` | latest accel / gyro / mag / baro values |
| `print` / `clear` | dump / wipe the flash log |
| `new` | new run id (new files); also clears `flown` and the flight clock |
| `disarm` | clear `flown` + flight clock: launch gate armed at next boot |
| `fly` | set `flown`, fresh flight clock: next boot goes straight to mission mode |
| `land` | force the flight-over stop now |
| `thermal N` / `thermal auto` | pin the thermal level / automatic |
| `volt N` | set the PA bias DAC (0..3300 mV) |
| `cw on` / `cw off` | continuous carrier for RF measurements (heats fast) |

Per cycle the monitor prints one `[TX]` line per frame, `[TX] PACKET 64
bytes` per packet (plus the hex bytes when `PRINT_TX_HEX` is 1), the `[ROW]`
line and the LoRa counters; a failed transmit prints its RadioLib error code
and the packet goes to the retry queue (`LORA_RETRY_QUEUE` 16 entries,
`LORA_MAX_ATTEMPTS` 3).

## Checking against the real ground-station behaviour

The local `ground_station` project has `env:adapter_sim`, a receive-only
firmware for the Nano 33 IoT that forwards the fixed 128-byte buffer exactly
like the real adapter, wrapped in `#PKT … #END` markers with RSSI/SNR (the
Nano's DIO1 is not wired, so it receives with SPI-polled IRQs). On the PC:

```bash
g++ -std=c++17 -O1 -Wno-address-of-packed-member -I include/mavlink/library \
    -I include/Stack_with_LoRa tools/gs_decode_sim.cpp -o /tmp/gs_decode_sim   # once
~/.platformio/penv/bin/python tools/gs_capture.py \
    /dev/serial/by-id/usb-Arduino_LLC_Arduino_NANO_33_IoT_5177072C50304D484D2E3120FF131B20-if00 \
    3600 capture.bin --decode /tmp/gs_decode_sim        # live RSSI + decoded messages, Ctrl-C to stop
/tmp/gs_decode_sim --capture capture.bin                # decode a recording again
```

Run it in a second terminal next to the payload monitor (payload port:
`/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_*`). Without a
radio link, `tools/hex_to_capture.py` builds the same capture from the
payload's `[TX] PACKET bytes:` lines. For range tests use `env:test_lora_range`
on the payload: the flight packet layout with a counter in the radiation
value, no sensors or SD needed.

## Configuration switches (`include/Stack_with_LoRa/Config.hpp`)

```cpp
#define USE_SERIAL       1   // status text on USB CDC
#define PRINT_TX         1   // one [TX] line per frame / packet
#define PRINT_TX_HEX     1   // also the packet bytes in hex (needed by hex_to_capture.py)
#define USE_LORA         1   // SX1280 + MAVLink packets
#define USE_SD           1   // microSD row log
#define USE_FLASH        1   // LittleFS row log
#define USE_GEIGER       1   // CRD1 + CRD2 pulse counters
#define USE_IMU          1   // ICM-45686
#define USE_BARO         1   // BMP388/390
#define USE_MAG          1   // BMM350
#define USE_LORA_TEMP    1   // TMP1075 (thermal throttle input)
#define USE_LORA_VOLT    1   // DAC43401 PA bias
#define USE_LORA_THERMAL 1   // the throttle itself
#define USE_LAUNCH_GATE  1   // wait for launch on the pad (-DUSE_LAUNCH_GATE=0 to build without)
#define LAUNCH_USE_LIGHT_SLEEP 0 // 1 = esp_light_sleep on the pad (kills USB serial; flight only)
```

## Source layout

```
include/Stack_with_LoRa/
  Config.hpp         switches, pins, radio profile, timing, thermal levels, flight stop
  Sensors.hpp        reading structs + Sensors (ICM-45686, BMP3, BMM350, TMP1075, DAC43401)
  MavlinkFrames.hpp  one builder per message the GS decodes
  Packer.hpp         several frames -> one fixed-length packet
  LoraTx.hpp         SX1280 init, IRQ-polled TX, retry queue, sleep/wake, CW
  Thermal.hpp        level selection with hysteresis
  Storage.hpp        SD + LittleFS row mirror
  App.hpp            launch gate, cycle, throttle, flight clock, serial commands
src/Stack_with_LoRa/ the matching .cpp files + main.cpp (globals, Geiger ISRs)
```
