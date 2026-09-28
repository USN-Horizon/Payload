# USN Horizon payload firmware (ESP32-S3)

Three identical sensor stacks fly in the rocket. Each stack is a main board
(ESP32-S3-WROOM-1U-N16R2, microSD), a LoRa board and a telemetry board
(ICM-45686 IMU, BMP388 barometer, BMM350 magnetometer, Geiger inputs). All
three share one Geiger (CRD) board with two tubes: one stack powers it, the
others only take the two pulse lines. Only **one** LoRa board carries an
SX1280 radio, so there are two firmwares:

| Environment | Runs on | Purpose | Details |
|---|---|---|---|
| `Stack_with_LoRa` | the stack whose LoRa board has the SX1280 | MAVLink telemetry over 2.4 GHz LoRa to the team's ground station, plus a low-rate row log on SD / flash. Launch gate on the pad, thermal throttle for the radio, automatic stop 30 min after launch. | [README_Stack_with_LoRa.md](README_Stack_with_LoRa.md) |
| `Stack_without_LoRa` | the two other stacks | Logs every sensor at full rate to the SD card: IMU 400 Hz from the chip FIFO, baro 50 Hz, mag 100 Hz, every Geiger pulse time-stamped. | [README_Stack_without_LoRa.md](README_Stack_without_LoRa.md) |
| `test_*` | one peripheral each | Hardware sanity checks, PASS / FAIL on the serial monitor. | below |

## Build, flash, monitor

`pio` and a Python with pyserial live in PlatformIO's own environment. Either
use the VS Code PlatformIO buttons, or add
`export PATH="$HOME/.platformio/penv/bin:$PATH"` to `~/.bashrc`.

```bash
pio run -e Stack_with_LoRa -t upload          # or Stack_without_LoRa, test_imu, ...
pio device monitor -e Stack_with_LoRa --dtr 1 --rts 1
```

Two quirks of this main board:

- **After every upload the board sits in the ROM bootloader.** A USB-triggered
  reset on this board always selects download mode (GPIO0 reads low), so the
  app does not start until you press the reset button, or run:
  ```bash
  ~/.platformio/penv/bin/python ~/.platformio/packages/tool-esptoolpy/esptool.py \
      --chip esp32s3 --port /dev/ttyACM0 --before default_reset --after watchdog_reset --no-stub chip_id
  ```
  (the port may re-enumerate as a different `/dev/ttyACMn` afterwards).
- **Open the monitor with `--dtr 1 --rts 1`.** Opening the port with the
  control lines dropped resets the board into the bootloader as well.

Use the `/dev/serial/by-id/...` names when two boards are connected; the
`/dev/ttyACMn` numbers swap around on every reset.

## Hardware summary

| Signal | GPIO | Notes |
|---|---|---|
| SPI MOSI / SCK / MISO | 4 / 5 / 6 | shared by SD, IMU, SX1280 |
| I2C SDA / SCL | 7 / 15 | BMP388 0x76, BMM350 0x14, TMP1075 0x49, DAC43401 0x48 |
| ICM-45686 CS / INT1 / INT2 | 16 / 17 / 18 | IF8 / IF9 / IF10 |
| Geiger CRD1 / CRD2 | 8 / 3 | IF11 / IF12, pulse inputs |
| SX1280 CS / power enable / BUSY / RST | 9 / 10 / 13 / 14 | IF13 / IF14 / IF17 / IF18 |
| Antenna switch CXT / CrX | 11 / 12 | IF15 / IF16 (the PDF calls them DIO1/DIO2; they are not the chip's DIOs) |
| microSD CS | 21 | |

Full pin table and schematics: `Ekstra_Info/PCB Dokumentasjon/` (not in
git). Note the board document's I2C addresses for the DAC (0x72) and TMP1075
(0x73) are wrong; the chips answer at 0x48 and 0x49.

## Component tests

```bash
pio run -e test_imu -t upload && pio device monitor -e test_imu --dtr 1 --rts 1
```

| Environment | Checks |
|---|---|
| `test_imu` | ICM-45686 over SPI: init, gravity on one axis, gyro still |
| `test_baro` | BMP388/390: init, plausible pressure and temperature |
| `test_mag` | BMM350: init, Earth-field magnitude |
| `test_geiger_dual` | both Geiger inputs, counts per 10 s |
| `test_sdcard` | SD mount, write, read back |
| `test_flash` | LittleFS mount, write, read back |
| `test_i2c_scan` | which addresses answer on the I2C bus |
| `test_lora` | SX1280 init with the flight profile, one 64-byte TX |
| `test_lora_temp` | TMP1075 on the LoRa board |
| `test_lora_volt` | DAC43401 (PA bias) register round-trips |
| `test_lowpower_wake` | launch detection on the accelerometer (shake to trigger) |
| `test_lora_range` | production radio code sending the flight packet layout with a counter, for range tests against the ground station |
| `test_sensors_fifo` | the production sensor layer of `Stack_without_LoRa`: measures the real IMU / baro / mag sample rates |

## Tools (PC side)

| File | Purpose |
|---|---|
| `tools/split_log.py` | splits a `Stack_without_LoRa` run file into one CSV per record type |
| `tools/gs_capture.py` | records / live-decodes what the ground-station adapter forwards (needs the Nano with the `adapter_sim` firmware from the `ground_station` project) |
| `tools/gs_decode_sim.cpp` | PC simulation of the adapter + Qt ground station parser; decodes captures, flags stale frames |
| `tools/hex_to_capture.py` | builds such a capture from the payload's `[TX] PACKET bytes:` serial lines (no radio needed) |

## Documents

- [docs/LoRa_RF_notes.md](docs/LoRa_RF_notes.md): what is known about the RF
  path, the radio profile, the ground-station constraints and the thermal
  behaviour, with the measurements behind it.
- `Needs_to_be_done.txt`: open items.
- `Ekstra_Info/` (not in git): datasheets, PCB documentation, and
  `Avionics-main`, the team's avionics + ground-station repository. The
  ground station (adapter firmware + Nidhoggr Qt app) is the reference the
  payload must stay compatible with.

## Repository layout

```
platformio.ini                  environments (see above)
include/Stack_with_LoRa/  src/Stack_with_LoRa/       radio stack firmware
include/Stack_without_LoRa/  src/Stack_without_LoRa/ SD-logger firmware
include/mavlink/                MAVLink C library with the HorizonDialect (USN-Horizon/mavlink)
src/tests/                      component tests, one file per env
tools/                          PC-side scripts
docs/                           notes
a_BACKUP/                       manual snapshots of earlier firmware versions
```
