# Libraries: what the payload code uses, and what for

Every external firmware library is declared in `platformio.ini` (`lib_deps`).
PlatformIO downloads them separately for each environment into
`.pio/libdeps/<env>/`. The one exception is MAVLink: its generated C headers
are vendored in `include/mavlink/`. Everything else comes with the
Arduino-ESP32 core.

## Overview

| Library | Version | Source | Used for |
|---|---|---|---|
| [ICM45686](#icm45686-imu) (TDK InvenSense) | `1.1.1`, pinned git tag | `github.com/tdk-invn-oss/motion.arduino.ICM45686` | ICM-45686 IMU: accelerometer + gyro over SPI, launch detection, 400 Hz FIFO logging |
| [Adafruit BMP3XX Library](#adafruit-bmp3xx-barometer) | `^2.1.6` (2.1.6 installed) | PlatformIO registry | BMP388/390 barometer: pressure + temperature |
| ↳ Adafruit BusIO | 1.17.4 | pulled in by BMP3XX | I2C/SPI device helper the Adafruit driver is built on |
| ↳ Adafruit Unified Sensor | 1.1.15 | pulled in by BMP3XX | `Adafruit_Sensor` base class, not used directly |
| [DFRobot_BMM350](#dfrobot_bmm350-magnetometer) | 1.0.0, **unpinned** (git HEAD) | `github.com/DFRobot/DFRobot_BMM350` | BMM350 magnetometer |
| [RadioLib](#radiolib-sx1280-lora-radio) | `^7.6.0` (7.8.0 installed for `Stack_with_LoRa`) | PlatformIO registry `jgromes/RadioLib` | SX1280 2.4 GHz LoRa radio |
| [MAVLink, HorizonDialect](#mavlink-horizondialect-telemetry-frames) | `USN-Horizon/mavlink` @ `78727f8` | vendored in `include/mavlink/` | building the telemetry frames the ground station decodes |
| [Arduino-ESP32 core](#arduino-esp32-core-built-in-libraries) | 2.0.17 (platform `espressif32` 6.13.0) | PlatformIO platform | `Wire`, `SPI`, `SD`, `LittleFS`, `Preferences`, interrupts, ... |
| [ESP-IDF](#esp-idf-apis) | 4.4.7 (bundled with the core) | n/a | light sleep, microsecond timer, reset reason |
| [pyserial](#pc-side-tools) | 3.5 | PlatformIO's Python (`~/.platformio/penv`) | `tools/gs_capture.py` reading the ground-station adapter |

## Which environment uses what

| Environment | ICM45686 | BMP3XX | BMM350 | RadioLib | MAVLink |
|---|:-:|:-:|:-:|:-:|:-:|
| `Stack_with_LoRa` | ✓ | ✓ (Adafruit class) | ✓ | ✓ | ✓ |
| `Stack_without_LoRa` | ✓ (FIFO) | ✓ (Bosch API only) | ✓ | | |
| `test_imu` | ✓ | | | | |
| `test_baro` | | ✓ | | | |
| `test_mag` | | | ✓ | | |
| `test_lora` | | | | ✓ | |
| `test_lowpower_wake` | ✓ | | | | |
| `test_lora_range` | headers¹ | headers¹ | headers¹ | ✓ | ✓ |
| `test_sensors_fifo` | ✓ | ✓ | ✓ | | |
| `test_geiger_dual`, `test_sdcard`, `test_flash`, `test_i2c_scan`, `test_lora_temp`, `test_lora_volt` | core only | | | | |

¹ `test_lora_range` builds `LoraTx.cpp` and `MavlinkFrames.cpp` from
`Stack_with_LoRa`. Their `Config.hpp` includes the sensor headers, so the
sensor libraries must be present for it to compile. No sensor is read.

---

## ICM45686 (IMU)

`https://github.com/tdk-invn-oss/motion.arduino.ICM45686.git#1.1.1`

TDK InvenSense's official Arduino driver for the ICM-45686 6-axis IMU. The
chip sits on the shared SPI bus (CS GPIO 16).

- **`Stack_with_LoRa`** ([src/Stack_with_LoRa/Sensors.cpp](src/Stack_with_LoRa/Sensors.cpp)):
  an `ICM456xx` object. `begin()`, `startAccel(100, 16)` (100 Hz, ±16 g) and
  `startGyro(100, 2000)` (100 Hz, ±2000 dps). It is polled at 20 Hz with
  `getDataFromRegisters()`. It supplies:
  - acceleration for `SCALED_IMU` and rotation for `SCALED_IMU2`;
  - the **launch gate**: on the pad, |a| > 1.5 g ends the low-power wait;
  - the THRUSTING → COASTING phase change.

  At mission end `stopAccel()` / `stopGyro()` switch the IMU off.
- **`Stack_without_LoRa`** ([src/Stack_without_LoRa/Sensors.cpp](src/Stack_without_LoRa/Sensors.cpp)):
  `class ImuFifo : public ICM456xx` ([include/Stack_without_LoRa/Sensors.hpp](include/Stack_without_LoRa/Sensors.hpp)).
  It runs accel + gyro at 400 Hz into the chip's 8 kB FIFO (snapshot mode,
  20-byte high-resolution packets) and burst-reads the FIFO. The Arduino
  wrapper has no burst FIFO read, so the subclass calls the driver's
  low-level C functions on the `icm_driver` member: `inv_imu_set_fifo_config`,
  `inv_imu_get_frame_count`, `inv_imu_read_reg(FIFO_DATA)` and
  `inv_imu_flush_fifo`. The packets are decoded in `drainImu()`, and the
  layout copies `inv_imu_get_fifo_frame()`.
- **Tests:** `test_imu` (gravity / rest check), `test_lowpower_wake` (launch
  detection with light sleep in between), `test_sensors_fifo` (measures the
  real FIFO rate).

> Keep the version pinned. `ImuFifo` depends on driver internals
> (`icm_driver`, register and enum names), which are not part of the public
> Arduino API and may change between releases.

## Adafruit BMP3XX (barometer)

`adafruit/Adafruit BMP3XX Library@^2.1.6`

The Adafruit driver for the Bosch BMP388/390. The fitted chip reports id 0x50
(BMP388) on I2C address 0x76. The library also ships Bosch's own C driver
(`bmp3.h` / `bmp3.c`), and the two firmwares use different layers of it:

- **`Stack_with_LoRa`** uses the Adafruit class `Adafruit_BMP3XX`:
  `begin_I2C(0x76, &Wire)`, 8× temperature / 4× pressure oversampling,
  IIR coefficient 3, and one `performReading()` per poll (forced mode).
  It supplies pressure and temperature for `SCALED_PRESSURE`, and the
  barometric vertical speed sent as `VFR_HUD.groundspeed`.
- **`Stack_without_LoRa`** skips the Adafruit class and calls the **Bosch
  BMP3 API** directly (`#include "bmp3.h"` inside `extern "C"`). It supplies
  its own `Wire`-based read / write / delay callbacks to a `bmp3_dev`,
  then calls `bmp3_init()`, `bmp3_set_sensor_settings()` and
  `bmp3_set_op_mode(BMP3_MODE_NORMAL)`. The reason: the Adafruit class only
  does blocking forced-mode reads. In normal (continuous) mode at 50 Hz the
  firmware polls the data-ready flag, so the main loop never waits on the
  barometer. If the chip rejects 50 Hz for the chosen oversampling, it falls
  back to 25 Hz.
- **Tests:** `test_baro` (Adafruit class, plausible pressure / temperature),
  `test_sensors_fifo`.

PlatformIO pulls in **Adafruit BusIO** (the `Adafruit_I2CDevice` /
`Adafruit_SPIDevice` layer used by `Adafruit_BMP3XX`) and **Adafruit Unified
Sensor** automatically as dependencies. Our code never includes either one.

## DFRobot_BMM350 (magnetometer)

`https://github.com/DFRobot/DFRobot_BMM350.git`

DFRobot's driver for the Bosch BMM350 3-axis magnetometer, on I2C address
0x14. It is used as a `DFRobot_BMM350_I2C(&Wire, 0x14)` object.

- **`Stack_with_LoRa`**: `begin()`, then `setOperationMode(eBmm350NormalMode)`.
  Without that step the chip stays suspended and returns garbage. Each poll
  calls `getGeomagneticData()` and uses `float_x/y/z` (µT) × 10 to get
  mgauss for `SCALED_IMU3`. The integer `x/y/z` fields are internal scaled
  values, not µT. At mission end `setOperationMode(eBmm350SuspendMode)`
  puts the magnetometer back into suspend mode.
- **`Stack_without_LoRa`**: `setPresetMode(BMM350_PRESETMODE_ENHANCED,
  BMM350_DATA_RATE_100HZ)` (average of 4, 100 Hz), then
  `setDataReadyPin(BMM350_ENABLE_INTERRUPT, ...)` and normal mode. The INT
  pin itself is not wired. Enabling the interrupt is what makes the polled
  data-ready flag work; without it the magnetometer delivered only ~33 Hz.
- **Tests:** `test_mag` (Earth-field magnitude), `test_sensors_fifo`.

> Not pinned: every fresh install takes the repository's current HEAD. Add a
> `#<commit>` to the URL in `platformio.ini` if the build must be
> reproducible.

## RadioLib (SX1280 LoRa radio)

`jgromes/RadioLib@^7.6.0`

A general radio library. Here it drives the Semtech SX1280 on the LoRa board
(CS 9, BUSY 13, RST 14, shared SPI bus). It is only used where a radio
exists: `Stack_with_LoRa`, `test_lora`, `test_lora_range`.

- **[src/Stack_with_LoRa/LoraTx.cpp](src/Stack_with_LoRa/LoraTx.cpp)**:
  `Module(CS, RADIOLIB_NC, RST, BUSY)` + `SX1280`. DIO1 is `RADIOLIB_NC`
  because GPIO 11 is the antenna-switch line, not the chip's DIO1.
  - `begin()`, then `setFrequency`, `setBandwidth`, `setSpreadingFactor`,
    `setCodingRate` and `setOutputPower` with the flight profile from
    `Config.hpp`.
  - Transmit: `startTransmit()`, then `getIrqStatus()` is polled for
    `SX128X_IRQ_TX_DONE`, then `finishTransmit()`. This replaces the usual
    DIO1 interrupt, which is not connected.
  - `sleep(true)` (retention sleep, configuration kept) between pad beacons
    and after the mission; `standby()` to wake up.
  - The error codes (`RADIOLIB_ERR_*`) are printed on the serial monitor
    when a transmit fails.
- **Tests:** `test_lora` (init + one 64-byte TX), `test_lora_range` (the
  production `LoraTx` sending the flight packet layout).
- Quirks found on this board, such as `setCRC(1)` being rejected and RST
  needing to be held low until `begin()`, are listed in
  [docs/LoRa_RF_notes.md](docs/LoRa_RF_notes.md).

> `^7.6.0` accepts any 7.x release, and each environment keeps whatever
> version it installed first: `Stack_with_LoRa` and `test_lora_range`
> currently have 7.8.0, `test_lora` has 7.6.0. Run `pio pkg update -e <env>`,
> or pin an exact version, if they must match.

## MAVLink, HorizonDialect (telemetry frames)

`include/mavlink/`: a copy of the team's fork
[USN-Horizon/mavlink](https://github.com/USN-Horizon/mavlink) (commit
`78727f8`, "Implemented Library"). It is checked into this repository, not
fetched by PlatformIO.

MAVLink is a header-only message library. `library/` holds the C headers
generated from the XML message definitions. The firmware includes
`<mavlink/library/HorizonDialect/mavlink.h>` (from
[include/Stack_with_LoRa/Config.hpp](include/Stack_with_LoRa/Config.hpp)).
That header contains the standard messages plus the team's own messages
`COSMIC_RADIATION`, `FLIGHT_STATES` and `PAYLOAD_TEMPERATURE`.

- **[src/Stack_with_LoRa/MavlinkFrames.cpp](src/Stack_with_LoRa/MavlinkFrames.cpp)**
  packs one frame per value the ground station reads, then calls
  `mavlink_msg_to_send_buffer()`:

  | Function | Payload data |
  |---|---|
  | `mavlink_msg_heartbeat_pack` | status, flight phase, thermal level |
  | `mavlink_msg_scaled_imu_pack` | acceleration |
  | `mavlink_msg_scaled_imu2_pack` | rotation |
  | `mavlink_msg_scaled_imu3_pack` | magnetic field |
  | `mavlink_msg_scaled_pressure_pack` | pressure, temperature |
  | `mavlink_msg_cosmic_radiation_pack` | Geiger counts (HorizonDialect) |
  | `mavlink_msg_vfr_hud_pack` | vertical speed |

  `Packer.hpp` then packs these frames into fixed-length 64-byte LoRa
  packets (see [README_Stack_with_LoRa.md](README_Stack_with_LoRa.md)).
- **[tools/gs_decode_sim.cpp](tools/gs_decode_sim.cpp)** (PC) uses the same
  headers the other way round. It runs `mavlink_frame_char()` on every byte
  and calls `mavlink_msg_*_decode()`, the same as the ground station's parser.
- `FLIGHT_STATES` and `PAYLOAD_TEMPERATURE` exist in the dialect but are not
  sent, because the ground station does not decode them.
- `pymavlink` and the rest of the fork (generator, XML, docs) are only needed
  to regenerate the headers after a change to the dialect.
- `Stack_without_LoRa` sends nothing over the air and does not use MAVLink.

## Arduino-ESP32 core (built-in libraries)

These ship with `framework = arduino` (Arduino-ESP32 2.0.17 on platform
`espressif32` 6.13.0). They are not listed in `lib_deps`.

| Header | Used for | Where |
|---|---|---|
| `Arduino.h` | `Serial` console and commands, `millis` / `delay`, GPIO. `attachInterrupt(..., RISING)` counts the Geiger pulses on CRD1 (GPIO 8) and CRD2 (GPIO 3) in `IRAM_ATTR` ISRs. In `Stack_without_LoRa`, `temperatureRead()` reads the ESP32 die temperature and `ESP.getFreeHeap()` the free heap for the STAT rows. | everywhere |
| `Wire.h` | I2C bus (SDA 7 / SCL 15, 400 kHz): BMP388, BMM350, plus the two chips that have no library (below). | both firmwares, `test_baro`, `test_mag`, `test_i2c_scan`, `test_lora_temp`, `test_lora_volt` |
| `SPI.h` | The shared SPI bus (MOSI 4 / SCK 5 / MISO 6): IMU, microSD, SX1280. | both firmwares, `test_imu`, `test_sdcard`, `test_lora`, `test_lowpower_wake` |
| `SD.h` + `FS.h` | microSD card (CS 21). Row log in `Stack_with_LoRa` ([Storage.cpp](src/Stack_with_LoRa/Storage.cpp)). One full-rate file per run, `/logs/run_NNNNNN.csv`, in `Stack_without_LoRa` ([Logger.cpp](src/Stack_without_LoRa/Logger.cpp)). | both firmwares, `test_sdcard` |
| `LittleFS.h` | Filesystem on the ESP32's own 16 MB flash. In `Stack_with_LoRa` it holds a copy of the row log. In `Stack_without_LoRa` it is a fallback log, written only while the SD card is down. | both firmwares, `test_flash` |
| `Preferences.h` | NVS key/value storage that survives a reset. `Stack_with_LoRa` stores `flown` (the launch gate has tripped), `fl_s` (flight time so far) and `run`. `Stack_without_LoRa` stores `run` (run counter for the file name) and `flown`. | both firmwares |

### Chips driven without a library

Both chips are on the LoRa board and talk a small register protocol, so the
code uses plain `Wire` calls
([src/Stack_with_LoRa/Sensors.cpp](src/Stack_with_LoRa/Sensors.cpp)):

| Chip | I2C address | What the code does |
|---|---|---|
| TMP1075 temperature sensor | 0x49 | Reads register 0x00 (2 bytes). The temperature is the input to the radio's thermal throttle ([Thermal.hpp](include/Stack_with_LoRa/Thermal.hpp)). |
| DAC43401 (PA bias voltage) | 0x48 | Wakes the DAC with DEVICE_UNLOCK (0x36) and a cleared GENERAL_CONFIG (0xD1). DAC_DATA (0x21, code << 4) then sets the PA bias: 3.3 V in flight, lower at the top thermal levels, 0 V at mission end. |

## ESP-IDF APIs

C APIs from the ESP-IDF 4.4.7 build that the Arduino core is based on:

| Header | Function | Used for |
|---|---|---|
| `esp_sleep.h` | `esp_sleep_enable_timer_wakeup()`, `esp_light_sleep_start()` | Light sleep between accelerometer checks while waiting on the pad (`Stack_with_LoRa`, `test_lowpower_wake`). |
| `esp_timer.h` | `esp_timer_get_time()` | 64-bit microsecond timestamps for every log row and every Geiger pulse (`Stack_without_LoRa`). |
| `esp_system.h` | `esp_reset_reason()` | Writes the reset reason into the log header (`Stack_without_LoRa`). |

## PC-side tools

| Tool | Libraries |
|---|---|
| [tools/gs_capture.py](tools/gs_capture.py) | **pyserial** (`import serial`) reads the ground-station adapter's USB serial port. It is already installed in PlatformIO's Python, so run it with `~/.platformio/penv/bin/python`, or `pip install pyserial`. Otherwise only the standard library (`subprocess`, `sys`, `time`). |
| [tools/split_log.py](tools/split_log.py) | Standard library only (`argparse`, `csv`, `collections`, `re`, ...). |
| [tools/hex_to_capture.py](tools/hex_to_capture.py) | Standard library only (`re`, `sys`). |
| [tools/gs_decode_sim.cpp](tools/gs_decode_sim.cpp) | C++17 standard library, the vendored MAVLink HorizonDialect headers and `include/Stack_with_LoRa/Packer.hpp`. It needs no Arduino code. Build with `g++ -std=c++17 -I include/mavlink/library -I include/Stack_with_LoRa ...`. |
