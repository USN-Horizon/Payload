# `Stack_without_LoRa` — the two payload stacks without a radio

Firmware for the two stacks whose LoRa board has no SX1280. There is nothing
to transmit, so the only goal is to put as much sensor data on the microSD
card as the hardware can produce. The stack with the radio runs
[`Stack_with_LoRa`](README_Stack_with_LoRa.md).

```bash
pio run -e Stack_without_LoRa -t upload
pio device monitor -e Stack_without_LoRa --dtr 1 --rts 1
```

After the upload press the reset button (or use the esptool watchdog reset
from [README.md](README.md)); the board otherwise stays in the bootloader.

## What is logged, and how fast

| Record | Source | Rate in FLIGHT | Rate on PAD / POST | Columns |
|---|---|---|---|---|
| `IMU` | ICM-45686, 20-bit FIFO packets | 400 Hz | 20 Hz | `t_us,ax_mg,ay_mg,az_mg,gx_dps,gy_dps,gz_dps` |
| `BARO` | BMP388/390, continuous mode | 50 Hz | 5 Hz | `t_us,press_hPa,temp_C,alt_m` |
| `MAG` | BMM350, 4× averaging | 100 Hz | 5 Hz | `t_us,mx_uT,my_uT,mz_uT,temp_C` |
| `GEIG` | Geiger pulse, time-stamped in the ISR | every pulse | every pulse | `t_us,ch` |
| `RATE` | Geiger counts per window + totals | 1 Hz | 1 Hz | `t_us,dt_ms,ch1,ch2,ch1_total,ch2_total,ts_dropped` |
| `TEMP` | IMU die, ESP32 die, TMP1075 on the LoRa board | 1 Hz | 1 Hz | `t_us,imu_C,esp_C,lora_C` |
| `STAT` | firmware health | 1 Hz | 1 Hz | see the header line in the file |
| `EVT` | boot, sensor init, mode changes, SD errors | as they happen | | `t_us,text` |

- `t_us` is microseconds since boot (64-bit, never wraps). IMU rows are
  stamped by a small phase-locked sample clock (one step per FIFO frame,
  nudged toward the ESP32 clock), so they are evenly spaced instead of
  jittering with the 2 ms FIFO poll; they can lag the other rows by ~1 ms.
- IMU values come from the chip's 8 kB hardware FIFO in high-resolution mode:
  20-bit samples always scaled to ±32 g / ±4000 dps (0.06 mg, 0.008 °/s per
  LSB). The FIFO holds about one second, so an SD-card stall never loses a
  sample.
- Geiger pulses are stamped inside the interrupt (µs) and buffered 256 deep
  per channel, so the two tubes can be compared pulse by pulse. The inputs
  use the internal pull-up, so a stack with an empty CRD connector stays
  quiet instead of counting noise.
- `alt_m` is standard-atmosphere altitude against 1013.25 hPa.

Measured on the bench: 397 IMU rows/s (the chip's real ODR), ~25 kB/s to
the card in FLIGHT with no drops, ~2 kB/s on the pad.

## Modes

```
boot ─► PAD ─(|a| > 2.0 g for 100 ms)─► FLIGHT ─(30 min)─► POST ─(launch again)─► FLIGHT
```

- **PAD**: low rates while waiting on the rail. The last second of IMU data
  is always kept in RAM at full rate and written out when FLIGHT starts, so
  the ignition transient is in the file although PAD was decimating.
- **FLIGHT**: full rate. Sets `flown=1` in NVS (own namespace, separate from
  `Stack_with_LoRa`). A brownout or reset during the flight boots straight
  back into FLIGHT.
- **POST**: back to PAD rates after `FLIGHT_FULL_RATE_S` (30 min) so a long
  recovery does not produce a giant file. A new launch-like acceleration
  re-enters FLIGHT, so a false trigger only costs bytes.
- No IMU at boot: FLIGHT immediately (no way to detect launch, so log all).

Thresholds and rates: `include/Stack_without_LoRa/Config.hpp`.

## Files on the card

```
/logs/run_000012.csv        one file per run; the run id counts up in NVS at every boot
/logs/run_000012_p2.csv     next "part" after an SD error + remount
```

The first lines are `#` header lines with the firmware version, reset reason
and the column names of every record type. Rows are written in 4 kB chunks
(or every 250 ms) and fsync'ed every second, so a power cut loses at most
about one second. Split a run into one CSV per record type (also sorts each
type by `t_us`, needed because the pre-trigger dump writes IMU rows that are
older than rows already on the card):

```bash
python3 tools/split_log.py run_000012.csv
python3 tools/split_log.py run_000012.csv --summary    # counts and rates only
```

pandas: `df = pd.read_csv("run_000012.csv", comment="#", header=None, names=range(17)); imu = df[df[0]=="IMU"]`.

## If the SD card dies

If a write fails the firmware closes the file, unmounts, and retries the
mount every 10 s, opening a new part file when it succeeds. Meanwhile
LittleFS in the ESP32's flash receives a reduced stream (IMU 10 Hz, baro and
mag 2 Hz, all Geiger pulses, all 1 Hz rows) so a flight is never completely
lost; `EVT` rows always go to flash. The flash log is auto-wiped at boot
when 90 % full; read it with `print`. (Not yet exercised on hardware: pull
the card while it runs to test it.)

## Serial commands (115200 baud)

| Command | Effect |
|---|---|
| `status` | mode, run id, sensor state incl. the baro init trace, SD / flash counters |
| `last` | latest IMU / baro / mag values |
| `ls` | newest files in `/logs` on the card |
| `tail [n]` | last *n* lines of the current file (flushes first) |
| `head` | first 2 kB of the current file |
| `sync` | flush pending rows to the card now |
| `new` | start a new run (new files) |
| `flight` / `pad` | force full-rate / low-rate mode |
| `arm` | clear the `flown` flag so the next boot starts in PAD |
| `print` / `clear` | dump / wipe the flash fallback log |

Serial output never blocks the logger: once booted, text is dropped if
nobody reads the USB port (`Serial.setTxTimeoutMs(0)`), so the firmware
behaves the same with or without a PC attached. Long dumps (`ls`, `tail`)
do block the loop briefly; the IMU FIFO covers up to about one second.

## Hardware notes for these stacks

- The LoRa board is present but has no SX1280. Its power enable (GPIO 10) is
  held LOW and the SX1280 reset / CS lines are parked (RST low, CS high),
  which also keeps the SPI bus clean for the SD card if a radio is fitted.
- The TMP1075 on the LoRa board is auto-detected; absent is fine.
- CRD board: one stack powers it (4-pin connector), the others only take the
  two signal lines (3-pin connector) with a shared ground, so every stack can
  log the pulses.
- `env:test_sensors_fifo` runs this firmware's sensor layer stand-alone and
  reports the achieved sample rates.

## Source layout

```
include/Stack_without_LoRa/
  Config.hpp      switches, pins, rates, thresholds, storage settings
  Sensors.hpp     IMU FIFO (subclass of the TDK driver), BMP3 normal mode,
                  BMM350, TMP1075, ESP32 die temperature
  Geiger.hpp      ISRs + per-channel timestamp rings
  Logger.hpp      RAM ring -> SD chunks / fsync / remount, LittleFS fallback
  RowFmt.hpp      integer-only CSV row builder (no printf in the hot path)
  App.hpp         modes, launch detector, pre-trigger ring, serial commands
src/Stack_without_LoRa/   the matching .cpp files + main.cpp
tools/split_log.py        run file -> one CSV per record type
```
