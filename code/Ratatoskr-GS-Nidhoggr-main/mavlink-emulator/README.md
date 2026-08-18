# MAVLink serial emulator (Horizon Ground Station)

A standalone, dependency-free Python script that pretends to be the rocket's
serial telemetry link. It creates a virtual serial port and streams MAVLink v2
frames matching exactly what `HorizonGroundStation` parses, including the custom
`HorizonDialect` messages.

## Run

```bash
./mavlink_emulator.py
```

It prints the virtual device it created, e.g.:

```
[emulator] streaming MAVLink v2 on: /dev/pts/5
[emulator] point the ground station at this device: /dev/pts/5
```

Options:

| flag | meaning |
|------|---------|
| `--port /dev/ttyX` | write to an existing device instead of creating a PTY |
| `--hz 50` | base update rate (default 50 Hz) |
| `--no-flight` | steady idle values instead of a simulated flight arc |
| `--heartbeat-period 5` | seconds between heartbeats, per board (default 5) |
| `--fallout` | periodically kill the whole link (including heartbeats) to simulate signal dropout |
| `--fallout-on 20` | seconds the signal stays up per fallout cycle (default 20) |
| `--fallout-off 6` | seconds the signal stays down per fallout cycle (default 6) |

By default it plays a full rocket arc: idle → thrust → coast → apogee → drogue →
main → touchdown, driving altitude, pressure, temperature, acceleration,
velocity, radiation and flight-state.

With `--fallout`, the link square-waves between fully up and fully silent
(no messages at all, heartbeats included) — useful for exercising the ground
station's "signal lost" / heartbeat-staleness behaviour:

```bash
./mavlink_emulator.py --fallout --fallout-on 15 --fallout-off 5
```

## Messages streamed

| MAVLink message | id | drives in the GS |
|-----------------|----|------------------|
| `SCALED_IMU` | 26 | acceleration graph |
| `SCALED_IMU2` | 116 | rotation graph |
| `SCALED_PRESSURE` | 29 | pressure / temperature / altitude |
| `VFR_HUD` | 74 | velocity |
| `NAMED_VALUE_INT` "BARO_T" | 252 | temperature |
| `COSMIC_RADIATION` | 16002 | radiation (custom dialect) |
| `HEARTBEAT` | 0 | heartbeat indicator (per board, every `--heartbeat-period` s) |
| `FLIGHT_STATES` | 16000 | flight phase (custom dialect) |
| `PAYLOAD_TEMPERATURE` | 16001 | stack temps (custom dialect) |

CRC_EXTRA values are taken straight from `HorizonDialect.h`, so frames pass the
GS's `mavlink_frame_char` CRC check.

## Feeding it into the ground station

The GS auto-discovers its port by the Arduino Nano 33 IoT USB VID/PID
(`SerialReader::findHorizonPort()`), and a virtual PTY has no such IDs — so the
GS will **not** pick it up automatically. Two ways to connect them:

**A. Point the GS at the PTY (small code change).** Have `main.cpp` honour an
env var that overrides auto-discovery when set:

```cpp
QString port = qEnvironmentVariable("HORIZON_SERIAL_PORT");  // explicit override
if (port.isEmpty())
    port = SerialReader::findHorizonPort();                  // else auto-discover
```

Then:

```bash
./mavlink_emulator.py                 # note the /dev/pts/N it prints
HORIZON_SERIAL_PORT=/dev/pts/N ./HorizonGroundstation
```

**B. Use socat for a fixed device name.** Create a linked pair and aim the
emulator at one end, the GS (via the env override above) at the other:

```bash
socat -d -d pty,raw,echo=0,link=/tmp/horizon_a pty,raw,echo=0,link=/tmp/horizon_b
./mavlink_emulator.py --port /tmp/horizon_a
HORIZON_SERIAL_PORT=/tmp/horizon_b ./HorizonGroundstation
```

## How it works

Pure-Python MAVLink v2 framing: `0xFD` start byte, 24-bit little-endian message
id, X25/MCRF4XX checksum seeded with each message's `CRC_EXTRA`, v2 trailing-zero
payload truncation. No `pymavlink` or generated dialect needed.
