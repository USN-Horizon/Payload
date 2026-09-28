#pragma once

// =============================================================================
//  Config.hpp
//  Central configuration for the Stack_with_LoRa flight firmware (the
//  stack whose LoRa board carries the SX1280).
//  Contains feature switches, pin map, I2C addresses, LoRa radio settings,
//  MAVLink identity, timing constants, and shared globals.
// =============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Preferences.h>
#include "esp_system.h"

// -----------------------------------------------------------------------------
//  Feature switches (1 = enabled, 0 = disabled).
//  Disable any peripheral that is missing on the bench so the firmware still
//  boots and continues to log whatever sensors are present.
// -----------------------------------------------------------------------------
#define USE_SERIAL     1   // Print status/debug to USB CDC.
#define PRINT_TX       1   // One [TX] line per MAVLink frame and per LoRa packet.
#define PRINT_TX_HEX   1   // Also the bytes (hex) of every packet; tools/hex_to_capture.py reads them.
#define USE_LORA       1   // SX1280 transmitter + MAVLink packets.
#define USE_SD         1   // microSD card on PIN_SD_CS.
#define USE_FLASH      1   // LittleFS log mirror in onboard flash.
#define USE_GEIGER     1   // Geiger pulse counter (CRD1 + CRD2).
#define USE_IMU        1   // ICM-45686 accelerometer + gyroscope.
#define USE_BARO       1   // BMP388/390 barometer (the fitted chip reports id 0x50 = BMP388).
#define USE_MAG        1   // BMM350 magnetometer.
#define USE_LORA_TEMP  1   // TMP1075 on the LoRa board (input of the thermal throttle).
#define USE_LORA_VOLT  1   // DAC43401 PA bias (the real TX power lever).

// -----------------------------------------------------------------------------
//  Conditional library includes.
// -----------------------------------------------------------------------------
#if USE_SD || USE_FLASH
  #include <FS.h>
#endif
#if USE_SD
  #include <SD.h>
#endif
#if USE_FLASH
  #include <LittleFS.h>
#endif
#if USE_LORA
  #include <RadioLib.h>
  #include <mavlink/library/HorizonDialect/mavlink.h>
#endif
#if USE_IMU
  #include <ICM45686.h>
#endif
#if USE_BARO
  #include <Adafruit_BMP3XX.h>
#endif
#if USE_MAG
  #include "DFRobot_BMM350.h"
#endif

// -----------------------------------------------------------------------------
//  Pin map - from the breakout pin table in Ekstra_Info/PCB Dokumentasjon/
//  "Main board testing.docx". Do not change without updating the hardware doc.
// -----------------------------------------------------------------------------
constexpr int PIN_I2C_SDA   = 7;    // I2C data line for all sensors.
constexpr int PIN_I2C_SCL   = 15;   // I2C clock line for all sensors.

constexpr int PIN_SPI_MOSI  = 4;    // Shared SPI MOSI (SD + LoRa + IMU).
constexpr int PIN_SPI_SCK   = 5;    // Shared SPI clock.
constexpr int PIN_SPI_MISO  = 6;    // Shared SPI MISO.

constexpr int PIN_IMU_CS    = 16;   // ICM-45686 chip-select  (IF8).
constexpr int PIN_IMU_INT1  = 17;   // ICM-45686 INT1         (IF9).
constexpr int PIN_IMU_INT2  = 18;   // ICM-45686 INT2         (IF10).

constexpr int PIN_GEIGER_1  = 8;    // Geiger CRD1 pulse input (IF11).
constexpr int PIN_GEIGER_2  = 3;    // Geiger CRD2 pulse input (IF12).

constexpr int PIN_LORA_CS    = 9;   // SX1280 chip-select       (IF13).
constexpr int PIN_LORA_PWR   = 10;  // SX1280 power-enable      (IF14).
constexpr int PIN_LORA_CXT   = 11;  // Antenna switch CXT       (IF15). "DIO1" on the pin table,
constexpr int PIN_LORA_CRX   = 12;  // Antenna switch CrX       (IF16). "DIO2" there; neither is
                                    // an SX1280 DIO (TxDone is polled over SPI instead).
constexpr int PIN_LORA_BUSY  = 13;  // SX1280 BUSY              (IF17).
constexpr int PIN_LORA_RST   = 14;  // SX1280 RESET             (IF18).

constexpr int PIN_SD_CS      = 21;  // microSD chip-select.

// -----------------------------------------------------------------------------
//  I2C device addresses.
// -----------------------------------------------------------------------------
constexpr uint8_t I2C_ADDR_BMM350   = 0x14;  // Magnetometer.
constexpr uint8_t I2C_ADDR_BMP388   = 0x76;  // Barometer (SDO low).
// NOTE: the board document says DAC=0x72 / TMP=0x73; the chips answer at
// 0x48 and 0x49 (measured, see test_i2c_scan).  0x47 also ACKs on this board
// but writing it does nothing for the PA.
constexpr uint8_t I2C_ADDR_DAC43401 = 0x48;  // Sets LoRa TX power voltage.
constexpr uint8_t I2C_ADDR_TMP1075  = 0x49;  // Monitors LoRa module temperature.

// -----------------------------------------------------------------------------
//  LoRa radio (SX1280, 2.4 GHz).
// -----------------------------------------------------------------------------
constexpr float   LORA_FREQ_MHZ = 2400.0f;   // Carrier frequency in MHz.
constexpr float   LORA_BW_KHZ   = 812.5f;    // kHz. SX1280 accepts only 203.125 /
                                             // 406.25 / 812.5 / 1625.0.
constexpr uint8_t LORA_SF       = 11;        // Spreading factor.
constexpr uint8_t LORA_CR       = 8;         // Coding rate denominator (4/8, max FEC).
constexpr int8_t  LORA_TX_PWR   = 10;        // SX1280 output in dBm (max +13). The
                                             // external PA saturates above ~+6, so
                                             // +10 and +13 radiate the same
                                             // (measured 2026-07-06); the real
                                             // power lever is the DAC bias.
// This profile is fixed by the ground-station adapter (Avionics-main,
// Ratatoskr-GS-adapter-firmware config.h: 2400 MHz / 812.5 kHz / SF11 / CR 4/8 /
// sync 0x12 / CRC on), which cannot be changed. A narrower bandwidth would buy
// ~4 dB but needs a TCXO (DS Table 6-3: +/-21 ppm total at 200 kHz).
// See docs/LoRa_RF_notes.md.

// -----------------------------------------------------------------------------
//  MAVLink identity for outgoing frames.
// -----------------------------------------------------------------------------
constexpr uint8_t MAV_SYSID   = 42;          // Payload's MAVLink system ID.
constexpr uint8_t MAV_COMPID  = 191;         // Component ID for "scientific payload".

// -----------------------------------------------------------------------------
//  Timing.
// -----------------------------------------------------------------------------
constexpr uint32_t TX_CYCLE_INTERVAL_MS    = 1500; // Sensor cycle (log row + TX
                                                    // when the thermal throttle
                                                    // allows).  A full TX is 3
                                                    // packets of LORA_PACKET_LEN
                                                    // bytes = ~0.96 s on air at
                                                    // SF11/BW812.5, so 1.5 s = 64 %
                                                    // duty when cool.  The thermal
                                                    // levels below stretch the TX
                                                    // interval as the board warms,
                                                    // so this is the ceiling, not
                                                    // the sustained rate.  (The old
                                                    // one-message-per-packet scheme
                                                    // needed ~1.9 s on air per cycle.)
constexpr uint32_t SENSOR_POLL_INTERVAL_MS = 50;    // Refresh latest values 20 Hz.
constexpr uint32_t HEARTBEAT_INTERVAL_MS   = 5000;  // MAVLink heartbeat cadence (the GS
                                                    // shows "Xs ago" only after 10 s).

// -----------------------------------------------------------------------------
//  Launch gate (low-power wait).
//  On the pad the payload may sit powered for hours waiting on weather. While
//  USE_LAUNCH_GATE is set, App::begin() blocks in a low-power loop and does NOT
//  start logging / transmitting until it sees flight begin: the accelerometer
//  magnitude above LAUNCH_THRESHOLD_MG (a clear jump over the ~1000 mg it
//  reads at rest) for LAUNCH_CONFIRM_SAMPLES checks.  With 1 (the current
//  setting) the first check above the threshold trips the gate; higher values
//  debounce: a brief dip only decrements the count, so a noisy burn still
//  trips while a single knock does not.  Requires USE_IMU (no IMU = no gate).
//  Validated standalone by env:test_lowpower_wake.
//
//  While waiting, one beacon packet (HEARTBEAT with MAV_STATE_STANDBY + the
//  latest pressure frame) goes out every PAD_BEACON_INTERVAL_MS, stretched by
//  the thermal throttle like the flight TX, so the ground station knows the
//  payload is alive on the rail; between beacons the SX1280 sits in retention
//  sleep (~1 uA).
//
//  When the gate trips, "flown=1" is persisted to NVS BEFORE mission work
//  starts, and any later boot that sees the flag skips the gate entirely - a
//  brownout/watchdog reset mid-flight (coasting at ~0 g, chute at ~1 g) must
//  never strand the payload back in the pad wait. Clear with the `disarm`
//  (or `new`) serial command, set it with `fly`; preflight checklist: `status`
//  shows flown=0.
#ifndef USE_LAUNCH_GATE          // Overridable from the build: -DUSE_LAUNCH_GATE=0
#define USE_LAUNCH_GATE        1
#endif
// LAUNCH_USE_LIGHT_SLEEP: 1 = esp_light_sleep between the accelerometer checks
// for the lowest pad current. NOTE: light sleep tears down the USB-CDC link, so
// the serial monitor drops and USB uploads can be blocked while it runs - fine
// in flight (no host), awkward on the bench. Leave 0 for bench work; set 1 for
// the actual flight build.
#define LAUNCH_USE_LIGHT_SLEEP 0
constexpr int32_t  LAUNCH_THRESHOLD_MG      = 1500; // > resting ~1000 mg.
constexpr uint8_t  LAUNCH_CONFIRM_SAMPLES   = 1;    // Checks in a row above the threshold:
                                                    // 1 = first one trips, 3 = ~0.6 s sustained.
constexpr uint32_t LAUNCH_CHECK_INTERVAL_MS = 200;  // Accelerometer check period on the pad.
constexpr uint32_t PAD_BEACON_INTERVAL_MS   = 5000; // On-pad heartbeat cadence. The
                                                    // ground station only starts
                                                    // showing "Xs ago" after 10 s.

// -----------------------------------------------------------------------------
//  Thermal protection of the LoRa board (see Thermal.hpp).
//  SX1280 and SKY65383-11 PA: operating range -40..+85 degC, junction max
//  125 degC.  Thresholds are on the TMP1075 reading (board temperature next
//  to the module, which is cooler than the dies).  Each level: entry
//  temperature, TX interval as a multiple of TX_CYCLE_INTERVAL_MS in tenths
//  (10 = every cycle, 15 = every 1.5 cycles on average, 0 = no sensor frames
//  at all, only one heartbeat per LORA_THERMAL_HB_MS), whether to drop the
//  non-essential frames (gyro, mag, vertical speed), and the PA bias.  Rows
//  are still logged to SD / flash every cycle.
//
//  The levels act as a thermostat (a level is left LORA_THERMAL_HYST_cC below
//  its entry): cooler surroundings give the full rate, warmer ones a reduced
//  one.  Bench numbers: see the table and docs/LoRa_RF_notes.md.
// -----------------------------------------------------------------------------
#define USE_LORA_THERMAL 1
struct LoraThermalLevel {
  int16_t  on_cC;            // Enter this level at/above this TMP1075 reading.
  uint8_t  txIntervalTenths; // TX every (this/10) cycles; 0 = heartbeat only.
  bool     essentialOnly;    // Only heartbeat/accel/pressure/cosmic.
  uint16_t dacMv;            // DAC43401 PA bias while in this level.
};
constexpr LoraThermalLevel LORA_THERMAL_LEVELS[] = {
  { -32768, 10, false, 3300 },   // L0  cool: TX every cycle (64 % duty)
  {   6500, 15, false, 3300 },   // L1  65 C: every 1.5 cycles (43 %)
  {   6800, 20, false, 3300 },   // L2  68 C: every 2 cycles (32 %)
  {   7100, 30, true,  3300 },   // L3  71 C: every 3 cycles, essential frames only
  {   7400, 60, true,  2500 },   // L4  74 C: every 6 cycles, essential, PA bias down
  {   7800,  0, true,  2500 },   // L5  78 C: one heartbeat (+pressure) per minute
  // On the desk (2026-09-28, USB power, no airflow) the stack idles at ~55 C
  // and settled at 66-68 C while transmitting, i.e. it hovers around L1/L2.
};
constexpr uint8_t  LORA_THERMAL_LEVEL_COUNT = sizeof(LORA_THERMAL_LEVELS) / sizeof(LORA_THERMAL_LEVELS[0]);
constexpr int16_t  LORA_THERMAL_HYST_cC     = 200;    // Leave a level 2 C below its entry.
constexpr uint32_t LORA_THERMAL_POLL_MS     = 2000;   // TMP1075 read cadence.
constexpr uint32_t LORA_THERMAL_HB_MS       = 60000;  // Heartbeat cadence at the top level (L5), flight and pad.

// -----------------------------------------------------------------------------
//  Flight timeout.  The flight itself lasts ~10 min (9 km apogee, then
//  parachute); after FLIGHT_MAX_DURATION_S of flight time the payload stops
//  everything that matters: a last heartbeat (MAV_STATE_POWEROFF, phase
//  TOUCHDOWN in custom_mode) + row, radio to sleep, PA bias off, IMU/mag off,
//  no more logging.  Flight time counts from the launch gate (not from
//  power-on) and is saved to NVS once a minute, so a reset mid-flight cannot
//  restart the budget.  Serial still works; `disarm` / `new` clear the clock,
//  `fly` restarts it, `land` triggers the stop now.  0 = never stop.
// -----------------------------------------------------------------------------
constexpr uint32_t FLIGHT_MAX_DURATION_S    = 1800;
constexpr uint32_t FLIGHT_PERSIST_INTERVAL_MS = 60000;
constexpr uint32_t LANDED_BEACON_INTERVAL_MS = 0;    // >0: keep one heartbeat every N ms after stop.
constexpr uint32_t BURNOUT_CONFIRM_MS       = 2000;  // |a| below LAUNCH_THRESHOLD_MG this long = COASTING.

// -----------------------------------------------------------------------------
//  LoRa retry queue. If a packet fails to transmit it is pushed onto the queue
//  and re-tried on the next service tick until LORA_MAX_ATTEMPTS is reached,
//  after which the packet is dropped (and counted in totalDropped()).
// -----------------------------------------------------------------------------
constexpr size_t   LORA_FRAME_MAX     = 128; // Max bytes of one LoRa packet (retry queue slot).
// Every packet is padded to exactly this many bytes (see Packer.hpp for why the
// ground-station adapter needs constant-length packets).  64 keeps packets
// short enough to survive at the range limit (a 64-byte packet is ~116 symbols
// vs ~240 for 128, so it is decoded far more often when the SNR is marginal)
// and one flight cycle fits in three of them.  Must be <= LORA_FRAME_MAX.
constexpr size_t   LORA_PACKET_LEN    = 64;
// Pause between the packets of one cycle.  The adapter's blocking receive()
// has to get back to listening before the next preamble (12 symbols = 30 ms
// at SF11/BW812.5); the bench receiver caught all back-to-back packets, but
// the real adapter also reads its temperature sensor between packets.
constexpr uint32_t LORA_PACKET_GAP_MS = 40;
constexpr size_t   LORA_RETRY_QUEUE   = 16;  // Number of packets the queue may hold.
constexpr uint8_t  LORA_MAX_ATTEMPTS  = 3;   // Drop after this many failed sends.
constexpr uint16_t LORA_BACKOFF_MS    = 30;  // Pause after a failed retry before the next queue entry.

// -----------------------------------------------------------------------------
//  Storage paths.
// -----------------------------------------------------------------------------
constexpr const char* FLASH_LOG_DIR = "/logs";
constexpr size_t      FLASH_QUOTA_B = static_cast<size_t>(1.5 * 1024 * 1024);
constexpr const char* SD_LOG_DIR    = "/logs";

// -----------------------------------------------------------------------------
//  CSV header for both SD and flash row logs.
// -----------------------------------------------------------------------------
extern const char* CSV_HEADER;

// -----------------------------------------------------------------------------
//  Globals shared across translation units. Defined in main.cpp.
// -----------------------------------------------------------------------------
extern Preferences prefs;                          // Boot-persistent key/value store.
extern uint32_t    runId;                          // Increments every cold boot.
extern volatile uint32_t geigerCh1Pulses;          // ISR counter for CRD1.
extern volatile uint32_t geigerCh2Pulses;          // ISR counter for CRD2.

// -----------------------------------------------------------------------------
//  Geiger ISRs — defined in main.cpp.
// -----------------------------------------------------------------------------
void IRAM_ATTR geigerCh1Isr();
void IRAM_ATTR geigerCh2Isr();
