#pragma once

// =============================================================================
//  Config.hpp
//  Central configuration for the "New" payload firmware.
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
#define PRINT_TX       1   // Print every MAVLink frame as it goes out the radio.
#define PRINT_TX_HEX   1   // Also dump raw bytes (hex) of every transmitted frame.
#define USE_LORA       1   // SX1280 transmitter + MAVLink frames.
#define USE_SD         1   // microSD card on PIN_SD_CS.
#define USE_FLASH      1   // LittleFS log mirror in onboard flash.
#define USE_GEIGER     1   // Geiger pulse counter (CRD1 + CRD2).
#define USE_IMU        1   // ICM-45686 accelerometer + gyroscope.
#define USE_BARO       1   // BMP388 barometer.
#define USE_MAG        1   // BMM350 magnetometer.
#define USE_LORA_TEMP  1   // TMP1075 temperature sensor next to LoRa module.
#define USE_LORA_VOLT  1   // DAC43401 voltage setting that drives LoRa TX power.

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
//  Pin map — values come straight from the breakout pin table in
//  "Main board testing.pdf". Do not change without updating the hardware doc.
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
constexpr int PIN_LORA_DIO1  = 11;  // SX1280 DIO1 / CXT        (IF15).
constexpr int PIN_LORA_DIO2  = 12;  // SX1280 DIO2 / CrX        (IF16).
constexpr int PIN_LORA_BUSY  = 13;  // SX1280 BUSY              (IF17).
constexpr int PIN_LORA_RST   = 14;  // SX1280 RESET             (IF18).

constexpr int PIN_SD_CS      = 21;  // microSD chip-select.

// -----------------------------------------------------------------------------
//  I2C device addresses.
// -----------------------------------------------------------------------------
constexpr uint8_t I2C_ADDR_BMM350   = 0x14;  // Magnetometer.
constexpr uint8_t I2C_ADDR_BMP388   = 0x76;  // Barometer (SDO low).
// NOTE: the breakout PDF says DAC=0x72 / TMP=0x73, but the actual board
// strapping puts these chips on the TI default range.  test_lora_temp
// confirmed TMP1075 at 0x49; DAC43401 has been observed at 0x48 (or 0x47
// when the ADDR pin is tied differently).
constexpr uint8_t I2C_ADDR_DAC43401 = 0x48;  // Sets LoRa TX power voltage.
constexpr uint8_t I2C_ADDR_TMP1075  = 0x49;  // Monitors LoRa module temperature.

// -----------------------------------------------------------------------------
//  LoRa radio (SX1280, 2.4 GHz).
// -----------------------------------------------------------------------------
constexpr float   LORA_FREQ_MHZ = 2400.0f;   // Carrier frequency in MHz.
constexpr float   LORA_BW_KHZ   = 812.5f;  // Bandwidth in kHz. SX1280 accepts
                                             // ONLY 203.125 / 406.25 / 812.5 /
                                             // 1625.0 — anything else makes
                                             // setBandwidth() fail and the radio
                                             // never starts. 203.125 buys
                                             // ~+6-12 dB sensitivity vs 812.5.
constexpr uint8_t LORA_SF       = 11;        // Spreading factor (long-range profile;
                                             // ~+6 dB link budget vs SF9, ~4x slower).
constexpr uint8_t LORA_CR       = 8;         // Coding rate denominator (8 = 4/8, max FEC).
constexpr int8_t  LORA_TX_PWR   = 10;       // Output power in dBm (max +13).
                                            // The external PA saturates: +6 and
                                            // +13 radiate the same (maxpower
                                            // test 2026-07-06), but +6 sits at
                                            // the PA knee — +13 keeps headroom
                                            // if the knee drifts with temp.
// NOTE: the ground-station receiver MUST match SF/CR/BW/freq/CRC/whitening
// exactly or it will not decode. Flight profile = SF11, CR 4/8, BW 203.125 kHz.

// -----------------------------------------------------------------------------
//  MAVLink identity for outgoing frames.
// -----------------------------------------------------------------------------
constexpr uint8_t MAV_SYSID   = 42;          // Payload's MAVLink system ID.
constexpr uint8_t MAV_COMPID  = 191;         // Component ID for "scientific payload".

// -----------------------------------------------------------------------------
//  Timing.
// -----------------------------------------------------------------------------
constexpr uint32_t TX_CYCLE_INTERVAL_MS    = 10000; // One TX cycle / 10 s.
                                                    // At SF11 / BW 203.125 each packet is
                                                    // ~0.8-1.1 s on air (4x slower than at
                                                    // BW 812.5), so a full ~8-packet cycle
                                                    // is ~7 s. 10 s keeps duty ~70% with
                                                    // margin for retries. If you need faster
                                                    // telemetry, drop packets from the cycle
                                                    // before shrinking this interval.
constexpr uint32_t SENSOR_POLL_INTERVAL_MS = 50;    // Refresh latest values 20 Hz.
constexpr uint32_t HEARTBEAT_INTERVAL_MS   = 5000;  // MAVLink heartbeat cadence.

// -----------------------------------------------------------------------------
//  Launch gate (low-power wait).
//  On the pad the payload may sit powered for hours waiting on weather. While
//  USE_LAUNCH_GATE is set, App::begin() blocks in a low-power loop and does NOT
//  start logging / transmitting until it sees flight begin: the accelerometer
//  magnitude staying above LAUNCH_THRESHOLD_MG (a clear jump over the ~1000 mg
//  it reads at rest) for LAUNCH_CONFIRM_SAMPLES samples. The debounce tolerates
//  brief dips (decrements, never hard-resets) so a sustained-but-noisy burn
//  still trips while a one-off knock on the pad cannot. Requires USE_IMU.
//  Validated standalone by env:test_lowpower_wake.
//
//  While waiting, a single MAVLink HEARTBEAT (MAV_STATE_STANDBY) goes out every
//  PAD_BEACON_INTERVAL_MS so the ground station knows the payload is alive on
//  the rail; between beacons the SX1280 sits in retention sleep (~1 uA).
//
//  When the gate trips, "flown=1" is persisted to NVS BEFORE mission work
//  starts, and any later boot that sees the flag skips the gate entirely - a
//  brownout/watchdog reset mid-flight (coasting at ~0 g, chute at ~1 g) must
//  never strand the payload back in the pad wait. Clear with the `disarm`
//  (or `new`) serial command; preflight checklist: `status` shows flown=0.
//  Validated standalone by env:test_pad_heartbeat.
#define USE_LAUNCH_GATE        1
// -----------------------------------------------------------------------------
// LAUNCH_USE_LIGHT_SLEEP
// Wait for launch before mission work.
// Set to 1 for the lowest pad current (esp_light_sleep between checks). NOTE:
// light sleep tears down the USB-CDC link, so the serial monitor drops and USB
// uploads can be blocked while it runs - fine in flight (no host), awkward on
// the bench. Leave 0 for bench work; set 1 for the actual flight build.
#define LAUNCH_USE_LIGHT_SLEEP 0
constexpr int32_t  LAUNCH_THRESHOLD_MG      = 1500; // > resting ~1000 mg.
constexpr uint8_t  LAUNCH_CONFIRM_SAMPLES   = 1;    // Sustained, not a tap.
constexpr uint32_t LAUNCH_CHECK_INTERVAL_MS = 200;  // 5 Hz duty cycle.
constexpr uint32_t PAD_BEACON_INTERVAL_MS   = 10000; // On-pad heartbeat cadence.

// -----------------------------------------------------------------------------
//  LoRa retry queue. If a frame fails to transmit it is pushed onto the queue
//  and re-tried on the next service tick until LORA_MAX_ATTEMPTS is reached,
//  after which the frame is dropped (and counted in totalDropped()).
// -----------------------------------------------------------------------------
constexpr size_t   LORA_FRAME_MAX     = 64;  // Max bytes any one MAVLink frame may take.
constexpr size_t   LORA_RETRY_QUEUE   = 16;  // Number of frames the queue may hold.
constexpr uint8_t  LORA_MAX_ATTEMPTS  = 3;   // Drop after this many failed sends.
constexpr uint16_t LORA_BACKOFF_MS    = 30;  // Small delay between back-to-back sends.

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
