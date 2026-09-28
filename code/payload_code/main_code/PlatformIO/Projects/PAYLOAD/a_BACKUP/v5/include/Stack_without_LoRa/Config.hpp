#pragma once

// =============================================================================
//  Config.hpp  -  Stack_without_LoRa
//
//  Central configuration for the two payload stacks that carry NO SX1280
//  radio.  There is nothing to send, so this firmware has one job: put as
//  much sensor data on the microSD card as the hardware can produce.
//
//  Hardware (per stack):  main board (ESP32-S3, SD card)
//                         LoRa board (no SX1280 fitted; TMP1075 may be there)
//                         telemetry board (ICM-45686, BMP388/390, BMM350,
//                                          Geiger signal connectors)
//  Shared between stacks: CRD board (two Geiger tubes), powered by ONE
//                         stack, ground common to all.
//
//  Everything below is a compile-time constant so the flight build has no
//  hidden runtime configuration.
// =============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Preferences.h>
#include <FS.h>
#include <SD.h>
#include <LittleFS.h>
#include "esp_system.h"
#include "esp_timer.h"

#define FW_NAME     "Stack_without_LoRa"
#define FW_VERSION  "1.0.0"

// -----------------------------------------------------------------------------
//  Feature switches (1 = enabled, 0 = disabled).  Disable a peripheral that is
//  missing on the bench; the rest keeps logging.
// -----------------------------------------------------------------------------
#define USE_SERIAL      1   // Status text on USB CDC.  Never blocks (see App).
#define USE_SD          1   // microSD card, the primary log.
#define USE_FLASH       1   // LittleFS fallback log (only written when SD is down).
#define USE_GEIGER      1   // Geiger pulse inputs CRD1 + CRD2.
#define USE_IMU         1   // ICM-45686 accel + gyro via the chip FIFO.
#define USE_BARO        1   // BMP388/BMP390 in continuous (normal) mode.
#define USE_MAG         1   // BMM350 magnetometer.
#define USE_LORA_TEMP   1   // TMP1075 on the LoRa board (auto-detected; fine if absent).
#define USE_ESP_TEMP    1   // ESP32-S3 internal die temperature.

// -----------------------------------------------------------------------------
//  Conditional driver includes.
// -----------------------------------------------------------------------------
#if USE_IMU
  #include <ICM45686.h>
#endif
#if USE_BARO
  extern "C" {
    #include "bmp3.h"          // Bosch driver shipped inside Adafruit BMP3XX.
  }
#endif
#if USE_MAG
  #include "DFRobot_BMM350.h"
#endif

// -----------------------------------------------------------------------------
//  Pin map  -  identical to Stack_with_LoRa (include/Stack_with_LoRa/Config.hpp) and the
//  breakout table in "Main board testing.docx".
// -----------------------------------------------------------------------------
constexpr int PIN_I2C_SDA   = 7;
constexpr int PIN_I2C_SCL   = 15;

constexpr int PIN_SPI_MOSI  = 4;
constexpr int PIN_SPI_SCK   = 5;
constexpr int PIN_SPI_MISO  = 6;

constexpr int PIN_IMU_CS    = 16;   // IF8
constexpr int PIN_IMU_INT1  = 17;   // IF9  (unused: FIFO is polled)
constexpr int PIN_IMU_INT2  = 18;   // IF10

constexpr int PIN_GEIGER_1  = 8;    // IF11, CRD1
constexpr int PIN_GEIGER_2  = 3;    // IF12, CRD2

// LoRa-board lines.  No SX1280 on these stacks, but the pins still exist on
// the board-to-board connector.  They are parked in a safe state at boot:
// CS high, RST low, POW_EN low (keeps the LoRa board's PA rail off).
constexpr int PIN_LORA_CS   = 9;    // IF13
constexpr int PIN_LORA_PWR  = 10;   // IF14, LoRa board power enable
constexpr int PIN_LORA_RST  = 14;   // IF18

constexpr int PIN_SD_CS     = 21;

// -----------------------------------------------------------------------------
//  I2C addresses.
// -----------------------------------------------------------------------------
constexpr uint8_t I2C_ADDR_BMM350  = 0x14;
constexpr uint8_t I2C_ADDR_BMP3XX  = 0x76;  // SDO low.
constexpr uint8_t I2C_ADDR_TMP1075 = 0x49;  // Measured on the real board.
constexpr uint32_t I2C_CLOCK_HZ    = 400000;

// -----------------------------------------------------------------------------
//  IMU  (ICM-45686, SPI, hardware FIFO)
//
//  The FIFO is drained by polling from loop().  It holds ~1 s of data at
//  400 Hz, so an SD-card stall (FAT allocation, flush) never loses a sample.
//
//  IMU_FIFO_HIRES = 1 uses the 20-byte high-resolution packet: 19/20-bit
//  samples that are ALWAYS scaled to +/-32 g and +/-4000 dps (datasheet 5.0),
//  i.e. 0.061 mg and 0.0076 dps per LSB, 8x finer than the 16-bit registers.
//  Set to 0 to fall back to the 16-byte packet at the FSR below.
// -----------------------------------------------------------------------------
#define  IMU_FIFO_HIRES         1
constexpr uint16_t IMU_ODR_HZ       = 400;   // 100/200/400/800 supported by the driver.
constexpr uint16_t IMU_ACCEL_FSR_G  = 32;    // Register FSR.  Ignored by hires FIFO data.
constexpr uint16_t IMU_GYRO_FSR_DPS = 4000;
constexpr uint32_t IMU_SPI_HZ       = 8000000;
constexpr uint16_t IMU_FIFO_MAX_FRAMES_PER_DRAIN = 64;  // Bounds one drain (~1.3 KB SPI).
constexpr uint32_t IMU_POLL_INTERVAL_US = 2000;         // Check the FIFO every 2 ms.

// -----------------------------------------------------------------------------
//  Barometer (BMP388/390, I2C).  Run in NORMAL mode at a fixed ODR and read
//  the data registers when the chip flags data-ready -> never blocks the
//  loop (the Adafruit forced-mode read would block ~20 ms per sample).
//  8x pressure / 1x temperature / IIR coeff 3 / 50 Hz is Bosch's "drone"
//  profile and fits inside the 20 ms ODR period.
// -----------------------------------------------------------------------------
constexpr uint8_t  BARO_ODR         = BMP3_ODR_50_HZ;
constexpr uint8_t  BARO_PRESS_OS    = BMP3_OVERSAMPLING_8X;
constexpr uint8_t  BARO_TEMP_OS     = BMP3_NO_OVERSAMPLING;
constexpr uint8_t  BARO_IIR         = BMP3_IIR_FILTER_COEFF_3;
constexpr uint32_t BARO_POLL_INTERVAL_US = 2000;
constexpr float    BARO_SEA_LEVEL_HPA = 1013.25f;   // For the alt_m column only.

// -----------------------------------------------------------------------------
//  Magnetometer (BMM350, I2C).  100 Hz with 4-sample averaging is the
//  fastest combination the chip allows with averaging enabled.
// -----------------------------------------------------------------------------
constexpr uint8_t  MAG_PRESET       = BMM350_PRESETMODE_ENHANCED;   // avg 4
constexpr eBmm350DataRates_t MAG_RATE = BMM350_DATA_RATE_100HZ;
constexpr uint32_t MAG_INTERVAL_US  = 10000;   // = 1 / 100 Hz.

// -----------------------------------------------------------------------------
//  Geiger.  Each pulse is time-stamped in the ISR (microseconds) and written
//  as its own row, plus a 1 Hz count summary.  The CRD board output idles
//  HIGH (10 k pull-up on the CRD board) and pulses LOW; the internal pull-up
//  keeps the input quiet on a stack whose CRD connector is empty.
// -----------------------------------------------------------------------------
constexpr size_t   GEIGER_EVENT_RING = 256;    // Pulses buffered per channel.
constexpr uint32_t GEIGER_RATE_INTERVAL_MS = 1000;

// -----------------------------------------------------------------------------
//  Logging modes.
//
//  PAD     from boot until launch is detected: low rates so a multi-hour
//          weather hold does not bury the flight in a 300 MB file.
//  FLIGHT  full rate.  Entered when |a| stays above LAUNCH_THRESHOLD_G,
//          by the `flight` serial command, or immediately at boot if the
//          NVS "flown" flag is set (mid-flight brownout/reset).
//  POST    after FLIGHT_FULL_RATE_S seconds of FLIGHT: back to PAD rates
//          (recovery can take a long time).  A new launch-like acceleration
//          re-enters FLIGHT, so a false trigger on the pad only costs bytes.
//
//  The 1 s before the trigger is kept in RAM at full rate and written out
//  when FLIGHT starts, so the ignition transient is never lost.
// -----------------------------------------------------------------------------
constexpr uint16_t PAD_IMU_DIVIDER   = 20;    // 400 Hz / 20 = 20 Hz on the pad.
constexpr uint16_t PAD_BARO_DIVIDER  = 10;    // 50 Hz / 10 = 5 Hz.
constexpr uint16_t PAD_MAG_DIVIDER   = 20;    // 100 Hz / 20 = 5 Hz.

constexpr float    LAUNCH_THRESHOLD_G      = 2.0f;  // Rest reads ~1.0 g.
constexpr uint16_t LAUNCH_CONFIRM_SAMPLES  = 40;    // 100 ms at 400 Hz, up/down counted.
constexpr uint16_t PRETRIGGER_SAMPLES      = 400;   // 1 s at 400 Hz kept in RAM.
constexpr uint32_t FLIGHT_FULL_RATE_S      = 1800;  // 30 min, 0 = stay in FLIGHT forever.

// -----------------------------------------------------------------------------
//  1 Hz housekeeping rows (TEMP + STAT).
// -----------------------------------------------------------------------------
constexpr uint32_t HOUSEKEEPING_INTERVAL_MS = 1000;

// -----------------------------------------------------------------------------
//  Storage.
//
//  Rows are formatted into a RAM ring buffer and written to the SD file in
//  SD_WRITE_CHUNK_B pieces (or every SD_WRITE_INTERVAL_MS when there is
//  less), and fsync'ed every SD_FLUSH_INTERVAL_MS so a power cut loses at
//  most that much.  The file stays open for the whole run.
// -----------------------------------------------------------------------------
constexpr const char* SD_LOG_DIR         = "/logs";
constexpr uint32_t    SD_SPI_HZ          = 8000000;
constexpr size_t      LOG_RING_B         = 48 * 1024;   // RAM row buffer.
constexpr size_t      SD_WRITE_CHUNK_B   = 4096;        // >= stdio buffer so errors surface.
constexpr size_t      SD_WRITE_MAX_B     = 8192;        // Upper bound for one write call.
constexpr uint32_t    SD_WRITE_INTERVAL_MS  = 250;
constexpr uint32_t    SD_FLUSH_INTERVAL_MS  = 1000;
constexpr uint32_t    SD_REMOUNT_INTERVAL_MS = 10000;  // A failed mount attempt stalls the loop; keep them rare.

constexpr const char* FLASH_LOG_DIR      = "/logs";
constexpr size_t      FLASH_QUOTA_B      = static_cast<size_t>(1.3 * 1024 * 1024);
constexpr size_t      FLASH_RING_B       = 4096;
constexpr uint32_t    FLASH_WRITE_INTERVAL_MS = 2000;
// Rates the flash fallback accepts while the SD is down (divide the sensor rate).
constexpr uint16_t    FLASH_IMU_DIVIDER  = 40;    // 400 Hz / 40 = 10 Hz.
constexpr uint16_t    FLASH_BARO_DIVIDER = 25;    // 2 Hz.
constexpr uint16_t    FLASH_MAG_DIVIDER  = 50;    // 2 Hz.

// -----------------------------------------------------------------------------
//  NVS.  Own namespace so bench tests of this firmware never touch the
//  Stack_with_LoRa run counter / launch-gate flag.
// -----------------------------------------------------------------------------
constexpr const char* NVS_NAMESPACE = "payload_nl";

// -----------------------------------------------------------------------------
//  Globals defined in main.cpp.
// -----------------------------------------------------------------------------
extern Preferences prefs;
extern uint32_t    runId;

// Microsecond timestamp used in every log row (64-bit, never wraps).
inline uint64_t nowUs() { return static_cast<uint64_t>(esp_timer_get_time()); }
