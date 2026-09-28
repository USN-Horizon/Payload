#pragma once

// =============================================================================
//  App.hpp  -  Stack_without_LoRa
//
//  Top-level orchestrator.  loop() -> tick():
//    1. serial commands
//    2. drain the IMU FIFO  (every sample -> launch detector -> IMU row)
//    3. barometer / magnetometer when they have new data
//    4. Geiger pulse events
//    5. once a second: RATE, TEMP, STAT rows + mode timeout
//    6. Logger::service()  (SD writes / fsync / remount, flash fallback)
//
//  Modes: PAD (low rate) -> FLIGHT (full rate, on launch detect) -> POST
//  (low rate after FLIGHT_FULL_RATE_S).  See Config.hpp.
// =============================================================================

#include "Config.hpp"
#include "Sensors.hpp"
#include "Logger.hpp"
#include "Geiger.hpp"

enum class Mode : uint8_t { PAD, FLIGHT, POST };

class App {
public:
  void begin();
  void tick();

private:
  void drainImu_(uint64_t nowUs);
  void emitImuRow_(uint64_t t, const ImuSample& s);
  void pollBaro_(uint64_t nowUs);
  void pollMag_(uint64_t nowUs);
  void drainGeiger_();
  void housekeeping_(uint64_t nowUs, uint32_t nowMs);

  void enterFlight_(const char* why, bool persistFlag);
  void enterLowRate_(Mode m, const char* why);
  static const char* modeName_(Mode m);

  void processSerial_();
  void printStatus_();
  void printHelp_();

  Sensors sensors_;
  Logger  logger_;
  Geiger  geiger_;

  Mode     mode_ = Mode::PAD;
  uint32_t flightStartMs_ = 0;
  bool     flownFlag_ = false;

  // IMU timing / stats.
  uint64_t lastImuPollUs_ = 0;
  uint64_t lastImuT_      = 0;
  // Sample clock: a 2nd-order PLL locked to host time.  Each FIFO frame is
  // time-stamped as clock += period_est, and the clock/period are nudged
  // toward the (jittery, +/- one poll interval) host estimate.  Result:
  // evenly spaced timestamps that still track the ESP32 clock long-term.
  bool     imuClkInit_    = false;
  double   imuClkT_       = 0.0;    // us
  double   imuPeriodEst_  = 0.0;    // us per sample
  uint32_t imuFrames_     = 0;     // Frames in the current 1 s window.
  uint16_t fifoMax_       = 0;     // Fullest the FIFO got in the window.
  uint16_t padImuDiv_ = 0, padBaroDiv_ = 0, padMagDiv_ = 0;
  ImuSample lastImu_{};
  BaroSample lastBaro_{};
  MagSample  lastMag_{};

  // Launch detector: |a|^2 in raw LSB^2, up/down counted.
  int64_t  threshRaw2_ = 0;
  uint16_t aboveCount_ = 0;

  // Pre-trigger ring: the last PRETRIGGER_SAMPLES IMU samples at full rate.
  struct PreEntry { uint64_t t; ImuSample s; bool logged; };
  static PreEntry pre_[PRETRIGGER_SAMPLES];
  uint16_t preHead_  = 0;
  uint16_t preCount_ = 0;

  // Housekeeping.
  uint32_t lastHkMs_ = 0;
  uint32_t loops_    = 0;
  uint32_t bootMs_   = 0;

  String serialBuf_;
};
