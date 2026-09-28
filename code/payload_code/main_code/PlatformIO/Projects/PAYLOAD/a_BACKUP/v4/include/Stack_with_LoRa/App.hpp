#pragma once

// =============================================================================
//  App.hpp
//  Top-level orchestrator. Runs the per-second telemetry cycle:
//    1. Drain the LoRa retry queue.
//    2. Refresh sensor readings.
//    3. Build one MAVLink frame per sensor and hand it to the radio.
//    4. Format a CSV row and append it to SD + flash.
//    5. Process serial commands (status / clear / dump / new / disarm).
// =============================================================================

#include "Config.hpp"
#include "Sensors.hpp"
#include "Storage.hpp"
#include "Thermal.hpp"
#if USE_LORA
  #include "LoraTx.hpp"
#endif

class App {
public:
  // Input  : (none)
  // What   : sets up serial, buses, NVS run id, all sensors, storage, radio.
  // Output : (none)
  void begin();

  // Input  : (none)
  // What   : called from loop(); polls sensors, runs the TX cycle, services
  //          the LoRa retry queue, processes any serial commands.
  // Output : (none)
  void tick();

private:
  // Input  : nowMs - current millis() timestamp.
  // What   : reads each sensor and sends one MAVLink frame per reading,
  //          one logical packet at a time, then logs a CSV row.
  // Output : (none)
  void runTxCycle_(uint32_t nowMs);

  // Input  : nowMs, accel, gyro, mag, baro, geig, lt, lv - latest readings.
  // What   : formats one CSV record carrying every sensor's most recent value.
  // Output : the formatted CSV line (no trailing newline).
  String buildCsvRow_(uint32_t nowMs,
                      const AccelReading& a,
                      const GyroReading&  g,
                      const MagReading&   m,
                      const BaroReading&  b,
                      const GeigerReading& gc,
                      const LoraTempReading& lt,
                      const LoraVoltReading& lv);

  // Input  : (none)
  // What   : reads any user input on Serial and runs `status`, `print`,
  //          `clear`, `new`, or `last` commands.
  // Output : (none)
  void processSerialCommands_();

  // Input  : (none)
  // What   : when USE_LAUNCH_GATE is set, blocks in a low-power loop until the
  //          accelerometer shows sustained launch acceleration (see Config.hpp).
  //          While waiting, sends one pad beacon every PAD_BEACON_INTERVAL_MS
  //          and keeps the radio in retention sleep in between. On trip,
  //          persists flown=1 to NVS before returning; if the flag is already
  //          set at entry (mid-flight reset) the gate is skipped entirely.
  //          Returns immediately if the gate is disabled or the IMU is absent,
  //          so a failed IMU can never strand the payload on the pad.
  // Output : (none)
  void waitForLaunch_();

  // Input  : (none)
  // What   : wakes the radio, transmits one HEARTBEAT (MAV_STATE_STANDBY) so
  //          the ground station knows we are alive on the rail, then puts the
  //          radio back into retention sleep. No-op when USE_LORA is disabled.
  // Output : (none)
  void sendPadBeacon_();

  // Helper that submits one prebuilt frame to the radio (when USE_LORA).
  void sendFrame_(const uint8_t* buf, size_t len);

  // Input  : nowMs
  // What   : every LORA_THERMAL_POLL_MS reads the TMP1075 and re-evaluates
  //          the throttle level; on a level change applies the PA bias of the
  //          new level and prints it.  Safe to call from the pad loop too.
  // Output : (none)
  void pollThermal_(uint32_t nowMs);

  // Input  : mavState - MAV_STATE for the heartbeat.
  // What   : transmits one HEARTBEAT (custom_mode = thermal level) followed
  //          by one FLIGHT_STATES frame with the current phase.
  // Output : (none)
  void sendHeartbeatPair_(uint8_t mavState);

  // Input  : phase - new ROCKET_STATE_TYPE.
  // What   : records the phase and sends one FLIGHT_STATES frame right away.
  // Output : (none)
  void setPhase_(uint8_t phase);

  // Input  : (none)
  // What   : seconds of flight so far (NVS-persisted part + this boot).
  // Output : seconds.
  uint32_t flightElapsedS_() const;

  // Input  : why - text for the serial log.
  // What   : the flight-timeout stop: final TOUCHDOWN + heartbeat, last row,
  //          radio to sleep, PA bias off, sensors off, logging stopped.
  // Output : (none)
  void enterLanded_(const char* why);

  // Input  : label - short tag like "ACCEL " / "BARO_P"; body - decoded
  //          values text; bytes/len - the raw frame about to be transmitted.
  // What   : prints "[TX] LABEL  BODY  (N bytes)" to serial, plus an
  //          optional hex dump if PRINT_TX_HEX is enabled.
  // Output : (none)
  void printTx_(const char* label, const char* body,
                const uint8_t* bytes, size_t len);

  Sensors  sensors_;
  Storage  storage_;
#if USE_LORA
  LoraTx   lora_;
#endif

  uint32_t lastTxCycleMs_      = 0;
  uint32_t lastSensorPollMs_   = 0;
  uint32_t lastHeartbeatMs_    = 0;
  String   serialBuf_;

  // Thermal throttle.
  LoraThermal thermal_;
  uint32_t lastThermalPollMs_  = 0;
  uint32_t cycleCount_         = 0;    // TX cycles since boot (for the divider).

  // Flight phase + timeout.
  uint8_t  phase_              = 0;    // ROCKET_STATE_TYPE.
  bool     flying_             = false;
  bool     landed_             = false;
  uint32_t flightStartMs_      = 0;
  uint32_t flightElapsedNvsS_  = 0;    // Flight seconds saved before this boot.
  uint32_t lastFlightPersistMs_= 0;
  uint32_t lastHighGMs_        = 0;    // Last time |a| was above the launch threshold.
  uint32_t lastLandedBeaconMs_ = 0;
};
