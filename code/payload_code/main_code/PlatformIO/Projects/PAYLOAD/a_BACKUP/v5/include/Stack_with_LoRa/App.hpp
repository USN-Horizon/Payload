#pragma once

// =============================================================================
//  App.hpp
//  Top-level orchestrator.  begin(): buses, NVS, storage, sensors, radio,
//  then the low-power launch gate.  tick(), every loop:
//    1. serial commands, LoRa retry queue, thermal throttle poll
//    2. sensor poll (20 Hz) + burnout detection + flight clock / 30 min stop
//    3. every TX_CYCLE_INTERVAL_MS: MAVLink frames -> fixed-length LoRa
//       packets (as the thermal level allows) and one CSV row to SD + flash
// =============================================================================

#include "Config.hpp"
#include "Sensors.hpp"
#include "Storage.hpp"
#include "Thermal.hpp"
#include "Packer.hpp"
#if USE_LORA
  #include "LoraTx.hpp"
#endif

class App {
public:
  // Input  : (none)
  // What   : sets up serial, buses, NVS (run id, flown flag, flight clock),
  //          sensors, storage, radio, then waits in the launch gate.
  // Output : (none)
  void begin();

  // Input  : (none)
  // What   : called from loop(); polls sensors, runs the TX cycle, services
  //          the LoRa retry queue, processes any serial commands.
  // Output : (none)
  void tick();

private:
  // Input  : nowMs - current millis() timestamp.
  // What   : builds the MAVLink frames the ground station decodes, packs
  //          them into LORA_PACKET_LEN-byte packets (subject to the thermal
  //          throttle) and logs one CSV row.
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
  // What   : reads any user input on Serial: status | print | clear | new |
  //          last | disarm | fly | land | thermal N|auto | volt N | cw on|off.
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
  // What   : wakes the radio, sends one beacon packet (HEARTBEAT with
  //          MAV_STATE_STANDBY + the latest pressure frame) so the ground
  //          station knows we are alive on the rail, then puts the radio back
  //          into retention sleep. No-op when USE_LORA is disabled.
  // Output : (none)
  void sendPadBeacon_();

  // Hands one finished packet to the radio (when USE_LORA); emitPacket_ calls it.
  void sendFrame_(const uint8_t* buf, size_t len);

  // Input  : nowMs
  // What   : every LORA_THERMAL_POLL_MS reads the TMP1075 and re-evaluates
  //          the throttle level; on a level change applies the PA bias of the
  //          new level and prints it.  Safe to call from the pad loop too.
  // Output : (none)
  void pollThermal_(uint32_t nowMs);

  // Input  : mavState - MAV_STATE for the heartbeat.
  // What   : queues one HEARTBEAT (custom_mode = thermal level | phase << 8)
  //          plus the latest pressure frame as filler, and sends the packet.
  //          Used for the pad beacon, the top-level (L5) heartbeat and the
  //          final one.
  // Output : (none)
  void sendHeartbeatPacket_(uint8_t mavState);

  // Input  : phase - new ROCKET_STATE_TYPE.
  // What   : records the phase (it travels in HEARTBEAT.custom_mode bits
  //          8..15; FLIGHT_STATES is not decoded by the ground station, so it
  //          is no longer sent on the air).
  // Output : (none)
  void setPhase_(uint8_t phase);

  // Input  : label / body for the serial print, buf / n - a MAVLink frame.
  // What   : prints the [TX] line and hands the frame to the packer.
  // Output : (none)
  void queueFrame_(const char* label, const char* body, const uint8_t* buf, size_t n);

  // Packer callback: one finished packet -> radio.
  static void emitPacket_(void* ctx, const uint8_t* packet, size_t len);

  // Input  : b - latest baro reading, nowMs.
  // What   : barometric vertical speed over the last cycle (m/s, + up).
  // Output : (none)
  void updateVerticalSpeed_(const BaroReading& b, uint32_t nowMs);

  // Input  : (none)
  // What   : seconds of flight so far (NVS-persisted part + this boot).
  // Output : seconds.
  uint32_t flightElapsedS_() const;

  // Input  : why - text for the serial log.
  // What   : the flight-timeout stop: final TOUCHDOWN + heartbeat, last row,
  //          radio to sleep, PA bias off, sensors off, logging stopped.
  // Output : (none)
  void enterLanded_(const char* why);

  // Input  : label - short tag like "ACCEL " / "BARO  "; body - decoded
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

  // Packet builder (several MAVLink frames per fixed-length LoRa packet).
  MavlinkPacker<LORA_FRAME_MAX> packer_;
  uint32_t packetsThisCycle_   = 0;

  // Barometric vertical speed for VFR_HUD.
  float    vspeed_mps_         = 0.0f;
  float    lastAlt_m_          = 0.0f;
  uint32_t lastAltMs_          = 0;
  bool     haveLastAlt_        = false;

  // Thermal throttle.
  LoraThermal thermal_;
  uint32_t lastThermalPollMs_  = 0;
  uint32_t cycleCount_         = 0;    // Sensor cycles since boot.
  uint32_t lastTxMs_           = 0;    // Last cycle that put sensor frames on the air.

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
