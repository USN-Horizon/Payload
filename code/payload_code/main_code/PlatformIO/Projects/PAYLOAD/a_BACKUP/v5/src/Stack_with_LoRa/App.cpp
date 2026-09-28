#include "Stack_with_LoRa/App.hpp"
#include <math.h>
#if USE_LORA
  #include "Stack_with_LoRa/MavlinkFrames.hpp"
#endif
#if USE_LAUNCH_GATE && LAUNCH_USE_LIGHT_SLEEP
  #include "esp_sleep.h"
#endif

// =============================================================================
//  App.cpp
//  Top-level orchestrator. setup() -> begin(), loop() -> tick().
//  See App.hpp for the per-cycle outline.
// =============================================================================

// =============================================================================
//  begin
// =============================================================================
void App::begin() {
#if USE_SERIAL
  Serial.begin(115200);
  for (uint32_t t = millis(); millis() - t < 200;) { /* small settling */ }
  Serial.println();
  Serial.println("==== Stack_with_LoRa booting ====");
#endif

  packer_.configure(LORA_PACKET_LEN, &App::emitPacket_, this);

  // Bring up shared buses once for everyone.
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

  // Drive every SPI chip-select high BEFORE first transfer.  The IMU has an
  // internal pull-down on CS, so leaving it floating would corrupt SD init.
#if USE_IMU
  pinMode(PIN_IMU_CS, OUTPUT); digitalWrite(PIN_IMU_CS, HIGH);
#endif
#if USE_LORA
  pinMode(PIN_LORA_CS, OUTPUT); digitalWrite(PIN_LORA_CS, HIGH);
  // Hold the SX1280 in reset until lora_.begin() runs. With RST floating
  // the chip half-powers itself, leaks onto MISO, and SD.begin() fails
  // ~75% of boots (measured 2026-07: 0/8 mounts with RST floating, 8/8
  // with RST held LOW). RadioLib's begin() releases RST and runs its own
  // reset sequence later.
  pinMode(PIN_LORA_RST, OUTPUT); digitalWrite(PIN_LORA_RST, LOW);
#endif
#if USE_SD
  pinMode(PIN_SD_CS, OUTPUT);  digitalWrite(PIN_SD_CS, HIGH);
#endif

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

  // Load / increment the cold-boot run id from NVS.
  prefs.begin("payload", false);
  runId = prefs.getUInt("run", 0) + 1;
  prefs.putUInt("run", runId);
  prefs.end();
#if USE_SERIAL
  Serial.printf("[APP] runId=%lu\n", static_cast<unsigned long>(runId));
#endif

  // Geiger ISRs.
#if USE_GEIGER
  pinMode(PIN_GEIGER_1, INPUT);
  pinMode(PIN_GEIGER_2, INPUT);
  attachInterrupt(PIN_GEIGER_1, geigerCh1Isr, RISING);
  attachInterrupt(PIN_GEIGER_2, geigerCh2Isr, RISING);
#endif

  // Mount SD first, while the SPI bus has not been touched by anything else.
  // SD.begin() can leave the bus in a state the InvenSense IMU driver
  // dislikes; Sensors::begin() retries the IMU up to 5x to cover that. If the
  // IMU ever fails all retries here, try the old order (sensors before SD).
  storage_.begin(runId);

  sensors_.begin();

#if USE_LORA
  bool ok = lora_.begin();
  if (!ok) {
#if USE_SERIAL
    Serial.println("[APP] LoRa init failed (continuing without radio)");
#endif
  }
#endif

  // Sit in low power on the pad until flight begins. Everything below this is
  // mission work (logging + TX); we don't want it running for hours of weather
  // hold. No-op when USE_LAUNCH_GATE is disabled or the IMU did not come up.
  waitForLaunch_();

  uint32_t now = millis();
  lastTxCycleMs_    = now;
  lastSensorPollMs_ = now;
  lastHeartbeatMs_  = now;
  lastThermalPollMs_ = now;
  sensors_.poll(now);

  // Flight clock.  waitForLaunch_() returned either because the gate tripped
  // just now or because flown=1 was already set (mid-flight reset, or the
  // gate is disabled): in both cases we are "flying" from here on.
  prefs.begin("payload", true);
  flightElapsedNvsS_ = prefs.getUInt("fl_s", 0);
  prefs.end();
  flying_ = true;
  flightStartMs_ = now;
  lastFlightPersistMs_ = now;
  lastHighGMs_ = now;
  if (phase_ == ROCKET_STATE_IDLE) phase_ = ROCKET_STATE_THRUSTING;
#if USE_SERIAL
  Serial.printf("[APP] flight clock: %lu s already used of %lu s\n",
                static_cast<unsigned long>(flightElapsedNvsS_),
                static_cast<unsigned long>(FLIGHT_MAX_DURATION_S));
#endif
  if (FLIGHT_MAX_DURATION_S > 0 && flightElapsedNvsS_ >= FLIGHT_MAX_DURATION_S) {
    enterLanded_("flight time already used up at boot");
  }

#if USE_SERIAL
  Serial.println("[APP] ready. Commands: status | print | clear | new | last | disarm | fly | land | thermal N|auto");
#endif
}

// =============================================================================
//  waitForLaunch_  -  low-power wait for the start of flight.
//  Mirrors env:test_lowpower_wake, but reuses Sensors::poll()/lastAccel() so
//  there is no second copy of the IMU read path.
// =============================================================================
void App::waitForLaunch_() {
#if USE_LAUNCH_GATE
  // A reset mid-flight (brownout, watchdog) must not strand the payload back
  // in the pad gate while it is coasting at ~0 g or hanging under the chute
  // at ~1 g - neither would ever re-trip the accel threshold. The gate writes
  // flown=1 the moment it trips; if the flag is already set, this boot is a
  // mid-flight restart and the gate is skipped. `disarm` / `new` clears it.
  prefs.begin("payload", true);
  bool flown = prefs.getUChar("flown", 0) != 0;
  prefs.end();
  if (flown) {
#if USE_SERIAL
    Serial.println("[GATE] flown=1 (mid-flight reset?) - skipping launch gate");
    Serial.println("[GATE] use `disarm` to re-arm the gate for the next boot");
#endif
    return;
  }

  // If the accelerometer never initialised we cannot detect launch - skip the
  // gate rather than block the payload on the pad forever.
  if (!sensors_.imuOk()) {
#if USE_SERIAL
    Serial.println("[GATE] IMU not available - skipping launch gate");
#endif
    return;
  }

  const int32_t threshSq =
      static_cast<int32_t>(LAUNCH_THRESHOLD_MG) * LAUNCH_THRESHOLD_MG;

#if USE_SERIAL
  Serial.printf("[GATE] LOW-POWER WAIT (light_sleep=%d): need |a| > %ld mg "
                "for %u samples, beacon every %lu s\n",
                LAUNCH_USE_LIGHT_SLEEP, static_cast<long>(LAUNCH_THRESHOLD_MG),
                LAUNCH_CONFIRM_SAMPLES,
                static_cast<unsigned long>(PAD_BEACON_INTERVAL_MS / 1000));
#endif

  // First beacon right away so the ground station locks on without waiting a
  // full interval; sendPadBeacon_() leaves the radio in retention sleep.
  sendPadBeacon_();

  uint8_t  aboveCount = 0;
  uint32_t checks     = 0;
  uint32_t lastBeaconMs = millis();
  for (;;) {
    // Idle the core for one interval while the accel keeps sampling.
#if LAUNCH_USE_LIGHT_SLEEP
    esp_sleep_enable_timer_wakeup(
        static_cast<uint64_t>(LAUNCH_CHECK_INTERVAL_MS) * 1000ULL);
    esp_light_sleep_start();
#else
    delay(LAUNCH_CHECK_INTERVAL_MS);
#endif

    // The beacon is timed on millis(), not on successful accel reads, so a
    // sick IMU cannot silence it.  The radio heats on the pad too: the
    // cadence follows the thermal level (stretched like the TX interval, or
    // once per LORA_THERMAL_HB_MS at the top level).
    uint32_t now = millis();
    pollThermal_(now);
    uint32_t beaconMs = thermal_.txAllowed()
        ? thermal_.txIntervalMs(PAD_BEACON_INTERVAL_MS)
        : LORA_THERMAL_HB_MS;
    if (static_cast<uint32_t>(now - lastBeaconMs) >= beaconMs) {
      sendPadBeacon_();
      lastBeaconMs = now;
    }

    sensors_.poll(now);
    AccelReading a = sensors_.lastAccel();
    if (!a.ok) continue;          // A failed read must not strand us here.
    checks++;

    // Compare squared magnitude to avoid a sqrt (and any float) in the loop.
    int32_t magSq = static_cast<int32_t>(a.x_mg) * a.x_mg
                  + static_cast<int32_t>(a.y_mg) * a.y_mg
                  + static_cast<int32_t>(a.z_mg) * a.z_mg;

    if (magSq > threshSq) {
      if (++aboveCount >= LAUNCH_CONFIRM_SAMPLES) break;
    } else if (aboveCount > 0) {
      // Tolerate brief dips so a noisy-but-sustained burn still trips while a
      // lone knock (+1 then -1) never accumulates.
      aboveCount--;
    }
#if USE_SERIAL
    // Occasional heartbeat so we can see it is alive without spamming.
    if (checks % 25 == 0) {
      Serial.printf("[GATE] waiting... (above %u/%u)\n",
                    aboveCount, LAUNCH_CONFIRM_SAMPLES);
    }
#endif
  }

  // Persist BEFORE any mission work: even a brownout on the very next line
  // must reboot on the flight side of the gate.
  prefs.begin("payload", false);
  prefs.putUChar("flown", 1);
  prefs.putUInt("fl_s", 0);          // Fresh flight clock.
  prefs.end();
  phase_ = ROCKET_STATE_THRUSTING;

#if USE_LORA
  // The pad beacons left the radio in retention sleep; mission TX needs it
  // back in standby.
  lora_.wake();
#endif

#if USE_SERIAL
  Serial.println("[GATE] FLIGHT DETECTED - starting mission (flown=1 saved)");
#endif
#endif // USE_LAUNCH_GATE
}

// =============================================================================
//  sendPadBeacon_  -  one on-pad beacon packet (heartbeat + pressure), radio
//  asleep before and after.
// =============================================================================
void App::sendPadBeacon_() {
#if USE_LORA
  lora_.wake();
  sendHeartbeatPacket_(MAV_STATE_STANDBY);
  lora_.sleep();
#endif
}

// =============================================================================
//  sendHeartbeatPacket_ / setPhase_ / queueFrame_ / emitPacket_
// =============================================================================
void App::sendHeartbeatPacket_(uint8_t mavState) {
#if USE_LORA
  uint8_t buf[LORA_FRAME_MAX];
  char body[48];
  const uint32_t customMode = static_cast<uint32_t>(thermal_.level()) |
                              (static_cast<uint32_t>(phase_) << 8);
  size_t n = MavlinkFrames::buildHeartbeat(buf, mavState, customMode);
  snprintf(body, sizeof(body), "state=%u thermal=L%u phase=%u", mavState, thermal_.level(), phase_);
  queueFrame_("HEARTBEAT", body, buf, n);
  // The packet is padded to LORA_PACKET_LEN anyway: put the latest pressure
  // (+ temperature) in the free bytes so the ground station sees the baro
  // even while we only beacon.
  BaroReading b = sensors_.lastBaro();
  if (b.ok) {
    n = MavlinkFrames::buildBaroPressure(buf, b);
    snprintf(body, sizeof(body), "press=%.2f hPa temp=%d cC", b.pressure_hPa, b.temperature_cC);
    queueFrame_("BARO  ", body, buf, n);
  }
  packer_.flush();
#else
  (void)mavState;
#endif
}

void App::setPhase_(uint8_t phase) {
  if (phase == phase_) return;
  phase_ = phase;
#if USE_SERIAL
  Serial.printf("[APP] flight phase -> %u\n", phase);
#endif
}

void App::queueFrame_(const char* label, const char* body, const uint8_t* buf, size_t n) {
  printTx_(label, body, buf, n);
  if (!packer_.add(buf, n)) {
#if USE_SERIAL
    Serial.printf("[LoRa] frame %s (%u B) does not fit a %u-byte packet - dropped\n",
                  label, static_cast<unsigned>(n), static_cast<unsigned>(LORA_PACKET_LEN));
#endif
  }
}

void App::emitPacket_(void* ctx, const uint8_t* packet, size_t len) {
  App* self = static_cast<App*>(ctx);
  if (self->packetsThisCycle_ > 0) delay(LORA_PACKET_GAP_MS);   // Let the receiver re-arm.
  self->packetsThisCycle_++;
#if USE_SERIAL && PRINT_TX
  Serial.printf("[TX] PACKET %u bytes\n", static_cast<unsigned>(len));
#if PRINT_TX_HEX
  // Whole packet as one hex line: tools/hex_to_capture.py turns these into
  // an adapter capture for tools/gs_decode_sim.cpp.
  Serial.print("[TX] PACKET bytes:");
  for (size_t i = 0; i < len; ++i) Serial.printf(" %02X", packet[i]);
  Serial.println();
#endif
#endif
  self->sendFrame_(packet, len);
}

void App::updateVerticalSpeed_(const BaroReading& b, uint32_t nowMs) {
  if (!b.ok) return;
  // Same formula the ground station uses for its altitude plot.
  float alt = 44330.0f * (1.0f - powf(b.pressure_hPa / 1013.25f, 0.1903f));
  if (haveLastAlt_) {
    float dt = static_cast<float>(nowMs - lastAltMs_) / 1000.0f;
    if (dt > 0.2f) vspeed_mps_ = (alt - lastAlt_m_) / dt;
  }
  lastAlt_m_  = alt;
  lastAltMs_  = nowMs;
  haveLastAlt_ = true;
}

// =============================================================================
//  pollThermal_  -  TMP1075 -> throttle level -> PA bias.
// =============================================================================
void App::pollThermal_(uint32_t nowMs) {
#if USE_LORA_THERMAL
  if (static_cast<uint32_t>(nowMs - lastThermalPollMs_) < LORA_THERMAL_POLL_MS) return;
  lastThermalPollMs_ = nowMs;
  LoraTempReading lt = sensors_.readLoraTemp(nowMs);
  if (!lt.ok) return;                       // No sensor: stay at the current level.
  uint16_t oldDac = thermal_.dacMv();
  if (thermal_.update(lt.t_cC)) {
#if USE_SERIAL
    Serial.printf("[THERM] LoRa %d.%02d C -> level L%u (TX every %u.%u cycles%s, PA %u mV)\n",
                  lt.t_cC / 100, abs(lt.t_cC % 100), thermal_.level(),
                  thermal_.txIntervalTenths() / 10, thermal_.txIntervalTenths() % 10,
                  thermal_.essentialOnly() ? ", essential only" : "", thermal_.dacMv());
#endif
    if (thermal_.dacMv() != oldDac) sensors_.setLoraVolt(thermal_.dacMv());
  }
#else
  (void)nowMs;
#endif
}

// =============================================================================
//  flightElapsedS_ / enterLanded_  -  the 30 minute stop.
// =============================================================================
uint32_t App::flightElapsedS_() const {
  if (!flying_) return flightElapsedNvsS_;
  return flightElapsedNvsS_ + static_cast<uint32_t>(millis() - flightStartMs_) / 1000u;
}

void App::enterLanded_(const char* why) {
  if (landed_) return;
  uint32_t elapsed = flightElapsedS_();
#if USE_SERIAL
  Serial.printf("[APP] FLIGHT OVER (%s) after %lu s - stopping TX, logging, sensors\n",
                why, static_cast<unsigned long>(elapsed));
#endif
  // Remember that the budget is used up, so a reboot on the ground stays quiet.
  prefs.begin("payload", false);
  prefs.putUInt("fl_s", elapsed);
  prefs.end();

  // Last words: phase + heartbeat, and one final row.
  setPhase_(ROCKET_STATE_TOUCHDOWN);
  uint32_t now = millis();
  runTxCycle_(now);
#if USE_LORA
  sendHeartbeatPacket_(MAV_STATE_POWEROFF);
  lora_.service(/*maxAttemptsThisTick=*/LORA_RETRY_QUEUE);
  lora_.sleep();
#endif
  sensors_.setLoraVolt(0);        // PA unbiased.
  sensors_.powerDown();
  landed_ = true;
  flying_ = false;                // Freeze the flight clock at `elapsed`.
  flightElapsedNvsS_ = elapsed;
  lastLandedBeaconMs_ = now;
}

// =============================================================================
//  tick
// =============================================================================
void App::tick() {
  uint32_t now = millis();

  processSerialCommands_();

  if (landed_) {
    // Everything relevant is stopped.  Optional slow beacon for recovery.
#if USE_LORA
    if (LANDED_BEACON_INTERVAL_MS > 0 &&
        static_cast<uint32_t>(now - lastLandedBeaconMs_) >= LANDED_BEACON_INTERVAL_MS) {
      lora_.wake();
      sendHeartbeatPacket_(MAV_STATE_POWEROFF);
      lora_.sleep();
      lastLandedBeaconMs_ = now;
    }
#endif
    delay(10);
    return;
  }

  // Retry a few queued packets every loop so the queue does not grow.
#if USE_LORA
  lora_.service(/*maxAttemptsThisTick=*/2);
#endif

  pollThermal_(now);

  // Refresh accel/gyro/mag/baro at SENSOR_POLL_INTERVAL_MS cadence.
  if (static_cast<uint32_t>(now - lastSensorPollMs_) >= SENSOR_POLL_INTERVAL_MS) {
    sensors_.poll(now);
    lastSensorPollMs_ = now;

    // THRUSTING -> COASTING once |a| has stayed below the launch threshold
    // for BURNOUT_CONFIRM_MS (uses the same squared-magnitude test as the gate).
    AccelReading a = sensors_.lastAccel();
    if (a.ok) {
      int32_t magSq = static_cast<int32_t>(a.x_mg) * a.x_mg
                    + static_cast<int32_t>(a.y_mg) * a.y_mg
                    + static_cast<int32_t>(a.z_mg) * a.z_mg;
      const int32_t threshSq = static_cast<int32_t>(LAUNCH_THRESHOLD_MG) * LAUNCH_THRESHOLD_MG;
      if (magSq > threshSq) lastHighGMs_ = now;
      else if (phase_ == ROCKET_STATE_THRUSTING &&
               static_cast<uint32_t>(now - lastHighGMs_) >= BURNOUT_CONFIRM_MS) {
        setPhase_(ROCKET_STATE_COASTING);
      }
    }
  }

  // Flight clock: persist once a minute, stop at the budget.
  if (flying_) {
    if (static_cast<uint32_t>(now - lastFlightPersistMs_) >= FLIGHT_PERSIST_INTERVAL_MS) {
      lastFlightPersistMs_ = now;
      prefs.begin("payload", false);
      prefs.putUInt("fl_s", flightElapsedS_());
      prefs.end();
    }
    if (FLIGHT_MAX_DURATION_S > 0 && flightElapsedS_() >= FLIGHT_MAX_DURATION_S) {
      enterLanded_("flight time budget reached");
      return;
    }
  }

  // Run a full TX + log cycle every TX_CYCLE_INTERVAL_MS.
  if (static_cast<uint32_t>(now - lastTxCycleMs_) >= TX_CYCLE_INTERVAL_MS) {
    runTxCycle_(now);
    lastTxCycleMs_ = now;
  }
}

// =============================================================================
//  runTxCycle_  -  the heart of the firmware.
//  One MAVLink frame per value the ground station shows, packed into
//  LORA_PACKET_LEN-byte LoRa packets (as the thermal level allows), plus one
//  log row.
// =============================================================================
void App::runTxCycle_(uint32_t nowMs) {
  AccelReading    a  = sensors_.lastAccel();
  GyroReading     g  = sensors_.lastGyro();
  MagReading      m  = sensors_.lastMag();
  BaroReading     b  = sensors_.lastBaro();
  GeigerReading   gc = sensors_.takeGeiger(nowMs);
  LoraTempReading lt = sensors_.readLoraTemp(nowMs);
  LoraVoltReading lv = sensors_.readLoraVolt(nowMs);

  cycleCount_++;
  packetsThisCycle_ = 0;
  updateVerticalSpeed_(b, nowMs);

#if USE_LORA
  uint8_t buf[LORA_FRAME_MAX];
  size_t  n;
  char    body[80];

  // Thermal throttle: the sensor frames go out when the level's TX interval
  // (a multiple of the cycle, e.g. 1.5x) has elapsed since the last TX; at
  // the higher levels only the essential ones, and at the top level nothing
  // but a heartbeat every LORA_THERMAL_HB_MS.  Logging below is never
  // throttled.
  const bool     txAllowed   = thermal_.txAllowed();
  const uint32_t txInterval  = thermal_.txIntervalMs(TX_CYCLE_INTERVAL_MS);
  const bool     txThisCycle = txAllowed &&
                               static_cast<uint32_t>(nowMs - lastTxMs_) + TX_CYCLE_INTERVAL_MS / 2 >= txInterval;
  const bool     essential   = thermal_.essentialOnly();
  const uint32_t hbInterval  = txAllowed ? HEARTBEAT_INTERVAL_MS : LORA_THERMAL_HB_MS;
  const bool     hbDue       = static_cast<uint32_t>(nowMs - lastHeartbeatMs_) >= hbInterval;
  const uint8_t  mavState    = !txAllowed               ? MAV_STATE_EMERGENCY
                             : thermal_.essentialOnly() ? MAV_STATE_CRITICAL
                             : MAV_STATE_ACTIVE;

  if (txThisCycle) {
    // What the ground station (Nidhoggr packetparser.cpp) actually decodes,
    // and nothing else:
    //   SCALED_IMU  -> acc only     SCALED_IMU2 -> gyro only
    //   SCALED_IMU3 -> mag only     SCALED_PRESSURE -> pressure + temperature
    //   COSMIC_RADIATION            VFR_HUD.groundspeed -> "velocity"
    //   HEARTBEAT                   (NAMED_VALUE_INT LORA_T / LORA_V: ignored)
    // Frames are queued most-important-first; the packer fills each
    // LORA_PACKET_LEN-byte packet in this order:
    //   [IMU 22 + IMU2 28] [IMU3 34 + PRESS 26] [COSMIC 22 + VFR 20 + HB 21]
    if (a.ok) {
      n = MavlinkFrames::buildAccel(buf, a);
      snprintf(body, sizeof(body), "t=%lu ms  x=%d y=%d z=%d mg",
               (unsigned long)a.t_ms, a.x_mg, a.y_mg, a.z_mg);
      queueFrame_("ACCEL ", body, buf, n);
    }
    if (!essential && g.ok) {
      n = MavlinkFrames::buildGyro(buf, g);
      snprintf(body, sizeof(body), "t=%lu ms  x=%d y=%d z=%d mrad/s",
               (unsigned long)g.t_ms, g.x_mrad_s, g.y_mrad_s, g.z_mrad_s);
      queueFrame_("GYRO  ", body, buf, n);
    }
    if (!essential && m.ok) {
      n = MavlinkFrames::buildMag(buf, m);
      snprintf(body, sizeof(body), "t=%lu ms  x=%d y=%d z=%d mgauss",
               (unsigned long)m.t_ms, m.x_mgauss, m.y_mgauss, m.z_mgauss);
      queueFrame_("MAG   ", body, buf, n);
    }
    if (b.ok) {
      n = MavlinkFrames::buildBaroPressure(buf, b);
      snprintf(body, sizeof(body), "t=%lu ms  press=%.2f hPa temp=%d cC",
               (unsigned long)b.t_ms, b.pressure_hPa, b.temperature_cC);
      queueFrame_("BARO  ", body, buf, n);
    }
    if (gc.ok) {
      n = MavlinkFrames::buildCosmicRadiation(buf, gc);
      snprintf(body, sizeof(body), "t=%lu ms  counts1=%lu counts2=%lu total=%lu",
               (unsigned long)gc.t_ms,
               (unsigned long)gc.counts1, (unsigned long)gc.counts2,
               (unsigned long)(gc.counts1 + gc.counts2));
      queueFrame_("COSMIC", body, buf, n);
    }
    if (!essential && b.ok && haveLastAlt_) {
      n = MavlinkFrames::buildVfrHud(buf, vspeed_mps_);
      snprintf(body, sizeof(body), "vspeed=%.2f m/s (baro)", vspeed_mps_);
      queueFrame_("VSPEED", body, buf, n);
    }
    if (hbDue) {
      const uint32_t customMode = static_cast<uint32_t>(thermal_.level()) |
                                  (static_cast<uint32_t>(phase_) << 8);
      n = MavlinkFrames::buildHeartbeat(buf, mavState, customMode);
      snprintf(body, sizeof(body), "state=%u thermal=L%u phase=%u", mavState, thermal_.level(), phase_);
      queueFrame_("HEARTBEAT", body, buf, n);
      lastHeartbeatMs_ = nowMs;
    }
    packer_.flush();
    lastTxMs_ = nowMs;
  } else if (hbDue) {
    // Throttled cycle: heartbeat (+ pressure filler) only.
    sendHeartbeatPacket_(mavState);
    lastHeartbeatMs_ = nowMs;
#if USE_SERIAL
    Serial.printf("[THERM] L%u: heartbeat only this cycle\n", thermal_.level());
#endif
  } else {
#if USE_SERIAL
    Serial.printf("[THERM] L%u: nothing sent this cycle (%s)\n", thermal_.level(),
                  txAllowed ? "TX interval stretched" : "heartbeat only");
#endif
  }
#endif

  // CSV row mirrors every sensor value into SD/flash.
  String row = buildCsvRow_(nowMs, a, g, m, b, gc, lt, lv);
  storage_.writeRow(row);

#if USE_SERIAL
  Serial.print("[ROW] "); Serial.println(row);
#if USE_LORA
  Serial.printf("[LoRa] packets=%lu this cycle, sent=%lu retries=%lu drops=%lu queued=%u\n",
                static_cast<unsigned long>(packetsThisCycle_),
                static_cast<unsigned long>(lora_.totalSent()),
                static_cast<unsigned long>(lora_.totalRetries()),
                static_cast<unsigned long>(lora_.totalDropped()),
                static_cast<unsigned>(lora_.queueSize()));
#endif
#endif
}

// =============================================================================
//  buildCsvRow_
// =============================================================================
String App::buildCsvRow_(uint32_t nowMs,
                         const AccelReading& a,
                         const GyroReading&  g,
                         const MagReading&   m,
                         const BaroReading&  b,
                         const GeigerReading& gc,
                         const LoraTempReading& lt,
                         const LoraVoltReading& lv) {
  String r;
  r.reserve(280);
  r += "Run_ID = "          + String(runId) + "|";
  r += "Now_Ms = "          + String(nowMs) + "|";
  r += "Accsloration_mg = " + String(a.ok ? a.x_mg : 0)     + "," + String(a.ok ? a.y_mg : 0)     + "," + String(a.ok ? a.z_mg : 0)     + "|";
  r += "Gyro_mrad_s = "     + String(g.ok ? g.x_mrad_s : 0) + "," + String(g.ok ? g.y_mrad_s : 0) + "," + String(g.ok ? g.z_mrad_s : 0) + "|";
  r += "Mag_mgauss = "      + String(m.ok ? m.x_mgauss : 0) + "," + String(m.ok ? m.y_mgauss : 0) + "," + String(m.ok ? m.z_mgauss : 0) + "|";
  r += "Pressure_hPa = "    + String(b.ok ? b.pressure_hPa : 0.0f, 2) + "|";
  r += "Temperature_cC = "  + String(b.ok ? b.temperature_cC : 0)     + "|";
  r += "Geiger_1 = "        + String(static_cast<unsigned long>(gc.counts1)) + "|";
  r += "Geiger_2 = "        + String(static_cast<unsigned long>(gc.counts2)) + "|";
  r += "LoRa_Temp_cC = "    + String(lt.ok ? lt.t_cC : 0) + "|";
  r += "LoRa_Volt_mV = "    + String(lv.ok ? lv.mV   : 0) + "|";
#if USE_LORA
  r += "LoRa_Retries = "    + String(static_cast<unsigned long>(lora_.totalRetries())) + "|";
  r += "LoRa_Drops = "      + String(static_cast<unsigned long>(lora_.totalDropped())) + "|";
#else
  r += "LoRa_Retries = 0|";
  r += "LoRa_Drops = 0|";
#endif
  r += "Storage_Fails = "   + String(static_cast<unsigned long>(storage_.failures())) + "|";
  r += "Flight_Phase = "    + String(phase_) + "|";
  r += "Flight_Time_s = "   + String(static_cast<unsigned long>(flightElapsedS_())) + "|";
  r += "LoRa_Thermal_Level = " + String(thermal_.level()) + "|";
  r += "Vert_Speed_mps = "  + String(vspeed_mps_, 2);
  return r;
}

// =============================================================================
//  printTx_
// =============================================================================
void App::printTx_(const char* label, const char* body,
                   const uint8_t* bytes, size_t len) {
#if USE_SERIAL && PRINT_TX
  Serial.printf("[TX] %s  %s  (%u bytes)\n",
                label, body, static_cast<unsigned>(len));
#if PRINT_TX_HEX
  Serial.print("[TX]   bytes:");
  for (size_t i = 0; i < len; ++i) {
    Serial.printf(" %02X", bytes[i]);
  }
  Serial.println();
#else
  (void)bytes;
#endif
#else
  (void)label; (void)body; (void)bytes; (void)len;
#endif
}

// =============================================================================
//  sendFrame_  -  one finished packet to the radio (retry queue on failure).
// =============================================================================
void App::sendFrame_(const uint8_t* buf, size_t len) {
#if USE_LORA
  if (len == 0 || len > LORA_FRAME_MAX) return;
  lora_.send(buf, static_cast<uint8_t>(len));
#else
  (void)buf; (void)len;
#endif
}

// =============================================================================
//  processSerialCommands_
// =============================================================================
void App::processSerialCommands_() {
#if USE_SERIAL
  while (Serial.available()) {
    char ch = static_cast<char>(Serial.read());
    if (ch == '\r') continue;
    if (ch == '\n') {
      String cmd = serialBuf_;
      serialBuf_.clear();
      cmd.trim();
      cmd.toLowerCase();

      if (cmd == "status") {
        prefs.begin("payload", true);
        uint8_t flown = prefs.getUChar("flown", 0);
        prefs.end();
        Serial.printf("[STAT] runId=%lu sd=%d flash=%d storageFails=%lu\n",
                      static_cast<unsigned long>(runId),
                      storage_.sdMounted() ? 1 : 0,
                      storage_.flashMounted() ? 1 : 0,
                      static_cast<unsigned long>(storage_.failures()));
        Serial.printf("[STAT] flown=%u (launch gate %s on next boot)\n",
                      flown, flown ? "SKIPPED - use `disarm`" : "armed");
        Serial.printf("[STAT] phase=%u flight_time=%lu/%lu s %s\n", phase_,
                      static_cast<unsigned long>(flightElapsedS_()),
                      static_cast<unsigned long>(FLIGHT_MAX_DURATION_S),
                      landed_ ? "LANDED (stopped)" : (flying_ ? "flying" : "on pad"));
        Serial.printf("[STAT] lora temp=%d.%02d C thermal=L%u%s (TX every %u.%u cycles%s, PA %u mV)\n",
                      thermal_.lastTemp() / 100, abs(thermal_.lastTemp() % 100), thermal_.level(),
                      thermal_.forced() ? " FORCED" : "",
                      thermal_.txIntervalTenths() / 10, thermal_.txIntervalTenths() % 10,
                      thermal_.essentialOnly() ? ", essential only" : "", thermal_.dacMv());
#if USE_LORA
        Serial.printf("[STAT] packets: %u bytes each, %lu sent\n",
                      static_cast<unsigned>(LORA_PACKET_LEN),
                      static_cast<unsigned long>(packer_.packetsSent()));
        Serial.printf("[STAT] lora sent=%lu retries=%lu drops=%lu queued=%u\n",
                      static_cast<unsigned long>(lora_.totalSent()),
                      static_cast<unsigned long>(lora_.totalRetries()),
                      static_cast<unsigned long>(lora_.totalDropped()),
                      static_cast<unsigned>(lora_.queueSize()));
#endif
      } else if (cmd == "print") {
        storage_.dumpFlashToSerial();
      } else if (cmd == "clear") {
        storage_.wipeFlash();
        Serial.println("[CMD] flash wiped");
      } else if (cmd == "new") {
        // Bump runId so the next written row goes into a new file. A new run
        // means a new flight, so re-arm the launch gate as well.
        prefs.begin("payload", false);
        runId = prefs.getUInt("run", 0) + 1;
        prefs.putUInt("run", runId);
        prefs.putUChar("flown", 0);
        prefs.putUInt("fl_s", 0);
        prefs.end();
        storage_.begin(runId);
        Serial.printf("[CMD] new run id=%lu (flown cleared - gate armed)\n",
                      static_cast<unsigned long>(runId));
      } else if (cmd == "disarm") {
        prefs.begin("payload", false);
        prefs.putUChar("flown", 0);
        prefs.putUInt("fl_s", 0);
        prefs.end();
        Serial.println("[CMD] flown + flight time cleared - launch gate armed for next boot");
      } else if (cmd == "fly") {
        // Bench aid, opposite of `disarm`: next boot skips the launch gate
        // and starts a fresh 30 min flight clock (no shaking needed).
        prefs.begin("payload", false);
        prefs.putUChar("flown", 1);
        prefs.putUInt("fl_s", 0);
        prefs.end();
        Serial.println("[CMD] flown set, flight clock reset - next boot goes straight to mission mode");
      } else if (cmd == "land") {
        enterLanded_("serial command");
      } else if (cmd.startsWith("thermal")) {
        // `thermal N` pins level N (bench test), `thermal auto` releases it.
        String arg = cmd.length() > 7 ? cmd.substring(8) : "";
        arg.trim();
        if (arg == "auto" || arg == "") {
          thermal_.force(-1);
          Serial.println("[CMD] thermal level: automatic");
        } else {
          thermal_.force(static_cast<int8_t>(arg.toInt()));
          sensors_.setLoraVolt(thermal_.dacMv());
          Serial.printf("[CMD] thermal level FORCED to L%u (TX every %u.%u cycles%s, PA %u mV)\n",
                        thermal_.level(), thermal_.txIntervalTenths() / 10, thermal_.txIntervalTenths() % 10,
                        thermal_.essentialOnly() ? ", essential only" : "", thermal_.dacMv());
        }
      } else if (cmd.startsWith("volt ")) {
        long mV = cmd.substring(5).toInt();
        if (mV < 0)    mV = 0;
        if (mV > 3300) mV = 3300;
        bool ok = sensors_.setLoraVolt(static_cast<uint16_t>(mV));
        Serial.printf("[CMD] LoRa volt -> %ld mV  %s\n", mV, ok ? "OK" : "FAIL");
      } else if (cmd == "cw on" || cmd == "cw") {
#if USE_LORA
        bool ok = lora_.startCw();
        Serial.printf("[CMD] CW on -> %s. Watch LoRa temp; type 'cw off' to stop.\n",
                      ok ? "OK" : "FAIL");
#else
        Serial.println("[CMD] CW unavailable: USE_LORA disabled");
#endif
      } else if (cmd == "cw off" || cmd == "stop") {
#if USE_LORA
        bool ok = lora_.stopCw();
        Serial.printf("[CMD] CW off -> %s\n", ok ? "OK" : "FAIL");
#else
        Serial.println("[CMD] CW unavailable: USE_LORA disabled");
#endif
      } else if (cmd == "last") {
        AccelReading a = sensors_.lastAccel();
        GyroReading  g = sensors_.lastGyro();
        MagReading   m = sensors_.lastMag();
        BaroReading  b = sensors_.lastBaro();
        Serial.printf("[LAST] acc(mg)=%d,%d,%d  gyro(mrad/s)=%d,%d,%d  "
                      "mag(mG)=%d,%d,%d  baro=%.2fhPa,%dcC\n",
                      a.x_mg, a.y_mg, a.z_mg,
                      g.x_mrad_s, g.y_mrad_s, g.z_mrad_s,
                      m.x_mgauss, m.y_mgauss, m.z_mgauss,
                      b.pressure_hPa, b.temperature_cC);
      } else if (cmd.length()) {
        Serial.print("[CMD] unknown: "); Serial.println(cmd);
        Serial.println("[CMD] try: status | print | clear | new | last | disarm | fly | land | thermal N|auto");
      }
    } else {
      if (serialBuf_.length() < 128) serialBuf_ += ch;
    }
  }
#endif
}
