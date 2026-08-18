#include "New/App.hpp"
#if USE_LORA
  #include "New/MavlinkFrames.hpp"
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
  Serial.println("==== PAYLOAD (NEW) booting ====");
#endif

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
  // ~75% of boots (proven by test_sd_minimal_prefix: 0/8 with LoRa
  // connected & RST floating, 8/8 with RST held LOW). RadioLib's begin()
  // will release this RST and run its own reset sequence later.
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
  // Earlier comment here noted SD.begin() can leave the bus in a state the
  // InvenSense IMU driver dislikes - that concern only applies when USE_IMU
  // is enabled (currently 0). If USE_IMU is ever re-enabled and SD-after-IMU
  // works better there, restore the old order (sensors / radio first).
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
  sensors_.poll(now);

#if USE_SERIAL
  Serial.println("[APP] ready. Commands: status | print | clear | new | last");
#endif
}

// =============================================================================
//  waitForLaunch_  -  low-power wait for the start of flight.
//  Mirrors env:test_lowpower_wake, but reuses Sensors::poll()/lastAccel() so
//  there is no second copy of the IMU read path.
// =============================================================================
void App::waitForLaunch_() {
#if USE_LAUNCH_GATE
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
                "for %u samples\n",
                LAUNCH_USE_LIGHT_SLEEP, static_cast<long>(LAUNCH_THRESHOLD_MG),
                LAUNCH_CONFIRM_SAMPLES);
#endif

  uint8_t  aboveCount = 0;
  uint32_t checks     = 0;
  for (;;) {
    // Idle the core for one interval while the accel keeps sampling.
#if LAUNCH_USE_LIGHT_SLEEP
    esp_sleep_enable_timer_wakeup(
        static_cast<uint64_t>(LAUNCH_CHECK_INTERVAL_MS) * 1000ULL);
    esp_light_sleep_start();
#else
    delay(LAUNCH_CHECK_INTERVAL_MS);
#endif

    uint32_t now = millis();
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

#if USE_SERIAL
  Serial.println("[GATE] FLIGHT DETECTED - starting mission");
#endif
#endif // USE_LAUNCH_GATE
}

// =============================================================================
//  tick
// =============================================================================
void App::tick() {
  uint32_t now = millis();

  processSerialCommands_();

  // Drain a few retry frames every loop so the queue does not grow.
#if USE_LORA
  lora_.service(/*maxAttemptsThisTick=*/2);
#endif

  // Refresh accel/gyro/mag/baro at SENSOR_POLL_INTERVAL_MS cadence.
  if (static_cast<uint32_t>(now - lastSensorPollMs_) >= SENSOR_POLL_INTERVAL_MS) {
    sensors_.poll(now);
    lastSensorPollMs_ = now;
  }

  // Run a full TX + log cycle every TX_CYCLE_INTERVAL_MS.
  if (static_cast<uint32_t>(now - lastTxCycleMs_) >= TX_CYCLE_INTERVAL_MS) {
    runTxCycle_(now);
    lastTxCycleMs_ = now;
  }
}

// =============================================================================
//  runTxCycle_  -  the heart of the firmware.
//  One MAVLink frame per sensor, one LoRa packet per frame.
// =============================================================================
void App::runTxCycle_(uint32_t nowMs) {
  AccelReading    a  = sensors_.lastAccel();
  GyroReading     g  = sensors_.lastGyro();
  MagReading      m  = sensors_.lastMag();
  BaroReading     b  = sensors_.lastBaro();
  GeigerReading   gc = sensors_.takeGeiger(nowMs);
  LoraTempReading lt = sensors_.readLoraTemp(nowMs);
  LoraVoltReading lv = sensors_.readLoraVolt(nowMs);

#if USE_LORA
  uint8_t buf[LORA_FRAME_MAX];
  size_t  n;

  // Heartbeat every HEARTBEAT_INTERVAL_MS so a ground station knows we are alive.
  if (static_cast<uint32_t>(nowMs - lastHeartbeatMs_) >= HEARTBEAT_INTERVAL_MS) {
    n = MavlinkFrames::buildHeartbeat(buf);
    printTx_("HEARTBEAT", "(no payload)", buf, n);
    sendFrame_(buf, n);
    lastHeartbeatMs_ = nowMs;
  }

  // Each sensor reading goes out as its own MAVLink frame / LoRa packet.
  char body[80];
  if (a.ok)  {
    n = MavlinkFrames::buildAccel(buf, a);
    snprintf(body, sizeof(body), "t=%lu ms  x=%d y=%d z=%d mg",
             (unsigned long)a.t_ms, a.x_mg, a.y_mg, a.z_mg);
    printTx_("ACCEL ", body, buf, n);
    sendFrame_(buf, n);
  }
  if (g.ok)  {
    n = MavlinkFrames::buildGyro(buf, g);
    snprintf(body, sizeof(body), "t=%lu ms  x=%d y=%d z=%d mrad/s",
             (unsigned long)g.t_ms, g.x_mrad_s, g.y_mrad_s, g.z_mrad_s);
    printTx_("GYRO  ", body, buf, n);
    sendFrame_(buf, n);
  }
  if (m.ok)  {
    n = MavlinkFrames::buildMag(buf, m);
    snprintf(body, sizeof(body), "t=%lu ms  x=%d y=%d z=%d mgauss",
             (unsigned long)m.t_ms, m.x_mgauss, m.y_mgauss, m.z_mgauss);
    printTx_("MAG   ", body, buf, n);
    sendFrame_(buf, n);
  }
  if (b.ok)  {
    n = MavlinkFrames::buildBaroPressure(buf, b);
    snprintf(body, sizeof(body), "t=%lu ms  press=%.2f hPa",
             (unsigned long)b.t_ms, b.pressure_hPa);
    printTx_("BARO_P", body, buf, n);
    sendFrame_(buf, n);
  }
  if (b.ok)  {
    n = MavlinkFrames::buildBaroTemp(buf, b);
    snprintf(body, sizeof(body), "t=%lu ms  temp=%d cC",
             (unsigned long)b.t_ms, b.temperature_cC);
    printTx_("BARO_T", body, buf, n);
    sendFrame_(buf, n);
  }
  if (gc.ok) {
    n = MavlinkFrames::buildCosmicRadiation(buf, gc);
    snprintf(body, sizeof(body), "t=%lu ms  counts1=%lu counts2=%lu total=%lu",
             (unsigned long)gc.t_ms,
             (unsigned long)gc.counts1, (unsigned long)gc.counts2,
             (unsigned long)(gc.counts1 + gc.counts2));
    printTx_("COSMIC", body, buf, n);
    sendFrame_(buf, n);
  }
  if (lt.ok) {
    n = MavlinkFrames::buildLoraTemp(buf, lt);
    snprintf(body, sizeof(body), "t=%lu ms  temp=%d cC",
             (unsigned long)lt.t_ms, lt.t_cC);
    printTx_("LORA_T", body, buf, n);
    sendFrame_(buf, n);
  }
  if (lv.ok) {
    n = MavlinkFrames::buildLoraVolt(buf, lv);
    snprintf(body, sizeof(body), "t=%lu ms  volt=%u mV",
             (unsigned long)lv.t_ms, lv.mV);
    printTx_("LORA_V", body, buf, n);
    sendFrame_(buf, n);
  }
#endif

  // CSV row mirrors every sensor value into SD/flash.
  String row = buildCsvRow_(nowMs, a, g, m, b, gc, lt, lv);
  storage_.writeRow(row);

#if USE_SERIAL
  Serial.print("[ROW] "); Serial.println(row);
#if USE_LORA
  Serial.printf("[LoRa] sent=%lu retries=%lu drops=%lu queued=%u\n",
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
  r += "Storage_Fails = "   + String(static_cast<unsigned long>(storage_.failures()));
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
//  sendFrame_
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
        Serial.printf("[STAT] runId=%lu sd=%d flash=%d storageFails=%lu\n",
                      static_cast<unsigned long>(runId),
                      storage_.sdMounted() ? 1 : 0,
                      storage_.flashMounted() ? 1 : 0,
                      static_cast<unsigned long>(storage_.failures()));
#if USE_LORA
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
        // Bump runId so the next written row goes into a new file.
        prefs.begin("payload", false);
        runId = prefs.getUInt("run", 0) + 1;
        prefs.putUInt("run", runId);
        prefs.end();
        storage_.begin(runId);
        Serial.printf("[CMD] new run id=%lu\n", static_cast<unsigned long>(runId));
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
        Serial.println("[CMD] try: status | print | clear | new | last");
      }
    } else {
      if (serialBuf_.length() < 128) serialBuf_ += ch;
    }
  }
#endif
}
