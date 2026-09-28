#include "Stack_without_LoRa/App.hpp"

// =============================================================================
//  App.cpp  -  Stack_without_LoRa
// =============================================================================

App::PreEntry App::pre_[PRETRIGGER_SAMPLES];

const char* App::modeName_(Mode m) {
  switch (m) {
    case Mode::PAD:    return "PAD";
    case Mode::FLIGHT: return "FLIGHT";
    case Mode::POST:   return "POST";
  }
  return "?";
}

// =============================================================================
//  begin
// =============================================================================
void App::begin() {
#if USE_SERIAL
  Serial.begin(115200);
  for (uint32_t t = millis(); millis() - t < 200;) { /* settle */ }
  Serial.println();
  Serial.printf("==== %s v%s booting ====\n", FW_NAME, FW_VERSION);
#endif
  bootMs_ = millis();

  // Shared buses.  Every SPI chip-select high BEFORE the first transfer
  // (the IMU has an internal pull-down on CS).  LoRa-board lines parked:
  // no SX1280 on this stack, keep its rail off and its bus pins passive.
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
  pinMode(PIN_IMU_CS,  OUTPUT); digitalWrite(PIN_IMU_CS,  HIGH);
  pinMode(PIN_SD_CS,   OUTPUT); digitalWrite(PIN_SD_CS,   HIGH);
  pinMode(PIN_LORA_CS, OUTPUT); digitalWrite(PIN_LORA_CS, HIGH);
  pinMode(PIN_LORA_RST,OUTPUT); digitalWrite(PIN_LORA_RST,LOW);
  pinMode(PIN_LORA_PWR,OUTPUT); digitalWrite(PIN_LORA_PWR,LOW);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);

  // Run id + flown flag from NVS (own namespace, see Config.hpp).
  prefs.begin(NVS_NAMESPACE, false);
  runId = prefs.getUInt("run", 0) + 1;
  prefs.putUInt("run", runId);
  flownFlag_ = prefs.getUChar("flown", 0) != 0;
  prefs.end();
#if USE_SERIAL
  Serial.printf("[APP] runId=%lu flown=%d\n", static_cast<unsigned long>(runId), flownFlag_ ? 1 : 0);
#endif

  // Storage first: the SD mount wants a quiet SPI bus (see Stack_with_LoRa).
  logger_.begin(runId);

  char msg[128];
  snprintf(msg, sizeof(msg), "BOOT %s v%s run=%lu", FW_NAME, FW_VERSION,
           static_cast<unsigned long>(runId));
  logger_.event(msg);

  geiger_.begin();
  sensors_.begin();

  snprintf(msg, sizeof(msg), "SENSORS imu=%d baro=%d(0x%02X) mag=%d lora_temp=%d sd=%d flash=%d",
           sensors_.imuOk(), sensors_.baroOk(), sensors_.baroChipId(), sensors_.magOk(),
           sensors_.loraTempOk(), logger_.sdOk(), logger_.flashOk());
  logger_.event(msg);
  {
    char diag[128];
    snprintf(diag, sizeof(diag), "BARO_INIT %s", sensors_.baroDiag());
    logger_.event(diag);
  }

  // Launch detector threshold in raw LSB^2.
  const int64_t lsbPerG = imuLsbPerG();
  const int64_t thr = static_cast<int64_t>(LAUNCH_THRESHOLD_G * static_cast<float>(lsbPerG));
  threshRaw2_ = thr * thr;

  if (flownFlag_) {
    // Mid-flight reset: never sit in PAD rates while the rocket is flying.
    enterFlight_("flown flag set at boot", false);
  } else if (!sensors_.imuOk()) {
    // No accelerometer = no launch detection: log everything at full rate.
    enterFlight_("IMU absent, no launch detect", false);
  } else {
    logger_.event("MODE PAD (waiting for launch)");
  }

  uint32_t now = millis();
  lastHkMs_ = now;
  lastImuPollUs_ = nowUs();

#if USE_SERIAL
  Serial.println("[APP] ready. Type `help` for commands.");
  // From here on serial never blocks the logger: if nobody drains the USB
  // buffer, text is dropped instead of stalling the loop.
  Serial.setTxTimeoutMs(0);
#endif
}

// =============================================================================
//  tick
// =============================================================================
void App::tick() {
  const uint64_t now = nowUs();
  const uint32_t nowMs = static_cast<uint32_t>(now / 1000ULL);
  loops_++;

  processSerial_();
  drainImu_(now);
  pollBaro_(now);
  pollMag_(now);
  drainGeiger_();

  if (static_cast<uint32_t>(nowMs - lastHkMs_) >= HOUSEKEEPING_INTERVAL_MS) {
    housekeeping_(now, nowMs);
    lastHkMs_ = nowMs;
  }

  logger_.service(nowMs);
}

// =============================================================================
//  IMU
// =============================================================================
void App::emitImuRow_(uint64_t t, const ImuSample& s) {
  Row r;
  r.tag("IMU");
  r.cu64(t);
  r.cfixed(imuAccelToMgx100(s.ax), 2);
  r.cfixed(imuAccelToMgx100(s.ay), 2);
  r.cfixed(imuAccelToMgx100(s.az), 2);
  r.cfixed(imuGyroToMdps(s.gx), 3);
  r.cfixed(imuGyroToMdps(s.gy), 3);
  r.cfixed(imuGyroToMdps(s.gz), 3);
  r.endl();
  logger_.push(RowType::IMU, r);
}

void App::drainImu_(uint64_t nowUs_) {
  if (!sensors_.imuOk()) return;
  if (nowUs_ - lastImuPollUs_ < IMU_POLL_INTERVAL_US) return;
  lastImuPollUs_ = nowUs_;

  static ImuSample buf[IMU_FIFO_MAX_FRAMES_PER_DRAIN];
  uint16_t inFifo = 0;
  int n = sensors_.drainImu(buf, IMU_FIFO_MAX_FRAMES_PER_DRAIN, inFifo);
  if (n <= 0) return;
  if (inFifo > fifoMax_) fifoMax_ = inFifo;
  imuFrames_ += static_cast<uint32_t>(n);

  const double periodNom = 1000000.0 / IMU_ODR_HZ;
  // The read happens on average half a poll interval after the newest
  // frame landed in the FIFO; centre the host estimate on the sample.
  const double tRead = static_cast<double>(nowUs()) - 0.5 * IMU_POLL_INTERVAL_US;

  for (int i = 0; i < n; ++i) {
    const ImuSample& s = buf[i];
    lastImu_ = s;

    // Host estimate: this batch holds the OLDEST frames; frame i was sampled
    // (inFifo - 1 - i) periods before the read.  Jitter = one poll interval.
    const double h = tRead - static_cast<double>(inFifo > i ? (inFifo - 1 - i) : 0) * periodNom;

    if (!imuClkInit_) {
      imuClkT_ = h; imuPeriodEst_ = periodNom; imuClkInit_ = true;
    } else {
      imuClkT_ += imuPeriodEst_;
      double err = h - imuClkT_;
      if (err > 50000.0 || err < -50000.0) {
        // Way off (FIFO overflow / first frames after a long gap): re-lock.
        imuClkT_ = h; imuPeriodEst_ = periodNom;
      } else {
        // Critically damped 2nd-order loop: Kp = 0.02, Ki = Kp^2 / 4.
        imuClkT_      += err * 0.02;
        imuPeriodEst_ += err * 0.0001;
        if (imuPeriodEst_ > periodNom * 1.05) imuPeriodEst_ = periodNom * 1.05;
        if (imuPeriodEst_ < periodNom * 0.95) imuPeriodEst_ = periodNom * 0.95;
      }
    }
    uint64_t t = imuClkT_ > 0.0 ? static_cast<uint64_t>(imuClkT_) : 0;
    if (t <= lastImuT_) t = lastImuT_ + 1;
    lastImuT_ = t;

    // Keep the newest second in RAM regardless of mode.
    PreEntry& e = pre_[preHead_];
    e.t = t; e.s = s; e.logged = false;
    preHead_ = static_cast<uint16_t>((preHead_ + 1) % PRETRIGGER_SAMPLES);
    if (preCount_ < PRETRIGGER_SAMPLES) preCount_++;

    // Launch detector (PAD and POST).  Up/down counter tolerates dips.
    if (mode_ != Mode::FLIGHT) {
      int64_t m2 = static_cast<int64_t>(s.ax) * s.ax +
                   static_cast<int64_t>(s.ay) * s.ay +
                   static_cast<int64_t>(s.az) * s.az;
      if (m2 > threshRaw2_) {
        if (++aboveCount_ >= LAUNCH_CONFIRM_SAMPLES) {
          enterFlight_("accel threshold", true);   // Dumps the pre-trigger ring.
          continue;                                // This sample went out with it.
        }
      } else if (aboveCount_ > 0) {
        aboveCount_--;
      }
    }

    if (mode_ == Mode::FLIGHT) {
      emitImuRow_(t, s);
      e.logged = true;
    } else if (++padImuDiv_ >= PAD_IMU_DIVIDER) {
      padImuDiv_ = 0;
      emitImuRow_(t, s);
      e.logged = true;
    }
  }
}

// =============================================================================
//  Baro / Mag / Geiger
// =============================================================================
void App::pollBaro_(uint64_t nowUs_) {
  BaroSample b;
  if (!sensors_.pollBaro(nowUs_, b)) return;
  lastBaro_ = b;
  if (mode_ != Mode::FLIGHT) {
    if (++padBaroDiv_ < PAD_BARO_DIVIDER) return;
    padBaroDiv_ = 0;
  }
  Row r;
  r.tag("BARO");
  r.cu64(b.t_us);
  r.cfixed(b.press_Pa_x10, 3);   // hPa with 3 decimals (0.1 Pa).
  r.cfixed(b.temp_cC, 2);
  r.cfixed(b.alt_cm, 2);
  r.endl();
  logger_.push(RowType::BARO, r);
}

void App::pollMag_(uint64_t nowUs_) {
  MagSample m;
  if (!sensors_.pollMag(nowUs_, m)) return;
  lastMag_ = m;
  if (mode_ != Mode::FLIGHT) {
    if (++padMagDiv_ < PAD_MAG_DIVIDER) return;
    padMagDiv_ = 0;
  }
  Row r;
  r.tag("MAG");
  r.cu64(m.t_us);
  r.cfixed(m.x_nT100, 2);
  r.cfixed(m.y_nT100, 2);
  r.cfixed(m.z_nT100, 2);
  r.cfixed(m.temp_cC, 2);
  r.endl();
  logger_.push(RowType::MAG, r);
}

void App::drainGeiger_() {
  GeigerEvent ev;
  for (int i = 0; i < 32 && geiger_.popEvent(ev); ++i) {
    Row r;
    r.tag("GEIG");
    r.cu64(ev.t_us);
    r.cu32(ev.channel);
    r.endl();
    logger_.push(RowType::GEIG, r);
  }
}

// =============================================================================
//  Housekeeping (1 Hz)
// =============================================================================
void App::housekeeping_(uint64_t nowUs_, uint32_t nowMs) {
  const uint32_t dtMs = nowMs - lastHkMs_;

  // RATE: Geiger counts in this window + running totals.
  {
    uint32_t c1, c2;
    geiger_.takeInterval(c1, c2);
    Row r;
    r.tag("RATE");
    r.cu64(nowUs_);
    r.cu32(dtMs);
    r.cu32(c1);
    r.cu32(c2);
    r.cu32(geiger_.total1());
    r.cu32(geiger_.total2());
    r.cu32(geiger_.dropped());
    r.endl();
    logger_.push(RowType::RATE, r);
  }

  // TEMP: IMU die (from the FIFO packet), ESP32 die, LoRa board.
  {
    int32_t esp = 0, lora = 0;
    bool haveEsp  = sensors_.readEspTemp(esp);
    bool haveLora = sensors_.readLoraTemp(lora);
    Row r;
    r.tag("TEMP");
    r.cu64(nowUs_);
    if (sensors_.imuOk()) r.cfixed(lastImu_.temp_cC, 2); else r.cempty();
    if (haveEsp)  r.cfixed(esp, 2);  else r.cempty();
    if (haveLora) r.cfixed(lora, 2); else r.cempty();
    r.endl();
    logger_.push(RowType::TEMP, r);
  }

  // STAT.
  const uint32_t loopHz = dtMs ? (loops_ * 1000u) / dtMs : 0;
  {
    Row r;
    r.tag("STAT");
    r.cu64(nowUs_);
    r.cstr(modeName_(mode_));
    r.cu32(runId);
    r.cu32(logger_.sdPart());
    r.cu32((nowMs - bootMs_) / 1000u);
    r.cu32(loopHz);
    r.cu32(imuFrames_);
    r.cu32(fifoMax_);
    r.cu32(sensors_.imuErrors());
    r.cu32(logger_.sdOk() ? 1 : 0);
    r.cu32(logger_.sdBytes());
    r.cu32(logger_.sdFails());
    r.cu32(logger_.ringDrops());
    r.cu32(logger_.ringHighWater());
    r.cu32(logger_.flashBytes());
    r.cu32(ESP.getFreeHeap());
    r.endl();
    logger_.push(RowType::STAT, r);
  }

#if USE_SERIAL
  Serial.printf("[%s] up=%lus loop=%luHz imu=%lu/s fifo_max=%u a=%.2f,%.2f,%.2f g p=%.2f hPa sd=%s %lu B fails=%lu drops=%lu geig=%lu/%lu\n",
                modeName_(mode_), static_cast<unsigned long>((nowMs - bootMs_) / 1000u),
                static_cast<unsigned long>(loopHz), static_cast<unsigned long>(imuFrames_), fifoMax_,
                imuAccelToMgx100(lastImu_.ax) / 100000.0f,
                imuAccelToMgx100(lastImu_.ay) / 100000.0f,
                imuAccelToMgx100(lastImu_.az) / 100000.0f,
                lastBaro_.press_Pa_x10 / 1000.0f,
                logger_.sdOk() ? "OK" : "DOWN", static_cast<unsigned long>(logger_.sdBytes()),
                static_cast<unsigned long>(logger_.sdFails()), static_cast<unsigned long>(logger_.ringDrops()),
                static_cast<unsigned long>(geiger_.total1()), static_cast<unsigned long>(geiger_.total2()));
#endif

  imuFrames_ = 0;
  fifoMax_   = 0;
  loops_     = 0;

  // FLIGHT -> POST after the full-rate window (0 = never).
  if (mode_ == Mode::FLIGHT && FLIGHT_FULL_RATE_S > 0 &&
      static_cast<uint32_t>(nowMs - flightStartMs_) >= FLIGHT_FULL_RATE_S * 1000u) {
    enterLowRate_(Mode::POST, "full-rate window elapsed");
  }
}

// =============================================================================
//  Mode changes
// =============================================================================
void App::enterFlight_(const char* why, bool persistFlag) {
  mode_ = Mode::FLIGHT;
  flightStartMs_ = millis();
  aboveCount_ = 0;

  char msg[80];
  snprintf(msg, sizeof(msg), "MODE FLIGHT (%s)", why);
  logger_.event(msg);

  // Write out the last second of full-rate IMU data that PAD mode skipped.
  uint16_t idx = static_cast<uint16_t>((preHead_ + PRETRIGGER_SAMPLES - preCount_) % PRETRIGGER_SAMPLES);
  uint16_t dumped = 0;
  for (uint16_t k = 0; k < preCount_; ++k) {
    PreEntry& e = pre_[idx];
    if (!e.logged) { emitImuRow_(e.t, e.s); e.logged = true; dumped++; }
    idx = static_cast<uint16_t>((idx + 1) % PRETRIGGER_SAMPLES);
  }
  if (dumped) {
    snprintf(msg, sizeof(msg), "PRETRIGGER %u samples written", dumped);
    logger_.event(msg);
  }

  if (persistFlag && !flownFlag_) {
    prefs.begin(NVS_NAMESPACE, false);
    prefs.putUChar("flown", 1);
    prefs.end();
    flownFlag_ = true;
  }
}

void App::enterLowRate_(Mode m, const char* why) {
  mode_ = m;
  aboveCount_ = 0;
  char msg[80];
  snprintf(msg, sizeof(msg), "MODE %s (%s)", modeName_(m), why);
  logger_.event(msg);
}

// =============================================================================
//  Serial
// =============================================================================
void App::printHelp_() {
#if USE_SERIAL
  Serial.println("[CMD] status        - mode, run, sensors, SD/flash counters");
  Serial.println("[CMD] last          - most recent IMU / baro / mag values");
  Serial.println("[CMD] ls            - list SD /logs");
  Serial.println("[CMD] tail [n]      - last n lines of the current SD file (default 20)");
  Serial.println("[CMD] head          - first 2 kB of the current SD file");
  Serial.println("[CMD] sync          - flush pending rows to the card now");
  Serial.println("[CMD] new           - start a new run (new files)");
  Serial.println("[CMD] flight | pad  - force full-rate / low-rate mode");
  Serial.println("[CMD] arm           - clear the flown flag (PAD mode at next boot)");
  Serial.println("[CMD] print | clear - dump / wipe the flash fallback log");
#endif
}

void App::printStatus_() {
#if USE_SERIAL
  Serial.printf("[STAT] %s v%s run=%lu part=%u mode=%s flown=%d up=%lus\n",
                FW_NAME, FW_VERSION, static_cast<unsigned long>(runId), logger_.sdPart(),
                modeName_(mode_), flownFlag_ ? 1 : 0,
                static_cast<unsigned long>((millis() - bootMs_) / 1000u));
  Serial.printf("[STAT] imu=%d(%u Hz, hires=%d, errs=%lu) baro=%d(0x%02X) mag=%d lora_temp=%d\n",
                sensors_.imuOk(), IMU_ODR_HZ, IMU_FIFO_HIRES,
                static_cast<unsigned long>(sensors_.imuErrors()),
                sensors_.baroOk(), sensors_.baroChipId(), sensors_.magOk(), sensors_.loraTempOk());
  Serial.printf("[STAT] baro init: %s\n", sensors_.baroDiag());
  Serial.printf("[STAT] sd=%s file=%s bytes=%lu fails=%lu ring_drops=%lu ring_max=%lu\n",
                logger_.sdOk() ? "OK" : "DOWN", logger_.sdPath(),
                static_cast<unsigned long>(logger_.sdBytes()),
                static_cast<unsigned long>(logger_.sdFails()),
                static_cast<unsigned long>(logger_.ringDrops()),
                static_cast<unsigned long>(logger_.ringHighWater()));
  Serial.printf("[STAT] flash=%d bytes=%lu  geiger totals=%lu/%lu dropped_ts=%lu  heap=%lu\n",
                logger_.flashOk(), static_cast<unsigned long>(logger_.flashBytes()),
                static_cast<unsigned long>(geiger_.total1()),
                static_cast<unsigned long>(geiger_.total2()),
                static_cast<unsigned long>(geiger_.dropped()),
                static_cast<unsigned long>(ESP.getFreeHeap()));
#endif
}

void App::processSerial_() {
#if USE_SERIAL
  while (Serial.available()) {
    char ch = static_cast<char>(Serial.read());
    if (ch == '\r') continue;
    if (ch != '\n') {
      if (serialBuf_.length() < 64) serialBuf_ += ch;
      continue;
    }
    String cmd = serialBuf_;
    serialBuf_ = "";
    cmd.trim();
    cmd.toLowerCase();
    if (!cmd.length()) continue;

    // Long dumps need the USB buffer to actually drain: allow a little
    // blocking for the duration of the command only (the IMU FIFO covers
    // ~1 s, so keep every command well under that).
    Serial.setTxTimeoutMs(50);

    if (cmd == "help" || cmd == "?") {
      printHelp_();
    } else if (cmd == "status") {
      printStatus_();
    } else if (cmd == "last") {
      Serial.printf("[LAST] a=%.2f,%.2f,%.2f mg  g=%.3f,%.3f,%.3f dps  imu_t=%.2f C\n",
                    imuAccelToMgx100(lastImu_.ax) / 100.0f, imuAccelToMgx100(lastImu_.ay) / 100.0f,
                    imuAccelToMgx100(lastImu_.az) / 100.0f, imuGyroToMdps(lastImu_.gx) / 1000.0f,
                    imuGyroToMdps(lastImu_.gy) / 1000.0f, imuGyroToMdps(lastImu_.gz) / 1000.0f,
                    lastImu_.temp_cC / 100.0f);
      Serial.printf("[LAST] p=%.3f hPa  t=%.2f C  alt=%.2f m   mag=%.2f,%.2f,%.2f uT (%.2f C)\n",
                    lastBaro_.press_Pa_x10 / 1000.0f, lastBaro_.temp_cC / 100.0f, lastBaro_.alt_cm / 100.0f,
                    lastMag_.x_nT100 / 100.0f, lastMag_.y_nT100 / 100.0f, lastMag_.z_nT100 / 100.0f,
                    lastMag_.temp_cC / 100.0f);
    } else if (cmd == "ls") {
      logger_.listSd();
    } else if (cmd == "tail" || cmd.startsWith("tail ")) {
      long n = cmd.length() > 5 ? cmd.substring(5).toInt() : 20;
      if (n <= 0) n = 20;
      if (n > 200) n = 200;
      logger_.tailSd(static_cast<size_t>(n));
    } else if (cmd == "head") {
      logger_.headSd();
    } else if (cmd == "sync") {
      logger_.flushNow();
      Serial.println("[CMD] synced");
    } else if (cmd == "new") {
      prefs.begin(NVS_NAMESPACE, false);
      runId = prefs.getUInt("run", 0) + 1;
      prefs.putUInt("run", runId);
      prefs.end();
      logger_.newRun(runId);
      char msg[48];
      snprintf(msg, sizeof(msg), "NEW RUN %lu (serial)", static_cast<unsigned long>(runId));
      logger_.event(msg);
      Serial.printf("[CMD] new run %lu\n", static_cast<unsigned long>(runId));
    } else if (cmd == "flight") {
      enterFlight_("serial command", false);
    } else if (cmd == "pad") {
      enterLowRate_(Mode::PAD, "serial command");
    } else if (cmd == "arm" || cmd == "disarm") {
      prefs.begin(NVS_NAMESPACE, false);
      prefs.putUChar("flown", 0);
      prefs.end();
      flownFlag_ = false;
      Serial.println("[CMD] flown cleared - next boot starts in PAD mode");
    } else if (cmd == "print") {
      logger_.dumpFlash();
    } else if (cmd == "clear") {
      logger_.wipeFlash();
    } else {
      Serial.print("[CMD] unknown: "); Serial.println(cmd);
      Serial.println("[CMD] type `help` for the command list");
    }

    Serial.setTxTimeoutMs(0);
  }
#endif
}
