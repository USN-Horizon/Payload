#include "Stack_without_LoRa/Logger.hpp"

// =============================================================================
//  Logger.cpp  -  Stack_without_LoRa
// =============================================================================

uint8_t Logger::ring_[LOG_RING_B];
uint8_t Logger::flashRing_[FLASH_RING_B];

static const char* resetReasonName() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "POWERON";
    case ESP_RST_SW:       return "SW";
    case ESP_RST_PANIC:    return "PANIC";
    case ESP_RST_INT_WDT:  return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT:      return "WDT";
    case ESP_RST_DEEPSLEEP:return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO:     return "SDIO";
    default:               return "UNKNOWN";
  }
}

// =============================================================================
//  Header
// =============================================================================
void Logger::writeHeader_(File& f) {
  f.printf("# %s v%s run=%lu part=%u reset=%s imu_hires=%d imu_odr=%u baro_odr_reg=%u mag_odr=100 fsr_a=%ug fsr_g=%udps\n",
           FW_NAME, FW_VERSION, static_cast<unsigned long>(run_), sdPart_,
           resetReasonName(), IMU_FIFO_HIRES, IMU_ODR_HZ, BARO_ODR,
           IMU_ACCEL_FSR_G, IMU_GYRO_FSR_DPS);
  f.println("# t_us = microseconds since boot (esp_timer). Column 1 = record type.");
  f.println("# IMU,t_us,ax_mg,ay_mg,az_mg,gx_dps,gy_dps,gz_dps");
  f.println("# BARO,t_us,press_hPa,temp_C,alt_m");
  f.println("# MAG,t_us,mx_uT,my_uT,mz_uT,temp_C");
  f.println("# GEIG,t_us,ch                      (one row per Geiger pulse, ch = 1|2)");
  f.println("# RATE,t_us,dt_ms,ch1,ch2,ch1_total,ch2_total,ts_dropped");
  f.println("# TEMP,t_us,imu_C,esp_C,lora_C");
  f.println("# STAT,t_us,mode,run,part,uptime_s,loop_hz,imu_frames,fifo_max,imu_errs,sd_ok,sd_bytes,sd_fails,ring_drops,ring_max,flash_bytes,heap_free");
  f.println("# EVT,t_us,text");
}

// =============================================================================
//  begin / newRun
// =============================================================================
void Logger::begin(uint32_t run) {
  run_ = run;
  sdPart_ = 0;
  head_ = tail_ = count_ = 0;

#if USE_SD
  sdOk_ = sdMount_();
  if (sdOk_) {
    if (!sdOpenPart_()) sdFail_("open");
  }
  lastSdMountMs_ = millis();
#endif

#if USE_FLASH
  flashOk_ = LittleFS.begin(true);
  if (flashOk_) {
    const size_t total = LittleFS.totalBytes();
    const size_t used  = LittleFS.usedBytes();
    if (total > 0 && used * 100 / total >= 90) {
#if USE_SERIAL
      Serial.printf("[STOR] LittleFS %u%% full, auto-wiping\n",
                    static_cast<unsigned>(used * 100 / total));
#endif
      flashDeleteRecursive_(FLASH_LOG_DIR);
    }
    LittleFS.mkdir(FLASH_LOG_DIR);
    flashOpenRun_();
#if USE_SERIAL
    Serial.printf("[STOR] LittleFS used=%u total=%u  fallback file: %s\n",
                  static_cast<unsigned>(LittleFS.usedBytes()),
                  static_cast<unsigned>(LittleFS.totalBytes()), flashPath_);
#endif
  } else {
#if USE_SERIAL
    Serial.println("[STOR] LittleFS mount failed");
#endif
  }
#endif
}

void Logger::newRun(uint32_t run) {
  flushNow();
#if USE_SD
  if (sdFile_) sdFile_.close();
#endif
  run_ = run;
  sdPart_ = 0;
#if USE_SD
  if (sdOk_ && !sdOpenPart_()) sdFail_("open");
#endif
#if USE_FLASH
  if (flashOk_) flashOpenRun_();
#endif
}

// =============================================================================
//  SD helpers
// =============================================================================
bool Logger::sdMount_() {
#if USE_SD
  bool ok = SD.begin(PIN_SD_CS, SPI, SD_SPI_HZ);
  if (ok) {
    SD.mkdir(SD_LOG_DIR);
#if USE_SERIAL
    Serial.printf("[STOR] SD mounted: %llu MB, type %d\n",
                  SD.cardSize() / (1024ULL * 1024ULL), static_cast<int>(SD.cardType()));
#endif
  } else {
#if USE_SERIAL
    Serial.println("[STOR] SD mount failed");
#endif
  }
  return ok;
#else
  return false;
#endif
}

bool Logger::sdOpenPart_() {
#if USE_SD
  sdPart_++;
  if (sdPart_ == 1) {
    snprintf(sdPath_, sizeof(sdPath_), "%s/run_%06lu.csv", SD_LOG_DIR,
             static_cast<unsigned long>(run_));
  } else {
    snprintf(sdPath_, sizeof(sdPath_), "%s/run_%06lu_p%u.csv", SD_LOG_DIR,
             static_cast<unsigned long>(run_), sdPart_);
  }
  sdFile_ = SD.open(sdPath_, FILE_APPEND);
  if (!sdFile_) return false;
  writeHeader_(sdFile_);
  sdFile_.flush();
  sdDirty_ = false;
  lastSdWriteMs_ = lastSdFlushMs_ = millis();
#if USE_SERIAL
  Serial.printf("[STOR] SD log: %s\n", sdPath_);
#endif
  return true;
#else
  return false;
#endif
}

void Logger::sdFail_(const char* why) {
#if USE_SD
  sdFails_++;
  sdOk_ = false;
  if (sdFile_) sdFile_.close();
  SD.end();
  lastSdMountMs_ = millis();
#if USE_SERIAL
  Serial.printf("[STOR] SD FAILED (%s) - fallback to flash, remount every %lu s\n",
                why, static_cast<unsigned long>(SD_REMOUNT_INTERVAL_MS / 1000));
#endif
  char msg[48];
  snprintf(msg, sizeof(msg), "SD_FAIL %s", why);
  event(msg);
#else
  (void)why;
#endif
}

void Logger::sdWrite_(uint32_t nowMs) {
#if USE_SD
  if (!sdOk_ || !sdFile_) return;
  size_t pending = count_;
  bool due = pending >= SD_WRITE_CHUNK_B ||
             (pending > 0 && static_cast<uint32_t>(nowMs - lastSdWriteMs_) >= SD_WRITE_INTERVAL_MS);
  if (due) {
    size_t budget = pending < SD_WRITE_MAX_B ? pending : SD_WRITE_MAX_B;
    while (budget > 0) {
      const uint8_t* p;
      size_t n = ringPeek_(p);
      if (n == 0) break;
      if (n > budget) n = budget;
      size_t w = sdFile_.write(p, n);
      if (w != n) { sdFail_("write"); return; }
      ringPop_(n);
      budget -= n;
      sdBytes_ += n;
      sdDirty_ = true;
    }
    lastSdWriteMs_ = nowMs;
  }
  if (sdDirty_ && static_cast<uint32_t>(nowMs - lastSdFlushMs_) >= SD_FLUSH_INTERVAL_MS) {
    sdFile_.flush();
    sdDirty_ = false;
    lastSdFlushMs_ = nowMs;
  }
#else
  (void)nowMs;
#endif
}

void Logger::flushNow() {
#if USE_SD
  if (!sdOk_ || !sdFile_) return;
  // Write everything pending, then fsync.
  while (count_ > 0) {
    const uint8_t* p;
    size_t n = ringPeek_(p);
    if (n == 0) break;
    size_t w = sdFile_.write(p, n);
    if (w != n) { sdFail_("write"); return; }
    ringPop_(n);
    sdBytes_ += n;
  }
  sdFile_.flush();
  sdDirty_ = false;
  lastSdFlushMs_ = lastSdWriteMs_ = millis();
#endif
}

// =============================================================================
//  service
// =============================================================================
void Logger::service(uint32_t nowMs) {
#if USE_SD
  if (sdOk_) {
    sdWrite_(nowMs);
  } else if (static_cast<uint32_t>(nowMs - lastSdMountMs_) >= SD_REMOUNT_INTERVAL_MS) {
    lastSdMountMs_ = nowMs;
    if (sdMount_()) {
      if (sdOpenPart_()) {
        sdOk_ = true;
        event("SD_REMOUNT_OK");
      } else {
        SD.end();
      }
    }
  }
#endif
#if USE_FLASH
  flashWrite_(nowMs);
#endif
}

// =============================================================================
//  push / event
// =============================================================================
void Logger::push(RowType type, const Row& row) {
  if (row.overflowed() || row.size() == 0) return;
  if (!ringPush_(row.data(), row.size())) ringDrops_++;
#if USE_FLASH
  if (flashOk_ && !sdOk_ && flashAccept_(type)) flashPush_(row.data(), row.size());
#endif
}

void Logger::event(const char* text) {
  Row r;
  r.tag("EVT");
  r.cu64(nowUs());
  r.cstr(text);
  r.endl();
  if (!ringPush_(r.data(), r.size())) ringDrops_++;
#if USE_FLASH
  if (flashOk_) flashPush_(r.data(), r.size());
#endif
#if USE_SERIAL
  Serial.printf("[EVT] %s\n", text);
#endif
}

// =============================================================================
//  Ring buffer
// =============================================================================
bool Logger::ringPush_(const char* p, size_t n) {
  if (n > LOG_RING_B - count_) return false;
  size_t first = LOG_RING_B - head_;
  if (first > n) first = n;
  memcpy(ring_ + head_, p, first);
  if (n > first) memcpy(ring_, p + first, n - first);
  head_ = (head_ + n) % LOG_RING_B;
  count_ += n;
  if (count_ > ringHigh_) ringHigh_ = count_;
  return true;
}

size_t Logger::ringPeek_(const uint8_t*& p) const {
  if (count_ == 0) return 0;
  p = ring_ + tail_;
  size_t contiguous = LOG_RING_B - tail_;
  return contiguous < count_ ? contiguous : count_;
}

void Logger::ringPop_(size_t n) {
  tail_ = (tail_ + n) % LOG_RING_B;
  count_ -= n;
}

// =============================================================================
//  LittleFS fallback
// =============================================================================
void Logger::flashOpenRun_() {
#if USE_FLASH
  snprintf(flashPath_, sizeof(flashPath_), "%s/run_%06lu.csv", FLASH_LOG_DIR,
           static_cast<unsigned long>(run_));
  File f = LittleFS.open(flashPath_, FILE_APPEND);
  if (f) {
    if (f.size() == 0) writeHeader_(f);
    f.close();
  }
  flashCount_ = 0;
  memset(flashDiv_, 0, sizeof(flashDiv_));
#endif
}

bool Logger::flashAccept_(RowType t) {
  uint16_t div;
  switch (t) {
    case RowType::IMU:  div = FLASH_IMU_DIVIDER;  break;
    case RowType::BARO: div = FLASH_BARO_DIVIDER; break;
    case RowType::MAG:  div = FLASH_MAG_DIVIDER;  break;
    default:            return true;              // GEIG/RATE/TEMP/STAT/EVT: all.
  }
  uint16_t& c = flashDiv_[static_cast<uint8_t>(t)];
  if (++c >= div) { c = 0; return true; }
  return false;
}

void Logger::flashPush_(const char* p, size_t n) {
  if (n > FLASH_RING_B - flashCount_) return;   // Drop silently; SD is the real log.
  memcpy(flashRing_ + flashCount_, p, n);
  flashCount_ += n;
}

void Logger::flashWrite_(uint32_t nowMs) {
#if USE_FLASH
  if (!flashOk_ || flashCount_ == 0) return;
  bool due = flashCount_ >= FLASH_RING_B / 2 ||
             static_cast<uint32_t>(nowMs - lastFlashWriteMs_) >= FLASH_WRITE_INTERVAL_MS;
  if (!due) return;
  lastFlashWriteMs_ = nowMs;
  if (LittleFS.usedBytes() >= FLASH_QUOTA_B) {
    flashCount_ = 0;
    return;                     // Quota reached: keep the older data.
  }
  File f = LittleFS.open(flashPath_, FILE_APPEND);
  if (f) {
    size_t w = f.write(flashRing_, flashCount_);
    f.close();
    flashBytes_ += w;
  }
  flashCount_ = 0;
#else
  (void)nowMs;
#endif
}

void Logger::flashDeleteRecursive_(const char* path) {
#if USE_FLASH
  File root = LittleFS.open(path);
  if (!root) return;
  if (!root.isDirectory()) { root.close(); LittleFS.remove(path); return; }
  File f = root.openNextFile();
  while (f) {
    String full = f.path() ? String(f.path()) : (String(path) + "/" + f.name());
    if (f.isDirectory()) { f.close(); flashDeleteRecursive_(full.c_str()); }
    else { f.close(); LittleFS.remove(full.c_str()); }
    f = root.openNextFile();
  }
  root.close();
#else
  (void)path;
#endif
}

void Logger::flashPrintRecursive_(const char* path) {
#if USE_FLASH && USE_SERIAL
  File root = LittleFS.open(path);
  if (!root) return;
  if (!root.isDirectory()) { root.close(); return; }
  File f = root.openNextFile();
  while (f) {
    String full = f.path() ? String(f.path()) : (String(path) + "/" + f.name());
    if (f.isDirectory()) { f.close(); flashPrintRecursive_(full.c_str()); }
    else {
      Serial.printf("[FILE] %s (%u b)\n", full.c_str(), static_cast<unsigned>(f.size()));
      uint8_t buf[256];
      while (f.available()) {
        size_t n = f.read(buf, sizeof(buf));
        if (n == 0) break;
        Serial.write(buf, n);
      }
      Serial.println();
      f.close();
    }
    f = root.openNextFile();
  }
  root.close();
#else
  (void)path;
#endif
}

// =============================================================================
//  Serial helpers
// =============================================================================
void Logger::listSd() {
#if USE_SD && USE_SERIAL
  if (!sdOk_) { Serial.println("[SD] not mounted"); return; }
  File dir = SD.open(SD_LOG_DIR);
  if (!dir || !dir.isDirectory()) { Serial.println("[SD] no /logs"); return; }
  // Only the newest entries: a long listing blocks the loop (bench only,
  // but the IMU FIFO holds ~1 s so keep it short).
  constexpr int kShow = 24;
  struct Ent { char name[32]; uint32_t size; };
  static Ent ents[kShow];
  int total = 0;
  File f = dir.openNextFile();
  while (f) {
    if (!f.isDirectory()) {
      Ent& e = ents[total % kShow];
      strncpy(e.name, f.name(), sizeof(e.name) - 1); e.name[sizeof(e.name) - 1] = 0;
      e.size = f.size();
      total++;
    }
    f.close();
    f = dir.openNextFile();
  }
  dir.close();
  Serial.printf("[SD] %s: %d files (card %llu MB, used %llu MB), newest %d:\n", SD_LOG_DIR, total,
                SD.cardSize() / (1024ULL * 1024ULL), SD.usedBytes() / (1024ULL * 1024ULL),
                total < kShow ? total : kShow);
  int n = total < kShow ? total : kShow;
  int start = total < kShow ? 0 : total - kShow;
  for (int i = 0; i < n; ++i) {
    const Ent& e = ents[(start + i) % kShow];
    Serial.printf("  %-28s %10lu b\n", e.name, static_cast<unsigned long>(e.size));
  }
#endif
}

void Logger::tailSd(size_t lines) {
#if USE_SD && USE_SERIAL
  if (!sdOk_) { Serial.println("[SD] not mounted"); return; }
  flushNow();
  File f = SD.open(sdPath_, FILE_READ);
  if (!f) { Serial.println("[SD] cannot open current file"); return; }
  const size_t window = 4096;
  size_t sz = f.size();
  size_t start = sz > window ? sz - window : 0;
  f.seek(start);
  static char buf[window + 1];
  size_t n = f.read(reinterpret_cast<uint8_t*>(buf), window);
  f.close();
  buf[n] = 0;
  // Walk back `lines` newlines from the end.
  size_t pos = n;
  size_t seen = 0;
  while (pos > 0) {
    if (buf[pos - 1] == '\n') {
      if (seen++ == lines) break;
    }
    pos--;
  }
  Serial.printf("[SD] tail %s (%u b total)\n", sdPath_, static_cast<unsigned>(sz));
  Serial.write(reinterpret_cast<uint8_t*>(buf + pos), n - pos);
#else
  (void)lines;
#endif
}

void Logger::headSd() {
#if USE_SD && USE_SERIAL
  if (!sdOk_) { Serial.println("[SD] not mounted"); return; }
  File f = SD.open(sdPath_, FILE_READ);
  if (!f) { Serial.println("[SD] cannot open current file"); return; }
  static char buf[2048];
  size_t n = f.read(reinterpret_cast<uint8_t*>(buf), sizeof(buf));
  f.close();
  Serial.printf("[SD] head %s\n", sdPath_);
  Serial.write(reinterpret_cast<uint8_t*>(buf), n);
  Serial.println();
#endif
}

void Logger::dumpFlash() {
#if USE_FLASH && USE_SERIAL
  if (!flashOk_) { Serial.println("[STOR] flash not mounted"); return; }
  flashWrite_(millis() + FLASH_WRITE_INTERVAL_MS);
  Serial.println("[STOR] BEGIN flash dump");
  flashPrintRecursive_(FLASH_LOG_DIR);
  Serial.println("[STOR] END flash dump");
#endif
}

void Logger::wipeFlash() {
#if USE_FLASH
  if (!flashOk_) return;
  flashCount_ = 0;
  flashDeleteRecursive_(FLASH_LOG_DIR);
  LittleFS.mkdir(FLASH_LOG_DIR);
  flashOpenRun_();
#if USE_SERIAL
  Serial.println("[STOR] flash logs wiped");
#endif
#endif
}
