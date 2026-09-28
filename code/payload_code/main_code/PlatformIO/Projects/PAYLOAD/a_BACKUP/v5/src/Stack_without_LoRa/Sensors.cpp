#include "Stack_without_LoRa/Sensors.hpp"
#include <math.h>

// =============================================================================
//  Sensors.cpp  -  Stack_without_LoRa
// =============================================================================

#if USE_IMU
static ImuFifo s_imu(SPI, PIN_IMU_CS, IMU_SPI_HZ);
#endif
#if USE_MAG
static DFRobot_BMM350_I2C s_bmm(&Wire, I2C_ADDR_BMM350);
#endif
#if USE_BARO
static struct bmp3_dev s_bmp;
#endif

static bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static int16_t clamp16(int32_t v) {
  if (v >  32767) return  32767;
  if (v < -32768) return -32768;
  return static_cast<int16_t>(v);
}

// -----------------------------------------------------------------------------
//  ImuFifo  -  FIFO access on top of the TDK driver.
// -----------------------------------------------------------------------------
#if USE_IMU
int ImuFifo::configureFifo() {
  inv_imu_fifo_config_t cfg;
  cfg.gyro_en    = INV_IMU_ENABLE;
  cfg.accel_en   = INV_IMU_ENABLE;
  cfg.hires_en   = IMU_FIFO_HIRES ? INV_IMU_ENABLE : INV_IMU_DISABLE;
  cfg.fifo_wm_th = 1;
  cfg.fifo_mode  = FIFO_CONFIG0_FIFO_MODE_SNAPSHOT;   // Stop when full, never corrupt.
  cfg.fifo_depth = FIFO_CONFIG0_FIFO_DEPTH_MAX;       // 8 kB (APEX off).
  return inv_imu_set_fifo_config(&icm_driver, &cfg);
}
int ImuFifo::frameCount(uint16_t& n) { return inv_imu_get_frame_count(&icm_driver, &n); }
int ImuFifo::readRaw(uint8_t* buf, size_t len) {
  return inv_imu_read_reg(&icm_driver, FIFO_DATA, static_cast<uint32_t>(len), buf);
}
int ImuFifo::flushFifo() { return inv_imu_flush_fifo(&icm_driver); }
#endif

// -----------------------------------------------------------------------------
//  BMP3 I2C glue for the Bosch driver (Wire based, no Adafruit wrapper so we
//  can run NORMAL mode and poll data-ready instead of blocking).
// -----------------------------------------------------------------------------
#if USE_BARO
static int8_t bmpRead(uint8_t reg, uint8_t* data, uint32_t len, void*) {
  Wire.beginTransmission(I2C_ADDR_BMP3XX);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(static_cast<int>(I2C_ADDR_BMP3XX), static_cast<int>(len)) != static_cast<int>(len)) return -1;
  for (uint32_t i = 0; i < len; ++i) data[i] = static_cast<uint8_t>(Wire.read());
  return 0;
}
static int8_t bmpWrite(uint8_t reg, const uint8_t* data, uint32_t len, void*) {
  Wire.beginTransmission(I2C_ADDR_BMP3XX);
  Wire.write(reg);
  Wire.write(data, len);
  return Wire.endTransmission() == 0 ? 0 : -1;
}
static void bmpDelayUs(uint32_t us, void*) { delayMicroseconds(us); }
#endif

// =============================================================================
//  begin
// =============================================================================
void Sensors::begin() {
#if USE_IMU
  // ~50-100 ms POR settling; begin() does not retry by itself.
  delay(150);
  imuOk_ = false;
  for (int attempt = 0; attempt < 5 && !imuOk_; ++attempt) {
    if (s_imu.begin() == 0) imuOk_ = true;
    else {
#if USE_SERIAL
      Serial.printf("[SENS] ICM-45686 begin() attempt %d failed\n", attempt + 1);
#endif
      delay(50);
    }
  }
  if (imuOk_) {
    int rc = 0;
    rc |= s_imu.startAccel(IMU_ODR_HZ, IMU_ACCEL_FSR_G);
    rc |= s_imu.startGyro(IMU_ODR_HZ, IMU_GYRO_FSR_DPS);
    rc |= s_imu.configureFifo();
    imuFrameSize_ = IMU_FIFO_HIRES ? 20 : 16;
    if (rc != 0) {
      imuOk_ = false;
#if USE_SERIAL
      Serial.printf("[SENS] ICM-45686 config failed rc=%d\n", rc);
#endif
    } else {
#if USE_SERIAL
      Serial.printf("[SENS] ICM-45686 OK: %u Hz, FIFO %s, frame %u B\n",
                    IMU_ODR_HZ, IMU_FIFO_HIRES ? "20-bit hires" : "16-bit",
                    static_cast<unsigned>(imuFrameSize_));
#endif
    }
  } else {
#if USE_SERIAL
    Serial.println("[SENS] ICM-45686 init failed (after retries)");
#endif
  }
#endif

#if USE_BARO
  memset(&s_bmp, 0, sizeof(s_bmp));
  s_bmp.intf       = BMP3_I2C_INTF;
  s_bmp.intf_ptr   = &s_bmp;     // Driver's null_ptr_check rejects NULL; value unused by our callbacks.
  s_bmp.read       = bmpRead;
  s_bmp.write      = bmpWrite;
  s_bmp.delay_us   = bmpDelayUs;
  s_bmp.dummy_byte = 0;
  baroOk_ = false;
  {
    // Raw probe first so the log tells exactly where an init failure sits.
    bool present = i2cPresent(I2C_ADDR_BMP3XX);
    uint8_t rawId = 0xFF, rawStatus = 0xFF;
    int8_t rProbe = bmpRead(BMP3_REG_CHIP_ID, &rawId, 1, nullptr);
    int8_t rStat  = bmpRead(BMP3_REG_SENS_STATUS, &rawStatus, 1, nullptr);
    int8_t rInit = -99, rSet = -99, rMode = -99;
    // NOTE: the BMP388 NACKs an empty (address-only) write, so `present` is
    // informational only; the chip-id read is the real presence test.
    if (rProbe == 0 && (rawId == BMP3_CHIP_ID || rawId == BMP390_CHIP_ID)) {
      rInit = bmp3_init(&s_bmp);           // Reads chip id, soft-resets, loads calibration.
      if (rInit == BMP3_OK) {
        baroChipId_ = s_bmp.chip_id;
        s_bmp.settings.press_en = BMP3_ENABLE;
        s_bmp.settings.temp_en  = BMP3_ENABLE;
        s_bmp.settings.odr_filter.press_os   = BARO_PRESS_OS;
        s_bmp.settings.odr_filter.temp_os    = BARO_TEMP_OS;
        s_bmp.settings.odr_filter.iir_filter = BARO_IIR;
        s_bmp.settings.odr_filter.odr        = BARO_ODR;
        const uint32_t sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS |
                             BMP3_SEL_TEMP_OS | BMP3_SEL_IIR_FILTER | BMP3_SEL_ODR;
        rSet = bmp3_set_sensor_settings(sel, &s_bmp);
        if (rSet == BMP3_E_INVALID_ODR_OSR_SETTINGS) {
          // Oversampling too slow for the ODR: drop to 25 Hz rather than fail.
          s_bmp.settings.odr_filter.odr = BMP3_ODR_25_HZ;
          rSet = bmp3_set_sensor_settings(sel, &s_bmp);
        }
        if (rSet == BMP3_OK) {
          s_bmp.settings.op_mode = BMP3_MODE_NORMAL;
          rMode = bmp3_set_op_mode(&s_bmp);
        }
        baroOk_ = (rSet == BMP3_OK && rMode == BMP3_OK);
      }
    }
    snprintf(baroDiag_, sizeof(baroDiag_),
             "present=%d probe=%d id=0x%02X status=%d/0x%02X init=%d set=%d mode=%d odr=%u",
             present ? 1 : 0, rProbe, rawId, rStat, rawStatus, rInit, rSet, rMode,
             s_bmp.settings.odr_filter.odr);
#if USE_SERIAL
    Serial.printf("[SENS] BMP3 %s (%s)\n", baroOk_ ? "OK, normal mode" : "FAILED", baroDiag_);
#endif
  }
#endif

#if USE_MAG
  magOk_ = (s_bmm.begin() == 0);
  if (magOk_) {
    // Set ODR/averaging while still suspended, then switch to normal mode
    // (the chip powers up suspended and would return garbage otherwise).
    s_bmm.setPresetMode(MAG_PRESET, MAG_RATE);
    // The INT pin is not used (the flag is polled over I2C), but enabling the
    // data-ready interrupt is what makes INT_STATUS.drdy_data_reg flag new
    // samples; without it the mag delivered only ~33 Hz.
    s_bmm.setDataReadyPin(BMM350_ENABLE_INTERRUPT, BMM350_ACTIVE_HIGH);
    s_bmm.setOperationMode(eBmm350NormalMode);
    delay(50);
#if USE_SERIAL
    Serial.printf("[SENS] BMM350 OK, %.1f Hz\n", s_bmm.getRate());
#endif
  } else {
#if USE_SERIAL
    Serial.println("[SENS] BMM350 init failed");
#endif
  }
#endif

#if USE_LORA_TEMP
  tmp1075Ok_ = i2cPresent(I2C_ADDR_TMP1075);
#if USE_SERIAL
  Serial.println(tmp1075Ok_ ? "[SENS] TMP1075 present (LoRa board)"
                            : "[SENS] TMP1075 absent (ok, no LoRa board temp)");
#endif
#endif
}

// =============================================================================
//  drainImu  -  burst-read the chip FIFO and decode packets.
//  Packet layout mirrors inv_imu_get_fifo_frame() in the TDK driver.
// =============================================================================
int Sensors::drainImu(ImuSample* out, uint16_t maxN, uint16_t& inFifo) {
  inFifo = 0;
#if USE_IMU
  if (!imuOk_) return 0;

  uint16_t count = 0;
  if (s_imu.frameCount(count) != 0) { imuErrs_++; return -1; }
  inFifo = count;
  if (count == 0) return 0;
  if (count > maxN) count = maxN;

  static uint8_t buf[IMU_FIFO_MAX_FRAMES_PER_DRAIN * 20];
  const size_t fs = imuFrameSize_;
  if (count * fs > sizeof(buf)) count = sizeof(buf) / fs;
  if (s_imu.readRaw(buf, count * fs) != 0) { imuErrs_++; return -1; }

  const uint8_t be = s_imu.endianness();
  auto rd16 = [be](const uint8_t* p) -> int16_t {
    uint16_t v = be ? static_cast<uint16_t>((p[0] << 8) | p[1])
                    : static_cast<uint16_t>((p[1] << 8) | p[0]);
    return static_cast<int16_t>(v);
  };

  int n = 0;
  for (uint16_t i = 0; i < count; ++i) {
    const uint8_t* f = buf + i * fs;
    const uint8_t hdr = f[0];
    const bool accelBit = hdr & 0x40;
    const bool gyroBit  = hdr & 0x20;
    if (!accelBit || !gyroBit) { imuErrs_++; continue; }   // Not a full packet.

    ImuSample& s = out[n];
    if (fs == 20) {
      int16_t r0 = rd16(f + 1), r1 = rd16(f + 3), r2 = rd16(f + 5);
      int16_t g0 = rd16(f + 7), g1 = rd16(f + 9), g2 = rd16(f + 11);
      s.ax = (static_cast<int32_t>(r0) << 4) | (f[17] >> 4);
      s.ay = (static_cast<int32_t>(r1) << 4) | (f[18] >> 4);
      s.az = (static_cast<int32_t>(r2) << 4) | (f[19] >> 4);
      s.gx = (static_cast<int32_t>(g0) << 4) | (f[17] & 0x0F);
      s.gy = (static_cast<int32_t>(g1) << 4) | (f[18] & 0x0F);
      s.gz = (static_cast<int32_t>(g2) << 4) | (f[19] & 0x0F);
      int16_t t16 = rd16(f + 13);
      s.temp_cC = clamp16(static_cast<int32_t>(t16) * 100 / 128 + 2500);
    } else {
      s.ax = rd16(f + 1); s.ay = rd16(f + 3);  s.az = rd16(f + 5);
      s.gx = rd16(f + 7); s.gy = rd16(f + 9);  s.gz = rd16(f + 11);
      int8_t t8 = static_cast<int8_t>(f[13]);
      s.temp_cC = static_cast<int16_t>(t8 * 50 + 2500);
    }
    n++;
  }
  return n;
#else
  (void)out; (void)maxN;
  return 0;
#endif
}

// =============================================================================
//  pollBaro  -  read when the chip says a new pressure sample is ready.
// =============================================================================
bool Sensors::pollBaro(uint64_t nowUs, BaroSample& out) {
#if USE_BARO
  if (!baroOk_) return false;
  if (nowUs - lastBaroPollUs_ < BARO_POLL_INTERVAL_US) return false;
  lastBaroPollUs_ = nowUs;

  uint8_t st = 0;
  if (bmp3_get_regs(BMP3_REG_SENS_STATUS, &st, 1, &s_bmp) != BMP3_OK) return false;
  if ((st & BMP3_DRDY_PRESS) == 0) return false;

  struct bmp3_data d;
  if (bmp3_get_sensor_data(BMP3_ALL, &d, &s_bmp) != BMP3_OK) return false;

  out.t_us = nowUs;
  out.press_Pa_x10 = static_cast<int32_t>(lround(d.pressure * 10.0));
  out.temp_cC      = static_cast<int32_t>(lround(d.temperature * 100.0));
  float hPa = static_cast<float>(d.pressure) / 100.0f;
  float alt = 44330.0f * (1.0f - powf(hPa / BARO_SEA_LEVEL_HPA, 0.1903f));
  out.alt_cm = static_cast<int32_t>(lroundf(alt * 100.0f));
  return true;
#else
  (void)nowUs; (void)out;
  return false;
#endif
}

// =============================================================================
//  pollMag  -  read on data-ready; timed fallback if the flag never shows.
// =============================================================================
bool Sensors::pollMag(uint64_t nowUs, MagSample& out) {
#if USE_MAG
  if (!magOk_) return false;
  if (nowUs - lastMagPollUs_ < BARO_POLL_INTERVAL_US) return false;   // ~2 ms cadence.
  lastMagPollUs_ = nowUs;

  bool ready = s_bmm.getDataReadyState();
  // Fallback: if data-ready does not fire, read once per ODR period anyway.
  if (!ready && (nowUs - lastMagSampleUs_) < MAG_INTERVAL_US) return false;

  sBmm350MagData_t m = s_bmm.getGeomagneticData();
  lastMagSampleUs_ = nowUs;
  out.t_us    = nowUs;
  out.x_nT100 = static_cast<int32_t>(lroundf(m.float_x * 100.0f));
  out.y_nT100 = static_cast<int32_t>(lroundf(m.float_y * 100.0f));
  out.z_nT100 = static_cast<int32_t>(lroundf(m.float_z * 100.0f));
  out.temp_cC = static_cast<int32_t>(lroundf(m.float_temperature * 100.0f));
  return true;
#else
  (void)nowUs; (void)out;
  return false;
#endif
}

// =============================================================================
//  Slow temperatures
// =============================================================================
bool Sensors::readLoraTemp(int32_t& cC) {
#if USE_LORA_TEMP
  if (!tmp1075Ok_) return false;
  Wire.beginTransmission(I2C_ADDR_TMP1075);
  Wire.write(static_cast<uint8_t>(0x00));
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(static_cast<int>(I2C_ADDR_TMP1075), 2) != 2) return false;
  uint8_t hi = Wire.read(), lo = Wire.read();
  int16_t raw = static_cast<int16_t>((hi << 8) | lo);
  int16_t v12 = raw >> 4;                         // 0.0625 degC / LSB
  cC = static_cast<int32_t>(v12) * 625 / 100;
  return true;
#else
  (void)cC;
  return false;
#endif
}

bool Sensors::readEspTemp(int32_t& cC) {
#if USE_ESP_TEMP
  float t = temperatureRead();
  if (isnan(t)) return false;
  cC = static_cast<int32_t>(lroundf(t * 100.0f));
  return true;
#else
  (void)cC;
  return false;
#endif
}
