#pragma once

// =============================================================================
//  Sensors.hpp  -  Stack_without_LoRa
//
//  All sensor access for the SD-only stacks.  Nothing here blocks the loop
//  for longer than one bus transaction:
//    - the IMU is read from its hardware FIFO in bursts,
//    - the barometer runs continuously and is read when it flags data-ready,
//    - the magnetometer is read when it flags data-ready (timed fallback),
//    - TMP1075 (LoRa board) and the ESP32 die temperature are read at 1 Hz.
// =============================================================================

#include "Config.hpp"

// -----------------------------------------------------------------------------
//  Raw IMU sample straight out of the FIFO.  Units depend on IMU_FIFO_HIRES:
//    hires : 20-bit, fixed +/-32 g and +/-4000 dps  (0.0610 mg, 0.00763 dps / LSB)
//    16-bit: +/-IMU_ACCEL_FSR_G, +/-IMU_GYRO_FSR_DPS
//  Use imuAccelToMgx100 / imuGyroToMdps to convert.
// -----------------------------------------------------------------------------
struct ImuSample {
  int32_t ax, ay, az;
  int32_t gx, gy, gz;
  int16_t temp_cC;      // Die temperature in centi-degC, from the same packet.
};

// accel raw -> milli-g * 100   (two decimals of mg)
inline int64_t imuAccelToMgx100(int32_t raw) {
#if IMU_FIFO_HIRES
  // raw * 32 g * 1000 mg * 100 / 2^19
  return (static_cast<int64_t>(raw) * 3200000LL + (1LL << 18)) >> 19;
#else
  return (static_cast<int64_t>(raw) * IMU_ACCEL_FSR_G * 100000LL + (1LL << 14)) >> 15;
#endif
}
// gyro raw -> milli-deg/s  (three decimals of dps)
inline int64_t imuGyroToMdps(int32_t raw) {
#if IMU_FIFO_HIRES
  return (static_cast<int64_t>(raw) * 4000000LL + (1LL << 18)) >> 19;
#else
  return (static_cast<int64_t>(raw) * IMU_GYRO_FSR_DPS * 1000LL + (1LL << 14)) >> 15;
#endif
}
// Raw LSB per g, for the launch detector (squared magnitude in raw units).
inline int64_t imuLsbPerG() {
#if IMU_FIFO_HIRES
  return 524288LL / 32;
#else
  return 32768LL / IMU_ACCEL_FSR_G;
#endif
}

struct BaroSample {
  uint64_t t_us;
  int32_t  press_Pa_x10;   // Pressure in 0.1 Pa  (= hPa * 1000).
  int32_t  temp_cC;        // Temperature in centi-degC.
  int32_t  alt_cm;         // Standard-atmosphere altitude in cm (vs BARO_SEA_LEVEL_HPA).
};

struct MagSample {
  uint64_t t_us;
  int32_t  x_nT100;        // uT * 100  (0.01 uT resolution)
  int32_t  y_nT100;
  int32_t  z_nT100;
  int32_t  temp_cC;
};

#if USE_IMU
// The stock class keeps the driver handle protected; this thin subclass adds
// the FIFO calls the library does not expose (no interrupt pin needed).
class ImuFifo : public ICM456xx {
public:
  using ICM456xx::ICM456xx;
  int  configureFifo();
  int  frameCount(uint16_t& n);
  int  readRaw(uint8_t* buf, size_t len);
  int  flushFifo();
  uint8_t endianness() const { return icm_driver.endianness_data; }
};
#endif

class Sensors {
public:
  // Bring up every enabled driver.  Each one may fail independently.
  void begin();

  // ---- IMU ---------------------------------------------------------------
  bool imuOk() const { return imuOk_; }
  // Read up to maxN frames from the chip FIFO.  `inFifo` receives the frame
  // count seen before the read (lets the caller back-date timestamps and
  // watch how full the FIFO gets).  Returns frames read, or -1 on bus error.
  int  drainImu(ImuSample* out, uint16_t maxN, uint16_t& inFifo);
  uint32_t imuErrors() const { return imuErrs_; }
  uint32_t imuFrameSize() const { return imuFrameSize_; }

  // ---- Baro --------------------------------------------------------------
  bool baroOk() const { return baroOk_; }
  uint8_t baroChipId() const { return baroChipId_; }
  const char* baroDiag() const { return baroDiag_; }   // Init trace for the log / `status`.
  // Polls the data-ready flag; fills `out` and returns true on a new sample.
  bool pollBaro(uint64_t nowUs, BaroSample& out);

  // ---- Mag ---------------------------------------------------------------
  bool magOk() const { return magOk_; }
  bool pollMag(uint64_t nowUs, MagSample& out);

  // ---- Slow temperatures (1 Hz) -----------------------------------------
  bool loraTempOk() const { return tmp1075Ok_; }
  bool readLoraTemp(int32_t& cC);     // TMP1075 on the LoRa board.
  bool readEspTemp(int32_t& cC);      // ESP32-S3 die sensor.

private:
  bool imuOk_      = false;
  bool baroOk_     = false;
  bool magOk_      = false;
  bool tmp1075Ok_  = false;
  uint8_t  baroChipId_   = 0;
  char     baroDiag_[96] = {0};
  uint32_t imuErrs_      = 0;
  uint32_t imuFrameSize_ = 0;
  uint64_t lastBaroPollUs_ = 0;
  uint64_t lastMagPollUs_  = 0;
  uint64_t lastMagSampleUs_ = 0;
};
