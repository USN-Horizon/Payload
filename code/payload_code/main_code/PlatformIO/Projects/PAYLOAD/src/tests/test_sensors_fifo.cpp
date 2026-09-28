// Component test: the production sensor layer of Stack_without_LoRa
// (env:test_sensors_fifo).
//
// Runs the real Sensors class (src/Stack_without_LoRa/Sensors.cpp) exactly
// as the flight firmware configures it and measures what it delivers:
//   - ICM-45686 through the 8 kB hardware FIFO, 20-bit high-resolution
//     packets at IMU_ODR_HZ (400 Hz)
//   - BMP388/390 in continuous (normal) mode, read on data-ready (50 Hz)
//   - BMM350 read on data-ready (100 Hz)
//   - TMP1075 on the LoRa board (optional) and the ESP32 die temperature
//
// PASS criteria (10 s window):
//   - IMU frames/s within 5 % of IMU_ODR_HZ, no bus errors
//   - baro samples/s 45..55, mag samples/s 90..110
//   - accel magnitude 0.8 .. 1.2 g at rest, pressure 700..1100 hPa,
//     mag magnitude 20..80 uT
//
// Upload:  pio run -e test_sensors_fifo -t upload
// Monitor: pio device monitor -e test_sensors_fifo --dtr 1 --rts 1

#include "Stack_without_LoRa/Config.hpp"
#include "Stack_without_LoRa/Sensors.hpp"
#include <math.h>

static Sensors sensors;

static void result(const char* label, bool pass) {
  Serial.printf("  [%s] %s\n", pass ? "PASS" : "FAIL", label);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("========================================");
  Serial.println("  COMPONENT TEST: sensor layer (FIFO / continuous modes)");
  Serial.println("========================================");

  // Same bus bring-up as App::begin (CS pins high before the first transfer,
  // LoRa lines parked, radio rail off).
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
  pinMode(PIN_IMU_CS,  OUTPUT); digitalWrite(PIN_IMU_CS,  HIGH);
  pinMode(PIN_SD_CS,   OUTPUT); digitalWrite(PIN_SD_CS,   HIGH);
  pinMode(PIN_LORA_CS, OUTPUT); digitalWrite(PIN_LORA_CS, HIGH);
  pinMode(PIN_LORA_RST,OUTPUT); digitalWrite(PIN_LORA_RST,LOW);
  pinMode(PIN_LORA_PWR,OUTPUT); digitalWrite(PIN_LORA_PWR,LOW);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);

  sensors.begin();
  result("ICM-45686 init (FIFO configured)", sensors.imuOk());
  result("BMP3 init (normal mode)", sensors.baroOk());
  Serial.printf("  [INFO] baro init trace: %s\n", sensors.baroDiag());
  result("BMM350 init", sensors.magOk());
  Serial.printf("  [INFO] TMP1075 on LoRa board: %s\n", sensors.loraTempOk() ? "present" : "absent");

  // ---- 10 s measurement window --------------------------------------------
  static ImuSample buf[IMU_FIFO_MAX_FRAMES_PER_DRAIN];
  uint32_t imuFrames = 0, baroN = 0, magN = 0;
  uint16_t fifoMax = 0;
  ImuSample lastImu{}; BaroSample lastBaro{}; MagSample lastMag{};
  uint64_t tStart = nowUs(), lastPoll = tStart, lastPrint = tStart;
  uint32_t secFrames = 0;

  while (nowUs() - tStart < 10000000ULL) {
    uint64_t now = nowUs();
    if (now - lastPoll >= IMU_POLL_INTERVAL_US) {
      lastPoll = now;
      uint16_t inFifo = 0;
      int n = sensors.drainImu(buf, IMU_FIFO_MAX_FRAMES_PER_DRAIN, inFifo);
      if (n > 0) { imuFrames += n; secFrames += n; lastImu = buf[n - 1]; }
      if (inFifo > fifoMax) fifoMax = inFifo;
    }
    BaroSample b; if (sensors.pollBaro(now, b)) { baroN++; lastBaro = b; }
    MagSample  m; if (sensors.pollMag(now, m))  { magN++;  lastMag  = m; }
    if (now - lastPrint >= 1000000ULL) {
      lastPrint = now;
      Serial.printf("  [1s] imu=%lu frames  fifo_max=%u  a=%.3f,%.3f,%.3f g  p=%.2f hPa  mag=%.1f,%.1f,%.1f uT\n",
                    static_cast<unsigned long>(secFrames), fifoMax,
                    imuAccelToMgx100(lastImu.ax) / 100000.0f, imuAccelToMgx100(lastImu.ay) / 100000.0f,
                    imuAccelToMgx100(lastImu.az) / 100000.0f, lastBaro.press_Pa_x10 / 1000.0f,
                    lastMag.x_nT100 / 100.0f, lastMag.y_nT100 / 100.0f, lastMag.z_nT100 / 100.0f);
      secFrames = 0;
    }
  }

  float secs = (nowUs() - tStart) / 1e6f;
  float imuRate = imuFrames / secs, baroRate = baroN / secs, magRate = magN / secs;
  float ax = imuAccelToMgx100(lastImu.ax) / 100000.0f, ay = imuAccelToMgx100(lastImu.ay) / 100000.0f,
        az = imuAccelToMgx100(lastImu.az) / 100000.0f;
  float aMag = sqrtf(ax * ax + ay * ay + az * az);
  float mx = lastMag.x_nT100 / 100.0f, my = lastMag.y_nT100 / 100.0f, mz = lastMag.z_nT100 / 100.0f;
  float mMag = sqrtf(mx * mx + my * my + mz * mz);
  float hPa = lastBaro.press_Pa_x10 / 1000.0f;

  Serial.printf("  [INFO] rates: imu %.1f/s (ODR %u)  baro %.1f/s  mag %.1f/s  imu bus errors %lu  fifo max %u frames\n",
                imuRate, IMU_ODR_HZ, baroRate, magRate, static_cast<unsigned long>(sensors.imuErrors()), fifoMax);
  Serial.printf("  [INFO] |a| = %.3f g   p = %.2f hPa, %.2f C   |mag| = %.1f uT   imu die %.2f C\n",
                aMag, hPa, lastBaro.temp_cC / 100.0f, mMag, lastImu.temp_cC / 100.0f);
  int32_t espC = 0, loraC = 0;
  if (sensors.readEspTemp(espC))  Serial.printf("  [INFO] ESP32 die %.2f C\n", espC / 100.0f);
  if (sensors.readLoraTemp(loraC)) Serial.printf("  [INFO] LoRa board %.2f C\n", loraC / 100.0f);

  result("IMU FIFO rate within 5 % of ODR, no bus errors",
         sensors.imuOk() && fabsf(imuRate - IMU_ODR_HZ) <= 0.05f * IMU_ODR_HZ && sensors.imuErrors() == 0);
  result("Baro 45..55 samples/s", sensors.baroOk() && baroRate >= 45 && baroRate <= 55);
  result("Mag 90..110 samples/s", sensors.magOk() && magRate >= 90 && magRate <= 110);
  result("Accel magnitude 0.8..1.2 g at rest", aMag > 0.8f && aMag < 1.2f);
  result("Pressure 700..1100 hPa", hPa > 700 && hPa < 1100);
  result("Mag magnitude 20..80 uT", mMag > 20 && mMag < 80);

  Serial.println("========================================");
  Serial.println("  TEST COMPLETE  -  restarting in 5 s");
  Serial.println("========================================");
  delay(5000);
  ESP.restart();
}

void loop() {}
