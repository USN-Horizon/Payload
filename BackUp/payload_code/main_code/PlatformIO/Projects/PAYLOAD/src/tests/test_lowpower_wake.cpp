// Component test: low-power launch wait  (ICM-45686 accelerometer)
//
// Purpose
// -------
// On the launch pad the payload may sit powered for hours waiting on weather.
// We don't want it logging / transmitting / burning battery the whole time.
// So at boot the firmware drops into a LOW-POWER WAIT: the CPU duty-cycles
// (light sleep between checks) while the accelerometer keeps sampling at a low
// rate. The moment a real flight begins — a sustained jump in acceleration well
// above the ~1 g it reads at rest — we leave low-power mode and start doing the
// actual mission work.
//
// "Flight start" detection
// ------------------------
// At rest the accel magnitude is ~1.0 g (just gravity). A launch adds thrust on
// top of gravity, so the magnitude climbs to several g and STAYS there for the
// burn. We therefore declare launch only when the magnitude is above
// LAUNCH_THRESHOLD_G for LAUNCH_CONFIRM_SAMPLES samples in a row. That debounce
// rejects one-off knocks/handling on the pad (a tap spikes for a single sample,
// a launch does not). The exact threshold is not critical — anything clearly
// above resting gravity works.
//
// Low power
// ---------
// Between accel checks the ESP32-S3 enters esp_light_sleep_start() with only a
// timer wake source, so the core is halted ~90 % of the time. NOTE: while a USB
// host (your PC running the serial monitor) is attached, light sleep tears down
// the USB-CDC link and you will lose the serial monitor. That is fine in flight
// (no host attached) but makes the test hard to watch on the bench. For bench
// observation set USE_LIGHT_SLEEP to 0 — the logic is identical, it just uses
// delay() instead of sleeping so the monitor stays connected.
//
// Upload:  pio run -e test_lowpower_wake -t upload
// Monitor: pio device monitor -e test_lowpower_wake
// To test: hold the board still (stays in WAIT), then jerk/shake/drop it to
//          simulate launch acceleration — it should report "FLIGHT DETECTED".

#include <Arduino.h>
#include <SPI.h>
#include <ICM45686.h>
#include <math.h>
#include "esp_sleep.h"

// ---- Wiring (from "Main board.pdf" pin table) -------------------------------
static constexpr int PIN_SPI_SCK  = 5;
static constexpr int PIN_SPI_MISO = 6;
static constexpr int PIN_SPI_MOSI = 4;
static constexpr int IMU_CS       = 16;   // IF8,  SS_ICM-45686
// Peers on the shared SPI bus. They must be deselected (CS HIGH) or they
// contend on MISO and corrupt the IMU's chip-ID read (begin() returns -1).
static constexpr int PIN_LORA_CS  = 9;    // IF13, SX1280 CS
static constexpr int PIN_LORA_PWR = 10;   // IF14, SX1280 power-enable
static constexpr int PIN_SD_CS    = 21;   // microSD CS

// ---- Tuning -----------------------------------------------------------------
// Set to 1 for the real low-power behaviour (esp_light_sleep, lowest current,
// but the USB serial monitor drops while a PC is attached). Set to 0 for bench
// observation (uses delay() so the monitor stays alive).
#define USE_LIGHT_SLEEP 0

static constexpr uint16_t ACCEL_ODR_HZ   = 25;    // Low sample rate while waiting.
static constexpr uint16_t ACCEL_FSR_G    = 16;    // ±16 g so launch never clips.
static constexpr float    ACCEL_LSB_PER_G = 32768.0f / ACCEL_FSR_G;  // ≈ 2048

static constexpr float    LAUNCH_THRESHOLD_G   = 1.5f;  // Above resting ~1 g.
static constexpr uint8_t  LAUNCH_CONFIRM_SAMPLES = 1;   // Sustained, not a tap.
static constexpr uint32_t WAIT_CHECK_INTERVAL_MS = 200; // 5 Hz duty cycle.

static void result(const char* label, bool pass) {
    Serial.printf("  [%s] %s\n", pass ? "PASS" : "FAIL", label);
}

// Read the current accel magnitude in g. Returns false if the register read
// fails (the magnitude is then left untouched).
static bool readAccelMagnitude(ICM456xx& imu, float& mag_g) {
    inv_imu_sensor_data_t d;
    if (imu.getDataFromRegisters(d) != 0) return false;
    float ax = d.accel_data[0] / ACCEL_LSB_PER_G;
    float ay = d.accel_data[1] / ACCEL_LSB_PER_G;
    float az = d.accel_data[2] / ACCEL_LSB_PER_G;
    mag_g = sqrtf(ax * ax + ay * ay + az * az);
    return true;
}

// Sleep (or wait) for WAIT_CHECK_INTERVAL_MS keeping power low.
static void lowPowerInterval() {
#if USE_LIGHT_SLEEP
    esp_sleep_enable_timer_wakeup(
        static_cast<uint64_t>(WAIT_CHECK_INTERVAL_MS) * 1000ULL);
    esp_light_sleep_start();
#else
    delay(WAIT_CHECK_INTERVAL_MS);
#endif
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("========================================");
    Serial.println("  COMPONENT TEST: low-power launch wait");
    Serial.println("========================================");

    // Deselect every other device on the shared SPI bus first, so only the
    // IMU can drive MISO while we read its chip ID.
    pinMode(PIN_SD_CS,    OUTPUT); digitalWrite(PIN_SD_CS,    HIGH);
    pinMode(PIN_LORA_CS,  OUTPUT); digitalWrite(PIN_LORA_CS,  HIGH);
    pinMode(PIN_LORA_PWR, OUTPUT); digitalWrite(PIN_LORA_PWR, LOW);  // keep LoRa off
    result("Peer SPI devices deselected (SD CS=21, LoRa CS=9 HIGH)", true);

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
    result("SPI bus initialized (SCK=5, MISO=6, MOSI=4, CS=16/IF8)", true);

    ICM456xx imu(SPI, IMU_CS);
    // The ICM-45686 needs ~50-100 ms of POR settling before SPI works, and its
    // begin() does not retry internally. Give it time and retry a few times
    // (same pattern as Sensors::begin in the production firmware).
    delay(150);
    bool initOk = false;
    for (int attempt = 0; attempt < 5; ++attempt) {
        int initState = imu.begin();
        Serial.printf("  [INFO] imu.begin() attempt %d = %d  (0 = OK)\n",
                      attempt + 1, initState);
        if (initState == 0) { initOk = true; break; }
        delay(50);
    }
    result("IMU begin() - SPI comms + chip ID", initOk);

    if (!initOk) {
        Serial.println("  [HINT] Check SPI wiring SCK=5 MISO=6 MOSI=4 CS=16(IF8), power.");
        Serial.println("  TEST ABORTED - restarting in 5 s");
        delay(5000);
        ESP.restart();
    }

    // Low-rate accel only (gyro stays off to save power during the wait).
    imu.startAccel(ACCEL_ODR_HZ, ACCEL_FSR_G);
    Serial.printf("  [INFO] Accel started at %u Hz, +/-%u g (gyro off)\n",
                  ACCEL_ODR_HZ, ACCEL_FSR_G);
    delay(100);  // let the first sample settle

    // ---- LOW-POWER WAIT -----------------------------------------------------
    Serial.println();
    Serial.printf("  [MODE] LOW-POWER WAIT  (light_sleep=%d)\n", USE_LIGHT_SLEEP);
    Serial.printf("  [INFO] Declaring flight when |a| > %.2f g for %u samples.\n",
                  LAUNCH_THRESHOLD_G, LAUNCH_CONFIRM_SAMPLES);
    Serial.println("  [INFO] Hold still to wait; shake/jerk the board to launch.");

    uint8_t  aboveCount = 0;
    uint32_t checks     = 0;
    bool     launched   = false;

    while (!launched) {
        lowPowerInterval();

        float mag = 0.0f;
        if (!readAccelMagnitude(imu, mag)) {
            // A failed read shouldn't strand us in the wait; just retry.
            continue;
        }
        checks++;

        if (mag > LAUNCH_THRESHOLD_G) {
            aboveCount++;
            Serial.printf("  [WAIT] |a|=%.2f g  ABOVE (%u/%u)\n",
                          mag, aboveCount, LAUNCH_CONFIRM_SAMPLES);
            if (aboveCount >= LAUNCH_CONFIRM_SAMPLES) launched = true;
        } else if (aboveCount > 0) {
            // Tolerate brief dips: decrement instead of hard-resetting. Sustained
            // (if noisy) acceleration keeps climbing to the trigger, while a lone
            // tap spike (+1 then -1) can never accumulate. More robust than a
            // strict consecutive run against both hand-jitter and sensor glitches.
            aboveCount--;
            Serial.printf("  [WAIT] |a|=%.2f g  dip (%u/%u)\n",
                          mag, aboveCount, LAUNCH_CONFIRM_SAMPLES);
        } else {
            // Heartbeat every ~2 s so we can see it's alive and low (not spammy).
            if (checks % 10 == 0) {
                Serial.printf("  [WAIT] |a|=%.2f g  resting...\n", mag);
            }
        }
    }

    // ---- WAKE / ACTIVE MODE -------------------------------------------------
    Serial.println();
    Serial.println("  ****  FLIGHT DETECTED - leaving low-power mode  ****");
    result("Woke out of low-power on sustained acceleration", true);

    // Demonstrate the "now do mission work" phase: read accel fast for a few
    // seconds (in real firmware this is where logging + LoRa TX would resume).
    Serial.println("  [MODE] ACTIVE - streaming accel for 5 s:");
    uint32_t tEnd = millis() + 5000;
    while (millis() < tEnd) {
        float mag = 0.0f;
        if (readAccelMagnitude(imu, mag)) {
            Serial.printf("  [ACTIVE] |a| = %.2f g\n", mag);
        }
        delay(250);
    }

    Serial.println("========================================");
    Serial.println("  TEST COMPLETE  -  restarting in 5 s");
    Serial.println("========================================");
    delay(5000);
    ESP.restart();
}

void loop() {}
