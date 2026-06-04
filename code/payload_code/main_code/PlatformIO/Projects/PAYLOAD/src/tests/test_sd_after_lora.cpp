// Component test: which difference between production LoraTx::begin()
// and the working test stage [D] causes SD.begin() to fail right after
// LoRa init?
//
// Each variation:
//   1. Hard-resets the SX1280 (PWR off + RST low + wait + PWR on)
//   2. Reinitializes the radio with one variable changed
//   3. Tries SD.begin() and reports PASS/FAIL
//   4. Tears down (SD.end + radio.reset)
//
// Variations:
//   [A] Baseline: production verbatim - 8 MHz Module SPI, SF=9, CR=7,
//       setCRC+setWhitening, NO explicit standby() at end.
//   [B] Baseline + explicit radio.standby() after the setters.
//   [C] Baseline + 500 ms delay before SD.begin (rules out warm-up time).
//   [D] Module SPI at 2 MHz instead of 8 MHz, otherwise production.
//   [E] SF=7 / CR=5 (matches the working test [D]), otherwise production.
//   [F] Skip setCRC + setWhitening, otherwise production.
//
// PASS criteria: variation B is the one expected to pass if the hypothesis
// "missing radio.standby()" is right. If a different variation passes
// uniquely, that one identifies the real root cause.
//
// Upload:  pio run -e test_sd_after_lora -t upload
// Monitor: pio device monitor -e test_sd_after_lora

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Preferences.h>
#include <RadioLib.h>
#include <SD.h>

static constexpr int PIN_I2C_SDA = 7;
static constexpr int PIN_I2C_SCL = 15;
static constexpr uint8_t I2C_ADDR_DAC43401 = 0x48;
static constexpr uint8_t I2C_ADDR_TMP1075  = 0x49;

static constexpr int PIN_SPI_MOSI  = 4;
static constexpr int PIN_SPI_SCK   = 5;
static constexpr int PIN_SPI_MISO  = 6;

static constexpr int PIN_LORA_CS   = 9;
static constexpr int PIN_LORA_PWR  = 10;
static constexpr int PIN_LORA_DIO1 = 11;
static constexpr int PIN_LORA_BUSY = 13;
static constexpr int PIN_LORA_RST  = 14;

static constexpr int PIN_SD_CS     = 21;

static int passCount = 0;
static int failCount = 0;

static void result(const char* label, bool pass) {
    Serial.printf("  [%s] %s\n", pass ? "PASS" : "FAIL", label);
    if (pass) passCount++; else failCount++;
}

// Hard-reset the SX1280 between variations: drop power, hold RST low for
// long enough for the chip to fully release, then bring power back.
static void coldResetLora() {
    pinMode(PIN_LORA_CS,  OUTPUT); digitalWrite(PIN_LORA_CS,  HIGH);
    pinMode(PIN_LORA_PWR, OUTPUT); digitalWrite(PIN_LORA_PWR, LOW);
    pinMode(PIN_LORA_RST, OUTPUT); digitalWrite(PIN_LORA_RST, LOW);
    delay(50);
    digitalWrite(PIN_LORA_PWR, HIGH);
    delay(10);
    // Let RadioLib's begin() drive RST; we just leave PWR HIGH.
}

// Configure the radio the way production does, with one variable changed.
// Returns true if begin() succeeded.
static bool initLora(SX1280& radio,
                     uint8_t sf, uint8_t cr,
                     bool useCrcWhitening,
                     bool callStandby) {
    int s = radio.begin();
    if (s != RADIOLIB_ERR_NONE) {
        Serial.printf("  [INFO] radio.begin = %d\n", s);
        return false;
    }
    radio.setFrequency(2400.0f);
    radio.setBandwidth(812.5f);
    radio.setSpreadingFactor(sf);
    radio.setCodingRate(cr);
    radio.setOutputPower(13);
    if (useCrcWhitening) {
        radio.setCRC(true);
        radio.setWhitening(true);
    }
    if (callStandby) {
        int sb = radio.standby();
        Serial.printf("  [INFO] radio.standby = %d\n", sb);
    }
    return true;
}

static bool tryMountSd() {
    bool ok = SD.begin(PIN_SD_CS, SPI, 8000000);
    if (ok) SD.end();
    return ok;
}

// Run one variation. Each variation gets a fresh Module so the SPISettings
// stored on the Module object can vary.
struct Variation {
    const char* label;
    uint32_t    moduleSpiHz;
    uint8_t     sf;
    uint8_t     cr;
    bool        useCrcWhitening;
    bool        callStandby;
    uint32_t    postInitDelayMs;
};

static void runVariation(const Variation& v) {
    Serial.printf("\n--- %s ---\n", v.label);
    Serial.printf("  [INFO] moduleSpiHz=%lu sf=%u cr=%u crc/whiten=%d "
                  "standby=%d postDelay=%lu ms\n",
                  (unsigned long)v.moduleSpiHz,
                  (unsigned)v.sf, (unsigned)v.cr,
                  v.useCrcWhitening ? 1 : 0,
                  v.callStandby ? 1 : 0,
                  (unsigned long)v.postInitDelayMs);

    coldResetLora();

    // Fresh Module so SPISettings reflects this variation.
    Module* mod = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RST,
                             PIN_LORA_BUSY, SPI,
                             SPISettings(v.moduleSpiHz, MSBFIRST, SPI_MODE0));
    SX1280 radio(mod);

    bool ok = initLora(radio, v.sf, v.cr, v.useCrcWhitening, v.callStandby);
    if (!ok) {
        result(v.label, false);
        delete mod;
        return;
    }

    if (v.postInitDelayMs) delay(v.postInitDelayMs);

    uint32_t tBefore = millis();
    bool sdOk = tryMountSd();
    Serial.printf("  [INFO] SD.begin at boot t=%lu ms = %s\n",
                  (unsigned long)tBefore, sdOk ? "OK" : "FAIL");
    result(v.label, sdOk);

    delete mod;
}

void setup() {
    Serial.begin(115200);
    // Match production timing: App::begin() only spins ~200 ms before
    // SPI.begin. The previous test had delay(800) here which was masking
    // a timing-related cause - removed.
    delay(100);

    Serial.println();
    Serial.println("==================================================");
    Serial.println("  TEST: SD.begin after LoRa init - variation sweep");
    Serial.printf("  (boot t=%lu ms when test starts)\n",
                  (unsigned long)millis());
    Serial.println("==================================================");

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
    pinMode(PIN_SD_CS, OUTPUT); digitalWrite(PIN_SD_CS, HIGH);

    // [A] Production verbatim.
    runVariation({
        "[A] baseline (production verbatim)",
        /*moduleSpiHz*/ 8000000,
        /*sf*/ 9, /*cr*/ 7,
        /*useCrcWhitening*/ true,
        /*callStandby*/ false,
        /*postInitDelayMs*/ 0
    });

    // [B] + explicit standby() at end.
    runVariation({
        "[B] baseline + radio.standby()",
        8000000, 9, 7, true, true, 0
    });

    // [C] + 500 ms delay between init and SD mount.
    runVariation({
        "[C] baseline + 500 ms delay",
        8000000, 9, 7, true, false, 500
    });

    // [D] Module SPI at 2 MHz instead of 8 MHz.
    runVariation({
        "[D] Module SPI 2 MHz (not 8 MHz)",
        2000000, 9, 7, true, false, 0
    });

    // [E] SF=7 / CR=5 (matches the working test).
    runVariation({
        "[E] SF=7 CR=5 (working-test settings)",
        8000000, 7, 5, true, false, 0
    });

    // [F] Skip setCRC + setWhitening.
    runVariation({
        "[F] no setCRC/setWhitening",
        8000000, 9, 7, false, false, 0
    });

    // [H] Production mimic + actual DAC43401 write to 3300 mV. This is the
    // last production-only step the test was skipping. If [H] fails while
    // [G] passes, setting the LoRa PA bias rail to 3.3 V is what breaks
    // the SD card mount (likely a brown-out from the inrush current when
    // the PA bias caps charge).
    {
        Serial.println("\n--- [H] production mimic + DAC set to 3300 mV ---");
        coldResetLora();

        Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

        // DAC43401 unlock + general-config clear (matches Sensors.cpp).
        Wire.beginTransmission(I2C_ADDR_DAC43401);
        Wire.write(uint8_t(0x36));
        Wire.write(uint8_t(0x50));
        Wire.write(uint8_t(0x00));
        Wire.endTransmission();
        delay(2);
        Wire.beginTransmission(I2C_ADDR_DAC43401);
        Wire.write(uint8_t(0x09));
        Wire.write(uint8_t(0x00));
        Wire.write(uint8_t(0x00));
        Wire.endTransmission();
        delay(2);

        // Write DAC_DATA (0x21) to set output to 3300 mV (code 0xFF, reg 0x0FF0).
        Wire.beginTransmission(I2C_ADDR_DAC43401);
        Wire.write(uint8_t(0x21));
        Wire.write(uint8_t(0x0F));
        Wire.write(uint8_t(0xF0));
        int eD = Wire.endTransmission();
        Serial.printf("  [INFO] DAC43401 set 3300 mV write: endTransmission=%d\n",
                      eD);

        Module* mod = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RST,
                                 PIN_LORA_BUSY, SPI,
                                 SPISettings(8000000, MSBFIRST, SPI_MODE0));
        SX1280 radio(mod);
        bool initOk = initLora(radio, 9, 7, true, false);
        if (!initOk) {
            result("[H] production mimic + DAC 3300 mV", false);
        } else {
            uint32_t tBefore = millis();
            bool sdOk = tryMountSd();
            Serial.printf("  [INFO] SD.begin at boot t=%lu ms = %s\n",
                          (unsigned long)tBefore, sdOk ? "OK" : "FAIL");
            result("[H] production mimic + DAC 3300 mV", sdOk);
        }
        delete mod;
    }

    // [I] Full production mimic: NVS prefs write + full I2C/DAC sequence +
    // LoRa init, then SD. This is everything App::begin() does in order,
    // minus calls to drivers we don't link in (BMP/BMM/IMU all USE_x=0).
    // If [I] FAILS while [A]-[H] PASS, the NVS write is the last piece
    // that triggers the SD failure.
    {
        Serial.println("\n--- [I] full production mimic (NVS + I2C + DAC + LoRa) ---");
        coldResetLora();

        // NVS read+write (matches App::begin lines 41-44).
        Preferences testPrefs;
        testPrefs.begin("test_sd", false);
        uint32_t bootCount = testPrefs.getUInt("boot", 0) + 1;
        testPrefs.putUInt("boot", bootCount);
        testPrefs.end();
        Serial.printf("  [INFO] NVS bootCount=%u\n", static_cast<unsigned>(bootCount));

        Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

        // Probe TMP1075 and DAC43401 (matches sensors_.begin).
        Wire.beginTransmission(I2C_ADDR_TMP1075);
        Wire.endTransmission();
        Wire.beginTransmission(I2C_ADDR_DAC43401);
        Wire.endTransmission();

        // DAC unlock + clear + 3300 mV setpoint.
        Wire.beginTransmission(I2C_ADDR_DAC43401);
        Wire.write(uint8_t(0x36));
        Wire.write(uint8_t(0x50));
        Wire.write(uint8_t(0x00));
        Wire.endTransmission();
        delay(2);
        Wire.beginTransmission(I2C_ADDR_DAC43401);
        Wire.write(uint8_t(0x09));
        Wire.write(uint8_t(0x00));
        Wire.write(uint8_t(0x00));
        Wire.endTransmission();
        delay(2);
        Wire.beginTransmission(I2C_ADDR_DAC43401);
        Wire.write(uint8_t(0x21));
        Wire.write(uint8_t(0x0F));
        Wire.write(uint8_t(0xF0));
        Wire.endTransmission();

        Module* mod = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RST,
                                 PIN_LORA_BUSY, SPI,
                                 SPISettings(8000000, MSBFIRST, SPI_MODE0));
        SX1280 radio(mod);
        bool initOk = initLora(radio, 9, 7, true, false);
        if (!initOk) {
            result("[I] full production mimic", false);
        } else {
            uint32_t tBefore = millis();
            bool sdOk = tryMountSd();
            Serial.printf("  [INFO] SD.begin at boot t=%lu ms = %s\n",
                          (unsigned long)tBefore, sdOk ? "OK" : "FAIL");
            result("[I] full production mimic", sdOk);
        }
        delete mod;
    }

    // [G] Production mimic: Wire.begin + I2C probes for TMP1075/DAC43401
    // BEFORE the LoRa init. Production's sensors_.begin() runs first and
    // touches the I2C bus and the DAC. If [G] FAILS while [A]-[F] PASS,
    // I2C/DAC activity is what's breaking SD - not the LoRa init itself.
    {
        Serial.println("\n--- [G] production mimic (Wire+I2C before LoRa) ---");
        coldResetLora();

        Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
        Wire.beginTransmission(I2C_ADDR_TMP1075);
        int e1 = Wire.endTransmission();
        Wire.beginTransmission(I2C_ADDR_DAC43401);
        int e2 = Wire.endTransmission();
        Serial.printf("  [INFO] TMP1075 probe=%d  DAC43401 probe=%d "
                      "(0=present)\n", e1, e2);

        Module* mod = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RST,
                                 PIN_LORA_BUSY, SPI,
                                 SPISettings(8000000, MSBFIRST, SPI_MODE0));
        SX1280 radio(mod);
        bool initOk = initLora(radio, 9, 7, true, false);
        if (!initOk) {
            result("[G] production mimic (Wire+I2C)", false);
        } else {
            uint32_t tBefore = millis();
            bool sdOk = tryMountSd();
            Serial.printf("  [INFO] SD.begin at boot t=%lu ms = %s\n",
                          (unsigned long)tBefore, sdOk ? "OK" : "FAIL");
            result("[G] production mimic (Wire+I2C)", sdOk);
        }
        delete mod;
    }

    Serial.println("\n==================================================");
    Serial.printf("  TEST COMPLETE - %d pass, %d fail\n", passCount, failCount);
    Serial.println("  Restarting in 8 s...");
    Serial.println("==================================================");
    delay(8000);
    ESP.restart();
}

void loop() {}
