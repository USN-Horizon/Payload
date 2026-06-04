// Minimal-prefix SD reliability test.
//
// Production env:new (post-reorder) does this prefix before SD.begin:
//   1. SPI.begin
//   2. CS pins HIGH (LoRa CS, SD CS)
//   3. Wire.begin (no transactions yet)
//   4. NVS prefs read + write
//   5. SD.begin
//
// And it fails ~75% of boots. The standalone test_sdcard test (which only
// does step 1 + 5) passes 100%. So one of steps 2-4 is the trigger.
//
// This test walks each step in isolation, repeats SD.begin N times in each
// configuration, and reports the failure rate. The configuration whose
// failure rate jumps from 0% to >0% identifies the trigger.
//
// Each phase is run multiple times in a row (ESP.restart between full
// passes) so the user can observe whether the failure rate is consistent.
//
// Upload:  pio run -e test_sd_minimal_prefix -t upload
// Monitor: pio device monitor -e test_sd_minimal_prefix

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Preferences.h>
#include <SD.h>

static constexpr int PIN_SPI_MOSI = 4;
static constexpr int PIN_SPI_SCK  = 5;
static constexpr int PIN_SPI_MISO = 6;
static constexpr int PIN_I2C_SDA  = 7;
static constexpr int PIN_I2C_SCL  = 15;
static constexpr int PIN_LORA_CS   = 9;
static constexpr int PIN_LORA_PWR  = 10;
static constexpr int PIN_LORA_RST  = 14;
static constexpr int PIN_SD_CS     = 21;

static constexpr int ATTEMPTS_PER_PHASE = 8;

static int tryMountN(const char* label, int n) {
    int passes = 0;
    for (int i = 0; i < n; ++i) {
        bool ok = SD.begin(PIN_SD_CS, SPI, 8000000);
        Serial.printf("  [%s][%d] SD.begin at boot t=%lu ms = %s\n",
                      label, i, (unsigned long)millis(),
                      ok ? "OK" : "FAIL");
        if (ok) {
            passes++;
            SD.end();
        }
        delay(80);
    }
    Serial.printf("  [%s] TOTAL: %d/%d passed\n", label, passes, n);
    return passes;
}

void setup() {
    Serial.begin(115200);
    // Match production's busy-wait warm-up exactly.
    for (uint32_t t = millis(); millis() - t < 200; ) {}

    Serial.println();
    Serial.println("==================================================");
    Serial.println("  MINIMAL-PREFIX SD reliability sweep");
    Serial.printf("  (boot t=%lu ms when test starts)\n",
                  (unsigned long)millis());
    Serial.println("==================================================");

    // Always done first - matches the very start of App::begin().
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

    // PHASE 1: bare minimum (SPI only, SD CS not even driven explicitly).
    Serial.println("\n--- [1] SPI.begin only, no CS driven, no Wire, no NVS ---");
    int p1 = tryMountN("1", ATTEMPTS_PER_PHASE);

    // PHASE 2: add explicit CS HIGH drive for both LoRa and SD.
    Serial.println("\n--- [2] + drive LoRa_CS and SD_CS HIGH ---");
    pinMode(PIN_LORA_CS, OUTPUT); digitalWrite(PIN_LORA_CS, HIGH);
    pinMode(PIN_SD_CS,   OUTPUT); digitalWrite(PIN_SD_CS,   HIGH);
    int p2 = tryMountN("2", ATTEMPTS_PER_PHASE);

    // PHASE 3: + Wire.begin (I2C bus init, no transactions).
    Serial.println("\n--- [3] + Wire.begin(SDA=7, SCL=15, 400k) ---");
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    int p3 = tryMountN("3", ATTEMPTS_PER_PHASE);

    // PHASE 4: + NVS prefs read + write (same namespace as production).
    Serial.println("\n--- [4] + Preferences read/write under \"payload\" ---");
    {
        Preferences prefs;
        prefs.begin("payload", false);
        uint32_t runId = prefs.getUInt("run", 0) + 1;
        prefs.putUInt("run", runId);
        prefs.end();
        Serial.printf("  [INFO] runId=%lu\n", (unsigned long)runId);
    }
    int p4 = tryMountN("4", ATTEMPTS_PER_PHASE);

    // PHASE 5: + drive LoRa_PWR LOW (cut power to SX1280).
    Serial.println("\n--- [5] + drive LoRa_PWR LOW (cut SX1280 power) ---");
    pinMode(PIN_LORA_PWR, OUTPUT); digitalWrite(PIN_LORA_PWR, LOW);
    delay(20);
    int p5 = tryMountN("5", ATTEMPTS_PER_PHASE);

    // PHASE 6: + drive LoRa_RST LOW too (hold in reset, PWR still LOW).
    Serial.println("\n--- [6] + drive LoRa_RST LOW (hold in reset) ---");
    pinMode(PIN_LORA_RST, OUTPUT); digitalWrite(PIN_LORA_RST, LOW);
    delay(20);
    int p6 = tryMountN("6", ATTEMPTS_PER_PHASE);

    // PHASE 7: restore LoRa_PWR HIGH but keep RST LOW (powered, in reset).
    Serial.println("\n--- [7] LoRa_PWR HIGH, RST LOW (powered, held in reset) ---");
    digitalWrite(PIN_LORA_PWR, HIGH);
    digitalWrite(PIN_LORA_RST, LOW);
    delay(20);
    int p7 = tryMountN("7", ATTEMPTS_PER_PHASE);

    Serial.println("\n==================================================");
    Serial.println("  SUMMARY  (passes / attempts per phase)");
    Serial.printf("    [1] SPI only                  : %d / %d\n", p1, ATTEMPTS_PER_PHASE);
    Serial.printf("    [2] + CS HIGH                 : %d / %d\n", p2, ATTEMPTS_PER_PHASE);
    Serial.printf("    [3] + Wire.begin              : %d / %d\n", p3, ATTEMPTS_PER_PHASE);
    Serial.printf("    [4] + NVS read/write          : %d / %d\n", p4, ATTEMPTS_PER_PHASE);
    Serial.printf("    [5] + LoRa_PWR LOW            : %d / %d\n", p5, ATTEMPTS_PER_PHASE);
    Serial.printf("    [6] + LoRa_RST LOW (PWR LOW)  : %d / %d\n", p6, ATTEMPTS_PER_PHASE);
    Serial.printf("    [7] LoRa PWR HIGH + RST LOW   : %d / %d\n", p7, ATTEMPTS_PER_PHASE);
    Serial.println();
    Serial.println("  The cheapest passing phase is the production fix.");
    Serial.println("==================================================");
    Serial.println("  Restarting in 8 s...");
    delay(8000);
    ESP.restart();
}

void loop() {}
