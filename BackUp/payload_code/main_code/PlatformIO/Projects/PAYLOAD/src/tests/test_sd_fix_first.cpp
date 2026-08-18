// Reproduce production EXACTLY: RST held LOW from the start of boot, no
// prior SD activity, single SD.begin attempt. If [A] fails, my one-line
// production fix isn't sufficient and we need to add ONE more thing.
// Walk the candidates one at a time and find the cheapest combination
// that works on FIRST attempt.
//
// Each phase ends with ESP.restart() between runs so every phase always
// starts from a clean cold-boot state and we never benefit from a card
// "warm-up" by previous failed attempts.
//
// Variations to test (one per cold boot):
//   [A] Production-fix-only: RST LOW, no delay, single SD.begin.
//       Expect: FAIL (matches current production behavior).
//   [B] RST LOW + 50 ms delay before SD.begin.
//   [C] RST LOW + 8 dummy SPI bytes at 400 kHz with SD_CS HIGH.
//   [D] RST LOW + 50 ms delay + 8 dummy SPI bytes.
//
// Since we can only run one phase per cold boot, the test reads a counter
// from NVS to cycle through them automatically.
//
// Upload:  pio run -e test_sd_fix_first -t upload
// Monitor: pio device monitor -e test_sd_fix_first

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Preferences.h>

static constexpr int PIN_SPI_MOSI = 4;
static constexpr int PIN_SPI_SCK  = 5;
static constexpr int PIN_SPI_MISO = 6;
static constexpr int PIN_LORA_CS  = 9;
static constexpr int PIN_LORA_RST = 14;
static constexpr int PIN_SD_CS    = 21;

void setup() {
    Serial.begin(115200);
    for (uint32_t t = millis(); millis() - t < 200; ) {}

    // Pick variation based on a rotating NVS counter.
    Preferences prefs;
    prefs.begin("sdfix", false);
    uint8_t phase = prefs.getUChar("phase", 0);
    prefs.putUChar("phase", uint8_t((phase + 1) % 4));
    prefs.end();

    const char* labels[] = {
        "[A] RST LOW + immediate SD.begin",
        "[B] RST LOW + 50 ms delay + SD.begin",
        "[C] RST LOW + 8 dummy SPI bytes + SD.begin",
        "[D] RST LOW + 50 ms delay + 8 dummy SPI bytes + SD.begin",
    };

    Serial.println();
    Serial.println("==================================================");
    Serial.printf("  Variation %s\n", labels[phase]);
    Serial.printf("  (boot t=%lu ms)\n", (unsigned long)millis());
    Serial.println("==================================================");

    // Common: drive LoRa_CS HIGH and SD_CS HIGH, then assert RST LOW.
    pinMode(PIN_LORA_CS, OUTPUT); digitalWrite(PIN_LORA_CS, HIGH);
    pinMode(PIN_LORA_RST, OUTPUT); digitalWrite(PIN_LORA_RST, LOW);
    pinMode(PIN_SD_CS,   OUTPUT); digitalWrite(PIN_SD_CS,   HIGH);

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

    // Phase-specific additions.
    if (phase == 1 || phase == 3) {
        delay(50);
    }
    if (phase == 2 || phase == 3) {
        // 8 dummy bytes (64 SCK cycles) at 400 kHz with SD_CS HIGH.
        SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));
        digitalWrite(PIN_SD_CS, HIGH);
        for (int i = 0; i < 8; ++i) SPI.transfer(0xFF);
        SPI.endTransaction();
    }

    // Single SD.begin attempt - matches production's behavior.
    uint32_t t0 = millis();
    bool ok = SD.begin(PIN_SD_CS, SPI, 8000000);
    uint32_t dt = millis() - t0;
    Serial.printf("  SD.begin = %s  (took %lu ms, boot t=%lu ms)\n",
                  ok ? "OK" : "FAIL", (unsigned long)dt,
                  (unsigned long)millis());
    if (ok) SD.end();

    Serial.println("\n  Restarting in 5 s (next boot will run next variation)...");
    delay(5000);
    ESP.restart();
}

void loop() {}
