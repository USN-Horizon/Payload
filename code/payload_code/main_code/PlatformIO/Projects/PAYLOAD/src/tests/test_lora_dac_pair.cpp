// Diagnostic test (TX side): drive BOTH DACs (0x47 + 0x48) together.
//
// The earlier per-address test (test_lora_dac_probe) only ever set one DAC at a
// time and left the other at an uncontrolled mid-scale value, so "both DACs at
// max simultaneously" was never actually tested. If the two DAC43401s feed the
// LoRa PA together (e.g. two bias/supply controls, or a summed pair), only one
// at max would not show the effect. This test always sets BOTH to known levels
// and steps through the four corners so you can read RSSI per combo:
//
//   47=0    48=0      (both min)
//   47=max  48=0
//   47=0    48=max
//   47=max  48=max    (both max)  <- the case of interest
//
// If "both max" is clearly stronger than the others on your ground station, the
// two DACs drive the PA together and we have a real power lever. If all four are
// the same, the DACs do not control output power (RF-path fault stands).
//
// Antenna switch held best (CXT=LOW, CrX=HIGH); radio at full power. 10 s window
// per combo with a 1 s silent gap. Radio config 2400/812.5/SF11/CR4-8.
//
// Upload:  pio run -e test_lora_dac_pair -t upload -t monitor

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>

static constexpr int PIN_SPI_MOSI = 4;
static constexpr int PIN_SPI_SCK  = 5;
static constexpr int PIN_SPI_MISO = 6;
static constexpr int LORA_NSS     = 9;
static constexpr int LORA_PWR     = 10;
static constexpr int PIN_CXT      = 11;
static constexpr int PIN_CRX      = 12;
static constexpr int LORA_BUSY    = 13;
static constexpr int LORA_RST     = 14;
static constexpr int IMU_CS       = 16;
static constexpr int SD_CS        = 21;
static constexpr int PIN_I2C_SDA  = 7;
static constexpr int PIN_I2C_SCL  = 15;

static constexpr float   RF_FREQ_MHZ = 2400.0f;
static constexpr float   RF_BW_KHZ   = 812.5f;
static constexpr uint8_t RF_SF       = 11;
static constexpr uint8_t RF_CR       = 8;

static constexpr uint8_t DAC_A = 0x47;
static constexpr uint8_t DAC_B = 0x48;
static constexpr uint8_t DAC_GENERAL_CFG = 0xD1;
static constexpr uint8_t DAC_DAC_DATA    = 0x21;
static constexpr uint8_t DAC_UNLOCK      = 0x36;

static SX1280 radio = new Module(LORA_NSS, RADIOLIB_NC, LORA_RST, LORA_BUSY,
                                 SPI, SPISettings(2000000, MSBFIRST, SPI_MODE0));
static uint32_t seq = 0;

static void result(const char* label, bool pass) {
    Serial.printf("  [%s] %s\n", pass ? "PASS" : "FAIL", label);
}

static bool dacWrite16(uint8_t a, uint8_t reg, uint16_t v) {
    Wire.beginTransmission(a);
    Wire.write(reg);
    Wire.write(static_cast<uint8_t>(v >> 8));
    Wire.write(static_cast<uint8_t>(v & 0xFF));
    return Wire.endTransmission() == 0;
}

static void dacWake(uint8_t a) {
    dacWrite16(a, DAC_UNLOCK, 0x5000); delay(2);
    dacWrite16(a, DAC_GENERAL_CFG, 0x0000); delay(2);
}

static void dacSetMv(uint8_t a, uint16_t mV) {
    if (mV > 3300) mV = 3300;
    uint8_t code = static_cast<uint8_t>((static_cast<uint32_t>(mV) * 255u) / 3300u);
    dacWrite16(a, DAC_DAC_DATA, static_cast<uint16_t>(code) << 4);
}

// Combo table: {mV for 0x47, mV for 0x48, label}.
struct Combo { uint16_t a; uint16_t b; const char* name; };
static const Combo COMBOS[] = {
    {0,    0,    "47=0     48=0     (both min)"},
    {3300, 0,    "47=MAX   48=0"},
    {0,    3300, "47=0     48=MAX"},
    {3300, 3300, "47=MAX   48=MAX   (both max)"},
};
static constexpr int N_COMBO = sizeof(COMBOS) / sizeof(COMBOS[0]);

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("========================================");
    Serial.println("  DIAGNOSTIC (TX): both DACs together");
    Serial.println("========================================");

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
    pinMode(IMU_CS, OUTPUT); digitalWrite(IMU_CS, HIGH);
    pinMode(SD_CS,  OUTPUT); digitalWrite(SD_CS,  HIGH);

    pinMode(LORA_PWR, OUTPUT); digitalWrite(LORA_PWR, HIGH);
    delay(10);
    pinMode(LORA_NSS, OUTPUT); digitalWrite(LORA_NSS, HIGH);
    pinMode(LORA_RST, OUTPUT); digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(20);

    pinMode(PIN_CXT, OUTPUT); digitalWrite(PIN_CXT, HIGH);
    pinMode(PIN_CRX, OUTPUT); digitalWrite(PIN_CRX, LOW);

    int st = radio.begin();
    Serial.printf("  [INFO] radio.begin() = %d (0=OK)\n", st);
    result("radio.begin()", st == RADIOLIB_ERR_NONE);
    if (st != RADIOLIB_ERR_NONE) {
        Serial.println("  ABORT - restarting in 5 s"); delay(5000); ESP.restart();
    }
    radio.setFrequency(RF_FREQ_MHZ);
    radio.setBandwidth(RF_BW_KHZ);
    radio.setSpreadingFactor(RF_SF);
    radio.setCodingRate(RF_CR);
    radio.setOutputPower(13);
    radio.setCRC(true);
    radio.setWhitening(true);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    dacWake(DAC_A);
    dacWake(DAC_B);
    Serial.println("  [INFO] Both DACs woken. Stepping combos, 10 s each.");
    Serial.println("========================================");
}

void loop() {
    for (int c = 0; c < N_COMBO; ++c) {
        // Always set BOTH DACs explicitly so neither is left floating mid-scale.
        dacSetMv(DAC_A, COMBOS[c].a);
        dacSetMv(DAC_B, COMBOS[c].b);
        delay(20);

        Serial.println();
        Serial.println("  ----------------------------------------------------");
        Serial.printf("  >>> %s  - settle 1 s, RSSI window ~10 s\n", COMBOS[c].name);
        Serial.println("  ----------------------------------------------------");
        delay(1000);  // silent gap to separate windows on the ground station.

        uint32_t tEnd = millis() + 10000;
        while (millis() < tEnd) {
            char msg[56];
            int len = snprintf(msg, sizeof(msg), "DACPAIR c%d 47=%u 48=%u seq=%lu",
                               c, COMBOS[c].a, COMBOS[c].b,
                               static_cast<unsigned long>(seq));
            radio.transmit(reinterpret_cast<uint8_t*>(msg), len);
            Serial.printf("  [TX] c%d (47=%u 48=%u) seq=%lu\n",
                          c, COMBOS[c].a, COMBOS[c].b,
                          static_cast<unsigned long>(seq));
            seq++;
            delay(120);
        }
    }
}
