// Diagnostic test (TX side): steady single-config beacon for antenna/connector
// debugging.
//
// Transmits continuously in ONE fixed best-case configuration so you can watch a
// stable RSSI baseline on the ground station while you change ONLY the antenna
// connection - everything else held still. This isolates the antenna/connector
// from position/fading (which confounded the earlier "I reconnected and moved
// it" reading).
//
// Fixed config: antenna switch CXT=LOW/CrX=HIGH (best), both DACs full scale,
// TX power +13 dBm, GPIO10 ~40 mA. Radio 2400/812.5/SF11/CR4-8, CRC+whiten.
//
// How to use (keep the unit and ground station completely still):
//   1. Note the baseline RSSI.
//   2. Re-seat the antenna connector firmly - does RSSI jump UP (toward -20s)?
//   3. Gently wiggle/press the connector - does RSSI swing many dB?
//   4. Try a different antenna / different u.FL or SMA - does it improve?
//   If re-seating alone (nothing else moved) swings RSSI by 10s of dB, the
//   connector/antenna is marginal -> very likely the ~50 dB fault.
//   If RSSI is rock-steady no matter what you do to the connector, the loss is
//   upstream (SX1280 output / antenna-side matching) instead.
//
// Upload:  pio run -e test_lora_beacon -t upload -t monitor

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include "driver/gpio.h"

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

static constexpr uint8_t DAC_ADDRS[]     = {0x47, 0x48};
static constexpr uint8_t DAC_GENERAL_CFG = 0x09;
static constexpr uint8_t DAC_DAC_DATA    = 0x21;
static constexpr uint8_t DAC_UNLOCK      = 0x36;

static SX1280 radio = new Module(LORA_NSS, RADIOLIB_NC, LORA_RST, LORA_BUSY,
                                 SPI, SPISettings(2000000, MSBFIRST, SPI_MODE0));
static uint32_t seq = 0;

static void result(const char* label, bool pass) {
    Serial.printf("  [%s] %s\n", pass ? "PASS" : "FAIL", label);
}

static void dacWrite16(uint8_t a, uint8_t reg, uint16_t v) {
    Wire.beginTransmission(a);
    Wire.write(reg);
    Wire.write(static_cast<uint8_t>(v >> 8));
    Wire.write(static_cast<uint8_t>(v & 0xFF));
    Wire.endTransmission();
}

static void dacsMax() {
    for (uint8_t a : DAC_ADDRS) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() != 0) continue;
        dacWrite16(a, DAC_UNLOCK, 0x5000); delay(2);
        dacWrite16(a, DAC_GENERAL_CFG, 0x0000); delay(2);
        dacWrite16(a, DAC_DAC_DATA, 0x0FF0); delay(2);
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("========================================");
    Serial.println("  DIAGNOSTIC (TX): steady antenna beacon");
    Serial.println("========================================");

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
    pinMode(IMU_CS, OUTPUT); digitalWrite(IMU_CS, HIGH);
    pinMode(SD_CS,  OUTPUT); digitalWrite(SD_CS,  HIGH);

    pinMode(LORA_PWR, OUTPUT); digitalWrite(LORA_PWR, HIGH);
    gpio_set_drive_capability(static_cast<gpio_num_t>(LORA_PWR), GPIO_DRIVE_CAP_3);
    delay(10);
    pinMode(LORA_NSS, OUTPUT); digitalWrite(LORA_NSS, HIGH);
    pinMode(LORA_RST, OUTPUT); digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(20);

    // Fixed best switch state.
    pinMode(PIN_CXT, OUTPUT); digitalWrite(PIN_CXT, LOW);
    pinMode(PIN_CRX, OUTPUT); digitalWrite(PIN_CRX, HIGH);

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
    dacsMax();

    Serial.println("  [INFO] Steady beacon (best switch, DACs max, +13 dBm).");
    Serial.println("  [INFO] Hold everything still; change ONLY the antenna and");
    Serial.println("  [INFO] watch the ground-station RSSI for jumps.");
    Serial.println("========================================");
}

void loop() {
    char msg[40];
    int len = snprintf(msg, sizeof(msg), "BEACON seq=%lu",
                       static_cast<unsigned long>(seq));
    radio.transmit(reinterpret_cast<uint8_t*>(msg), len);
    Serial.printf("  [TX] beacon seq=%lu\n", static_cast<unsigned long>(seq));
    seq++;
    delay(150);
}
