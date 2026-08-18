// Diagnostic test (TX side): find which DAC (if any) controls LoRa TX power.
//
// The I2C scan shows BOTH 0x47 and 0x48 ACKing - likely two DAC43401 chips.
// Production / earlier sweeps only drove 0x48 and RSSI never moved. If the DAC
// wired to the PA is actually 0x47, that explains it. This test drives EACH
// candidate address through min/max voltage while transmitting, so you can see
// on your ground station whether either one changes the signal strength.
//
// For each address it first confirms the chip behaves like a DAC (DAC_DATA
// round-trips), then sweeps it. Antenna switch held in the proven-best TX state
// (CXT=LOW, CrX=HIGH); radio at full power.
//
// Read RSSI per window on your MC:
//   addr X @ 0 mV   vs   addr X @ 3300 mV
//   If RSSI jumps between 0 and 3300 mV for an address -> THAT DAC controls the
//   PA. If neither address changes anything -> no DAC controls output power and
//   the weak signal is the RF path (switch/matching), as the other tests show.
//
// Radio config matches your receiver: 2400 MHz, BW 812.5 kHz, SF11, CR 4/8.
//
// Upload:  pio run -e test_lora_dac_probe -t upload -t monitor

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>

// ---- Pins (match Config.hpp) ------------------------------------------------
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

// DAC43401 registers.
static constexpr uint8_t DAC_GENERAL_CFG = 0x09;
static constexpr uint8_t DAC_DAC_DATA    = 0x21;
static constexpr uint8_t DAC_UNLOCK      = 0x36;

// Candidate DAC addresses from the I2C scan.
static const uint8_t DAC_ADDRS[] = {0x47, 0x48};
static constexpr int  N_ADDR     = sizeof(DAC_ADDRS) / sizeof(DAC_ADDRS[0]);

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

static bool dacRead16(uint8_t a, uint8_t reg, uint16_t& out) {
    Wire.beginTransmission(a);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(static_cast<int>(a), 2) != 2) return false;
    out = (static_cast<uint16_t>(Wire.read()) << 8) | Wire.read();
    return true;
}

// Confirm chip at addr behaves like a DAC (DAC_DATA round-trips two patterns).
static bool isDac(uint8_t a) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() != 0) return false;
    dacWrite16(a, DAC_UNLOCK, 0x5000); delay(2);
    dacWrite16(a, DAC_GENERAL_CFG, 0x0000); delay(2);
    for (uint16_t p : {0x0A50, 0x05A0}) {
        if (!dacWrite16(a, DAC_DAC_DATA, p)) return false;
        delay(2);
        uint16_t rb = 0;
        if (!dacRead16(a, DAC_DAC_DATA, rb) || rb != p) return false;
    }
    return true;
}

static void dacSetMv(uint8_t a, uint16_t mV) {
    if (mV > 3300) mV = 3300;
    uint8_t code = static_cast<uint8_t>((static_cast<uint32_t>(mV) * 255u) / 3300u);
    dacWrite16(a, DAC_DAC_DATA, static_cast<uint16_t>(code) << 4);
}

// Transmit for ~10 s, announcing the address/voltage under test. A short silent
// settle first gives a clear gap on the ground station between windows.
static void txWindow(const char* label) {
    Serial.println();
    Serial.println("  ----------------------------------------------------");
    Serial.printf("  >>> %s  - settling 1 s, then RSSI window (~10 s)\n", label);
    Serial.println("  ----------------------------------------------------");
    delay(1000);  // brief quiet gap so the windows are easy to tell apart
    uint32_t tEnd = millis() + 10000;
    while (millis() < tEnd) {
        char msg[56];
        int len = snprintf(msg, sizeof(msg), "DACP %s seq=%lu",
                           label, static_cast<unsigned long>(seq));
        radio.transmit(reinterpret_cast<uint8_t*>(msg), len);
        Serial.printf("  [TX] seq=%lu\n", static_cast<unsigned long>(seq));
        seq++;
        delay(120);
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("========================================");
    Serial.println("  DIAGNOSTIC (TX): which DAC drives the PA?");
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

    pinMode(PIN_CXT, OUTPUT); digitalWrite(PIN_CXT, LOW);   // best TX path
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
    for (uint8_t a : DAC_ADDRS) {
        bool dac = isDac(a);
        Serial.printf("  [INFO] 0x%02X behaves like a DAC: %s\n", a, dac ? "YES" : "no");
    }
    Serial.println("========================================");
}

void loop() {
    for (int i = 0; i < N_ADDR; ++i) {
        uint8_t a = DAC_ADDRS[i];
        if (!isDac(a)) continue;   // skip anything that isn't a DAC

        char lab[24];

        dacSetMv(a, 0);    delay(20);
        snprintf(lab, sizeof(lab), "0x%02X @ 0mV", a);
        txWindow(lab);

        dacSetMv(a, 3300); delay(20);
        snprintf(lab, sizeof(lab), "0x%02X @ 3300mV", a);
        txWindow(lab);
    }
}
