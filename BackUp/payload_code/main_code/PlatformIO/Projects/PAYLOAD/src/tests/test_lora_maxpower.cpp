// Diagnostic test (TX side): push the LoRa signal as strong as possible and
// find which lever actually moves it.
//
// Holds the antenna switch in the proven-best state (CXT=LOW, CrX=HIGH, from
// test_lora_rfswitch_probe) and the DAC at full scale, then steps through the
// remaining power levers while transmitting steadily. Read RSSI on your MC for
// each labelled experiment window:
//
//   E1  power +13 dBm, GPIO10 drive ~20 mA   <- reference (near current default)
//   E2  power +13 dBm, GPIO10 drive ~40 mA   <- does more supply current help?
//                                               (tests the "LoRa not getting a
//                                               full 3.3 V under TX" hypothesis)
//   E3  power  +6 dBm, GPIO10 drive ~40 mA   <- does the TX power setting move
//   E4  power  -6 dBm, GPIO10 drive ~40 mA      RSSI at all? confirms the PA
//   E5  power -18 dBm, GPIO10 drive ~40 mA      responds to setOutputPower.
//
// How to read it:
//   - E2 clearly > E1  -> the module was current-starved; 40 mA helps (and the
//        real fix is proper power delivery, not a GPIO).
//   - E1 > E3 > E4 > E5 -> setOutputPower works; +13 dBm is the ceiling and the
//        weak absolute level is fixed loss (antenna match / connector).
//   - E1..E5 all the same -> the PA output is NOT changing with either knob ->
//        the loss is upstream of the PA entirely (supply or RF path hardware).
//
// Radio config matches production / your receiver: 2400 MHz, BW 812.5 kHz, SF11,
// CR 4/8, CRC on, whitening on. Your MC MUST be on the same settings.
//
// Upload:  pio run -e test_lora_maxpower -t upload -t monitor

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include "driver/gpio.h"   // gpio_set_drive_capability

// ---- Pins (match Config.hpp) ------------------------------------------------
static constexpr int PIN_SPI_MOSI = 4;
static constexpr int PIN_SPI_SCK  = 5;
static constexpr int PIN_SPI_MISO = 6;
static constexpr int LORA_NSS     = 9;    // IF13
static constexpr int LORA_PWR     = 10;   // IF14 power enable (the GPIO in question)
static constexpr int PIN_CXT      = 11;   // IF15 antenna-switch CXT
static constexpr int PIN_CRX      = 12;   // IF16 antenna-switch CrX
static constexpr int LORA_BUSY    = 13;   // IF17
static constexpr int LORA_RST     = 14;   // IF18
static constexpr int IMU_CS       = 16;
static constexpr int SD_CS        = 21;
static constexpr int PIN_I2C_SDA  = 7;
static constexpr int PIN_I2C_SCL  = 15;

// ---- Radio config (MUST match your receiver/MC) -----------------------------
static constexpr float   RF_FREQ_MHZ = 2400.0f;
static constexpr float   RF_BW_KHZ   = 812.5f;
static constexpr uint8_t RF_SF       = 11;
static constexpr uint8_t RF_CR       = 8;

// ---- DAC43401 (held at full scale; proven not to change power, max anyway) --
static constexpr uint8_t DAC_ADDR[]      = {0x48, 0x47};
static constexpr uint8_t DAC_GENERAL_CFG = 0x09;
static constexpr uint8_t DAC_DAC_DATA    = 0x21;
static constexpr uint8_t DAC_UNLOCK      = 0x36;

// DIO1 = RADIOLIB_NC: GPIO11 is the antenna-switch CXT line, not a chip DIO.
static SX1280 radio = new Module(LORA_NSS, RADIOLIB_NC, LORA_RST, LORA_BUSY,
                                 SPI, SPISettings(2000000, MSBFIRST, SPI_MODE0));

// Experiment table.
struct Exp { int8_t dbm; gpio_drive_cap_t drive; const char* name; };
static const Exp EXPS[] = {
    { 13, GPIO_DRIVE_CAP_2, "E1 +13dBm  ~20mA (reference)"},
    { 13, GPIO_DRIVE_CAP_3, "E2 +13dBm  ~40mA (more supply current)"},
    {  6, GPIO_DRIVE_CAP_3, "E3  +6dBm  ~40mA"},
    { -6, GPIO_DRIVE_CAP_3, "E4  -6dBm  ~40mA"},
    {-18, GPIO_DRIVE_CAP_3, "E5 -18dBm  ~40mA"},
};
static constexpr int N_EXP = sizeof(EXPS) / sizeof(EXPS[0]);
static constexpr uint32_t DWELL_MS = 6000;

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

// Wake whichever DAC address ACKs and drive it to full scale (code 0xFF<<4).
static bool dacMax() {
    for (uint8_t a : DAC_ADDR) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() != 0) continue;
        dacWrite16(a, DAC_UNLOCK, 0x5000); delay(2);
        dacWrite16(a, DAC_GENERAL_CFG, 0x0000); delay(2);
        dacWrite16(a, DAC_DAC_DATA, 0x0FF0); delay(2);   // 0xFF in bits [11:4]
        return true;
    }
    return false;
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("========================================");
    Serial.println("  DIAGNOSTIC (TX): LoRa max-power finder");
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

    // Antenna switch: proven-best TX path.
    pinMode(PIN_CXT, OUTPUT); digitalWrite(PIN_CXT, LOW);
    pinMode(PIN_CRX, OUTPUT); digitalWrite(PIN_CRX, HIGH);

    int st = radio.begin();
    Serial.printf("  [INFO] radio.begin() = %d (0=OK)\n", st);
    result("radio.begin()", st == RADIOLIB_ERR_NONE);
    if (st != RADIOLIB_ERR_NONE) {
        Serial.println("  ABORT - restarting in 5 s"); delay(5000); ESP.restart();
    }

    bool cfg = true;
    cfg &= radio.setFrequency(RF_FREQ_MHZ)  == RADIOLIB_ERR_NONE;
    cfg &= radio.setBandwidth(RF_BW_KHZ)    == RADIOLIB_ERR_NONE;
    cfg &= radio.setSpreadingFactor(RF_SF)  == RADIOLIB_ERR_NONE;
    cfg &= radio.setCodingRate(RF_CR)       == RADIOLIB_ERR_NONE;
    radio.setCRC(true);
    radio.setWhitening(true);
    result("radio config (2400/812.5/SF11/CR4-8, CRC+whiten)", cfg);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    bool dac = dacMax();
    Serial.printf("  [INFO] DAC %s (held at full scale)\n", dac ? "OK" : "not found");

    Serial.println("  [INFO] Antenna switch: CXT=LOW CrX=HIGH (best). DAC=full.");
    Serial.println("  [INFO] Read RSSI per experiment window on your MC.");
    Serial.println("========================================");
}

void loop() {
    for (int e = 0; e < N_EXP; ++e) {
        // Apply this experiment's GPIO supply-current limit and TX power.
        pinMode(LORA_PWR, OUTPUT);
        digitalWrite(LORA_PWR, HIGH);
        gpio_set_drive_capability(static_cast<gpio_num_t>(LORA_PWR), EXPS[e].drive);
        int ps = radio.setOutputPower(EXPS[e].dbm);

        Serial.println();
        Serial.println("  ====================================================");
        Serial.printf("  >>> %s\n", EXPS[e].name);
        Serial.printf("  >>> setOutputPower(%d) = %d (0=OK). Read RSSI ~%lus.\n",
                      EXPS[e].dbm, ps, static_cast<unsigned long>(DWELL_MS / 1000));
        Serial.println("  ====================================================");

        uint32_t tEnd = millis() + DWELL_MS;
        while (millis() < tEnd) {
            char msg[56];
            int len = snprintf(msg, sizeof(msg), "MAXP e%d dbm=%d seq=%lu",
                               e, EXPS[e].dbm, static_cast<unsigned long>(seq));
            // Blocking transmit; returns -5 (timeout) because DIO1=NC but the
            // packet still goes out (your MC receiving it confirms TX).
            radio.transmit(reinterpret_cast<uint8_t*>(msg), len);
            Serial.printf("  [TX] e%d dbm=%d seq=%lu\n",
                          e, EXPS[e].dbm, static_cast<unsigned long>(seq));
            seq++;
            delay(120);
        }
    }
}
