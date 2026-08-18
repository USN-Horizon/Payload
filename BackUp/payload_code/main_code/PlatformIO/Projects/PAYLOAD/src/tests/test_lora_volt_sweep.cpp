// Diagnostic test (TX side): sweep the LoRa PA voltage while transmitting.
//
// Purpose
// -------
// After seeing ~-70 dBm at 50 cm we want to know whether the DAC43401 PA-drive
// voltage actually changes the radiated power. This board keys a LoRa packet at
// each voltage step and embeds the voltage in the payload. On the ground-station
// side (env:test_lora_rx_rssi, or your existing receiver) you read the RSSI of
// each packet and see whether RSSI tracks the voltage:
//   - RSSI climbs with voltage  -> the DAC/PA path works; tune for best power.
//   - RSSI flat across voltage   -> the DAC does NOT drive output power here
//                                   (the loss is elsewhere: antenna / RF switch).
//
// Radio config matches production (Config.hpp): 2400 MHz, BW 812.5 kHz, SF11,
// CR 4/8, CRC on, whitening on, +13 dBm. The receiver MUST match exactly.
//
// Upload:  pio run -e test_lora_volt_sweep -t upload
// Monitor: pio device monitor -e test_lora_volt_sweep

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>

// ---- Pins (match Config.hpp) ------------------------------------------------
static constexpr int PIN_SPI_MOSI = 4;
static constexpr int PIN_SPI_SCK  = 5;
static constexpr int PIN_SPI_MISO = 6;
static constexpr int LORA_NSS     = 9;    // IF13
static constexpr int LORA_PWR     = 10;   // IF14 power enable
static constexpr int LORA_DIO1    = 11;   // IF15
static constexpr int LORA_BUSY    = 13;   // IF17
static constexpr int LORA_RST     = 14;   // IF18
static constexpr int IMU_CS       = 16;   // IF8  - deselect on shared SPI
static constexpr int SD_CS        = 21;   // microSD - deselect on shared SPI
static constexpr int PIN_I2C_SDA  = 7;
static constexpr int PIN_I2C_SCL  = 15;

// ---- Radio config (MUST match the receiver) ---------------------------------
static constexpr float   RF_FREQ_MHZ = 2400.0f;
static constexpr float   RF_BW_KHZ   = 812.5f;
static constexpr uint8_t RF_SF       = 11;
static constexpr uint8_t RF_CR       = 8;     // 4/8
static constexpr int8_t  RF_PWR_DBM  = 13;

// ---- DAC43401 (LoRa PA voltage) ---------------------------------------------
static constexpr uint8_t DAC_ADDR_PRIMARY  = 0x48;
static constexpr uint8_t DAC_ADDR_FALLBACK = 0x47;
static constexpr uint8_t DAC_REG_GENERAL_CFG   = 0xD1;
static constexpr uint8_t DAC_REG_DAC_DATA      = 0x21;
static constexpr uint8_t DAC_REG_DEVICE_UNLOCK = 0x36;
static constexpr uint16_t DAC_UNLOCK_MAGIC     = 0x5000;

// Voltage steps to sweep (mV). Covers zero -> full scale.
static const uint16_t SWEEP_MV[] = {0, 660, 1320, 1980, 2640, 3300};
static constexpr int   SWEEP_N   = sizeof(SWEEP_MV) / sizeof(SWEEP_MV[0]);
static constexpr int   PKTS_PER_STEP = 5;     // packets sent at each voltage.

static SX1280 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY,
                                 SPI, SPISettings(2000000, MSBFIRST, SPI_MODE0));
static uint8_t  dacAddr = 0;
static uint32_t seq     = 0;

static void result(const char* label, bool pass) {
    Serial.printf("  [%s] %s\n", pass ? "PASS" : "FAIL", label);
}

static bool dacWrite16(uint8_t addr, uint8_t reg, uint16_t v) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(static_cast<uint8_t>(v >> 8));
    Wire.write(static_cast<uint8_t>(v & 0xFF));
    return (Wire.endTransmission() == 0);
}

// 8-bit code lives in DAC_DATA bits [11:4], so write code << 4.
static bool dacSetMv(uint16_t mV) {
    if (mV > 3300) mV = 3300;
    uint8_t code = static_cast<uint8_t>((static_cast<uint32_t>(mV) * 255u) / 3300u);
    return dacWrite16(dacAddr, DAC_REG_DAC_DATA, static_cast<uint16_t>(code) << 4);
}

static uint8_t resolveDac() {
    for (uint8_t a : {DAC_ADDR_PRIMARY, DAC_ADDR_FALLBACK}) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() != 0) continue;
        // Wake: unlock magic + clear power-down.
        dacWrite16(a, DAC_REG_DEVICE_UNLOCK, DAC_UNLOCK_MAGIC);
        delay(2);
        dacWrite16(a, DAC_REG_GENERAL_CFG, 0x0000);
        delay(2);
        return a;
    }
    return 0;
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("========================================");
    Serial.println("  DIAGNOSTIC (TX): LoRa PA volt sweep");
    Serial.println("========================================");

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
    // Deselect other SPI devices so they don't corrupt the bus.
    pinMode(IMU_CS, OUTPUT); digitalWrite(IMU_CS, HIGH);
    pinMode(SD_CS,  OUTPUT); digitalWrite(SD_CS,  HIGH);

    // Power + reset the SX1280.
    pinMode(LORA_PWR, OUTPUT); digitalWrite(LORA_PWR, HIGH);
    delay(10);
    pinMode(LORA_NSS, OUTPUT); digitalWrite(LORA_NSS, HIGH);
    pinMode(LORA_RST, OUTPUT); digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(20);

    int st = radio.begin();
    Serial.printf("  [INFO] radio.begin() = %d (0=OK)\n", st);
    result("radio.begin()", st == RADIOLIB_ERR_NONE);
    if (st != RADIOLIB_ERR_NONE) {
        Serial.println("  ABORT - restarting in 5 s"); delay(5000); ESP.restart();
    }

    bool cfg = true;
    cfg &= radio.setFrequency(RF_FREQ_MHZ)       == RADIOLIB_ERR_NONE;
    cfg &= radio.setBandwidth(RF_BW_KHZ)         == RADIOLIB_ERR_NONE;
    cfg &= radio.setSpreadingFactor(RF_SF)       == RADIOLIB_ERR_NONE;
    cfg &= radio.setCodingRate(RF_CR)            == RADIOLIB_ERR_NONE;
    cfg &= radio.setOutputPower(RF_PWR_DBM)      == RADIOLIB_ERR_NONE;
    radio.setCRC(true);
    radio.setWhitening(true);
    result("radio config (2400/812.5/SF11/CR4-8/13dBm, CRC+whiten)", cfg);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    dacAddr = resolveDac();
    Serial.printf("  [INFO] DAC43401 at 0x%02X %s\n", dacAddr,
                  dacAddr ? "(OK)" : "(NOT FOUND - sweep will not change voltage!)");
    result("DAC43401 located", dacAddr != 0);

    Serial.println("  [INFO] Sweeping voltage; read RSSI per packet on the RX side.");
    Serial.println("========================================");
}

void loop() {
    for (int i = 0; i < SWEEP_N; ++i) {
        uint16_t mV = SWEEP_MV[i];
        if (dacAddr) dacSetMv(mV);
        delay(20);  // let the PA rail settle before keying TX.

        for (int p = 0; p < PKTS_PER_STEP; ++p) {
            char msg[48];
            int len = snprintf(msg, sizeof(msg),
                               "VSWEEP seq=%lu mV=%u step=%d/%d",
                               static_cast<unsigned long>(seq), mV, i + 1, SWEEP_N);
            int st = radio.transmit(reinterpret_cast<uint8_t*>(msg), len);
            Serial.printf("  [TX] seq=%lu mV=%u  -> %s\n",
                          static_cast<unsigned long>(seq), mV,
                          st == RADIOLIB_ERR_NONE ? "sent" : "FAIL");
            seq++;
            delay(150);  // gap between packets.
        }
    }
}
