// Diagnostic test (RX / ground-station side): receive LoRa packets and print
// signal strength.
//
// Purpose
// -------
// Pair this with env:test_lora_volt_sweep on the payload board. For every packet
// received it prints the payload (which carries the TX's current PA voltage) plus
// the measured RSSI and SNR, so you can see whether signal strength tracks the
// swept voltage:
//   RSSI rises with mV  -> DAC/PA drive affects output power.
//   RSSI flat vs mV     -> it does not; the weak link is antenna / RF front-end.
// It also works as a general link monitor for the real payload firmware (same
// radio settings), printing RSSI/SNR of whatever it hears.
//
// Radio config matches production (Config.hpp) and the sweep test: 2400 MHz,
// BW 812.5 kHz, SF11, CR 4/8, CRC on, whitening on. MUST match the transmitter.
//
// Upload:  pio run -e test_lora_rx_rssi -t upload
// Monitor: pio device monitor -e test_lora_rx_rssi

#include <Arduino.h>
#include <SPI.h>
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

// ---- Radio config (MUST match the transmitter) ------------------------------
static constexpr float   RF_FREQ_MHZ = 2400.0f;
static constexpr float   RF_BW_KHZ   = 812.5f;
static constexpr uint8_t RF_SF       = 11;
static constexpr uint8_t RF_CR       = 8;     // 4/8

static SX1280 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY,
                                 SPI, SPISettings(2000000, MSBFIRST, SPI_MODE0));

static uint32_t rxCount = 0;
static uint32_t errCount = 0;

static void result(const char* label, bool pass) {
    Serial.printf("  [%s] %s\n", pass ? "PASS" : "FAIL", label);
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("========================================");
    Serial.println("  DIAGNOSTIC (RX): LoRa RSSI monitor");
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

    int st = radio.begin();
    Serial.printf("  [INFO] radio.begin() = %d (0=OK)\n", st);
    result("radio.begin()", st == RADIOLIB_ERR_NONE);
    if (st != RADIOLIB_ERR_NONE) {
        Serial.println("  ABORT - restarting in 5 s"); delay(5000); ESP.restart();
    }

    bool cfg = true;
    cfg &= radio.setFrequency(RF_FREQ_MHZ)   == RADIOLIB_ERR_NONE;
    cfg &= radio.setBandwidth(RF_BW_KHZ)     == RADIOLIB_ERR_NONE;
    cfg &= radio.setSpreadingFactor(RF_SF)   == RADIOLIB_ERR_NONE;
    cfg &= radio.setCodingRate(RF_CR)        == RADIOLIB_ERR_NONE;
    radio.setCRC(true);
    radio.setWhitening(true);
    result("radio config (2400/812.5/SF11/CR4-8, CRC+whiten)", cfg);

    Serial.println("  [INFO] Listening. Each line: payload | RSSI | SNR");
    Serial.println("========================================");
}

void loop() {
    String data;
    int st = radio.receive(data);   // blocking with internal timeout.

    if (st == RADIOLIB_ERR_NONE) {
        rxCount++;
        Serial.printf("  [RX #%lu] \"%s\"  RSSI=%.1f dBm  SNR=%.1f dB\n",
                      static_cast<unsigned long>(rxCount),
                      data.c_str(), radio.getRSSI(), radio.getSNR());
    } else if (st == RADIOLIB_ERR_RX_TIMEOUT) {
        // No packet this window; keep listening quietly.
    } else if (st == RADIOLIB_ERR_CRC_MISMATCH) {
        errCount++;
        Serial.printf("  [RX] CRC mismatch (#%lu)  RSSI=%.1f dBm\n",
                      static_cast<unsigned long>(errCount), radio.getRSSI());
    } else {
        Serial.printf("  [RX] error %d\n", st);
    }
}
