// Component test: SX1280 LoRa radio
//
// Verifies SPI wiring, chip presence, the flight radio profile, and one
// transmission, the way production (Stack_with_LoRa/LoraTx.cpp) does it:
//   - antenna switch lines CXT=HIGH / CrX=LOW (TX path),
//   - TX completion by polling the SX1280 IRQ register (DIO1 is not wired
//     to the chip on this board, BUSY is not a reliable TxDone indicator),
//   - profile 2400 MHz / BW 812.5 / SF11 / CR 4/8 / CRC on (must match the
//     ground station adapter).
//
// PASS criteria:
//   - BUSY goes LOW after reset
//   - radio.begin() returns RADIOLIB_ERR_NONE  (SPI comms + chip status)
//   - every profile setter succeeds
//   - TxDone is seen within 2 s (a 64-byte packet is ~365 ms on air)
//
// The PA bias DAC is NOT touched here, so the packet leaves at low power;
// use env:test_lora_range for anything that involves a ground station.
//
// Upload:  pio run -e test_lora -t upload
// Monitor: pio device monitor -e test_lora --dtr 1 --rts 1

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

// Pin map: must match include/Stack_with_LoRa/Config.hpp
static constexpr int PIN_SPI_MOSI  = 4;
static constexpr int PIN_SPI_SCK   = 5;
static constexpr int PIN_SPI_MISO  = 6;
static constexpr int LORA_NSS      = 9;   // IF13 CS_rf
static constexpr int LORA_POWER_EN = 10;  // IF14 power enable
static constexpr int LORA_CXT      = 11;  // IF15 antenna switch CXT ("DIO1" on the PDF)
static constexpr int LORA_CRX      = 12;  // IF16 antenna switch CrX ("DIO2" on the PDF)
static constexpr int LORA_BUSY     = 13;  // IF17 rf_busy
static constexpr int LORA_RST      = 14;  // IF18 rf_rst
static constexpr int IMU_CS        = 16;  // Deselect the other SPI devices.
static constexpr int SD_CS         = 21;

static constexpr uint16_t SX128X_IRQ_TX_DONE = 0x0001;

static void result(const char* label, bool pass) {
    Serial.printf("  [%s] %s\n", pass ? "PASS" : "FAIL", label);
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("========================================");
    Serial.println("  COMPONENT TEST: SX1280 LoRa Radio");
    Serial.println("========================================");

    pinMode(IMU_CS, OUTPUT); digitalWrite(IMU_CS, HIGH);
    pinMode(SD_CS,  OUTPUT); digitalWrite(SD_CS,  HIGH);
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
    result("SPI bus initialized (SCK=5, MISO=6, MOSI=4)", true);

    pinMode(LORA_POWER_EN, OUTPUT); digitalWrite(LORA_POWER_EN, HIGH);
    delay(10);
    result("LoRa board power enabled (GPIO10/IF14)", true);

    // Antenna switch to the TX path (measured 2026-07-06: CXT=HIGH/CrX=LOW is
    // ~20 dB stronger than the other state once the PA is biased).
    pinMode(LORA_CXT, OUTPUT); digitalWrite(LORA_CXT, HIGH);
    pinMode(LORA_CRX, OUTPUT); digitalWrite(LORA_CRX, LOW);

    // Manual reset pulse before handing control to RadioLib.
    pinMode(LORA_NSS, OUTPUT); digitalWrite(LORA_NSS, HIGH);
    pinMode(LORA_RST, OUTPUT); digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(20);

    pinMode(LORA_BUSY, INPUT);
    uint32_t t0 = millis();
    while (digitalRead(LORA_BUSY) == HIGH && millis() - t0 < 100) {}
    bool busyOk = (digitalRead(LORA_BUSY) == LOW);
    Serial.printf("  [INFO] BUSY after reset = %s (waited %lu ms)\n",
                  busyOk ? "LOW (OK)" : "HIGH (stuck!)", millis() - t0);
    result("BUSY pin LOW after reset", busyOk);

    // DIO1 = RADIOLIB_NC: GPIO11 is the antenna switch, not the chip's DIO1.
    Module* mod = new Module(LORA_NSS, RADIOLIB_NC, LORA_RST, LORA_BUSY,
                             SPI, SPISettings(8000000, MSBFIRST, SPI_MODE0));
    SX1280 radio(mod);

    int state = radio.begin();
    Serial.printf("  [INFO] radio.begin() = %d  (0 = OK)\n", state);
    bool initOk = (state == RADIOLIB_ERR_NONE);
    result("radio.begin() - SPI comms + chip status", initOk);

    if (!initOk) {
        Serial.println("  [HINT] Check: NSS=9(IF13), RST=14(IF18), BUSY=13(IF17), PWR_EN=10(IF14), 3.3 V.");
        goto done;
    }

    {
        bool cfgOk = true;
        int s;
        s = radio.setFrequency(2400.0f);    if (s != RADIOLIB_ERR_NONE) { Serial.printf("  [FAIL] setFrequency: %d\n", s); cfgOk = false; }
        s = radio.setBandwidth(812.5f);     if (s != RADIOLIB_ERR_NONE) { Serial.printf("  [FAIL] setBandwidth: %d\n", s); cfgOk = false; }
        s = radio.setCodingRate(8);         if (s != RADIOLIB_ERR_NONE) { Serial.printf("  [FAIL] setCodingRate: %d\n", s); cfgOk = false; }
        s = radio.setSpreadingFactor(11);   if (s != RADIOLIB_ERR_NONE) { Serial.printf("  [FAIL] setSpreadingFactor: %d\n", s); cfgOk = false; }
        s = radio.setOutputPower(10);       if (s != RADIOLIB_ERR_NONE) { Serial.printf("  [FAIL] setOutputPower: %d\n", s); cfgOk = false; }
        s = radio.setCRC(2);                if (s != RADIOLIB_ERR_NONE) { Serial.printf("  [FAIL] setCRC(2): %d\n", s); cfgOk = false; }
        result("Flight profile (2400 MHz / BW 812.5 / SF11 / CR4-8 / CRC / +10 dBm)", cfgOk);

        if (cfgOk) {
            // A 64-byte packet like production sends.
            uint8_t pkt[64];
            for (size_t i = 0; i < sizeof(pkt); ++i) pkt[i] = static_cast<uint8_t>(i);
            uint32_t tTx = millis();
            int txState = radio.startTransmit(pkt, sizeof(pkt));
            Serial.printf("  [INFO] startTransmit() = %d  (0 = OK)\n", txState);
            bool txDone = false;
            if (txState == RADIOLIB_ERR_NONE) {
                while (millis() - tTx < 2000) {
                    if (radio.getIrqStatus() & SX128X_IRQ_TX_DONE) { txDone = true; break; }
                    delay(1);
                }
                Serial.printf("  [INFO] TxDone via IRQ register = %s after %lu ms (expect ~365 ms)\n",
                              txDone ? "YES" : "NO (timeout)", millis() - tTx);
                radio.finishTransmit();
            }
            result("Transmit one 64-byte packet", txDone);
        }
    }

done:
    Serial.println("========================================");
    Serial.println("  TEST COMPLETE  -  restarting in 5 s");
    Serial.println("========================================");
    delay(5000);
    ESP.restart();
}

void loop() {}
