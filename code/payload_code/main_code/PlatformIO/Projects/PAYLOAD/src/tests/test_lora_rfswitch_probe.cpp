// Diagnostic test (TX side): probe the CXT / CrX lines as an antenna RF switch.
//
// Hypothesis
// ----------
// The board's IF15/IF16 pins ("CXT" GPIO11, "CrX" GPIO12) are labelled DIO1/DIO2
// but DIO1 never asserts on TxDone (see New_README) - which is what you'd see if
// they are NOT chip DIOs but the control lines of an antenna TX/RX switch the
// firmware never drives. If so, the switch state would change how much power
// reaches the antenna and explain the weak signal.
//
// What it does
// ------------
// Transmits continuously while stepping CXT/CrX through several states, holding
// each ~6 s and announcing it on serial. Read RSSI on your ground station and
// note which state is strongest.
//
//   FLOAT/FLOAT  = both pins high-Z. THIS MATCHES THE WORKING SWEEP TEST, so it
//                  is the control: the link should behave exactly as before
//                  (~-67..-87). If even this state shows nothing on the ground
//                  station, the problem is this test transmitting, not the pins.
//   the driven states then show whether forcing the lines helps or hurts.
//
// Proven TX path: plain blocking radio.transmit() (same as test_lora_volt_sweep,
// which the ground station did receive). DIO1 = RADIOLIB_NC so GPIO11 is free to
// drive; transmit() falls back to an airtime timeout for completion.
//
// Radio config matches the receiver: 2400 MHz, BW 812.5 kHz, SF11, CR 4/8,
// CRC on, whitening on, +13 dBm.
//
// Upload:  pio run -e test_lora_rfswitch_probe -t upload -t monitor

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

// ---- Pins (match Config.hpp) ------------------------------------------------
static constexpr int PIN_SPI_MOSI = 4;
static constexpr int PIN_SPI_SCK  = 5;
static constexpr int PIN_SPI_MISO = 6;
static constexpr int LORA_NSS     = 9;    // IF13
static constexpr int LORA_PWR     = 10;   // IF14 power enable
static constexpr int PIN_CXT      = 11;   // IF15 "CXT"
static constexpr int PIN_CRX      = 12;   // IF16 "CrX"
static constexpr int LORA_BUSY    = 13;   // IF17
static constexpr int LORA_RST     = 14;   // IF18
static constexpr int IMU_CS       = 16;
static constexpr int SD_CS        = 21;

// ---- Radio config (MUST match the receiver) ---------------------------------
static constexpr float   RF_FREQ_MHZ = 2400.0f;
static constexpr float   RF_BW_KHZ   = 812.5f;
static constexpr uint8_t RF_SF       = 11;
static constexpr uint8_t RF_CR       = 8;
static constexpr int8_t  RF_PWR_DBM  = 13;

// DIO1 = RADIOLIB_NC so RadioLib leaves GPIO11 free for us to drive as CXT.
static SX1280 radio = new Module(LORA_NSS, RADIOLIB_NC, LORA_RST, LORA_BUSY,
                                 SPI, SPISettings(2000000, MSBFIRST, SPI_MODE0));

// Pin level encoding: -1 = float (INPUT/high-Z), 0 = LOW, 1 = HIGH.
struct SwState { int cxt; int crx; const char* name; };
static const SwState STATES[] = {
    {-1, -1, "FLOAT/FLOAT (control=working sweep)"},
    { 1,  0, "CXT-hi  CrX-lo"},
    { 0,  1, "CXT-lo  CrX-hi"},
    { 1,  1, "CXT-hi  CrX-hi"},
    { 0,  0, "CXT-lo  CrX-lo"},
};
static constexpr int N_STATES = sizeof(STATES) / sizeof(STATES[0]);
static constexpr uint32_t STATE_DWELL_MS = 6000;

static uint32_t seq = 0;

static void result(const char* label, bool pass) {
    Serial.printf("  [%s] %s\n", pass ? "PASS" : "FAIL", label);
}

static void applyPin(int pin, int level) {
    if (level < 0) { pinMode(pin, INPUT); }            // high-Z (float)
    else           { pinMode(pin, OUTPUT); digitalWrite(pin, level ? HIGH : LOW); }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("========================================");
    Serial.println("  DIAGNOSTIC (TX): CXT/CrX RF-switch probe");
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

    // Start with both lines floating (the known-working state).
    pinMode(PIN_CXT, INPUT);
    pinMode(PIN_CRX, INPUT);

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
    cfg &= radio.setOutputPower(RF_PWR_DBM)  == RADIOLIB_ERR_NONE;
    radio.setCRC(true);
    radio.setWhitening(true);
    result("radio config (2400/812.5/SF11/CR4-8/13dBm)", cfg);

    Serial.println("  [INFO] Stepping CXT/CrX; read RSSI per state on the ground station.");
    Serial.println("  [INFO] FLOAT/FLOAT should match your earlier ~-67..-87 link.");
    Serial.println("========================================");
}

void loop() {
    for (int s = 0; s < N_STATES; ++s) {
        applyPin(PIN_CXT, STATES[s].cxt);
        applyPin(PIN_CRX, STATES[s].crx);
        delay(5);  // let the switch settle.

        Serial.println();
        Serial.println("  ====================================================");
        Serial.printf("  >>> STATE %d/%d:  %s\n", s + 1, N_STATES, STATES[s].name);
        Serial.printf("  >>> Watch ground-station RSSI now for ~%lu s\n",
                      static_cast<unsigned long>(STATE_DWELL_MS / 1000));
        Serial.println("  ====================================================");

        uint32_t tEnd = millis() + STATE_DWELL_MS;
        int pkts = 0, okPkts = 0;
        while (millis() < tEnd) {
            char msg[56];
            int len = snprintf(msg, sizeof(msg), "RFSW s%d seq=%lu [%s]",
                               s, static_cast<unsigned long>(seq), STATES[s].name);
            // Proven blocking transmit (same call the sweep used).
            int tx = radio.transmit(reinterpret_cast<uint8_t*>(msg), len);
            pkts++;
            if (tx == RADIOLIB_ERR_NONE) okPkts++;
            Serial.printf("  [TX] state%d seq=%lu -> %s\n",
                          s, static_cast<unsigned long>(seq),
                          tx == RADIOLIB_ERR_NONE ? "sent" : "FAIL");
            seq++;
            delay(120);
        }
        Serial.printf("  >>> STATE %d done: %d/%d transmit() OK.\n", s, okPkts, pkts);
    }
}
