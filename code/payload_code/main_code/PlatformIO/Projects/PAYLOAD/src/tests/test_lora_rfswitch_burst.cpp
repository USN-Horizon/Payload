// Diagnostic test (TX side): RF-switch states, self-identifying on an RSSI-only
// ground station (no payload/seq visibility needed).
//
// For each switch state s (1..N) it does:
//   1. ID BLIPS: switch set to the known-good state (CXT=LOW, CrX=HIGH), then
//      send s separate single-packet blips spaced by silence. These ALWAYS reach
//      the ground station, so you can COUNT them: s blips = "state s coming".
//   2. short gap
//   3. MEASURE: switch set to state s, send a steady ~6 s burst. Read the RSSI
//      level of this window (or note it shows nothing if the state is dead).
//   4. long silent gap before the next state.
//
// On your RSSI-only display you therefore see, per state:
//   "s blips  ...gap...  [strong | weak | nothing]"
// so you can identify each state by counting blips AND read its strength, with
// no serial monitor or sequence numbers.
//
// States (cxt/crx: -1=float, 0=LOW, 1=HIGH):
//   1 float/float   2 CXT-hi/CrX-lo   3 CXT-lo/CrX-hi(best)   4 hi/hi   5 lo/lo
//
// Radio config matches your receiver: 2400 MHz, BW 812.5 kHz, SF11, CR 4/8.
// Proven blocking radio.transmit() (returns -5 with DIO1=NC but TX still goes).
//
// Upload:  pio run -e test_lora_rfswitch_burst -t upload -t monitor

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

// Both DAC43401s, held at full scale so the switch ranking is measured with the
// DACs maxed (proven inert by test_lora_dac_pair, but removes any doubt).
static constexpr uint8_t DAC_ADDRS[]     = {0x47, 0x48};
static constexpr uint8_t DAC_GENERAL_CFG = 0xD1;
static constexpr uint8_t DAC_DAC_DATA    = 0x21;
static constexpr uint8_t DAC_UNLOCK      = 0x36;

static constexpr float   RF_FREQ_MHZ = 2400.0f;
static constexpr float   RF_BW_KHZ   = 812.5f;
static constexpr uint8_t RF_SF       = 11;
static constexpr uint8_t RF_CR       = 8;

// Timing.
static constexpr uint32_t BLIP_GAP_MS   = 700;   // silence between ID blips.
static constexpr uint32_t ID_MEAS_GAP_MS = 1500; // gap between blips and window.
static constexpr uint32_t MEAS_MS       = 6000;  // measurement window length.
static constexpr uint32_t STATE_GAP_MS  = 2500;  // silent gap between states.

static SX1280 radio = new Module(LORA_NSS, RADIOLIB_NC, LORA_RST, LORA_BUSY,
                                 SPI, SPISettings(2000000, MSBFIRST, SPI_MODE0));
static uint32_t seq = 0;

struct SwState { int cxt; int crx; const char* name; };
static const SwState STATES[] = {
    {-1, -1, "float/float"},
    { 1,  0, "CXT-hi/CrX-lo"},
    { 0,  1, "CXT-lo/CrX-hi (best)"},
    { 1,  1, "CXT-hi/CrX-hi"},
    { 0,  0, "CXT-lo/CrX-lo"},
};
static constexpr int N_STATES = sizeof(STATES) / sizeof(STATES[0]);

// Known-good state used for the always-received ID blips.
static constexpr int BEST_CXT = 1;   // HIGH (PA-on ranking, 2026-07-06: state 2 wins, ~-29 dBm)
static constexpr int BEST_CRX = 0;   // LOW

static void result(const char* label, bool pass) {
    Serial.printf("  [%s] %s\n", pass ? "PASS" : "FAIL", label);
}

static void applyPin(int pin, int level) {
    if (level < 0) pinMode(pin, INPUT);
    else { pinMode(pin, OUTPUT); digitalWrite(pin, level ? HIGH : LOW); }
}

static void dacWrite16(uint8_t a, uint8_t reg, uint16_t v) {
    Wire.beginTransmission(a);
    Wire.write(reg);
    Wire.write(static_cast<uint8_t>(v >> 8));
    Wire.write(static_cast<uint8_t>(v & 0xFF));
    Wire.endTransmission();
}

// Wake every candidate DAC and drive it to full scale (code 0xFF in bits 11:4).
static void dacsMax() {
    for (uint8_t a : DAC_ADDRS) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() != 0) continue;
        dacWrite16(a, DAC_UNLOCK, 0x5000); delay(2);
        dacWrite16(a, DAC_GENERAL_CFG, 0x0000); delay(2);
        dacWrite16(a, DAC_DAC_DATA, 0x0FF0); delay(2);
        Serial.printf("  [INFO] DAC 0x%02X set to full scale\n", a);
    }
}

static void setSwitch(int cxt, int crx) {
    applyPin(PIN_CXT, cxt);
    applyPin(PIN_CRX, crx);
    delay(5);
}

static void sendOne(const char* tag) {
    char msg[40];
    int len = snprintf(msg, sizeof(msg), "%s seq=%lu", tag,
                       static_cast<unsigned long>(seq));
    radio.transmit(reinterpret_cast<uint8_t*>(msg), len);
    seq++;
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("========================================");
    Serial.println("  DIAGNOSTIC (TX): RF-switch burst-ID");
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

    setSwitch(BEST_CXT, BEST_CRX);

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

    // Hold both DACs at full scale for the whole run.
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    dacsMax();

    Serial.println("  [INFO] Per state: count the ID blips, then read the window.");
    Serial.println("  [INFO]   s blips -> [strong|weak|nothing] = state s result.");
    Serial.println("========================================");
}

void loop() {
    for (int s = 0; s < N_STATES; ++s) {
        int id = s + 1;  // 1-based blip count.

        Serial.println();
        Serial.printf("  >>> STATE %d (%s): sending %d ID blip(s)...\n",
                      id, STATES[s].name, id);

        // 1. ID blips in the known-good state (always received).
        setSwitch(BEST_CXT, BEST_CRX);
        for (int b = 0; b < id; ++b) {
            sendOne("BLIP");
            delay(BLIP_GAP_MS);   // silence between blips
        }

        delay(ID_MEAS_GAP_MS);    // gap before measurement

        // 2. Measurement window in the state under test.
        Serial.printf("  >>> STATE %d measure window (~%lu s) - read RSSI now\n",
                      id, static_cast<unsigned long>(MEAS_MS / 1000));
        setSwitch(STATES[s].cxt, STATES[s].crx);
        uint32_t tEnd = millis() + MEAS_MS;
        while (millis() < tEnd) {
            sendOne("MEAS");
            delay(120);
        }

        // 3. Silent gap before next state (switch back to safe best state).
        setSwitch(BEST_CXT, BEST_CRX);
        delay(STATE_GAP_MS);
    }
}
