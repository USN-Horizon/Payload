// DIAGNOSTIC ground-station RX scanner (env:scanner).
//
// Finds the working receive configuration by brute force while the payload
// transmits. Differences from the production adapter firmware:
//   - RX detected by polling SX1280 IRQ status over SPI (DIO1 not relied on).
//   - Scans radio profile x FEM polarity, locks on first decoded packet:
//       profiles: NEW SF12/CR4-8 | PREV SF11/CR4-8 (both sync 0x12, CRC on)
//       FEM RX:   CTX=0/CRX=1 (per fem_control.cpp) | CTX=1/CRX=0 (swapped)
//   - High sensitivity mode enabled (+2 dB, DS §4.2.1 / Table 3-5).
//   - Prints CRC/header errors (a wrong-parameter signal is still a signal).
//
// Pins per Ratatoskr-GS-adapter-firmware pins.h.

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

static constexpr int LED_EXT  = A1;
static constexpr int PIN_RST  = 10;
static constexpr int PIN_BUSY = 9;
static constexpr int PIN_CSD  = 8;
static constexpr int PIN_CTX  = 7;
static constexpr int PIN_DIO1 = 5;
static constexpr int PIN_CRX  = 4;
static constexpr int PIN_CS   = 2;

// Must match PAYLOAD/include/New/Config.hpp. 2400.0 was the exact bottom of both
// the SX1280 synth range (DS Table 3-3) and the ISM band; 2450 is mid-band.
static constexpr float    RF_FREQ_MHZ    = 2450.0f;
static constexpr float    RF_BW_KHZ     = 812.5f;
static constexpr uint32_t COMBO_DWELL_MS = 15000;

struct Profile { const char* name; uint8_t sf; uint8_t cr; };
static constexpr Profile PROFILES[] = {
  { "NEW  SF12/CR4-8", 12, 8 },   // current flight profile
  { "PREV SF11/CR4-8", 11, 8 },   // payload still on pre-SF12 firmware
};
struct FemMode { const char* name; uint8_t ctx; uint8_t crx; };
static constexpr FemMode FEMS[] = {
  { "FEM CTX=0/CRX=1", 0, 1 },
  { "FEM CTX=1/CRX=0", 1, 0 },
};
static constexpr uint8_t N_COMBOS = 4;

static uint8_t  comboIdx    = 0;   // profile = comboIdx/2, fem = comboIdx%2
static bool     comboLocked = false;
static uint32_t comboStart  = 0;

static SX1280 radio = new Module(PIN_CS, PIN_DIO1, PIN_RST, PIN_BUSY);

static int      beginStatus = -1;
static bool     cfgOk    = false;
static uint32_t rxCount  = 0;
static uint32_t errCount = 0;
static float    lastRssi = 0.0f;

static const Profile& curProfile() { return PROFILES[comboIdx / 2]; }
static const FemMode& curFem()     { return FEMS[comboIdx % 2]; }

static void femApply(const FemMode& f) {
  analogWrite(DAC0, 0);
  delayMicroseconds(500);
  digitalWrite(PIN_CSD, 1);
  digitalWrite(PIN_CTX, f.ctx);
  digitalWrite(PIN_CRX, f.crx);
}

static bool applyCombo() {
  radio.standby();
  const Profile& p = curProfile();
  bool ok = true;
  ok &= radio.setFrequency(RF_FREQ_MHZ) == RADIOLIB_ERR_NONE;
  ok &= radio.setBandwidth(RF_BW_KHZ)   == RADIOLIB_ERR_NONE;
  ok &= radio.setCodingRate(p.cr)       == RADIOLIB_ERR_NONE;
  // setSpreadingFactor LAST: DS §14.4.1 requires the SF-dependent write to
  // register 0x925 (+ bit 0 of 0x93C) after SetModulationParams, and RadioLib
  // only does those writes inside setSpreadingFactor().
  ok &= radio.setSpreadingFactor(p.sf)  == RADIOLIB_ERR_NONE;
  // +2 dB sensitivity for 700 uA (DS §4.2.1, Table 3-5). Re-applied per combo
  // because it is a raw register write and costs nothing to repeat.
  ok &= radio.setHighSensitivityMode(true) == RADIOLIB_ERR_NONE;
  // setCRC takes a LENGTH IN BYTES: LoRa accepts only 0 (off) or 2 (on).
  // setCRC(true) == setCRC(1) silently fails and changes nothing.
  ok &= radio.setCRC(2) == RADIOLIB_ERR_NONE;
  // No setWhitening(): not a LoRa packet parameter (DS Table 11-60).
  femApply(curFem());
  ok &= radio.startReceive() == RADIOLIB_ERR_NONE;  // clears pending IRQs
  comboStart = millis();
  return ok;
}

static void printCombo() {
  Serial.print(curProfile().name);
  Serial.print(" + ");
  Serial.print(curFem().name);
}

static void printStatus() {
  Serial.println();
  Serial.println("========================================");
  Serial.println("  GS DIAGNOSTIC SCANNER (IRQ polled)");
  Serial.println("  2450 MHz / BW 812.5 / high-sens ON");
  Serial.print  ("  radio.begin() = ");
  Serial.print  (beginStatus);
  Serial.println(" (0=OK)");
  Serial.print  ("  radio config: ");
  Serial.println(cfgOk ? "OK" : "FAIL");
  Serial.print  ("  combo: ");
  printCombo();
  Serial.println(comboLocked ? " (LOCKED)" : " (scanning)");
  Serial.println("========================================");
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 5000) {
  }

  analogWriteResolution(10);
  pinMode(LED_EXT, OUTPUT);
  pinMode(DAC0, OUTPUT);
  pinMode(PIN_CSD, OUTPUT);
  pinMode(PIN_CTX, OUTPUT);
  pinMode(PIN_CRX, OUTPUT);

  beginStatus = radio.begin();
  if (beginStatus == RADIOLIB_ERR_NONE) {
    cfgOk = applyCombo();
  }
  if (!cfgOk) {
    while (true) {
      printStatus();
      digitalWrite(LED_EXT, 1); delay(100);
      digitalWrite(LED_EXT, 0); delay(900);
    }
  }

  printStatus();
}

void loop() {
  static bool wasConnected = false;
  bool connected = (bool)Serial;
  if (connected && !wasConnected) {
    printStatus();
  }
  wasConnected = connected;

  if (!comboLocked && millis() - comboStart >= COMBO_DWELL_MS) {
    comboIdx = (comboIdx + 1) % N_COMBOS;
    applyCombo();
    Serial.print("trying: ");
    printCombo();
    Serial.println();
  }

  static uint32_t lastBeat = 0;
  if (millis() - lastBeat >= 5000) {
    lastBeat = millis();
    Serial.print("listening [");
    printCombo();
    Serial.print(comboLocked ? " LOCKED" : " scan");
    Serial.print("]  packets=");
    Serial.print(rxCount);
    Serial.print("  crc_err=");
    Serial.print(errCount);
    if (rxCount > 0) {
      Serial.print("  last RSSI=");
      Serial.print(lastRssi, 2);
      Serial.print(" dBm");
    }
    Serial.println();
  }

  uint16_t irq = radio.getIrqStatus();

  if (irq & (RADIOLIB_SX128X_IRQ_CRC_ERROR | RADIOLIB_SX128X_IRQ_HEADER_ERROR)) {
    errCount++;
    Serial.print("CRC/header error (#");
    Serial.print(errCount);
    Serial.print(")  RSSI: ");
    Serial.print(radio.getRSSI(), 2);
    Serial.print(" dBm  [");
    printCombo();
    Serial.println("]");
    radio.startReceive();
  } else if (irq & RADIOLIB_SX128X_IRQ_RX_DONE) {
    uint8_t buf[256];
    size_t  len = radio.getPacketLength();
    int st = radio.readData(buf, len > sizeof(buf) ? sizeof(buf) : len);
    if (st == RADIOLIB_ERR_NONE) {
      rxCount++;
      lastRssi = radio.getRSSI();
      if (!comboLocked) {
        comboLocked = true;
        Serial.print(">>> packet decoded - locking: ");
        printCombo();
        Serial.println();
      }
      digitalWrite(LED_EXT, 1);
      Serial.print("Packet RSSI: ");
      Serial.print(lastRssi, 2);
      Serial.print(" dBm  SNR: ");
      Serial.print(radio.getSNR(), 2);
      Serial.print(" dB  (#");
      Serial.print(rxCount);
      Serial.print(", ");
      Serial.print(len);
      Serial.println(" bytes)");
      digitalWrite(LED_EXT, 0);
    } else {
      errCount++;
      Serial.print("readData error ");
      Serial.println(st);
    }
    radio.startReceive();
  }

  delay(5);
}
