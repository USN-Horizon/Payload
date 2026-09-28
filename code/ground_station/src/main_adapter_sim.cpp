// ADAPTER-FAITHFUL receiver for bench tests (env:adapter_sim).  RECEIVE ONLY.
//
// The real ground-station adapter (Avionics-main / Ratatoskr-GS-adapter-
// firmware, which we cannot change) does:
//
//     uint8_t received[128];
//     if (radio->receive(received, 128) == RADIOLIB_ERR_NONE)
//         Serial.write(received, 128);          // always 128 bytes
//
// and the Qt ground station feeds every forwarded byte to mavlink_frame_char.
// This firmware reproduces that byte-for-byte on THIS Nano (whose DIO1 is not
// wired, so it uses the SPI-polled IRQ receive path from main_scanner.cpp
// instead of the blocking receive()), and wraps each 128-byte buffer in text
// markers so tools/gs_capture.py on the PC can extract it and decode it with
// the same MAVLink parser (tools/gs_decode_sim.cpp --capture).  Per packet it
// also prints RSSI / SNR / length for the range check.
//
// Output framing (all on the same serial port, 115200):
//     #PKT len=<n> rssi=<dBm> snr=<dB>\n   <128 raw bytes>   \n#END\n
//     #ERR rssi=<dBm>\n                     (CRC / header error)
//     #STAT ...\n                           (every 5 s)
//
// Radio profile = the production one (2400 MHz, BW 812.5, SF11, CR 4/8,
// sync 0x12, CRC on, high-sensitivity RX).  FEM stays in RX (CTX=0/CRX=1,
// the combination the 2 km test locked on); the PA is never keyed, so the
// board cannot heat up.
//
// Upload:  pio run -e adapter_sim -t upload      (restore with -e scanner)

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

// Must match PAYLOAD/include/Stack_with_LoRa/Config.hpp.
static constexpr float   RF_FREQ_MHZ = 2400.0f;
static constexpr float   RF_BW_KHZ   = 812.5f;
static constexpr uint8_t RF_SF       = 11;
static constexpr uint8_t RF_CR       = 8;

static SX1280 radio = new Module(PIN_CS, PIN_DIO1, PIN_RST, PIN_BUSY);

static uint8_t  received[128];      // Like the adapter: never cleared between packets.
static uint32_t rxCount = 0, errCount = 0;
static float    lastRssi = 0.0f, lastSnr = 0.0f;
static int      beginStatus = -1;
static bool     cfgOk = false;

static bool configureRadio() {
  radio.standby();
  bool ok = true;
  ok &= radio.setFrequency(RF_FREQ_MHZ) == RADIOLIB_ERR_NONE;
  ok &= radio.setBandwidth(RF_BW_KHZ)   == RADIOLIB_ERR_NONE;
  ok &= radio.setCodingRate(RF_CR)      == RADIOLIB_ERR_NONE;
  ok &= radio.setSpreadingFactor(RF_SF) == RADIOLIB_ERR_NONE;   // last (DS 14.4.1)
  ok &= radio.setHighSensitivityMode(true) == RADIOLIB_ERR_NONE;
  ok &= radio.setCRC(2) == RADIOLIB_ERR_NONE;
  // FEM: RX path, PA off.
  analogWrite(DAC0, 0);
  delayMicroseconds(500);
  digitalWrite(PIN_CSD, 1);
  digitalWrite(PIN_CTX, 0);
  digitalWrite(PIN_CRX, 1);
  ok &= radio.startReceive() == RADIOLIB_ERR_NONE;
  return ok;
}

static void printStatus() {
  Serial.print("#STAT begin=");
  Serial.print(beginStatus);
  Serial.print(" cfg=");
  Serial.print(cfgOk ? "OK" : "FAIL");
  Serial.print(" profile=2400/812.5/SF11/CR8 fem=RX packets=");
  Serial.print(rxCount);
  Serial.print(" errors=");
  Serial.print(errCount);
  if (rxCount) {
    Serial.print(" last_rssi=");
    Serial.print(lastRssi, 2);
    Serial.print(" last_snr=");
    Serial.print(lastSnr, 2);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 5000) {}

  analogWriteResolution(10);
  pinMode(LED_EXT, OUTPUT);
  pinMode(DAC0, OUTPUT);
  pinMode(PIN_CSD, OUTPUT);
  pinMode(PIN_CTX, OUTPUT);
  pinMode(PIN_CRX, OUTPUT);
  memset(received, 0xA5, sizeof(received));   // Known garbage, like an uninitialised stack slot.

  beginStatus = radio.begin();
  if (beginStatus == RADIOLIB_ERR_NONE) cfgOk = configureRadio();
  printStatus();
}

void loop() {
  static uint32_t lastStat = 0;
  if (millis() - lastStat >= 5000) { lastStat = millis(); printStatus(); }

  uint16_t irq = radio.getIrqStatus();
  if (irq & (RADIOLIB_SX128X_IRQ_CRC_ERROR | RADIOLIB_SX128X_IRQ_HEADER_ERROR)) {
    errCount++;
    Serial.print("#ERR rssi=");
    Serial.println(radio.getRSSI(), 2);
    radio.startReceive();
  } else if (irq & RADIOLIB_SX128X_IRQ_RX_DONE) {
    size_t len = radio.getPacketLength();
    // Exactly what radio->receive(received, 128) ends up doing in the
    // adapter: readData with a 128-byte cap; a shorter packet only
    // overwrites its own length, the rest of the buffer keeps old bytes.
    int st = radio.readData(received, 128);
    if (st == RADIOLIB_ERR_NONE) {
      rxCount++;
      lastRssi = radio.getRSSI();
      lastSnr  = radio.getSNR();
      digitalWrite(LED_EXT, 1);
      Serial.print("#PKT len=");
      Serial.print(len);
      Serial.print(" rssi=");
      Serial.print(lastRssi, 2);
      Serial.print(" snr=");
      Serial.println(lastSnr, 2);
      Serial.write(received, 128);               // <- the adapter's Serial.write(received, 128)
      Serial.print("\n#END\n");
      digitalWrite(LED_EXT, 0);
    } else {
      errCount++;
      Serial.print("#ERR readData=");
      Serial.println(st);
    }
    radio.startReceive();
  }
  delay(2);
}
