// Range / ground-station test transmitter (env:test_lora_range).
//
// Sends exactly what Stack_with_LoRa sends in flight - the same radio code
// (LoraTx), the same MAVLink builders (MavlinkFrames) and the same packing
// (Packer, 3 x 64-byte packets per cycle) - but with synthetic sensor values
// and a counter, so a ground station can measure RSSI and count lost packets
// without the sensors, SD card or launch gate being involved:
//
//   COSMIC_RADIATION.radiation  = cycle counter (the GS "radiation" plot
//                                  rises by 1 per cycle: gaps = lost packets)
//   SCALED_PRESSURE.press_abs   = 1000 + (cycle % 100) hPa
//   SCALED_IMU acc              = 0, 0, 1000 mg
//
// Only the main board + LoRa board are needed.  The TMP1075 is read every
// cycle and printed; TX pauses above 75 C and resumes below 70 C (the
// datasheet limit for the SX1280 / PA is 85 C).
//
// Serial commands:  volt <mV>   PA bias (0..3300, default 3300)
//                   interval <ms>  cycle time (default TX_CYCLE_INTERVAL_MS)
//                   pause | go
//
// Upload:  pio run -e test_lora_range -t upload
// Monitor: pio device monitor -e test_lora_range --dtr 1 --rts 1

#include "Stack_with_LoRa/Config.hpp"
#include "Stack_with_LoRa/LoraTx.hpp"
#include "Stack_with_LoRa/MavlinkFrames.hpp"
#include "Stack_with_LoRa/Packer.hpp"

static LoraTx lora;
static MavlinkPacker<LORA_FRAME_MAX> packer;
static uint32_t cycle = 0;
static uint32_t intervalMs = TX_CYCLE_INTERVAL_MS;
static bool paused = false;
static bool tooHot = false;
static bool dacOk = false;
static bool tmpOk = false;
static uint32_t packetsThisCycle = 0;
static String cmdBuf;

static bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// DAC43401 PA bias, same sequence as Sensors::begin / setLoraVolt.
static bool dacSet(uint16_t mV) {
  if (mV > 3300) mV = 3300;
  uint8_t code = static_cast<uint8_t>((static_cast<uint32_t>(mV) * 255u) / 3300u);
  uint16_t reg = static_cast<uint16_t>(code) << 4;
  Wire.beginTransmission(I2C_ADDR_DAC43401);
  Wire.write(static_cast<uint8_t>(0x21));
  Wire.write(static_cast<uint8_t>(reg >> 8));
  Wire.write(static_cast<uint8_t>(reg & 0xFF));
  return Wire.endTransmission() == 0;
}
static void dacInit() {
  dacOk = i2cPresent(I2C_ADDR_DAC43401);
  if (!dacOk) return;
  Wire.beginTransmission(I2C_ADDR_DAC43401);
  Wire.write(static_cast<uint8_t>(0x36)); Wire.write(static_cast<uint8_t>(0x50)); Wire.write(static_cast<uint8_t>(0x00));
  Wire.endTransmission();
  delay(2);
  Wire.beginTransmission(I2C_ADDR_DAC43401);
  Wire.write(static_cast<uint8_t>(0xD1)); Wire.write(static_cast<uint8_t>(0x00)); Wire.write(static_cast<uint8_t>(0x00));
  Wire.endTransmission();
  delay(2);
  dacSet(3300);
}

// TMP1075 in centi-degC, same as Sensors::readLoraTemp.
static bool readTemp(int16_t& cC) {
  if (!tmpOk) return false;
  Wire.beginTransmission(I2C_ADDR_TMP1075);
  Wire.write(static_cast<uint8_t>(0x00));
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(static_cast<int>(I2C_ADDR_TMP1075), 2) != 2) return false;
  uint8_t hi = Wire.read(), lo = Wire.read();
  int16_t v12 = static_cast<int16_t>((hi << 8) | lo) >> 4;
  cC = static_cast<int16_t>(static_cast<int32_t>(v12) * 625 / 100);
  return true;
}

static void emitPacket(void*, const uint8_t* packet, size_t len) {
  if (packetsThisCycle > 0) delay(LORA_PACKET_GAP_MS);
  packetsThisCycle++;
  uint32_t t0 = millis();
  bool ok = lora.send(packet, static_cast<uint8_t>(len));
  Serial.printf("    packet %lu: %u bytes %s (%lu ms)\n",
                static_cast<unsigned long>(packetsThisCycle), static_cast<unsigned>(len),
                ok ? "sent" : "FAILED/queued", static_cast<unsigned long>(millis() - t0));
}

static void sendCycle() {
  uint8_t buf[LORA_FRAME_MAX];
  const uint32_t t = millis();
  packetsThisCycle = 0;

  AccelReading a; a.ok = true; a.t_ms = t; a.x_mg = 0; a.y_mg = 0; a.z_mg = 1000;
  GyroReading  g; g.ok = true; g.t_ms = t; g.x_mrad_s = 0; g.y_mrad_s = 0; g.z_mrad_s = 0;
  MagReading   m; m.ok = true; m.t_ms = t; m.x_mgauss = 500; m.y_mgauss = 0; m.z_mgauss = static_cast<int16_t>(cycle % 1000);
  BaroReading  b; b.ok = true; b.t_ms = t; b.pressure_hPa = 1000.0f + (cycle % 100); b.temperature_cC = 2500;
  GeigerReading gc; gc.ok = true; gc.t_ms = t; gc.counts1 = cycle & 0xFFFF; gc.counts2 = 0;

  size_t n;
  n = MavlinkFrames::buildAccel(buf, a);          packer.add(buf, n);
  n = MavlinkFrames::buildGyro(buf, g);           packer.add(buf, n);
  n = MavlinkFrames::buildMag(buf, m);            packer.add(buf, n);
  n = MavlinkFrames::buildBaroPressure(buf, b);   packer.add(buf, n);
  n = MavlinkFrames::buildCosmicRadiation(buf, gc); packer.add(buf, n);
  n = MavlinkFrames::buildVfrHud(buf, 0.0f);      packer.add(buf, n);
  n = MavlinkFrames::buildHeartbeat(buf, MAV_STATE_ACTIVE, 0); packer.add(buf, n);
  packer.flush();
}

static void handleSerial() {
  while (Serial.available()) {
    char ch = static_cast<char>(Serial.read());
    if (ch == '\r') continue;
    if (ch != '\n') { if (cmdBuf.length() < 40) cmdBuf += ch; continue; }
    String cmd = cmdBuf; cmdBuf = ""; cmd.trim(); cmd.toLowerCase();
    if (cmd.startsWith("volt ")) {
      long mV = cmd.substring(5).toInt();
      if (mV < 0) mV = 0; if (mV > 3300) mV = 3300;
      Serial.printf("[CMD] PA bias -> %ld mV %s\n", mV, dacSet(static_cast<uint16_t>(mV)) ? "OK" : "FAIL");
    } else if (cmd.startsWith("interval ")) {
      long ms = cmd.substring(9).toInt();
      if (ms < 500) ms = 500;
      intervalMs = static_cast<uint32_t>(ms);
      Serial.printf("[CMD] cycle interval -> %lu ms\n", static_cast<unsigned long>(intervalMs));
    } else if (cmd == "pause") { paused = true;  Serial.println("[CMD] paused"); }
    else if (cmd == "go")      { paused = false; Serial.println("[CMD] running"); }
    else if (cmd.length())     { Serial.println("[CMD] volt <mV> | interval <ms> | pause | go"); }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("========================================");
  Serial.println("  RANGE TEST TX: production packets + counter");
  Serial.println("========================================");

  packer.configure(LORA_PACKET_LEN, emitPacket, nullptr);

  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
  pinMode(PIN_IMU_CS, OUTPUT); digitalWrite(PIN_IMU_CS, HIGH);
  pinMode(PIN_SD_CS,  OUTPUT); digitalWrite(PIN_SD_CS,  HIGH);
  pinMode(PIN_LORA_CS, OUTPUT); digitalWrite(PIN_LORA_CS, HIGH);
  pinMode(PIN_LORA_RST, OUTPUT); digitalWrite(PIN_LORA_RST, LOW);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

  bool radioOk = lora.begin();
  Serial.printf("  radio: %s  (%.1f MHz, BW %.1f, SF%u, CR4/%u, %d dBm)\n",
                radioOk ? "OK" : "FAILED", LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR, LORA_TX_PWR);
  tmpOk = i2cPresent(I2C_ADDR_TMP1075);
  dacInit();
  Serial.printf("  TMP1075: %s   DAC43401: %s (PA bias 3300 mV)\n",
                tmpOk ? "present" : "absent", dacOk ? "present" : "absent");
  Serial.printf("  one cycle = %u-byte packets every %lu ms; commands: volt | interval | pause | go\n",
                static_cast<unsigned>(LORA_PACKET_LEN), static_cast<unsigned long>(intervalMs));
  if (!radioOk) { Serial.println("  no radio - nothing to do"); }
}

void loop() {
  static uint32_t lastCycle = 0;
  handleSerial();

  int16_t tempCc = 0;
  bool haveTemp = readTemp(tempCc);
  if (haveTemp) {
    if (!tooHot && tempCc >= 7500) { tooHot = true;  Serial.println("[TEMP] >= 75 C - TX paused until < 70 C"); }
    if (tooHot  && tempCc <  7000) { tooHot = false; Serial.println("[TEMP] < 70 C - TX resumed"); }
  }

  if (!paused && !tooHot && millis() - lastCycle >= intervalMs) {
    lastCycle = millis();
    cycle++;
    Serial.printf("[CYCLE %lu] temp=%s%d.%02d C\n", static_cast<unsigned long>(cycle),
                  haveTemp ? "" : "n/a ", tempCc / 100, abs(tempCc % 100));
    sendCycle();
    lora.service(2);
  }
  delay(5);
}
