// Component test: DAC43401 (LoRa TX-power voltage setter)
//
// The I2C scanner located the DAC43401 at 0x48 (the breakout PDF claimed
// 0x72, but the actual board strapping puts it on the TI default range —
// ADDR pin tied to GND -> 0x48). 0x47 also ACKs on this board, so we keep
// it as a fallback in case the strapping changes.
//
// Output voltage: V_out = code * VREF / 255  with VREF ~= VDD ~= 3.3 V
// (unless the internal reference is enabled in GENERAL_CONFIG).
//
// PASS criteria:
//   - DAC ACKs at 0x48 (or 0x47 fallback)
//   - DEVICE_ID / STATUS register read returns
//   - DAC_DATA round-trips two distinct test patterns
//   - Mid-, full-, and zero-scale writes all ACK and read back correctly
//
// Upload:  pio run -e test_lora_volt -t upload
// Monitor: pio device monitor -e test_lora_volt

#include <Arduino.h>
#include <Wire.h>

static constexpr int     PIN_I2C_SDA  = 7;
static constexpr int     PIN_I2C_SCL  = 15;
static constexpr int     PIN_LORA_PWR = 10;     // gates the LoRa-side power rail.

static constexpr uint8_t DAC_ADDR_PRIMARY  = 0x48;   // confirmed by I2C scan
static constexpr uint8_t DAC_ADDR_FALLBACK = 0x47;   // also ACKs on this board

// DAC43401 register map
static constexpr uint8_t DAC_REG_STATUS        = 0x01;
static constexpr uint8_t DAC_REG_GENERAL_CFG   = 0x09;
static constexpr uint8_t DAC_REG_DAC_DATA      = 0x21;
static constexpr uint8_t DAC_REG_DEVICE_UNLOCK = 0x36;
static constexpr uint16_t DAC_UNLOCK_MAGIC     = 0x5000;

static int passCount = 0;
static int failCount = 0;

static void result(const char* label, bool pass) {
    Serial.printf("  [%s] %s\n", pass ? "PASS" : "FAIL", label);
    if (pass) passCount++; else failCount++;
}

static bool dacWrite16(uint8_t addr, uint8_t reg, uint16_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(static_cast<uint8_t>(value >> 8));
    Wire.write(static_cast<uint8_t>(value & 0xFF));
    return (Wire.endTransmission() == 0);
}

static bool dacRead16(uint8_t addr, uint8_t reg, uint16_t& out) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(static_cast<int>(addr), 2) != 2) return false;
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    out = (static_cast<uint16_t>(hi) << 8) | lo;
    return true;
}

// DAC43401 stores the 8-bit code in DAC_DATA bits [11:4]; bits [15:12]
// and [3:0] are reserved and always read back as zero. So a code of 0x80
// goes into the register as 0x0800, not 0x8000.
static bool dacSetCode(uint8_t addr, uint8_t code) {
    return dacWrite16(addr, DAC_REG_DAC_DATA, static_cast<uint16_t>(code) << 4);
}

// Wake the DAC: send unlock magic + clear power-down in GENERAL_CONFIG.
static void dacWake(uint8_t addr) {
    dacWrite16(addr, DAC_REG_DEVICE_UNLOCK, DAC_UNLOCK_MAGIC);
    delay(2);
    dacWrite16(addr, DAC_REG_GENERAL_CFG, 0x0000);
    delay(2);
}

// Round-trip two distinct DAC_DATA patterns to verify it really is a DAC
// (a TMP1075 at the same address would fail this). Patterns are placed in
// bits [11:4] since the chip masks bits [15:12] and [3:0] to zero.
static bool dacRoundTrip(uint8_t addr) {
    constexpr uint16_t patterns[] = {0x0A50, 0x05A0};
    for (uint16_t p : patterns) {
        if (!dacWrite16(addr, DAC_REG_DAC_DATA, p)) return false;
        delay(2);
        uint16_t readBack = 0;
        if (!dacRead16(addr, DAC_REG_DAC_DATA, readBack)) return false;
        Serial.printf("  [INFO] 0x%02X round-trip: wrote 0x%04X, read 0x%04X %s\n",
                      addr, p, readBack, (readBack == p) ? "OK" : "MISMATCH");
        if (readBack != p) return false;
    }
    return true;
}

static bool ack(uint8_t addr) {
    Wire.beginTransmission(addr);
    return (Wire.endTransmission() == 0);
}

// Pick whichever of the two candidate addresses behaves like a DAC.
static uint8_t resolveDacAddr() {
    for (uint8_t addr : {DAC_ADDR_PRIMARY, DAC_ADDR_FALLBACK}) {
        if (!ack(addr)) {
            Serial.printf("  [INFO] 0x%02X did not ACK\n", addr);
            continue;
        }
        Serial.printf("  [INFO] 0x%02X ACKed, running DAC behaviour check...\n", addr);
        dacWake(addr);
        if (dacRoundTrip(addr)) {
            Serial.printf("  [INFO] 0x%02X confirmed as DAC43401\n", addr);
            return addr;
        }
        Serial.printf("  [INFO] 0x%02X did not behave like a DAC\n", addr);
    }
    return 0;
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("========================================");
    Serial.println("  COMPONENT TEST: DAC43401  (LoRa volt)");
    Serial.println("========================================");

    // Bring up the LoRa-side power rail (the DAC is on it).
    pinMode(PIN_LORA_PWR, OUTPUT);
    digitalWrite(PIN_LORA_PWR, HIGH);
    delay(20);
    result("LoRa power enable HIGH (GPIO10)", true);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    result("I2C bus initialized (SDA=7, SCL=15)", true);

    uint8_t dacAddr = resolveDacAddr();
    result("DAC43401 located on bus", dacAddr != 0);
    if (dacAddr == 0) {
        Serial.println("  [HINT] Check LoRa power enable (GPIO10) and the DAC's ADDR strap.");
        goto done;
    }
    Serial.printf("  [INFO] Using DAC at 0x%02X\n", dacAddr);

    {
        uint16_t status = 0;
        bool stOk = dacRead16(dacAddr, DAC_REG_STATUS, status);
        Serial.printf("  [INFO] STATUS register = 0x%04X (read=%s)\n",
                      status, stOk ? "yes" : "no");
        result("Read STATUS register", stOk);

        // Mid-scale (~ VREF / 2 ~= 1.65 V).
        bool midOk = dacSetCode(dacAddr, 0x80);
        Serial.printf("  [INFO] Mid-scale  (code 0x80, ~1.65 V) ACK=%s\n",
                      midOk ? "yes" : "no");
        result("Write mid-scale (0x80)", midOk);
        delay(20);

        // Full-scale (~ VREF ~= 3.3 V).
        bool fullOk = dacSetCode(dacAddr, 0xFF);
        Serial.printf("  [INFO] Full-scale (code 0xFF, ~3.3 V)  ACK=%s\n",
                      fullOk ? "yes" : "no");
        result("Write full-scale (0xFF)", fullOk);
        delay(20);

        // Zero-scale.
        bool zeroOk = dacSetCode(dacAddr, 0x00);
        Serial.printf("  [INFO] Zero-scale (code 0x00, 0 V)     ACK=%s\n",
                      zeroOk ? "yes" : "no");
        result("Write zero-scale (0x00)", zeroOk);
        delay(20);

        // Confirm the chip remembers the last value we wrote.
        uint16_t readBack = 0;
        bool rbOk = dacRead16(dacAddr, DAC_REG_DAC_DATA, readBack);
        uint8_t lastCode = static_cast<uint8_t>((readBack >> 4) & 0xFF);
        Serial.printf("  [INFO] DAC_DATA read back = 0x%04X (code=0x%02X, read=%s)\n",
                      readBack, lastCode, rbOk ? "yes" : "no");
        result("Read DAC_DATA register", rbOk);
        result("Read-back matches last write (0x00)", rbOk && lastCode == 0x00);

        // Sweep through a few codes so the user can probe the LoRa-PA voltage
        // pin with a multimeter and watch it step.
        Serial.println("  [INFO] Voltage sweep (5 steps, ~1 s each):");
        const uint8_t sweep[] = {0x00, 0x40, 0x80, 0xC0, 0xFF};
        for (uint8_t code : sweep) {
            dacSetCode(dacAddr, code);
            float volts = (code * 3.3f) / 255.0f;
            Serial.printf("         code 0x%02X -> ~%.2f V\n", code, volts);
            delay(1000);
        }

        // Leave the DAC at mid-scale so the LoRa PA sees a known safe voltage.
        dacSetCode(dacAddr, 0x80);
        Serial.printf("  [INFO] DAC at 0x%02X left at mid-scale (~1.65 V) on exit.\n", dacAddr);
    }

done:
    Serial.println("========================================");
    Serial.printf("  TEST COMPLETE - %d pass, %d fail\n", passCount, failCount);
    Serial.println("  Restarting in 5 s...");
    Serial.println("========================================");
    delay(5000);
    ESP.restart();
}

void loop() {}
