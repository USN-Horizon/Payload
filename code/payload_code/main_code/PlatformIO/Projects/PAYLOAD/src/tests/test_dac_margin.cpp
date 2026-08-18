// Diagnostic test (DAC only): drive the DAC43401 to a 3.3 V target via the
// margin-output feature, on the DAC at 0x48 (the one wired to the LoRa PA).
//
// Reference: VDD (REF_EN = 0), gain 1x, so the output range is 0 .. VDD. This
// matches the existing LoRa volt-sweep tests, which assume VDD = 3.3 V full
// scale. Codes are MSB-left-aligned in bits[11:4]; the 8-bit DAC43401 uses the
// top byte, so code8 = round(V / 3.3 * 255) and the 12-bit value = code8 << 4.
//
// SYNTAX mirrored below:  WRITE <REG (hex)>, <MSB>, <LSB>
//
//   level     volts   code8   12-bit   MSB/LSB
//   nominal   1.650   0x80    0x0800   0x08 0x00   (midscale)
//   HIGH      3.300   0xFF    0x0FF0   0x0F 0xF0   (target / PA full bias)
//   LOW       0.000   0x00    0x0000   0x00 0x00   (PA off)
//
// Init (once):
//   WRITE DAC_DATA(0x21),       0x08, 0x00   ; nominal 1.65 V
//   WRITE DAC_MARGIN_HIGH(0x25),0x0F, 0xF0   ; margin-high 3.3 V
//   WRITE DAC_MARGIN_LOW(0x26), 0x00, 0x00   ; margin-low  0.0 V
//   WRITE GENERAL_CONFIG(0xD1), 0x12, 0x00   ; VDD ref, powered up, EN_PMBUS=1,
//                                            ; CODE_STEP 2 LSB, SLEW 25.6 us
// Loop (so each level can be measured):
//   WRITE TRIGGER(0xD3), 0x00, 0x80          ; bit7 PMBUS_MARGIN_HIGH -> OUT 3.3 V
//   WRITE TRIGGER(0xD3), 0x00, 0x40          ; bit6 PMBUS_MARGIN_LOW  -> OUT 0.0 V
//   WRITE DAC_DATA(0x21),0x0F, 0xF0          ; back to nominal 3.3 V
//
// No device unlock is issued: DEVICE_LOCK defaults to 0 (GENERAL_CONFIG reset
// 0x01F0), so the config registers are writable out of POR.
//
// NOTE: true 3.3 V at full scale needs the DAC VDD rail >= 3.3 V; output headroom
// to VDD is ~0.8 %FSR, so the real max sits a few tens of mV below VDD.
//
// Upload:  pio run -e test_dac_margin -t upload && pio device monitor -e test_dac_margin

#include <Arduino.h>
#include <Wire.h>

// ---- I2C pins (match Config.hpp) --------------------------------------------
static constexpr int PIN_I2C_SDA = 7;
static constexpr int PIN_I2C_SCL = 15;

// ---- DAC43401 address (the one wired to the LoRa PA) -------------------------
static const uint8_t DAC_ADDRS[] = {0x48};

// ---- DAC43401 registers (datasheet Table 16) --------------------------------
static constexpr uint8_t REG_DAC_DATA    = 0x21;
static constexpr uint8_t REG_MARGIN_HIGH = 0x25;
static constexpr uint8_t REG_MARGIN_LOW  = 0x26;
static constexpr uint8_t REG_GENERAL_CFG = 0xD1;
static constexpr uint8_t REG_TRIGGER     = 0xD3;

// GENERAL_CONFIG = 0x1200: EN_PMBUS=1, CODE_STEP 2 LSB, SLEW 25.6 us,
// DAC_PDN=00 (powered up), REF_EN=0 (VDD reference, range 0..VDD).
static constexpr uint8_t GENERAL_CFG_MSB = 0x12;
static constexpr uint8_t GENERAL_CFG_LSB = 0x00;

// How long to hold each output level so it can be measured (ms).
static constexpr uint32_t HOLD_MS = 3000;

// WRITE <reg>, <msb>, <lsb>  -> returns true on I2C ACK. Logs every write so the
// serial log reads like the pseudocode.
static bool dacWrite(uint8_t addr, uint8_t reg, uint8_t msb, uint8_t lsb) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(msb);
    Wire.write(lsb);
    bool ok = (Wire.endTransmission() == 0);
    Serial.printf("    WRITE 0x%02X <- 0x%02X 0x%02X  [%s]\n",
                  reg, msb, lsb, ok ? "ACK" : "NAK");
    return ok;
}

// Run the one-time init sequence on one address.
static void dacInit(uint8_t addr) {
    Serial.printf("  --- init 0x%02X ---\n", addr);
    dacWrite(addr, REG_DAC_DATA,    0x08, 0x00);             // nominal     1.65 V
    dacWrite(addr, REG_MARGIN_HIGH, 0x0F, 0xF0);            // margin-high 3.3 V
    dacWrite(addr, REG_MARGIN_LOW,  0x00, 0x00);            // margin-low  0.0 V
    dacWrite(addr, REG_GENERAL_CFG, GENERAL_CFG_MSB, GENERAL_CFG_LSB);
    delay(2);
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("========================================");
    Serial.println("  DIAGNOSTIC: DAC43401 -> 3.3 V target");
    Serial.printf ("  GENERAL_CONFIG = 0x%02X%02X (VDD ref, powered up)\n",
                   GENERAL_CFG_MSB, GENERAL_CFG_LSB);
    Serial.println("========================================");

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

    for (uint8_t a : DAC_ADDRS) {
        Wire.beginTransmission(a);
        bool present = (Wire.endTransmission() == 0);
        Serial.printf("  [INFO] 0x%02X %s\n", a, present ? "present" : "NO ACK");
        if (present) dacInit(a);
    }
    Serial.println("  [INFO] Cycling HIGH(3.3V) -> LOW(0V) -> nominal(1.65V), ~3 s each.");
    Serial.println("========================================");
}

void loop() {
    for (uint8_t a : DAC_ADDRS) {
        Serial.printf("  >>> 0x%02X  TRIGGER margin-HIGH -> OUT = 3.3 V\n", a);
        dacWrite(a, REG_TRIGGER, 0x00, 0x80);   // bit7 PMBUS_MARGIN_HIGH
        delay(HOLD_MS);

        Serial.printf("  >>> 0x%02X  TRIGGER margin-LOW  -> OUT = 0.0 V\n", a);
        dacWrite(a, REG_TRIGGER, 0x00, 0x40);   // bit6 PMBUS_MARGIN_LOW
        delay(HOLD_MS);

        Serial.printf("  >>> 0x%02X  DAC_DATA nominal    -> OUT = 1.65 V\n", a);
        dacWrite(a, REG_DAC_DATA, 0x08, 0x00);
        delay(HOLD_MS);
    }
}
