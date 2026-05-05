// Component test: I2C bus address scan
//
// Walks every address 0x00..0x7F on the main I2C bus (SDA=7, SCL=15)
// at both 100 kHz and 400 kHz, prints which addresses ACK, and labels
// the chips that are expected on this board.
//
// Specifically calls out 0x72 (the breakout PDF claims DAC43401 lives
// there; in practice it has been observed at 0x48).
//
// Upload:  pio run -e test_i2c_scan -t upload
// Monitor: pio device monitor -e test_i2c_scan

#include <Arduino.h>
#include <Wire.h>

static constexpr int PIN_I2C_SDA = 7;
static constexpr int PIN_I2C_SCL = 15;

// Known-device labels for the addresses we expect to see.
static const char* labelFor(uint8_t addr) {
    switch (addr) {
        case 0x14: return "BMM350 magnetometer";
        case 0x47: return "DAC43401 (alt addr)";
        case 0x48: return "DAC43401 (observed)";
        case 0x49: return "TMP1075 temp sensor";
        case 0x68: return "ICM-45686 (if I2C mode)";
        case 0x69: return "ICM-45686 (alt addr)";
        case 0x72: return "DAC43401 per PDF spec";
        case 0x73: return "TMP1075 per PDF spec";
        case 0x76: return "BMP388 barometer";
        case 0x77: return "BMP388 (alt addr)";
        default:   return nullptr;
    }
}

static int scanOnce(uint32_t freqHz) {
    Wire.setClock(freqHz);
    Serial.printf("\n--- Scanning at %u kHz ---\n", freqHz / 1000);

    // Print a 16x8 grid header
    Serial.print("       ");
    for (uint8_t col = 0; col < 16; ++col) Serial.printf(" %X ", col);
    Serial.println();

    int found = 0;
    for (uint8_t row = 0; row < 8; ++row) {
        Serial.printf("  0x%X0:", row);
        for (uint8_t col = 0; col < 16; ++col) {
            uint8_t a = (row << 4) | col;
            // Reserved ranges (0x00-0x07 and 0x78-0x7F) — still poke them so
            // the user sees if anything weird answers there.
            Wire.beginTransmission(a);
            uint8_t err = Wire.endTransmission();
            if (err == 0) {
                Serial.printf(" %02X", a);
                found++;
            } else {
                Serial.print(" --");
            }
        }
        Serial.println();
    }

    Serial.printf("  Total devices that ACKed: %d\n", found);
    return found;
}

static void detailedList(uint32_t freqHz) {
    Wire.setClock(freqHz);
    Serial.printf("\n--- Detailed list at %u kHz ---\n", freqHz / 1000);
    int found = 0;
    for (uint8_t a = 0x00; a <= 0x7F; ++a) {
        Wire.beginTransmission(a);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            const char* lbl = labelFor(a);
            Serial.printf("  [FOUND] 0x%02X  %s\n", a, lbl ? lbl : "(unknown)");
            found++;
        }
    }
    if (found == 0) {
        Serial.println("  (no devices responded)");
        Serial.println("  [HINT] Check SDA=7, SCL=15 wiring, pull-ups, and 3.3 V supply.");
    }
}

void setup() {
    Serial.begin(115200);
    delay(800);

    Serial.println();
    Serial.println("==================================================");
    Serial.println("  COMPONENT TEST: I2C bus scan (SDA=7, SCL=15)");
    Serial.println("==================================================");

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

    scanOnce(100000);
    scanOnce(400000);

    detailedList(100000);

    // Targeted probe of 0x72 since the user is specifically checking for it.
    Serial.println("\n--- Targeted probe: 0x72 ---");
    Wire.setClock(100000);
    Wire.beginTransmission(0x72);
    uint8_t err72 = Wire.endTransmission();
    Serial.printf("  endTransmission(0x72) = %u  (0 = ACK, 2 = NACK addr, "
                  "3 = NACK data, 4 = other)\n", err72);
    if (err72 == 0) {
        Serial.println("  [INFO] 0x72 IS PRESENT on the bus.");
    } else {
        Serial.println("  [INFO] 0x72 is NOT present (no ACK).");
    }

    Serial.println("\n==================================================");
    Serial.println("  TEST COMPLETE - restarting in 8 s");
    Serial.println("==================================================");
    delay(8000);
    ESP.restart();
}

void loop() {}
