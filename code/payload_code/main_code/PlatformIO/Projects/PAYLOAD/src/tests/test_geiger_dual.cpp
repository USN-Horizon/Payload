// Component test: Geiger Counter, BOTH channels
//
// test_geiger only ever verified CRD1 on GPIO 8. Production (env:new) also
// counts CRD2 on GPIO 3 (IF12, from the board PDF) which has never been
// hardware-verified — and GPIO 3 is an ESP32-S3 strapping pin, so the PDF
// being wrong here would silently zero the second channel in flight logs.
// This test counts both channels side by side and reports per-channel rates
// every 10 s so a mismatch (one channel silent) is obvious.
//
// PASS criteria per window:
//   - Both channels see pulses (tube powered + background/source present).
//   - Rates within the same order of magnitude (same tube type assumed).
//
// Upload:  pio run -e test_geiger_dual -t upload
// Monitor: pio device monitor -e test_geiger_dual

#include <Arduino.h>

static constexpr int GEIGER_PIN_1 = 8;  // CRD1, IF11 
static constexpr int GEIGER_PIN_2 = 3;  // CRD2, IF12 

static volatile uint32_t count1 = 0;
static volatile uint32_t count2 = 0;

void IRAM_ATTR geiger1ISR() { count1++; }
void IRAM_ATTR geiger2ISR() { count2++; }

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("========================================");
    Serial.println("  COMPONENT TEST: Geiger dual channel");
    Serial.println("========================================");

    pinMode(GEIGER_PIN_1, INPUT);
    pinMode(GEIGER_PIN_2, INPUT);
    attachInterrupt(digitalPinToInterrupt(GEIGER_PIN_1), geiger1ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(GEIGER_PIN_2), geiger2ISR, RISING);
    Serial.println("  [INFO] CRD1 on GPIO 8, CRD2 on GPIO 3. 10 s windows.");
    Serial.println("  [INFO] CRD2 silent while CRD1 counts => GPIO 3 mapping is wrong.");
    Serial.println("========================================");
}

void loop() {
    uint32_t start = millis();
    while (millis() - start < 10000) delay(100);

    noInterrupts();
    uint32_t c1 = count1; count1 = 0;
    uint32_t c2 = count2; count2 = 0;
    interrupts();

    Serial.printf("  [10s] CRD1(G8)=%lu  CRD2(G3)=%lu  %s\n",
                  static_cast<unsigned long>(c1),
                  static_cast<unsigned long>(c2),
                  (c1 > 0 && c2 > 0) ? "[BOTH OK]"
                  : (c1 > 0)         ? "[CRD2 SILENT - check GPIO 3 / IF12]"
                  : (c2 > 0)         ? "[CRD1 SILENT - check GPIO 8 / IF11]"
                                     : "[no pulses - tube powered?]");
}
