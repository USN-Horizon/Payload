// Raw SPI-level CMD0 (GO_IDLE_STATE) probe for the SD card.
//
// Why this test exists:
//   Production fails to mount the SD card ~75% of boots with the error
//   "Card Failed! cmd: 0x00" - meaning the card is not returning a valid
//   R1 response to CMD0. The Arduino SD library handles this at a higher
//   level, so we never see WHAT the card actually returned, only that the
//   library gave up.
//
// What this test does:
//   Bypasses the SD library and talks to the card directly over SPI.
//   Sends CMD0 with the correct frame (0x40 0x00 0x00 0x00 0x00 0x95),
//   reads the response byte, and reports it verbatim. Several variations
//   of warm-up cycles, SPI clock, and "force-reset" attempts are tried
//   so we can correlate which conditions get a valid response and which
//   don't.
//
// Interpretation key:
//   R1 = 0x01 -> Card responded correctly: "in idle state". Hardware fine.
//   R1 = 0xFF -> No response at all. Card not responding to SPI
//                (loose contact, missing card, wedged card, dead card).
//   R1 = 0x00 -> Card responded but with "no error" R1, unusual.
//   R1 = other -> Card responded with some other state (e.g. 0x05 =
//                 illegal command). Useful info: the card IS on the bus.
//
// Use:
//   1. Run env:new until you see a production SD-mount failure.
//   2. WITHOUT removing or reseating the card, flash this test.
//   3. Read the output. The CMD0 byte tells us whether the failure is
//      "card not responding at all" (hardware) or "card responding but
//      something else is wrong" (firmware/protocol).
//
// Upload:  pio run -e test_sd_raw_cmd0 -t upload
// Monitor: pio device monitor -e test_sd_raw_cmd0

#include <Arduino.h>
#include <SPI.h>

static constexpr int PIN_SPI_MOSI = 4;
static constexpr int PIN_SPI_SCK  = 5;
static constexpr int PIN_SPI_MISO = 6;
static constexpr int PIN_SD_CS    = 21;

// SD power-up "init clocks": 74+ SCK cycles with CS HIGH and MOSI HIGH.
// We do at least 10 dummy bytes (80 cycles) by default.
static void sdWarmupClocks(uint16_t clockCount, uint32_t spiHz) {
    SPI.beginTransaction(SPISettings(spiHz, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_SD_CS, HIGH);
    uint16_t bytes = (clockCount + 7) / 8;
    for (uint16_t i = 0; i < bytes; ++i) {
        SPI.transfer(0xFF);
    }
    SPI.endTransaction();
}

// Send a 6-byte command frame with CS LOW and read response bytes,
// waiting for the first one whose MSB is 0 (a valid R1). Returns the
// first valid byte found within `maxAttempts` reads, or 0xFF if none.
static uint8_t sdSendCommand(uint8_t cmd, uint32_t arg, uint8_t crc,
                             uint32_t spiHz, uint8_t maxAttempts = 16) {
    SPI.beginTransaction(SPISettings(spiHz, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_SD_CS, LOW);

    SPI.transfer(uint8_t(0x40 | (cmd & 0x3F)));
    SPI.transfer(uint8_t((arg >> 24) & 0xFF));
    SPI.transfer(uint8_t((arg >> 16) & 0xFF));
    SPI.transfer(uint8_t((arg >>  8) & 0xFF));
    SPI.transfer(uint8_t( arg        & 0xFF));
    SPI.transfer(crc);

    uint8_t r1 = 0xFF;
    for (uint8_t i = 0; i < maxAttempts; ++i) {
        r1 = SPI.transfer(0xFF);
        if ((r1 & 0x80) == 0) break;
    }

    digitalWrite(PIN_SD_CS, HIGH);
    SPI.transfer(0xFF);  // 8 idle clocks after deselect
    SPI.endTransaction();
    return r1;
}

static const char* describeR1(uint8_t r) {
    if (r == 0xFF) return "no response";
    if (r == 0x01) return "idle - OK";
    if (r == 0x00) return "no error";
    if (r & 0x04)  return "illegal command";
    if (r & 0x40)  return "param error";
    if (r & 0x20)  return "address error";
    if (r & 0x10)  return "erase seq error";
    if (r & 0x08)  return "crc error";
    if (r & 0x02)  return "erase reset";
    return "other";
}

void setup() {
    Serial.begin(115200);
    delay(800);

    Serial.println();
    Serial.println("==================================================");
    Serial.println("  RAW SD CMD0 probe - hardware vs firmware test");
    Serial.println("==================================================");

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, HIGH);

    // Probe MISO before any SD interaction: if it reads LOW with internal
    // pull-up applied, the card or breakout is actively holding MISO LOW
    // (which would prevent any normal SD response).
    pinMode(PIN_SPI_MISO, INPUT_PULLUP);
    delay(2);
    int misoPullUp = digitalRead(PIN_SPI_MISO);
    pinMode(PIN_SPI_MISO, INPUT_PULLDOWN);
    delay(2);
    int misoPullDn = digitalRead(PIN_SPI_MISO);
    pinMode(PIN_SPI_MISO, INPUT);
    Serial.printf("\n  [INFO] idle MISO: pullup=%d pulldown=%d "
                  "(both 1 = SD breakout pull-up, OK; "
                  "both 0 = something pulling LOW)\n",
                  misoPullUp, misoPullDn);

    // ----------- Variation series -----------
    uint8_t r;

    Serial.println("\n--- [A] 80 warm-up clocks, CMD0 @ 400 kHz ---");
    sdWarmupClocks(80, 400000);
    r = sdSendCommand(0x00, 0x00000000, 0x95, 400000);
    Serial.printf("  R1 = 0x%02X (%s)\n", r, describeR1(r));

    Serial.println("\n--- [B] 200 warm-up clocks, CMD0 @ 400 kHz ---");
    sdWarmupClocks(200, 400000);
    r = sdSendCommand(0x00, 0x00000000, 0x95, 400000);
    Serial.printf("  R1 = 0x%02X (%s)\n", r, describeR1(r));

    Serial.println("\n--- [C] 2000 warm-up clocks, CMD0 @ 400 kHz ---");
    sdWarmupClocks(2000, 400000);
    r = sdSendCommand(0x00, 0x00000000, 0x95, 400000);
    Serial.printf("  R1 = 0x%02X (%s)\n", r, describeR1(r));

    Serial.println("\n--- [D] 200 warm-up clocks @ 100 kHz, CMD0 @ 100 kHz ---");
    sdWarmupClocks(200, 100000);
    r = sdSendCommand(0x00, 0x00000000, 0x95, 100000);
    Serial.printf("  R1 = 0x%02X (%s)\n", r, describeR1(r));

    Serial.println("\n--- [E] Send CMD12 (STOP_TRANSMISSION) first, then CMD0 ---");
    sdWarmupClocks(200, 400000);
    uint8_t r12 = sdSendCommand(0x0C, 0x00000000, 0x61, 400000);
    Serial.printf("  CMD12 R1 = 0x%02X (%s)\n", r12, describeR1(r12));
    sdWarmupClocks(200, 400000);
    r = sdSendCommand(0x00, 0x00000000, 0x95, 400000);
    Serial.printf("  CMD0  R1 = 0x%02X (%s)\n", r, describeR1(r));

    Serial.println("\n--- [F] 10x CMD0 attempts (each with fresh 80-clk warmup) ---");
    int successes = 0;
    int responseAtAll = 0;
    for (int i = 0; i < 10; ++i) {
        sdWarmupClocks(80, 400000);
        r = sdSendCommand(0x00, 0x00000000, 0x95, 400000);
        Serial.printf("  [%d] R1 = 0x%02X (%s)\n", i, r, describeR1(r));
        if (r == 0x01) successes++;
        if (r != 0xFF) responseAtAll++;
        delay(50);
    }
    Serial.printf("  TOTAL: %d/10 returned R1=0x01, %d/10 returned anything "
                  "other than 0xFF\n", successes, responseAtAll);

    Serial.println("\n==================================================");
    Serial.println("  Verdict guide:");
    Serial.println("  - All 0xFF                -> Card never responds.");
    Serial.println("                                Hardware: bad card,");
    Serial.println("                                loose contact, or no");
    Serial.println("                                pull-up on MISO. Swap");
    Serial.println("                                the card.");
    Serial.println("  - Mostly 0x01             -> Card is fine; the SD");
    Serial.println("                                library or its timing");
    Serial.println("                                is the problem.");
    Serial.println("  - Mixed 0x01 / 0xFF       -> Card responds inter-");
    Serial.println("                                mittently; marginal");
    Serial.println("                                contact or power.");
    Serial.println("  - 0x05 / other non-0xFF   -> Card on the bus but in");
    Serial.println("                                a non-idle state.");
    Serial.println("==================================================");
    Serial.println("  Restarting in 8 s...");
    delay(8000);
    ESP.restart();
}

void loop() {}
