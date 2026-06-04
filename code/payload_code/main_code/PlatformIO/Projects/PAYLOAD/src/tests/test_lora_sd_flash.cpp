// Component test: LoRa + SD card + LittleFS flash interaction
//
// Diagnoses the symptom: "SD card stops working when the LoRa PCB is
// connected to the mainboard". All three peripherals share the same
// SPI bus (SCK=5, MISO=6, MOSI=4) so the most common root causes are:
//
//   1. MISO bus contention - the SX1280 is not properly deselected
//      (NSS floating or LOW), so it keeps driving MISO while the SD
//      transaction is in progress.
//   2. Power brown-out - the LoRa PCB pulls enough current that the
//      3.3 V rail dips and the SD card's init handshake fails.
//   3. LoRa stuck in reset / unknown state - RST or PWR_EN floating.
//   4. Pin conflict - something on the LoRa PCB is electrically
//      tied to one of the SD pins.
//
// The test runs a sequence of stages so the failure point can be
// located precisely from the serial log:
//
//   [A] SD alone, LoRa power OFF, all LoRa pins INPUT (high-Z)
//   [B] SD with LoRa pins parked (NSS=HIGH, RST=HIGH), still no power
//   [C] SD with LoRa power ON, but radio NOT begin()'d
//   [D] SD after radio.begin() succeeds (radio idle on bus)
//   [E] Interleaved: SD write -> LoRa TX -> SD read-back, repeated
//   [F] SD while LoRa TX is in flight (BUSY high)
//   [H] SD mount retry burst (10x, LoRa idle) - catches transient init.
//   [I] SD SPI clock sweep - find highest stable speed with LoRa idle.
//   [J] LoRa SPI round-trip after SD activity - confirms radio still alive.
//   [K] MISO / LoRa CS / BUSY line probe - detects bus contention directly.
//   [L] 20-round heavy interleave, reports first failing iteration.
//   [M] DIO1 watch during a single TX - does TxDone IRQ ever fire?
//   [N] Read SX1280 IRQ status after TX - does the radio think TX done?
//   [P] MISO probe with SX1280 in RESET - isolates pull-up source.
//   [Q] DIO1 vs DIO2 during TX - detects swapped pins.
//   [R] DIO1 connectivity probe - tells if GPIO11 reaches the chip pad.
//   [S] Polling-based TX (no DIO1) - firmware workaround proof.
//   [G] LittleFS sanity (internal flash, independent of SPI)
//
// PASS criteria: every SD write/read in every stage succeeds.
// If A passes and B/C/D/E/F fail, the log tells you exactly which
// interaction breaks the bus.
//
// Upload:  pio run -e test_lora_sd_flash -t upload
// Monitor: pio device monitor -e test_lora_sd_flash

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <SD.h>
#include <LittleFS.h>

// Pin map - must match include/New/Config.hpp
static constexpr int PIN_SPI_MOSI  = 4;
static constexpr int PIN_SPI_SCK   = 5;
static constexpr int PIN_SPI_MISO  = 6;

static constexpr int PIN_LORA_CS   = 9;   // IF13
static constexpr int PIN_LORA_PWR  = 10;  // IF14
static constexpr int PIN_LORA_DIO1 = 11;  // IF15
static constexpr int PIN_LORA_DIO2 = 12;  // IF16
static constexpr int PIN_LORA_BUSY = 13;  // IF17
static constexpr int PIN_LORA_RST  = 14;  // IF18

static constexpr int PIN_SD_CS     = 21;

static const char* SD_FILE      = "/lora_sd_test.txt";
static const char* FLASH_FILE   = "/lora_flash_test.txt";
static const char* TEST_PAYLOAD = "LORA_SD_FLASH_OK";

static int passCount = 0;
static int failCount = 0;

static void result(const char* label, bool pass) {
    Serial.printf("  [%s] %s\n", pass ? "PASS" : "FAIL", label);
    if (pass) passCount++; else failCount++;
}

// Park all LoRa control pins in a known idle state. NSS HIGH = chip
// deselected, RST HIGH = out of reset (no power means it draws nothing).
static void parkLoraPins() {
    pinMode(PIN_LORA_CS,  OUTPUT); digitalWrite(PIN_LORA_CS,  HIGH);
    pinMode(PIN_LORA_RST, OUTPUT); digitalWrite(PIN_LORA_RST, HIGH);
    pinMode(PIN_LORA_PWR, OUTPUT); digitalWrite(PIN_LORA_PWR, LOW);
    pinMode(PIN_LORA_DIO1, INPUT);
    pinMode(PIN_LORA_DIO2, INPUT);
    pinMode(PIN_LORA_BUSY, INPUT);
}

static void floatLoraPins() {
    pinMode(PIN_LORA_CS,   INPUT);
    pinMode(PIN_LORA_RST,  INPUT);
    pinMode(PIN_LORA_PWR,  INPUT);
    pinMode(PIN_LORA_DIO1, INPUT);
    pinMode(PIN_LORA_DIO2, INPUT);
    pinMode(PIN_LORA_BUSY, INPUT);
}

static bool sdMount() {
    return SD.begin(PIN_SD_CS, SPI, 8000000);
}

static bool sdWriteRead(const char* tag) {
    SD.remove(SD_FILE);

    File fw = SD.open(SD_FILE, FILE_WRITE);
    bool writeOk = false;
    if (fw) {
        writeOk = (fw.print(TEST_PAYLOAD) > 0);
        fw.close();
    }

    File fr = SD.open(SD_FILE, FILE_READ);
    bool readOk = false;
    String content;
    if (fr) {
        content = fr.readString();
        fr.close();
        readOk = (content == String(TEST_PAYLOAD));
    }

    SD.remove(SD_FILE);

    Serial.printf("  [INFO] %s: write=%s read=%s data=\"%s\"\n",
                  tag,
                  writeOk ? "OK" : "FAIL",
                  readOk  ? "OK" : "FAIL",
                  content.c_str());
    return writeOk && readOk;
}

static void dumpPins(const char* tag) {
    Serial.printf("  [PINS %s] LORA_CS=%d LORA_BUSY=%d LORA_RST=%d "
                  "LORA_DIO1=%d LORA_DIO2=%d LORA_PWR=%d SD_CS=%d\n",
                  tag,
                  digitalRead(PIN_LORA_CS),
                  digitalRead(PIN_LORA_BUSY),
                  digitalRead(PIN_LORA_RST),
                  digitalRead(PIN_LORA_DIO1),
                  digitalRead(PIN_LORA_DIO2),
                  digitalRead(PIN_LORA_PWR),
                  digitalRead(PIN_SD_CS));
}

void setup() {
    Serial.begin(115200);
    delay(800);

    Serial.println();
    Serial.println("==================================================");
    Serial.println("  COMPONENT TEST: LoRa + SD + LittleFS interaction");
    Serial.println("==================================================");

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
    pinMode(PIN_SD_CS, OUTPUT); digitalWrite(PIN_SD_CS, HIGH);
    Serial.println("  [INFO] SPI bus started (SCK=5 MISO=6 MOSI=4)");

    // -------------------------------------------------------------------
    // [A] SD alone, every LoRa pin floating (high-Z).
    // If this fails, the issue is not LoRa interaction at all - check
    // SD wiring, card, or supply.
    // -------------------------------------------------------------------
    Serial.println("\n--- [A] SD alone, LoRa pins floating (high-Z) ---");
    floatLoraPins();
    dumpPins("A-pre");
    {
        bool mounted = sdMount();
        Serial.printf("  [INFO] SD.begin() = %s\n", mounted ? "true" : "false");
        result("[A] SD mount with LoRa pins floating", mounted);
        if (mounted) {
            result("[A] SD write/read with LoRa floating", sdWriteRead("A"));
            SD.end();
        }
    }

    // -------------------------------------------------------------------
    // [B] SD with LoRa control pins parked, no LoRa supply.
    // If A passes but B fails, NSS or RST is leaking into the bus even
    // without supply (e.g. ESD diode path through the SX1280 to MISO).
    // -------------------------------------------------------------------
    Serial.println("\n--- [B] SD with LoRa pins parked, power OFF ---");
    parkLoraPins();
    delay(20);
    dumpPins("B-pre");
    {
        bool mounted = sdMount();
        Serial.printf("  [INFO] SD.begin() = %s\n", mounted ? "true" : "false");
        result("[B] SD mount with LoRa parked, no power", mounted);
        if (mounted) {
            result("[B] SD write/read with LoRa parked, no power",
                   sdWriteRead("B"));
            SD.end();
        }
    }

    // -------------------------------------------------------------------
    // [C] SD with LoRa supply ON, radio NOT begin()'d.
    // If B passes and C fails, supply brown-out is likely, OR the SX1280
    // is powering up into a state that pulls MISO before reset.
    // -------------------------------------------------------------------
    Serial.println("\n--- [C] SD with LoRa power ON, radio not init'd ---");
    pinMode(PIN_LORA_CS,  OUTPUT); digitalWrite(PIN_LORA_CS,  HIGH);
    pinMode(PIN_LORA_RST, OUTPUT); digitalWrite(PIN_LORA_RST, LOW);   // hold in reset
    pinMode(PIN_LORA_PWR, OUTPUT); digitalWrite(PIN_LORA_PWR, HIGH);
    delay(50);
    digitalWrite(PIN_LORA_RST, HIGH);  // release reset, leave radio idle
    delay(20);
    dumpPins("C-pre");
    {
        bool mounted = sdMount();
        Serial.printf("  [INFO] SD.begin() = %s\n", mounted ? "true" : "false");
        result("[C] SD mount with LoRa powered (idle)", mounted);
        if (mounted) {
            result("[C] SD write/read with LoRa powered (idle)",
                   sdWriteRead("C"));
            SD.end();
        }
    }

    // -------------------------------------------------------------------
    // [D] Bring SX1280 fully online via RadioLib, THEN test SD.
    // If C passes and D fails, RadioLib's begin() is leaving NSS or MISO
    // in a bad state, or its SPISettings clash with the SD driver's.
    // -------------------------------------------------------------------
    Serial.println("\n--- [D] SD after radio.begin() succeeds ---");
    Module* mod = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RST,
                             PIN_LORA_BUSY, SPI,
                             SPISettings(2000000, MSBFIRST, SPI_MODE0));
    SX1280 radio(mod);

    int rstate = radio.begin();
    Serial.printf("  [INFO] radio.begin() = %d  (0 = OK)\n", rstate);
    bool radioOk = (rstate == RADIOLIB_ERR_NONE);
    result("[D] radio.begin() (LoRa init)", radioOk);

    if (radioOk) {
        radio.setFrequency(2400.0f);
        radio.setBandwidth(812.5f);
        radio.setSpreadingFactor(7);
        radio.setCodingRate(5);
        radio.setOutputPower(13);
        radio.standby();   // park radio idle, not in RX
    }

    dumpPins("D-pre");
    {
        bool mounted = sdMount();
        Serial.printf("  [INFO] SD.begin() = %s\n", mounted ? "true" : "false");
        result("[D] SD mount after radio.begin()", mounted);
        if (mounted) {
            result("[D] SD write/read after radio.begin()", sdWriteRead("D"));
            SD.end();
        }
    }

    // -------------------------------------------------------------------
    // [E] Interleave LoRa TX with SD writes for several rounds.
    // If D passes and E fails, sharing the bus under real load is the
    // problem. Audit beginTransaction()/endTransaction() pairing and
    // whether the SX1280 is being deselected after each SPI burst.
    // -------------------------------------------------------------------
    Serial.println("\n--- [E] Interleaved: TX -> SD write/read x 5 ---");
    if (radioOk) {
        bool mounted = sdMount();
        result("[E] SD mount before interleave", mounted);
        if (mounted) {
            bool allOk = true;
            for (int i = 0; i < 5; ++i) {
                String msg = "PKT_" + String(i);
                int s = radio.transmit(msg);
                Serial.printf("  [INFO] iter %d: transmit() = %d\n", i, s);
                bool sdOk = sdWriteRead("E");
                if (s != RADIOLIB_ERR_NONE || !sdOk) allOk = false;
                delay(50);
            }
            result("[E] 5x interleaved TX + SD write/read", allOk);
            SD.end();
        }
    } else {
        result("[E] interleaved (skipped - radio failed)", false);
    }

    // -------------------------------------------------------------------
    // [F] Start a non-blocking TX, then access SD while BUSY may still
    // be HIGH. If E passes and F fails, an in-flight LoRa command is
    // disturbing the SD driver (e.g. NSS held LOW by RadioLib's poll
    // loop, or SX1280 driving MISO mid-TX).
    // -------------------------------------------------------------------
    Serial.println("\n--- [F] SD while LoRa TX in flight ---");
    if (radioOk) {
        bool mounted = sdMount();
        result("[F] SD mount", mounted);
        if (mounted) {
            int s = radio.startTransmit("BUSY_TEST_PAYLOAD");
            Serial.printf("  [INFO] startTransmit() = %d  busy=%d\n",
                          s, digitalRead(PIN_LORA_BUSY));
            bool sdOk = sdWriteRead("F-during-TX");
            uint32_t t0 = millis();
            while (digitalRead(PIN_LORA_BUSY) == HIGH && millis() - t0 < 2000) {}
            radio.finishTransmit();
            result("[F] SD write/read during in-flight TX",
                   sdOk && (s == RADIOLIB_ERR_NONE));
            SD.end();
        }
    } else {
        result("[F] SD during TX (skipped - radio failed)", false);
    }

    // -------------------------------------------------------------------
    // [H] SD mount retry burst with LoRa idle.
    // SD initialization is finicky and can fail on the first attempt
    // even when the wiring is fine. If this reports 9-10/10 successes,
    // any earlier mount failure was probably transient: bump retries.
    // If it reports <5/10, mount-time stability is genuinely poor.
    // -------------------------------------------------------------------
    Serial.println("\n--- [H] SD mount retry burst (10x, LoRa idle) ---");
    if (radioOk) {
        int mountSuccesses = 0;
        for (int i = 0; i < 10; ++i) {
            bool ok = sdMount();
            if (ok) {
                mountSuccesses++;
                SD.end();
            }
            delay(50);
        }
        Serial.printf("  [INFO] SD.begin() succeeded %d/10 times\n",
                      mountSuccesses);
        result("[H] SD mounts >= 9/10 times", mountSuccesses >= 9);
    } else {
        result("[H] retry burst (skipped - radio failed)", false);
    }

    // -------------------------------------------------------------------
    // [I] SD SPI clock sweep.
    // Probe each speed from 400 kHz up to 25 MHz. If only the lowest
    // speeds work, signal integrity / brown-out is hurting the SD at
    // higher clocks. If all speeds work, signal integrity is fine and
    // the earlier failures must come from something else (NSS, ordering).
    // -------------------------------------------------------------------
    Serial.println("\n--- [I] SD SPI clock sweep ---");
    if (radioOk) {
        const uint32_t speeds[] = {400000UL, 1000000UL, 4000000UL,
                                   8000000UL, 16000000UL, 20000000UL,
                                   25000000UL};
        long bestSpeed = -1;
        for (size_t i = 0; i < sizeof(speeds) / sizeof(speeds[0]); ++i) {
            bool mounted = SD.begin(PIN_SD_CS, SPI, speeds[i]);
            bool rwOk = false;
            if (mounted) {
                rwOk = sdWriteRead("I");
                SD.end();
            }
            Serial.printf("  [INFO] @%lu Hz: mount=%s rw=%s\n",
                          (unsigned long)speeds[i],
                          mounted ? "OK" : "FAIL",
                          rwOk ? "OK" : "FAIL");
            if (mounted && rwOk) bestSpeed = static_cast<long>(speeds[i]);
            delay(50);
        }
        Serial.printf("  [INFO] Highest stable speed: %ld Hz\n", bestSpeed);
        result("[I] At least 1 MHz stable", bestSpeed >= 1000000L);
    } else {
        result("[I] clock sweep (skipped - radio failed)", false);
    }

    // -------------------------------------------------------------------
    // [J] LoRa SPI round-trip BEFORE and AFTER touching SD.
    // standby() and getRSSI() both perform SPI transactions with the
    // SX1280. If standby() returns OK before SD activity but errors
    // after, then SD operations corrupted the LoRa SPI state (most
    // likely an unbalanced beginTransaction/endTransaction pair).
    // -------------------------------------------------------------------
    Serial.println("\n--- [J] LoRa SPI still responsive after SD activity ---");
    if (radioOk) {
        int s1 = radio.standby();
        bool mounted = sdMount();
        bool rwOk = false;
        if (mounted) {
            rwOk = sdWriteRead("J");
            SD.end();
        }
        int s2 = radio.standby();
        float rssi = radio.getRSSI();
        Serial.printf("  [INFO] pre-SD standby=%d  post-SD standby=%d  "
                      "RSSI=%.1f dBm\n", s1, s2, rssi);
        result("[J] LoRa standby OK before and after SD",
               s1 == RADIOLIB_ERR_NONE && s2 == RADIOLIB_ERR_NONE && rwOk);
    } else {
        result("[J] LoRa responsiveness (skipped - radio failed)", false);
    }

    // -------------------------------------------------------------------
    // [K] MISO / LoRa CS / BUSY line probe.
    // With both CS pins HIGH, no slave should be driving MISO. The pin
    // is therefore floating. We force MISO into an INPUT_PULLUP and
    // then INPUT_PULLDOWN configuration and read it back: a truly
    // floating line follows the pull, but a line that is being driven
    // ignores the pull and reads the same value either way (-> bus
    // contention). LORA_CS must read HIGH and LORA_BUSY LOW when idle.
    // -------------------------------------------------------------------
    Serial.println("\n--- [K] MISO / LoRa line probe (bus contention) ---");
    {
        pinMode(PIN_LORA_CS, OUTPUT); digitalWrite(PIN_LORA_CS, HIGH);
        pinMode(PIN_SD_CS,   OUTPUT); digitalWrite(PIN_SD_CS,   HIGH);
        delay(5);

        pinMode(PIN_SPI_MISO, INPUT_PULLUP);
        delay(1);
        int misoPullUp = digitalRead(PIN_SPI_MISO);
        pinMode(PIN_SPI_MISO, INPUT_PULLDOWN);
        delay(1);
        int misoPullDown = digitalRead(PIN_SPI_MISO);
        pinMode(PIN_SPI_MISO, INPUT);  // SPI driver will reconfigure later.

        Serial.printf("  [INFO] MISO with pullup=%d  with pulldown=%d  "
                      "(diff -> floating OK; same -> bus contention)\n",
                      misoPullUp, misoPullDown);
        bool misoFloating = (misoPullUp == HIGH) && (misoPullDown == LOW);
        result("[K] MISO floats when both CS are HIGH", misoFloating);

        int csNow   = digitalRead(PIN_LORA_CS);
        int busyNow = digitalRead(PIN_LORA_BUSY);
        Serial.printf("  [INFO] LORA_CS=%d (expect 1)  LORA_BUSY=%d "
                      "(expect 0 when idle)\n", csNow, busyNow);
        result("[K] LORA_CS HIGH and LORA_BUSY LOW when idle",
               csNow == HIGH && busyNow == LOW);
    }

    // -------------------------------------------------------------------
    // [L] Heavy interleave stress (20 rounds), reports first failing iter.
    // Stage E only does 5 rounds and only reports a single pass/fail.
    // This stage runs 20 and prints the index of the first failure (if
    // any), so an intermittent fault that needs N seconds to appear is
    // easier to spot.
    // -------------------------------------------------------------------
    Serial.println("\n--- [L] Heavy interleave stress (20x, abort on first fail) ---");
    if (radioOk) {
        bool mounted = sdMount();
        result("[L] SD mount before stress", mounted);
        if (mounted) {
            int firstFailIter = -1;
            int lastTxStatus  = RADIOLIB_ERR_NONE;
            bool lastSdOk     = true;
            for (int i = 0; i < 20; ++i) {
                String msg = "STRESS_" + String(i);
                int s = radio.transmit(msg);
                bool sdOk = sdWriteRead("L");
                lastTxStatus = s;
                lastSdOk     = sdOk;
                if (s != RADIOLIB_ERR_NONE || !sdOk) {
                    firstFailIter = i;
                    break;
                }
            }
            if (firstFailIter < 0) {
                Serial.println("  [INFO] all 20 iterations passed");
            } else {
                Serial.printf("  [INFO] FAIL at iter %d: tx=%d sd=%s\n",
                              firstFailIter, lastTxStatus,
                              lastSdOk ? "OK" : "FAIL");
            }
            result("[L] 20x stress with no failures", firstFailIter < 0);
            SD.end();
        }
    } else {
        result("[L] stress (skipped - radio failed)", false);
    }

    // -------------------------------------------------------------------
    // [M] DIO1 watch during a single transmit attempt.
    // -5 (RADIOLIB_ERR_TX_TIMEOUT) means RadioLib never saw TxDone. This
    // stage polls DIO1 directly after startTransmit() and reports
    // whether (and when) DIO1 goes HIGH. If DIO1 NEVER goes HIGH:
    //   - either the SX1280's IRQ mask doesn't route TxDone to DIO1
    //     (firmware config issue), or
    //   - the physical pin (GPIO 11) is not actually wired to the
    //     SX1280's DIO1 pad (PCB / pin-map issue).
    // If DIO1 DOES go HIGH but transmit() still returns -5, RadioLib
    // is missing the edge - check that the SPI bus isn't held by
    // another driver while RadioLib polls.
    // -------------------------------------------------------------------
    Serial.println("\n--- [M] DIO1 watch during single TX (does TxDone fire?) ---");
    if (radioOk) {
        radio.standby();
        radio.clearIrqFlags(0xFFFFFFFFu);
        int dio1Pre  = digitalRead(PIN_LORA_DIO1);
        int busyPre  = digitalRead(PIN_LORA_BUSY);
        int s = radio.startTransmit("DIO1_TEST");
        uint32_t t0 = millis();
        bool dio1Fired = false;
        uint32_t dio1FireMs = 0;
        int dio1Last = dio1Pre;
        int busyLast = busyPre;
        while (millis() - t0 < 200) {
            int d = digitalRead(PIN_LORA_DIO1);
            if (d == HIGH && !dio1Fired) {
                dio1Fired = true;
                dio1FireMs = millis() - t0;
            }
            dio1Last = d;
            busyLast = digitalRead(PIN_LORA_BUSY);
        }
        Serial.printf("  [INFO] startTransmit=%d  pre: DIO1=%d BUSY=%d\n",
                      s, dio1Pre, busyPre);
        Serial.printf("  [INFO] DIO1 fired=%d  at t=%lu ms  final DIO1=%d "
                      "BUSY=%d\n",
                      dio1Fired ? 1 : 0, (unsigned long)dio1FireMs,
                      dio1Last, busyLast);
        result("[M] DIO1 asserts within 200 ms (TxDone IRQ fires)",
               dio1Fired);
    } else {
        result("[M] DIO1 watch (skipped - radio failed)", false);
    }

    // -------------------------------------------------------------------
    // [N] Read SX1280 IRQ status after a transmit attempt.
    // This tells us what the radio THINKS happened, independent of
    // RadioLib's polling. SX1280 IRQ bits we care about:
    //   bit 0 = TxDone       -> TX completed
    //   bit 1 = RxDone
    //   bit 6 = CrcError
    //   bit 9 = RxTxTimeout  -> radio aborted internally
    // If TxDone is set but DIO1 didn't fire in [M], the IRQ mask isn't
    // routing TxDone to DIO1. If neither TxDone nor RxTxTimeout is set,
    // the radio never actually entered TX mode (PA / RFSWITCH issue).
    // -------------------------------------------------------------------
    Serial.println("\n--- [N] SX1280 IRQ status after TX attempt ---");
    if (radioOk) {
        radio.standby();
        radio.clearIrqFlags(0xFFFFFFFFu);
        int s = radio.startTransmit("IRQ_TEST");
        delay(80);  // generous TOA budget for a short payload
        uint16_t irq = radio.getIrqStatus();
        Serial.printf("  [INFO] startTransmit=%d  IRQ=0x%04X\n", s, irq);
        Serial.printf("  [INFO]   TxDone=%d  RxDone=%d  CrcError=%d  "
                      "RxTxTimeout=%d\n",
                      (irq >> 0) & 1,
                      (irq >> 1) & 1,
                      (irq >> 6) & 1,
                      (irq >> 9) & 1);
        result("[N] IRQ TxDone bit set after TX attempt",
               (irq & 0x0001) != 0);
        radio.clearIrqFlags(0xFFFFFFFFu);
    } else {
        result("[N] IRQ status (skipped - radio failed)", false);
    }

    // -------------------------------------------------------------------
    // [P] MISO probe with SX1280 held in RESET.
    // Re-run the [K] MISO test but with PIN_LORA_RST = LOW so the
    // SX1280 cannot drive any pin. If MISO still reads HIGH under
    // both pull-up and pull-down, the bias is from the SD breakout
    // (a board-level pull-up on MISO, which is very common) and the
    // earlier [K] FAIL is a benign false positive.
    // If MISO becomes floating (follows the pull) only when SX1280 is
    // in reset, the SX1280 IS driving MISO even when NSS is HIGH -
    // a real bus-contention bug.
    // -------------------------------------------------------------------
    Serial.println("\n--- [P] MISO probe with SX1280 held in RESET ---");
    {
        pinMode(PIN_LORA_RST, OUTPUT); digitalWrite(PIN_LORA_RST, LOW);
        pinMode(PIN_LORA_CS,  OUTPUT); digitalWrite(PIN_LORA_CS,  HIGH);
        pinMode(PIN_SD_CS,    OUTPUT); digitalWrite(PIN_SD_CS,    HIGH);
        delay(5);

        pinMode(PIN_SPI_MISO, INPUT_PULLUP);
        delay(1);
        int misoPullUp = digitalRead(PIN_SPI_MISO);
        pinMode(PIN_SPI_MISO, INPUT_PULLDOWN);
        delay(1);
        int misoPullDown = digitalRead(PIN_SPI_MISO);
        pinMode(PIN_SPI_MISO, INPUT);

        Serial.printf("  [INFO] (SX1280 in RESET) MISO pullup=%d  pulldown=%d\n",
                      misoPullUp, misoPullDown);
        bool misoFloating = (misoPullUp == HIGH) && (misoPullDown == LOW);
        bool misoStuckHigh = (misoPullUp == HIGH) && (misoPullDown == HIGH);
        if (misoFloating) {
            Serial.println("  [INFO] MISO floats with SX1280 in reset -> "
                           "earlier [K] FAIL was SX1280 driving MISO.");
        } else if (misoStuckHigh) {
            Serial.println("  [INFO] MISO still stuck HIGH with SX1280 in "
                           "reset -> bias is on the SD breakout (board "
                           "pull-up); [K] FAIL is benign.");
        }
        result("[P] MISO floats with SX1280 in reset", misoFloating);

        // Release reset so subsequent stages can use the radio if needed.
        digitalWrite(PIN_LORA_RST, HIGH);
        delay(10);
    }

    // -------------------------------------------------------------------
    // [Q] During a TX, poll BOTH DIO1 and DIO2.
    // SX1280 has 3 DIO pins; the silkscreen vs the actual pad routing
    // can disagree. If TxDone shows up on DIO2 instead of DIO1, the
    // pins are wired swapped on the PCB.
    // Interpretation:
    //   - DIO1 fires      -> previous [M] result was wrong (unlikely);
    //                        check whether [M]'s poll loop missed it.
    //   - DIO2 fires      -> pin swap: in firmware change PIN_LORA_DIO1
    //                        to 12 (or whatever GPIO is wired to the
    //                        true DIO1 pad), or rework the PCB.
    //   - Neither fires   -> DIO1 line is open (proceed to [R] / [S]).
    // -------------------------------------------------------------------
    Serial.println("\n--- [Q] DIO1 vs DIO2 watch during TX (swap detect) ---");
    if (radioOk) {
        radio.standby();
        radio.clearIrqFlags(0xFFFFFFFFu);
        pinMode(PIN_LORA_DIO1, INPUT);
        pinMode(PIN_LORA_DIO2, INPUT);
        int s = radio.startTransmit("DIO_SWAP_TEST");
        uint32_t t0 = millis();
        bool dio1Fired = false, dio2Fired = false;
        uint32_t dio1Ms = 0, dio2Ms = 0;
        while (millis() - t0 < 200) {
            if (digitalRead(PIN_LORA_DIO1) == HIGH && !dio1Fired) {
                dio1Fired = true;  dio1Ms = millis() - t0;
            }
            if (digitalRead(PIN_LORA_DIO2) == HIGH && !dio2Fired) {
                dio2Fired = true;  dio2Ms = millis() - t0;
            }
            if (dio1Fired && dio2Fired) break;
        }
        Serial.printf("  [INFO] startTransmit=%d\n", s);
        Serial.printf("  [INFO] DIO1 fired=%d @%lu ms   DIO2 fired=%d @%lu ms\n",
                      dio1Fired ? 1 : 0, (unsigned long)dio1Ms,
                      dio2Fired ? 1 : 0, (unsigned long)dio2Ms);
        if (dio2Fired && !dio1Fired) {
            Serial.println("  [INFO] Pin swap: TxDone shows up on DIO2 line, "
                           "not DIO1. Remap PIN_LORA_DIO1 to GPIO 12.");
        }
        result("[Q] At least one DIO line fires (TxDone reaches a pin)",
               dio1Fired || dio2Fired);
    } else {
        result("[Q] DIO swap test (skipped - radio failed)", false);
    }

    // -------------------------------------------------------------------
    // [R] DIO1 line connectivity probe.
    // With the SX1280 in standby (DIO outputs LOW when no IRQ active),
    // configure GPIO 11 as INPUT_PULLUP, then INPUT_PULLDOWN, and read.
    // If both reads follow the pull (pullup=1, pulldown=0), the line
    // is electrically open -> GPIO 11 is not connected to anything,
    // confirming a PCB / pin-map fault.
    // If both reads are 0 regardless of pull, the chip is actively
    // driving DIO1 LOW (so the trace IS connected) and the [M] failure
    // must be a mask config issue, not wiring.
    // -------------------------------------------------------------------
    Serial.println("\n--- [R] DIO1 connectivity probe (SX1280 in standby) ---");
    if (radioOk) {
        radio.standby();
        radio.clearIrqFlags(0xFFFFFFFFu);
        delay(5);
        pinMode(PIN_LORA_DIO1, INPUT_PULLUP);
        delay(2);
        int dio1Up = digitalRead(PIN_LORA_DIO1);
        pinMode(PIN_LORA_DIO1, INPUT_PULLDOWN);
        delay(2);
        int dio1Dn = digitalRead(PIN_LORA_DIO1);
        pinMode(PIN_LORA_DIO1, INPUT);

        Serial.printf("  [INFO] DIO1 with pullup=%d  pulldown=%d\n",
                      dio1Up, dio1Dn);
        if (dio1Up == HIGH && dio1Dn == LOW) {
            Serial.println("  [INFO] DIO1 floats -> GPIO 11 is NOT connected "
                           "to the SX1280 DIO1 pad. PCB issue.");
        } else if (dio1Up == LOW && dio1Dn == LOW) {
            Serial.println("  [INFO] DIO1 forced LOW by SX1280 -> trace is "
                           "connected. The IRQ-to-DIO mask is wrong.");
        } else {
            Serial.println("  [INFO] DIO1 reads inconsistent - check probe.");
        }
        // A "good" outcome here is the SX1280 holding the line LOW.
        result("[R] DIO1 line is driven by SX1280 (not floating)",
               dio1Up == LOW && dio1Dn == LOW);
    } else {
        result("[R] DIO1 connectivity (skipped - radio failed)", false);
    }

    // -------------------------------------------------------------------
    // [S] Polling-based TX workaround.
    // Bypass DIO1 entirely: call startTransmit(), poll the SX1280's
    // IRQ status register for the TxDone bit, then finishTransmit().
    // If this returns OK 5 times in a row, we have a firmware
    // workaround that doesn't depend on DIO1 being wired/configured.
    // -------------------------------------------------------------------
    Serial.println("\n--- [S] Polling-based TX (no DIO1) x5 ---");
    if (radioOk) {
        int okCount = 0;
        for (int i = 0; i < 5; ++i) {
            radio.standby();
            radio.clearIrqFlags(0xFFFFFFFFu);
            String msg = "POLL_" + String(i);
            int s = radio.startTransmit(msg);
            if (s != RADIOLIB_ERR_NONE) {
                Serial.printf("  [INFO] iter %d: startTransmit=%d\n", i, s);
                continue;
            }
            uint32_t t0 = millis();
            bool done = false;
            uint32_t tDone = 0;
            while (millis() - t0 < 200) {
                uint16_t irq = radio.getIrqStatus();
                if (irq & 0x0001) {
                    done = true;
                    tDone = millis() - t0;
                    break;
                }
            }
            radio.finishTransmit();
            Serial.printf("  [INFO] iter %d: done=%d  t=%lu ms\n",
                          i, done ? 1 : 0, (unsigned long)tDone);
            if (done) okCount++;
        }
        Serial.printf("  [INFO] %d/5 polled transmits completed\n", okCount);
        result("[S] All 5 polled transmits complete", okCount == 5);
    } else {
        result("[S] polled TX (skipped - radio failed)", false);
    }

    // -------------------------------------------------------------------
    // [G] LittleFS sanity check (internal flash, independent of SPI).
    // -------------------------------------------------------------------
    Serial.println("\n--- [G] LittleFS flash sanity ---");
    {
        bool mounted = LittleFS.begin(true);
        result("[G] LittleFS mount", mounted);
        if (mounted) {
            LittleFS.remove(FLASH_FILE);
            File fw = LittleFS.open(FLASH_FILE, FILE_WRITE);
            bool writeOk = false;
            if (fw) {
                writeOk = (fw.print(TEST_PAYLOAD) > 0);
                fw.close();
            }
            File fr = LittleFS.open(FLASH_FILE, FILE_READ);
            bool readOk = false;
            String content;
            if (fr) {
                content = fr.readString();
                fr.close();
                readOk = (content == String(TEST_PAYLOAD));
            }
            LittleFS.remove(FLASH_FILE);
            Serial.printf("  [INFO] flash data: \"%s\"\n", content.c_str());
            result("[G] LittleFS write/read", writeOk && readOk);
        }
    }

    Serial.println("\n==================================================");
    Serial.printf("  TEST COMPLETE - %d pass, %d fail\n", passCount, failCount);
    Serial.println("  Restarting in 8 s...");
    Serial.println("==================================================");
    delay(8000);
    ESP.restart();
}

void loop() {}
