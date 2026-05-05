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
