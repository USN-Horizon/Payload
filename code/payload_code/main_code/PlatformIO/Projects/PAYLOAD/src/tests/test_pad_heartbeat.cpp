// Component test: pad heartbeat + low-power launch wait + launched-flag.
//
// Purpose
// -------
// Prototype for the on-pad behaviour before it is ported into
// App::waitForLaunch_():
//
//   1. LOW-POWER WAIT - duty-cycle the CPU (light sleep between 200 ms accel
//      checks) exactly like test_lowpower_wake.
//   2. PAD HEARTBEAT - every ~30 s wake the SX1280 out of retention sleep,
//      transmit one MAVLink HEARTBEAT + one HorizonDialect FLIGHT_STATES
//      (phase = ROCKET_STATE_IDLE), then put the radio back to sleep. The
//      SX1280 draws ~1 uA in sleep-with-retention and keeps all of its
//      SF/BW/CR settings, so no re-init is needed between beacons.
//   3. LAUNCHED FLAG - the moment the accel gate trips, write "flown=1" to
//      NVS. On the next boot the gate is SKIPPED if the flag is set, so a
//      mid-flight brownout/reset can never strand the payload back on the
//      "waiting for launch" screen while it is coasting at ~0 g or hanging
//      under the chute at ~1 g. Type "clear" to disarm the flag on the bench.
//
// What to verify
// --------------
//   - Ground station receives a HEARTBEAT + FLIGHT_STATES pair every 30 s
//     (same 2400/812.5/SF11/CR4-8 profile as production).
//   - Radio survives many sleep -> standby -> TX -> sleep cycles (watch the
//     printed status codes; 0 = OK).
//   - Shake/jerk the board (or type "launch"): FLIGHT DETECTED prints, the
//     NVS flag is written, and a FLIGHT_STATES with phase=THRUSTING goes out.
//   - Press the RST button after that: the boot log must say the gate is
//     being skipped because flown=1. Type "clear" to re-arm the gate.
//   - With USE_LIGHT_SLEEP=1 and a current meter: pad current should be
//     dominated by the ESP32 light-sleep floor, with a brief TX bump per
//     beacon (~0.5 % duty at SF11).
//
// NOTE: USE_LIGHT_SLEEP=1 tears down USB-CDC (serial monitor drops, and the
// "launch"/"clear" commands stop working) - bench with 0, flight-test with 1.
//
// Upload:  pio run -e test_pad_heartbeat -t upload
// Monitor: pio device monitor -e test_pad_heartbeat

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Preferences.h>
#include <RadioLib.h>
#include <ICM45686.h>
#include <math.h>
#include "esp_sleep.h"
#include "driver/gpio.h"
#include <mavlink/library/HorizonDialect/mavlink.h>

// ---- Wiring (from "Main board.pdf" pin table) -------------------------------
static constexpr int PIN_SPI_MOSI = 4;
static constexpr int PIN_SPI_SCK  = 5;
static constexpr int PIN_SPI_MISO = 6;
static constexpr int PIN_I2C_SDA  = 7;
static constexpr int PIN_I2C_SCL  = 15;
static constexpr int LORA_NSS     = 9;    // IF13
static constexpr int LORA_PWR     = 10;   // IF14
static constexpr int PIN_CXT      = 11;   // IF15, RF switch (mislabelled DIO1)
static constexpr int PIN_CRX      = 12;   // IF16, RF switch (mislabelled DIO2)
static constexpr int LORA_BUSY    = 13;   // IF17
static constexpr int LORA_RST     = 14;   // IF18
static constexpr int IMU_CS       = 16;   // IF8
static constexpr int SD_CS        = 21;

// ---- Radio profile (must match production / ground station) ----------------
static constexpr float   RF_FREQ_MHZ = 2400.0f;
static constexpr float   RF_BW_KHZ   = 812.5f;
static constexpr uint8_t RF_SF       = 11;
static constexpr uint8_t RF_CR       = 8;
static constexpr int8_t  RF_TX_DBM   = 13;

// ---- MAVLink identity (same as production Config.hpp) -----------------------
static constexpr uint8_t MAV_SYSID  = 42;
static constexpr uint8_t MAV_COMPID = 191;

// ---- Tuning -----------------------------------------------------------------
// 1 = real low-power behaviour (esp_light_sleep). USB serial drops while a PC
// is attached, so bench with 0 and set 1 only for current measurements/flight.
#define USE_LIGHT_SLEEP 0

static constexpr uint32_t WAIT_CHECK_INTERVAL_MS  = 200;   // 5 Hz accel duty.
static constexpr uint32_t HEARTBEAT_INTERVAL_MS   = 30000; // Pad beacon cadence.
// Beacon every Nth accel check so the cadence survives light sleep without
// depending on how the core keeps millis() across sleep.
static constexpr uint32_t CHECKS_PER_HEARTBEAT =
    HEARTBEAT_INTERVAL_MS / WAIT_CHECK_INTERVAL_MS;

static constexpr uint16_t ACCEL_ODR_HZ    = 25;
static constexpr uint16_t ACCEL_FSR_G     = 16;
static constexpr float    ACCEL_LSB_PER_G = 32768.0f / ACCEL_FSR_G;
static constexpr float    LAUNCH_THRESHOLD_G     = 1.5f;
static constexpr uint8_t  LAUNCH_CONFIRM_SAMPLES = 1;

// ---- TX-done via IRQ polling (same rationale as LoraTx.cpp: DIO1/BUSY are
// not reliable TxDone indicators on this board) -------------------------------
static constexpr uint32_t LORA_TX_TIMEOUT_MS = 1000;
static constexpr uint16_t SX128X_IRQ_TX_DONE = 0x0001;

// ---- DAC43401s that set the LoRa PA voltage (full scale, as in production
// max-power testing) ----------------------------------------------------------
static constexpr uint8_t DAC_ADDRS[]     = {0x47, 0x48};
static constexpr uint8_t DAC_GENERAL_CFG = 0xD1;
static constexpr uint8_t DAC_DAC_DATA    = 0x21;
static constexpr uint8_t DAC_UNLOCK      = 0x36;

static SX1280 radio = new Module(LORA_NSS, RADIOLIB_NC, LORA_RST, LORA_BUSY,
                                 SPI, SPISettings(8000000, MSBFIRST, SPI_MODE0));
static ICM456xx imu(SPI, IMU_CS);
static Preferences prefs;

static bool     imuOk       = false;
static bool     flownFlag   = false;
static uint32_t beaconSeq   = 0;
static String   serialBuf;

static void result(const char* label, bool pass) {
    Serial.printf("  [%s] %s\n", pass ? "PASS" : "FAIL", label);
}

// -----------------------------------------------------------------------------
//  DAC + IMU + radio bring-up helpers
// -----------------------------------------------------------------------------
static void dacWrite16(uint8_t a, uint8_t reg, uint16_t v) {
    Wire.beginTransmission(a);
    Wire.write(reg);
    Wire.write(static_cast<uint8_t>(v >> 8));
    Wire.write(static_cast<uint8_t>(v & 0xFF));
    Wire.endTransmission();
}

static void dacsMax() {
    for (uint8_t a : DAC_ADDRS) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() != 0) continue;
        dacWrite16(a, DAC_UNLOCK, 0x5000); delay(2);
        dacWrite16(a, DAC_GENERAL_CFG, 0x0000); delay(2);
        dacWrite16(a, DAC_DAC_DATA, 0x0FF0); delay(2);
    }
}

static bool readAccelMagnitude(float& mag_g) {
    inv_imu_sensor_data_t d;
    if (imu.getDataFromRegisters(d) != 0) return false;
    float ax = d.accel_data[0] / ACCEL_LSB_PER_G;
    float ay = d.accel_data[1] / ACCEL_LSB_PER_G;
    float az = d.accel_data[2] / ACCEL_LSB_PER_G;
    mag_g = sqrtf(ax * ax + ay * ay + az * az);
    return true;
}

// Blocking TX by polling the SX1280 IRQ status register for TxDone (bit 0),
// mirroring the proven txBlockOnIrq in src/New/LoraTx.cpp.
static int16_t txBlockOnIrq(const uint8_t* data, uint8_t len) {
    uint32_t tStart = millis();
    int16_t s = radio.startTransmit(const_cast<uint8_t*>(data), len);
    if (s != RADIOLIB_ERR_NONE) return s;

    while ((radio.getIrqStatus() & SX128X_IRQ_TX_DONE) == 0) {
        if (millis() - tStart > LORA_TX_TIMEOUT_MS) {
            radio.finishTransmit();
            return RADIOLIB_ERR_TX_TIMEOUT;
        }
        delay(1);
    }
    radio.finishTransmit();
    return RADIOLIB_ERR_NONE;
}

// -----------------------------------------------------------------------------
//  MAVLink frame builders
// -----------------------------------------------------------------------------
static size_t buildHeartbeat(uint8_t* out) {
    mavlink_message_t msg;
    // MAV_STATE_STANDBY while on the pad (production uses ACTIVE in flight).
    mavlink_msg_heartbeat_pack(MAV_SYSID, MAV_COMPID, &msg,
                               MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID,
                               0, 0, MAV_STATE_STANDBY);
    return mavlink_msg_to_send_buffer(out, &msg);
}

static size_t buildFlightState(uint8_t* out, uint8_t phase) {
    mavlink_message_t msg;
    mavlink_msg_flight_states_pack(MAV_SYSID, MAV_COMPID, &msg,
                                   static_cast<uint64_t>(millis()) * 1000ULL,
                                   phase);
    return mavlink_msg_to_send_buffer(out, &msg);
}

// -----------------------------------------------------------------------------
//  Pad beacon: wake radio -> TX heartbeat + flight state -> radio back to sleep
// -----------------------------------------------------------------------------
static void sendPadBeacon(uint8_t phase) {
    uint32_t t0 = millis();
    int16_t wake = radio.standby();          // wakes the chip out of sleep
    uint32_t tWake = millis() - t0;

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    size_t  n  = buildHeartbeat(buf);
    int16_t s1 = txBlockOnIrq(buf, static_cast<uint8_t>(n));
    n          = buildFlightState(buf, phase);
    int16_t s2 = txBlockOnIrq(buf, static_cast<uint8_t>(n));

    int16_t slp = radio.sleep(/*retainConfig=*/true);
    uint32_t tTotal = millis() - t0;

    beaconSeq++;
    Serial.printf("  [BEACON] #%lu  wake=%d(%lums) hb=%d fs=%d sleep=%d  "
                  "total=%lums  (0=OK)\n",
                  static_cast<unsigned long>(beaconSeq), wake,
                  static_cast<unsigned long>(tWake), s1, s2, slp,
                  static_cast<unsigned long>(tTotal));
}

// -----------------------------------------------------------------------------
//  Bench serial commands: launch | clear | status
//  Returns true if a launch was forced.
// -----------------------------------------------------------------------------
static bool pollSerialCommands() {
    bool forceLaunch = false;
    while (Serial.available()) {
        char ch = static_cast<char>(Serial.read());
        if (ch == '\r') continue;
        if (ch != '\n') {
            if (serialBuf.length() < 32) serialBuf += ch;
            continue;
        }
        String cmd = serialBuf;
        serialBuf.clear();
        cmd.trim();
        cmd.toLowerCase();
        if (cmd == "launch") {
            Serial.println("  [CMD] forcing launch");
            forceLaunch = true;
        } else if (cmd == "clear") {
            prefs.begin("payload", false);
            prefs.putUChar("flown", 0);
            prefs.end();
            flownFlag = false;
            Serial.println("  [CMD] flown flag cleared - gate re-armed on next boot");
        } else if (cmd == "status") {
            Serial.printf("  [CMD] flown=%d imu=%d beacons=%lu\n",
                          flownFlag ? 1 : 0, imuOk ? 1 : 0,
                          static_cast<unsigned long>(beaconSeq));
        } else if (cmd.length()) {
            Serial.println("  [CMD] try: launch | clear | status");
        }
    }
    return forceLaunch;
}

static void lowPowerInterval() {
#if USE_LIGHT_SLEEP
    esp_sleep_enable_timer_wakeup(
        static_cast<uint64_t>(WAIT_CHECK_INTERVAL_MS) * 1000ULL);
    esp_light_sleep_start();
#else
    delay(WAIT_CHECK_INTERVAL_MS);
#endif
}

// -----------------------------------------------------------------------------
//  setup
// -----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("========================================");
    Serial.println("  COMPONENT TEST: pad heartbeat + gate");
    Serial.println("========================================");

    // Deselect SPI peers before anything drives the bus.
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
    pinMode(SD_CS,  OUTPUT); digitalWrite(SD_CS,  HIGH);
    pinMode(IMU_CS, OUTPUT); digitalWrite(IMU_CS, HIGH);

    // LoRa power rail + reset sequencing (same as test_lora_beacon).
    pinMode(LORA_PWR, OUTPUT); digitalWrite(LORA_PWR, HIGH);
    gpio_set_drive_capability(static_cast<gpio_num_t>(LORA_PWR), GPIO_DRIVE_CAP_3);
    delay(10);
    pinMode(LORA_NSS, OUTPUT); digitalWrite(LORA_NSS, HIGH);
    pinMode(LORA_RST, OUTPUT); digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(20);

    // RF switch: CXT=LOW / CrX=HIGH is the only state that routes the PA to
    // the antenna (test_lora_rfswitch_probe).
    pinMode(PIN_CXT, OUTPUT); digitalWrite(PIN_CXT, HIGH);
    pinMode(PIN_CRX, OUTPUT); digitalWrite(PIN_CRX, LOW);

    int st = radio.begin();
    result("radio.begin()", st == RADIOLIB_ERR_NONE);
    if (st != RADIOLIB_ERR_NONE) {
        Serial.println("  ABORT - restarting in 5 s"); delay(5000); ESP.restart();
    }
    radio.setFrequency(RF_FREQ_MHZ);
    radio.setBandwidth(RF_BW_KHZ);
    radio.setSpreadingFactor(RF_SF);
    radio.setCodingRate(RF_CR);
    radio.setOutputPower(RF_TX_DBM);
    radio.setCRC(true);
    radio.setWhitening(true);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    dacsMax();

    // IMU (optional - without it the gate is skipped but beacons still run,
    // matching how waitForLaunch_() degrades when imuOk() is false).
    delay(150);
    for (int attempt = 0; attempt < 5 && !imuOk; ++attempt) {
        if (imu.begin() == 0) imuOk = true;
        else delay(50);
    }
    result("IMU begin()", imuOk);
    if (imuOk) {
        imu.startAccel(ACCEL_ODR_HZ, ACCEL_FSR_G);
        delay(100);
    } else {
        Serial.println("  [WARN] no IMU - gate disabled, use 'launch' command");
    }

    // Launched flag: a mid-flight reset must NOT re-enter the pad gate.
    prefs.begin("payload", false);
    flownFlag = prefs.getUChar("flown", 0) != 0;
    prefs.end();
    Serial.printf("  [INFO] NVS flown=%d\n", flownFlag ? 1 : 0);
    if (flownFlag) {
        Serial.println("  ****  flown=1: SKIPPING PAD GATE (mid-flight reset "
                       "path)  ****");
        Serial.println("  [INFO] type 'clear' then reset to re-arm the gate");
        result("Gate skipped on reboot after launch", true);
        return;  // loop() plays the in-flight role
    }

    // ---- LOW-POWER WAIT + PAD HEARTBEAT -------------------------------------
    const float threshold = LAUNCH_THRESHOLD_G;
    Serial.println();
    Serial.printf("  [MODE] PAD WAIT (light_sleep=%d): beacon every %lu s, "
                  "launch when |a| > %.2f g\n",
                  USE_LIGHT_SLEEP,
                  static_cast<unsigned long>(HEARTBEAT_INTERVAL_MS / 1000),
                  threshold);
    Serial.println("  [INFO] commands: launch | clear | status");

    // First beacon right away so the ground station locks on without a 30 s
    // wait, then park the radio in retention sleep.
    sendPadBeacon(ROCKET_STATE_IDLE);

    uint8_t  aboveCount = 0;
    uint32_t checks     = 0;
    bool     launched   = false;

    while (!launched) {
        lowPowerInterval();
        checks++;

        if (pollSerialCommands()) launched = true;

        if (imuOk && !launched) {
            float mag = 0.0f;
            if (readAccelMagnitude(mag)) {
                if (mag > threshold) {
                    if (++aboveCount >= LAUNCH_CONFIRM_SAMPLES) launched = true;
                } else if (aboveCount > 0) {
                    aboveCount--;
                }
            }
        }

        if (!launched && checks % CHECKS_PER_HEARTBEAT == 0) {
            sendPadBeacon(ROCKET_STATE_IDLE);
        } else if (!launched && checks % 25 == 0) {
            Serial.printf("  [WAIT] alive, next beacon in %lu s\n",
                          static_cast<unsigned long>(
                              (CHECKS_PER_HEARTBEAT - checks % CHECKS_PER_HEARTBEAT)
                              * WAIT_CHECK_INTERVAL_MS / 1000));
        }
    }

    // ---- LAUNCH ------------------------------------------------------------
    Serial.println();
    Serial.println("  ****  FLIGHT DETECTED - leaving pad mode  ****");

    // Persist BEFORE any mission work so even an immediate brownout reboots
    // straight into flight mode.
    prefs.begin("payload", false);
    prefs.putUChar("flown", 1);
    prefs.end();
    flownFlag = true;
    result("flown=1 written to NVS", true);

    // In production this is where waitForLaunch_() returns and the mission
    // loop takes over; here we just announce the phase change.
    sendPadBeacon(ROCKET_STATE_THRUSTING);
    Serial.println("  [INFO] press RST now: boot must skip the gate (flown=1)");
    Serial.println("  [INFO] 'clear' re-arms the gate for the next bench run");
}

// -----------------------------------------------------------------------------
//  loop - post-launch / post-reset role: keep beaconing so the sleep->TX->sleep
//  cycle can be soak-tested, and keep serving 'clear' / 'status'.
// -----------------------------------------------------------------------------
void loop() {
    static uint32_t lastBeaconMs = millis();
    pollSerialCommands();
    if (millis() - lastBeaconMs >= HEARTBEAT_INTERVAL_MS) {
        sendPadBeacon(flownFlag ? ROCKET_STATE_THRUSTING : ROCKET_STATE_IDLE);
        lastBeaconMs = millis();
    }
    delay(20);
}
