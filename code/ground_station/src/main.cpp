#include <Arduino.h>
#include <pins.h>
#include <config.h>
#include <RadioLib.h>
#include <fem_control.h>
#include <sensors.h>
#include <thermal_control.h>

using namespace pins;

// circuit mode status
uint8_t circ_mode = 0;

// command mode status
uint8_t cmd_mode = 0;

//#define SENDER
#define SIGNAL_STRENGTH_TEST

static SX1280* radio = nullptr;

static float currentTemp = 0.0f;
static bool thermalThrottling = false;

void setup() {

    Serial.begin(115200);
    delay(200);

    analogWriteResolution(10); // => 0-1023
    analogReadResolution(12);

    pinMode(LED, OUTPUT);
    pinMode(DAC0, OUTPUT);
    pinMode(CSD, OUTPUT);
    pinMode(CTX, OUTPUT);
    pinMode(CRX, OUTPUT);
    pinMode(A3, INPUT);

    FEM(SLEEP, 0);

    radio = new SX1280(new Module(CS, DIO1, RST, BUSY));

    int state = radio->begin(
        config::RADIO_FREQUENCY_MHZ,
        config::RADIO_BANDWIDTH_KHZ,
        config::RADIO_SPREADING_FACTOR,
        config::RADIO_CODING_RATE,
        config::RADIO_SYNC_WORD,
        config::TX_POWER_DBM_MAX
    );

    if (state != RADIOLIB_ERR_NONE) {
        while (true) {
            digitalWrite(LED, 1);
            delay(100);
            digitalWrite(LED, 0);
            delay(100);
        }
    }

    // --- link-budget fixes, applied after begin() -----------------------------

    // +2 dB of receiver sensitivity for 700 uA. DS §4.2.1: high sensitivity mode
    // "improves the noise figure of the receiver by up to 3dB"; Table 3-5 shows a
    // consistent 2 dB (e.g. SF12/BW203: -130 low power vs -132 high sensitivity).
    // Note that DS Table 6-1 is the LOW POWER table, so this stacks on top of it.
    // Sets bits 6:7 of RxGain register 0x891 (DS Table 13-1). Cheapest dB in the
    // whole link — no time-on-air cost, no frequency-tolerance cost.
    radio->setHighSensitivityMode(true);

    // begin() applies bandwidth -> SF -> coding rate, and each of those issues its
    // own SetModulationParams. DS §14.4.1 requires the SF-dependent write to
    // register 0x925 (and bit 0 of 0x93C) AFTER SetModulationParams, and RadioLib
    // only performs those writes inside setSpreadingFactor(). Re-asserting SF here
    // makes it the last modulation call so those registers match the final params.
    radio->setSpreadingFactor(config::RADIO_SPREADING_FACTOR);

    // setCRC() takes a LENGTH IN BYTES, not a bool: LoRa accepts only 0 (off) or
    // 2 (on). setCRC(true) == setCRC(1) silently returns
    // RADIOLIB_ERR_INVALID_CRC_CONFIGURATION and changes nothing.
    radio->setCRC(2);

    // Whitening is deliberately NOT set: it is not a LoRa packet parameter
    // (DS Table 11-60 lists exactly five, none of them whitening).

#ifdef SENDER
    FEM(TX, 500);
#else
    FEM(RX, 500);
#endif

    sensorsInit();
}

void loop() {

    sensorsUpdateTemperature(currentTemp);

    checkThermalStatus(currentTemp, thermalThrottling);

#ifdef SENDER

    static unsigned long count = 0;

    if (!thermalThrottling) {

        String message =
            String("HRZN range test: ")
            + String(count)
            + " | "
            + String(config::DAC_POWER_MAX % 466);

        radio->transmit(message.c_str());

        Serial.println("Sendte: " + String(count));

        count++;
    }

#else

    uint8_t received[128];

    if (radio->receive(received, 128) == RADIOLIB_ERR_NONE) {


#ifdef SIGNAL_STRENGTH_TEST
        float rssi = radio->getRSSI();
        Serial.print("Packet RSSI: ");
        Serial.print(rssi);
        Serial.println(" dBm");
#else
        Serial.write(received, 128);
#endif

    }

#endif

    //sensorsPrintMetrics(currentTemp, thermalThrottling); not accepted by nidhoggr for now,
    // maybe integrate it as a mavlink device in the future

    digitalWrite(LED, 1);
    digitalWrite(LED, 0);
}