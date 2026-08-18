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