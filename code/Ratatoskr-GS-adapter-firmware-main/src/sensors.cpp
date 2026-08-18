#include <Arduino.h>
#include <TMP1075.h>
#include <Wire.h>
#include "pins.h"
#include "config.h"
#include "sensors.h"

static TMP1075::TMP1075 temp(Wire);

void sensorsInit() {
    Wire.begin();
    temp.begin();
}

void sensorsUpdateTemperature(float &currentTemp) {

    static unsigned long lastTime = millis();

    if (millis() - lastTime > 1000) {
        lastTime = millis();
        temp.setConversionTime(TMP1075::ConversionTime220ms);
        currentTemp = temp.getTemperatureCelsius();
    }
}

void sensorsPrintMetrics(float currentTemp, bool thermalThrottling) {

    const int current_raw = analogRead(A3);

    const double current =
        current_raw * config::CURRENT_SCALE_A_PER_COUNT;

    Serial.print("Current:");
    Serial.print(current);
    Serial.print(" A | ");

    Serial.print("Temperature: ");
    Serial.print(currentTemp);
    Serial.println(" C");

    Serial.print("Is thermally throttling: ");
    Serial.println(thermalThrottling ? "YES" : "NO");
}