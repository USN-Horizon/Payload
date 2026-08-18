#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

void sensorsInit();
void sensorsUpdateTemperature(float &currentTemp);
void sensorsPrintMetrics(float currentTemp, bool thermalThrottling);

#endif