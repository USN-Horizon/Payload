#include "config.h"
#include "thermal_control.h"

void checkThermalStatus(float currentTemp, bool &thermallyThrottling) {

    if (currentTemp > config::THERMAL_THROTTLE_THRESHOLD) {
        thermallyThrottling = true;
    } 
    else if (currentTemp < config::THERMAL_THROTTLE_THRESHOLD - 1) {
        thermallyThrottling = false;
    }
}