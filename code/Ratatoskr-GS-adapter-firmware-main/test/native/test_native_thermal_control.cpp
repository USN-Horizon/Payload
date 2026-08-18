//
// Created by syvers on 27.07.25.
//

#include <unity.h>
#include "thermal_control.h"

void test_thermal_throttle_condition() {
    bool isThermallyThrottling = false;

    checkThermalStatus(100, isThermallyThrottling);
    TEST_ASSERT_TRUE(isThermallyThrottling);

    checkThermalStatus(50, isThermallyThrottling);
    TEST_ASSERT_FALSE(isThermallyThrottling);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_thermal_throttle_condition);
    return UNITY_END();
}