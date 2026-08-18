//
// Created by syvers on 29.07.25.
//
#include <unity.h>
#include "native/test_native_thermal_control.cpp"
#include "native/testing_tested.cpp"

void setUp() {}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_thermal_throttle_condition);
    RUN_TEST(test_isEven);

    UNITY_END();
}

void tearDown() {}