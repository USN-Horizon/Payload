//
// Created by syvers on 29.07.25.
//
#include <unity.h>
#include "onboard/test_onboard_serial_connection.cpp"

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_serial_with_python_script);
    UNITY_END();
}

void loop() {}
