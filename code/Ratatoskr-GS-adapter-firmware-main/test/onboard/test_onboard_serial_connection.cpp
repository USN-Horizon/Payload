#include <Arduino.h>
#include <unity.h>

void test_serial_with_python_script() {
    Serial.begin(9600);
    delay(2000); // Wait for Python script to connect

    // Test ping-pong
    Serial.println("PING");
    Serial.flush();

    unsigned long timeout = millis() + 3000;
    String response = "";

    while(millis() < timeout) {
        if(Serial.available()) {
            response = Serial.readStringUntil('\n');
            break;
        }
    }

    TEST_ASSERT_EQUAL_STRING("PONG", response.c_str());
}


