#define UNITY_INCLUDE_CONFIG_H
#include <unity.h>

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    // RUN_TEST(...)
    return UNITY_END();
}