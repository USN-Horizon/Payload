//
// Created by syvers on 30.07.25.
//

#include <unity.h>
#include "../../lib/tested.cpp"

void test_isEven() {
   const int TEST_1 = 10;

   TEST_ASSERT_TRUE(isEven(TEST_1));

   const int TEST_2 = 11;

   TEST_ASSERT_FALSE(isEven(TEST_2));

}