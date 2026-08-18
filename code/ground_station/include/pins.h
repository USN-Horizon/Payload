//
// Created by syvers on 25.06.25.
//

#ifndef PINS_H
#define PINS_H

#include <Arduino.h>

// Definitons for GPIO-pins and other peripherals to be used in code

namespace pins {

    // !!! Only example values and pins !!!
    constexpr int LED = A1;
    constexpr int RST = 10;
    constexpr int BUSY = 9;
    constexpr int CSD = 8;
    constexpr int CTX = 7;
    constexpr int DIO2 = 6;
    constexpr int DIO1 = 5;
    constexpr int CRX = 4;
    constexpr int CS = 2;

    constexpr int miso = 12;
    constexpr int mosi = 11;
    constexpr int sck  = 13;
}

#endif