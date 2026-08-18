#include <Arduino.h>
#include "fem_control.h"
#include "pins.h"
#include "config.h"

using namespace pins;

/**
 * Front End Module
 * @param tx_invRx front end module tx_invRx = 1 for sending, 0 for receiving, 2 for sleep.
 * @param ref ref = voltage*1e3
 * @return
 */
int FEM(const TX_RX_MODE tx_invRx, int ref) {

#ifdef ARDUINO

    switch (tx_invRx) {

        case RX:

            analogWrite(DAC0, 0);
            delayMicroseconds(500);

            digitalWrite(CSD, 1);
            digitalWrite(CTX, 0);
            digitalWrite(CRX, 1);

            break;

        case TX:

            if (ref > 2850) {
                ref = 2850;
            }

            // !!!!!!!!!!!!!!!!!!!! DO NOT GO HIGHER THAN 465
            analogWrite(DAC0, config::DAC_POWER_MAX % 466);

            digitalWrite(CSD, 1);
            digitalWrite(CTX, 1);
            digitalWrite(CRX, 0);

            break;

        default:

            analogWrite(DAC0, 0);
            delayMicroseconds(500);

            digitalWrite(CSD, 0);
            digitalWrite(CTX, 0);
            digitalWrite(CRX, 0);

            break;
    }

#endif

    return 0;
}