#ifndef FEM_CONTROL_H
#define FEM_CONTROL_H

#include <Arduino.h>

// Front End Module
// @param tx_invRx front end module tx_invRx = 1 for sending, 0 for receiving, 2 for sleep.
// @param ref ref = voltage*1e3

enum TX_RX_MODE {
    RX,
    TX,
    SLEEP
};

int FEM(const TX_RX_MODE tx_invRx, int ref);

#endif