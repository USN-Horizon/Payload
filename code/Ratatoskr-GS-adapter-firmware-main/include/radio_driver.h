#ifndef RADIO_DRIVER_H
#define RADIO_DRIVER_H

#pragma once

#include <Arduino.h>
#include <RadioLib.h>
#include <cstdint>

// Wrapper around SX1280 (RadioLib) to isolate radio logic
class RadioDriver {
public:
    RadioDriver();

    // Initialize radio with predefined configuration
    int begin();

    // Transmit null-terminated data
    int transmit(const char* data);

    // Put radio in receive mode
    int startReceive();

    // Read received packet into String
    int readReceived(String& out);

    // Get last measured RSSI
    float getRSSI() const;

    // ISR handler for DIO1 rising edge
    void onDio1Rise();

    // Fetch and clear packet received flag
    bool fetchPacketReceivedFlag();

private:
    SX1280 _radio;

    // Set from ISR when packet is received
    volatile bool _packetReceived = false;
};

// Global accessor for singleton-style usage
RadioDriver& getRadioDriver();

#endif