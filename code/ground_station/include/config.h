//
// Created by syvers on 27.07.25.
//

#ifndef CONFIG_H
#define CONFIG_H

#include <cstdint>

// Global system configuration

namespace config {

    // !!!!!!!!!!!!!!!!!!!! DO NOT GO HIGHER THAN 465
    // measuring 2.3v buffered by an op amp (as in schematic) without any gain
    // This value was experimentally validated in previous firmware.
    constexpr int DAC_POWER_MAX = 465;

    // VELDIG VIKTIG! SKAL IKKE HØYERE ENN 6 dBm
    // Maximum TX power validated for this hardware revision.
    constexpr int8_t TX_POWER_DBM_MAX = 6;
    constexpr int8_t TX_POWER_DBM_MIN = 0;

    // MUST match the payload exactly (PAYLOAD/include/New/Config.hpp).
    // Payload reverted from the 2450/SF12 trial back to this profile.
    constexpr float RADIO_FREQUENCY_MHZ = 2403.0f;
    constexpr float RADIO_BANDWIDTH_KHZ = 812.5f;
    constexpr uint8_t RADIO_SPREADING_FACTOR = 11;
    constexpr uint8_t RADIO_CODING_RATE = 8;
    constexpr uint8_t RADIO_SYNC_WORD = 18;   // 0x12

    constexpr float THERMAL_THROTTLE_THRESHOLD = 80.0f;

    constexpr float ADC_REFERENCE_VOLTAGE = 3.3f;
    constexpr float ADC_RESOLUTION_COUNTS = 4096.0f;

    // Original scaling factor used in previous firmware
    constexpr float CURRENT_SCALE_A_PER_COUNT = 0.00040283203f;
}

#endif