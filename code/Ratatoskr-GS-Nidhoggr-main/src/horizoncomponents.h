#ifndef HORIZONCOMPONENTS_H
#define HORIZONCOMPONENTS_H

#include <cstdint>

// MAVLink component IDs for the two boards on the rocket. Both share the
// vehicle's system ID (42, see Payload firmware's include/New/Config.hpp);
// component ID is what tells the ground station which board a message came
// from.
namespace HorizonComponent {
constexpr uint8_t Avionics = 1;    // MAV_COMP_ID_AUTOPILOT1
constexpr uint8_t Payload  = 191;  // MAV_COMP_ID_ONBOARD_COMPUTER, repurposed for the scientific payload
}

#endif // HORIZONCOMPONENTS_H
