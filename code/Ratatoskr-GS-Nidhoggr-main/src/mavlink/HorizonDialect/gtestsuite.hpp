/** @file
 *  @brief MAVLink comm testsuite protocol generated from HorizonDialect.xml
 *  @see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "HorizonDialect.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(HorizonDialect, FLIGHT_STATES)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::HorizonDialect::msg::FLIGHT_STATES packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.phase = 29;

    mavlink::HorizonDialect::msg::FLIGHT_STATES packet1{};
    mavlink::HorizonDialect::msg::FLIGHT_STATES packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.phase, packet2.phase);
}

#ifdef TEST_INTEROP
TEST(HorizonDialect_interop, FLIGHT_STATES)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_flight_states_t packet_c {
         93372036854775807ULL, 29
    };

    mavlink::HorizonDialect::msg::FLIGHT_STATES packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.phase = 29;

    mavlink::HorizonDialect::msg::FLIGHT_STATES packet2{};

    mavlink_msg_flight_states_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.phase, packet2.phase);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(HorizonDialect, PAYLOAD_TEMPERATURE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::HorizonDialect::msg::PAYLOAD_TEMPERATURE packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.top_temp = 73.0;
    packet_in.mid_temp = 101.0;
    packet_in.bottom_temp = 129.0;

    mavlink::HorizonDialect::msg::PAYLOAD_TEMPERATURE packet1{};
    mavlink::HorizonDialect::msg::PAYLOAD_TEMPERATURE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.top_temp, packet2.top_temp);
    EXPECT_EQ(packet1.mid_temp, packet2.mid_temp);
    EXPECT_EQ(packet1.bottom_temp, packet2.bottom_temp);
}

#ifdef TEST_INTEROP
TEST(HorizonDialect_interop, PAYLOAD_TEMPERATURE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_payload_temperature_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0
    };

    mavlink::HorizonDialect::msg::PAYLOAD_TEMPERATURE packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.top_temp = 73.0;
    packet_in.mid_temp = 101.0;
    packet_in.bottom_temp = 129.0;

    mavlink::HorizonDialect::msg::PAYLOAD_TEMPERATURE packet2{};

    mavlink_msg_payload_temperature_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.top_temp, packet2.top_temp);
    EXPECT_EQ(packet_in.mid_temp, packet2.mid_temp);
    EXPECT_EQ(packet_in.bottom_temp, packet2.bottom_temp);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(HorizonDialect, COSMIC_RADIATION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::HorizonDialect::msg::COSMIC_RADIATION packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.radiation = 17651;

    mavlink::HorizonDialect::msg::COSMIC_RADIATION packet1{};
    mavlink::HorizonDialect::msg::COSMIC_RADIATION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.radiation, packet2.radiation);
}

#ifdef TEST_INTEROP
TEST(HorizonDialect_interop, COSMIC_RADIATION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_cosmic_radiation_t packet_c {
         93372036854775807ULL, 17651
    };

    mavlink::HorizonDialect::msg::COSMIC_RADIATION packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.radiation = 17651;

    mavlink::HorizonDialect::msg::COSMIC_RADIATION packet2{};

    mavlink_msg_cosmic_radiation_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.radiation, packet2.radiation);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
