// MESSAGE PAYLOAD_TEMPERATURE support class

#pragma once

namespace mavlink {
namespace HorizonDialect {
namespace msg {

/**
 * @brief PAYLOAD_TEMPERATURE message
 *
 * Temperature readings from three points on the payload stack
 */
struct PAYLOAD_TEMPERATURE : mavlink::Message {
    static constexpr msgid_t MSG_ID = 16001;
    static constexpr size_t LENGTH = 20;
    static constexpr size_t MIN_LENGTH = 20;
    static constexpr uint8_t CRC_EXTRA = 46;
    static constexpr auto NAME = "PAYLOAD_TEMPERATURE";


    uint64_t time_usec; /*< [us]  Timestamp(micro seconds since boot)  */
    float top_temp; /*< [degC] Top-Stack:  */
    float mid_temp; /*< [degC] Mid-Stack: */
    float bottom_temp; /*< [degC] Bottom-Stack */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  time_usec: " << time_usec << std::endl;
        ss << "  top_temp: " << top_temp << std::endl;
        ss << "  mid_temp: " << mid_temp << std::endl;
        ss << "  bottom_temp: " << bottom_temp << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_usec;                     // offset: 0
        map << top_temp;                      // offset: 8
        map << mid_temp;                      // offset: 12
        map << bottom_temp;                   // offset: 16
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_usec;                     // offset: 0
        map >> top_temp;                      // offset: 8
        map >> mid_temp;                      // offset: 12
        map >> bottom_temp;                   // offset: 16
    }
};

} // namespace msg
} // namespace HorizonDialect
} // namespace mavlink
