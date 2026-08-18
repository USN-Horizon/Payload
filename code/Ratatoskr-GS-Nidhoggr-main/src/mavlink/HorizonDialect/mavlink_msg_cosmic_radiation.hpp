// MESSAGE COSMIC_RADIATION support class

#pragma once

namespace mavlink {
namespace HorizonDialect {
namespace msg {

/**
 * @brief COSMIC_RADIATION message
 *
 * Cosmic radiation data
 */
struct COSMIC_RADIATION : mavlink::Message {
    static constexpr msgid_t MSG_ID = 16002;
    static constexpr size_t LENGTH = 10;
    static constexpr size_t MIN_LENGTH = 10;
    static constexpr uint8_t CRC_EXTRA = 134;
    static constexpr auto NAME = "COSMIC_RADIATION";


    uint64_t time_usec; /*< [us]  Timestamp(micro seconds since boot)  */
    uint16_t radiation; /*<   */


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
        ss << "  radiation: " << radiation << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_usec;                     // offset: 0
        map << radiation;                     // offset: 8
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_usec;                     // offset: 0
        map >> radiation;                     // offset: 8
    }
};

} // namespace msg
} // namespace HorizonDialect
} // namespace mavlink
