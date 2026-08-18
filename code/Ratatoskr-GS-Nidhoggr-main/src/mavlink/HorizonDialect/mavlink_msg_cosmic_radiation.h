#pragma once
// MESSAGE COSMIC_RADIATION PACKING

#include <stdint.h>

#define MAVLINK_MSG_ID_COSMIC_RADIATION 16002


typedef struct __mavlink_cosmic_radiation_t {
 uint64_t time_usec; /*< [us]  Timestamp(micro seconds since boot) */
 uint16_t radiation; /*<  */
} mavlink_cosmic_radiation_t;

#define MAVLINK_MSG_ID_COSMIC_RADIATION_LEN 10
#define MAVLINK_MSG_ID_COSMIC_RADIATION_MIN_LEN 10
#define MAVLINK_MSG_ID_16002_LEN 10
#define MAVLINK_MSG_ID_16002_MIN_LEN 10

#define MAVLINK_MSG_ID_COSMIC_RADIATION_CRC 134
#define MAVLINK_MSG_ID_16002_CRC 134



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_COSMIC_RADIATION { \
    16002, \
    "COSMIC_RADIATION", \
    2, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_cosmic_radiation_t, time_usec) }, \
         { "radiation", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_cosmic_radiation_t, radiation) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_COSMIC_RADIATION { \
    "COSMIC_RADIATION", \
    2, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_cosmic_radiation_t, time_usec) }, \
         { "radiation", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_cosmic_radiation_t, radiation) }, \
         } \
}
#endif

/**
 * @brief Pack a cosmic_radiation message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us]  Timestamp(micro seconds since boot) 
 * @param radiation  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cosmic_radiation_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint16_t radiation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COSMIC_RADIATION_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint16_t(buf, 8, radiation);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COSMIC_RADIATION_LEN);
#else
    mavlink_cosmic_radiation_t packet;
    packet.time_usec = time_usec;
    packet.radiation = radiation;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COSMIC_RADIATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COSMIC_RADIATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_COSMIC_RADIATION_MIN_LEN, MAVLINK_MSG_ID_COSMIC_RADIATION_LEN, MAVLINK_MSG_ID_COSMIC_RADIATION_CRC);
}

/**
 * @brief Pack a cosmic_radiation message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us]  Timestamp(micro seconds since boot) 
 * @param radiation  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cosmic_radiation_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, uint16_t radiation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COSMIC_RADIATION_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint16_t(buf, 8, radiation);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COSMIC_RADIATION_LEN);
#else
    mavlink_cosmic_radiation_t packet;
    packet.time_usec = time_usec;
    packet.radiation = radiation;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COSMIC_RADIATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COSMIC_RADIATION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_COSMIC_RADIATION_MIN_LEN, MAVLINK_MSG_ID_COSMIC_RADIATION_LEN, MAVLINK_MSG_ID_COSMIC_RADIATION_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_COSMIC_RADIATION_MIN_LEN, MAVLINK_MSG_ID_COSMIC_RADIATION_LEN);
#endif
}

/**
 * @brief Pack a cosmic_radiation message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us]  Timestamp(micro seconds since boot) 
 * @param radiation  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cosmic_radiation_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint16_t radiation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COSMIC_RADIATION_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint16_t(buf, 8, radiation);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COSMIC_RADIATION_LEN);
#else
    mavlink_cosmic_radiation_t packet;
    packet.time_usec = time_usec;
    packet.radiation = radiation;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COSMIC_RADIATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COSMIC_RADIATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_COSMIC_RADIATION_MIN_LEN, MAVLINK_MSG_ID_COSMIC_RADIATION_LEN, MAVLINK_MSG_ID_COSMIC_RADIATION_CRC);
}

/**
 * @brief Encode a cosmic_radiation struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param cosmic_radiation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cosmic_radiation_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_cosmic_radiation_t* cosmic_radiation)
{
    return mavlink_msg_cosmic_radiation_pack(system_id, component_id, msg, cosmic_radiation->time_usec, cosmic_radiation->radiation);
}

/**
 * @brief Encode a cosmic_radiation struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cosmic_radiation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cosmic_radiation_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_cosmic_radiation_t* cosmic_radiation)
{
    return mavlink_msg_cosmic_radiation_pack_chan(system_id, component_id, chan, msg, cosmic_radiation->time_usec, cosmic_radiation->radiation);
}

/**
 * @brief Encode a cosmic_radiation struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param cosmic_radiation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cosmic_radiation_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_cosmic_radiation_t* cosmic_radiation)
{
    return mavlink_msg_cosmic_radiation_pack_status(system_id, component_id, _status, msg,  cosmic_radiation->time_usec, cosmic_radiation->radiation);
}

/**
 * @brief Send a cosmic_radiation message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us]  Timestamp(micro seconds since boot) 
 * @param radiation  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_cosmic_radiation_send(mavlink_channel_t chan, uint64_t time_usec, uint16_t radiation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COSMIC_RADIATION_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint16_t(buf, 8, radiation);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COSMIC_RADIATION, buf, MAVLINK_MSG_ID_COSMIC_RADIATION_MIN_LEN, MAVLINK_MSG_ID_COSMIC_RADIATION_LEN, MAVLINK_MSG_ID_COSMIC_RADIATION_CRC);
#else
    mavlink_cosmic_radiation_t packet;
    packet.time_usec = time_usec;
    packet.radiation = radiation;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COSMIC_RADIATION, (const char *)&packet, MAVLINK_MSG_ID_COSMIC_RADIATION_MIN_LEN, MAVLINK_MSG_ID_COSMIC_RADIATION_LEN, MAVLINK_MSG_ID_COSMIC_RADIATION_CRC);
#endif
}

/**
 * @brief Send a cosmic_radiation message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_cosmic_radiation_send_struct(mavlink_channel_t chan, const mavlink_cosmic_radiation_t* cosmic_radiation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_cosmic_radiation_send(chan, cosmic_radiation->time_usec, cosmic_radiation->radiation);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COSMIC_RADIATION, (const char *)cosmic_radiation, MAVLINK_MSG_ID_COSMIC_RADIATION_MIN_LEN, MAVLINK_MSG_ID_COSMIC_RADIATION_LEN, MAVLINK_MSG_ID_COSMIC_RADIATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_COSMIC_RADIATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_cosmic_radiation_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint16_t radiation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint16_t(buf, 8, radiation);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COSMIC_RADIATION, buf, MAVLINK_MSG_ID_COSMIC_RADIATION_MIN_LEN, MAVLINK_MSG_ID_COSMIC_RADIATION_LEN, MAVLINK_MSG_ID_COSMIC_RADIATION_CRC);
#else
    mavlink_cosmic_radiation_t *packet = (mavlink_cosmic_radiation_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->radiation = radiation;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COSMIC_RADIATION, (const char *)packet, MAVLINK_MSG_ID_COSMIC_RADIATION_MIN_LEN, MAVLINK_MSG_ID_COSMIC_RADIATION_LEN, MAVLINK_MSG_ID_COSMIC_RADIATION_CRC);
#endif
}
#endif

#endif

// MESSAGE COSMIC_RADIATION UNPACKING


/**
 * @brief Get field time_usec from cosmic_radiation message
 *
 * @return [us]  Timestamp(micro seconds since boot) 
 */
static inline uint64_t mavlink_msg_cosmic_radiation_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field radiation from cosmic_radiation message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_cosmic_radiation_get_radiation(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Decode a cosmic_radiation message into a struct
 *
 * @param msg The message to decode
 * @param cosmic_radiation C-struct to decode the message contents into
 */
static inline void mavlink_msg_cosmic_radiation_decode(const mavlink_message_t* msg, mavlink_cosmic_radiation_t* cosmic_radiation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    cosmic_radiation->time_usec = mavlink_msg_cosmic_radiation_get_time_usec(msg);
    cosmic_radiation->radiation = mavlink_msg_cosmic_radiation_get_radiation(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_COSMIC_RADIATION_LEN? msg->len : MAVLINK_MSG_ID_COSMIC_RADIATION_LEN;
        memset(cosmic_radiation, 0, MAVLINK_MSG_ID_COSMIC_RADIATION_LEN);
    memcpy(cosmic_radiation, _MAV_PAYLOAD(msg), len);
#endif
}
