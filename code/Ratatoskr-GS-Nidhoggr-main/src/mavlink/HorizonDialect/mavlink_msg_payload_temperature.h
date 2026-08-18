#pragma once
// MESSAGE PAYLOAD_TEMPERATURE PACKING

#include <stdint.h>

#define MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE 16001


typedef struct __mavlink_payload_temperature_t {
 uint64_t time_usec; /*< [us]  Timestamp(micro seconds since boot) */
 float top_temp; /*< [degC] Top-Stack: */
 float mid_temp; /*< [degC] Mid-Stack:*/
 float bottom_temp; /*< [degC] Bottom-Stack*/
} mavlink_payload_temperature_t;

#define MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN 20
#define MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_MIN_LEN 20
#define MAVLINK_MSG_ID_16001_LEN 20
#define MAVLINK_MSG_ID_16001_MIN_LEN 20

#define MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_CRC 46
#define MAVLINK_MSG_ID_16001_CRC 46



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PAYLOAD_TEMPERATURE { \
    16001, \
    "PAYLOAD_TEMPERATURE", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_payload_temperature_t, time_usec) }, \
         { "top_temp", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_payload_temperature_t, top_temp) }, \
         { "mid_temp", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_payload_temperature_t, mid_temp) }, \
         { "bottom_temp", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_payload_temperature_t, bottom_temp) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PAYLOAD_TEMPERATURE { \
    "PAYLOAD_TEMPERATURE", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_payload_temperature_t, time_usec) }, \
         { "top_temp", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_payload_temperature_t, top_temp) }, \
         { "mid_temp", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_payload_temperature_t, mid_temp) }, \
         { "bottom_temp", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_payload_temperature_t, bottom_temp) }, \
         } \
}
#endif

/**
 * @brief Pack a payload_temperature message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us]  Timestamp(micro seconds since boot) 
 * @param top_temp [degC] Top-Stack: 
 * @param mid_temp [degC] Mid-Stack:
 * @param bottom_temp [degC] Bottom-Stack
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_payload_temperature_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float top_temp, float mid_temp, float bottom_temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, top_temp);
    _mav_put_float(buf, 12, mid_temp);
    _mav_put_float(buf, 16, bottom_temp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN);
#else
    mavlink_payload_temperature_t packet;
    packet.time_usec = time_usec;
    packet.top_temp = top_temp;
    packet.mid_temp = mid_temp;
    packet.bottom_temp = bottom_temp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_CRC);
}

/**
 * @brief Pack a payload_temperature message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us]  Timestamp(micro seconds since boot) 
 * @param top_temp [degC] Top-Stack: 
 * @param mid_temp [degC] Mid-Stack:
 * @param bottom_temp [degC] Bottom-Stack
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_payload_temperature_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, float top_temp, float mid_temp, float bottom_temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, top_temp);
    _mav_put_float(buf, 12, mid_temp);
    _mav_put_float(buf, 16, bottom_temp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN);
#else
    mavlink_payload_temperature_t packet;
    packet.time_usec = time_usec;
    packet.top_temp = top_temp;
    packet.mid_temp = mid_temp;
    packet.bottom_temp = bottom_temp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN);
#endif
}

/**
 * @brief Pack a payload_temperature message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us]  Timestamp(micro seconds since boot) 
 * @param top_temp [degC] Top-Stack: 
 * @param mid_temp [degC] Mid-Stack:
 * @param bottom_temp [degC] Bottom-Stack
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_payload_temperature_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float top_temp,float mid_temp,float bottom_temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, top_temp);
    _mav_put_float(buf, 12, mid_temp);
    _mav_put_float(buf, 16, bottom_temp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN);
#else
    mavlink_payload_temperature_t packet;
    packet.time_usec = time_usec;
    packet.top_temp = top_temp;
    packet.mid_temp = mid_temp;
    packet.bottom_temp = bottom_temp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_CRC);
}

/**
 * @brief Encode a payload_temperature struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param payload_temperature C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_payload_temperature_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_payload_temperature_t* payload_temperature)
{
    return mavlink_msg_payload_temperature_pack(system_id, component_id, msg, payload_temperature->time_usec, payload_temperature->top_temp, payload_temperature->mid_temp, payload_temperature->bottom_temp);
}

/**
 * @brief Encode a payload_temperature struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param payload_temperature C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_payload_temperature_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_payload_temperature_t* payload_temperature)
{
    return mavlink_msg_payload_temperature_pack_chan(system_id, component_id, chan, msg, payload_temperature->time_usec, payload_temperature->top_temp, payload_temperature->mid_temp, payload_temperature->bottom_temp);
}

/**
 * @brief Encode a payload_temperature struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param payload_temperature C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_payload_temperature_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_payload_temperature_t* payload_temperature)
{
    return mavlink_msg_payload_temperature_pack_status(system_id, component_id, _status, msg,  payload_temperature->time_usec, payload_temperature->top_temp, payload_temperature->mid_temp, payload_temperature->bottom_temp);
}

/**
 * @brief Send a payload_temperature message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us]  Timestamp(micro seconds since boot) 
 * @param top_temp [degC] Top-Stack: 
 * @param mid_temp [degC] Mid-Stack:
 * @param bottom_temp [degC] Bottom-Stack
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_payload_temperature_send(mavlink_channel_t chan, uint64_t time_usec, float top_temp, float mid_temp, float bottom_temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, top_temp);
    _mav_put_float(buf, 12, mid_temp);
    _mav_put_float(buf, 16, bottom_temp);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE, buf, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_CRC);
#else
    mavlink_payload_temperature_t packet;
    packet.time_usec = time_usec;
    packet.top_temp = top_temp;
    packet.mid_temp = mid_temp;
    packet.bottom_temp = bottom_temp;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE, (const char *)&packet, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_CRC);
#endif
}

/**
 * @brief Send a payload_temperature message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_payload_temperature_send_struct(mavlink_channel_t chan, const mavlink_payload_temperature_t* payload_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_payload_temperature_send(chan, payload_temperature->time_usec, payload_temperature->top_temp, payload_temperature->mid_temp, payload_temperature->bottom_temp);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE, (const char *)payload_temperature, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_CRC);
#endif
}

#if MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_payload_temperature_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float top_temp, float mid_temp, float bottom_temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, top_temp);
    _mav_put_float(buf, 12, mid_temp);
    _mav_put_float(buf, 16, bottom_temp);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE, buf, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_CRC);
#else
    mavlink_payload_temperature_t *packet = (mavlink_payload_temperature_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->top_temp = top_temp;
    packet->mid_temp = mid_temp;
    packet->bottom_temp = bottom_temp;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE, (const char *)packet, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_CRC);
#endif
}
#endif

#endif

// MESSAGE PAYLOAD_TEMPERATURE UNPACKING


/**
 * @brief Get field time_usec from payload_temperature message
 *
 * @return [us]  Timestamp(micro seconds since boot) 
 */
static inline uint64_t mavlink_msg_payload_temperature_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field top_temp from payload_temperature message
 *
 * @return [degC] Top-Stack: 
 */
static inline float mavlink_msg_payload_temperature_get_top_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field mid_temp from payload_temperature message
 *
 * @return [degC] Mid-Stack:
 */
static inline float mavlink_msg_payload_temperature_get_mid_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field bottom_temp from payload_temperature message
 *
 * @return [degC] Bottom-Stack
 */
static inline float mavlink_msg_payload_temperature_get_bottom_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a payload_temperature message into a struct
 *
 * @param msg The message to decode
 * @param payload_temperature C-struct to decode the message contents into
 */
static inline void mavlink_msg_payload_temperature_decode(const mavlink_message_t* msg, mavlink_payload_temperature_t* payload_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    payload_temperature->time_usec = mavlink_msg_payload_temperature_get_time_usec(msg);
    payload_temperature->top_temp = mavlink_msg_payload_temperature_get_top_temp(msg);
    payload_temperature->mid_temp = mavlink_msg_payload_temperature_get_mid_temp(msg);
    payload_temperature->bottom_temp = mavlink_msg_payload_temperature_get_bottom_temp(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN? msg->len : MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN;
        memset(payload_temperature, 0, MAVLINK_MSG_ID_PAYLOAD_TEMPERATURE_LEN);
    memcpy(payload_temperature, _MAV_PAYLOAD(msg), len);
#endif
}
