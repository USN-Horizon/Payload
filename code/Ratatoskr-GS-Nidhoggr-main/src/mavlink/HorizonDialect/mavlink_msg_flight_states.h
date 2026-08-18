#pragma once
// MESSAGE FLIGHT_STATES PACKING

#include <stdint.h>

#define MAVLINK_MSG_ID_FLIGHT_STATES 16000


typedef struct __mavlink_flight_states_t {
 uint64_t time_usec; /*< [us]  Timestamp(micro seconds since boot) */
 uint8_t phase; /*<  Current Flight State: */
} mavlink_flight_states_t;

#define MAVLINK_MSG_ID_FLIGHT_STATES_LEN 9
#define MAVLINK_MSG_ID_FLIGHT_STATES_MIN_LEN 9
#define MAVLINK_MSG_ID_16000_LEN 9
#define MAVLINK_MSG_ID_16000_MIN_LEN 9

#define MAVLINK_MSG_ID_FLIGHT_STATES_CRC 171
#define MAVLINK_MSG_ID_16000_CRC 171



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FLIGHT_STATES { \
    16000, \
    "FLIGHT_STATES", \
    2, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_flight_states_t, time_usec) }, \
         { "phase", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_flight_states_t, phase) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FLIGHT_STATES { \
    "FLIGHT_STATES", \
    2, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_flight_states_t, time_usec) }, \
         { "phase", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_flight_states_t, phase) }, \
         } \
}
#endif

/**
 * @brief Pack a flight_states message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us]  Timestamp(micro seconds since boot) 
 * @param phase  Current Flight State: 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flight_states_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t phase)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FLIGHT_STATES_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 8, phase);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLIGHT_STATES_LEN);
#else
    mavlink_flight_states_t packet;
    packet.time_usec = time_usec;
    packet.phase = phase;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLIGHT_STATES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FLIGHT_STATES;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FLIGHT_STATES_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_STATES_LEN, MAVLINK_MSG_ID_FLIGHT_STATES_CRC);
}

/**
 * @brief Pack a flight_states message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us]  Timestamp(micro seconds since boot) 
 * @param phase  Current Flight State: 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flight_states_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t phase)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FLIGHT_STATES_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 8, phase);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLIGHT_STATES_LEN);
#else
    mavlink_flight_states_t packet;
    packet.time_usec = time_usec;
    packet.phase = phase;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLIGHT_STATES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FLIGHT_STATES;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_FLIGHT_STATES_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_STATES_LEN, MAVLINK_MSG_ID_FLIGHT_STATES_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_FLIGHT_STATES_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_STATES_LEN);
#endif
}

/**
 * @brief Pack a flight_states message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us]  Timestamp(micro seconds since boot) 
 * @param phase  Current Flight State: 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flight_states_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint8_t phase)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FLIGHT_STATES_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 8, phase);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLIGHT_STATES_LEN);
#else
    mavlink_flight_states_t packet;
    packet.time_usec = time_usec;
    packet.phase = phase;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLIGHT_STATES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FLIGHT_STATES;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FLIGHT_STATES_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_STATES_LEN, MAVLINK_MSG_ID_FLIGHT_STATES_CRC);
}

/**
 * @brief Encode a flight_states struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param flight_states C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flight_states_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_flight_states_t* flight_states)
{
    return mavlink_msg_flight_states_pack(system_id, component_id, msg, flight_states->time_usec, flight_states->phase);
}

/**
 * @brief Encode a flight_states struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param flight_states C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flight_states_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_flight_states_t* flight_states)
{
    return mavlink_msg_flight_states_pack_chan(system_id, component_id, chan, msg, flight_states->time_usec, flight_states->phase);
}

/**
 * @brief Encode a flight_states struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param flight_states C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flight_states_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_flight_states_t* flight_states)
{
    return mavlink_msg_flight_states_pack_status(system_id, component_id, _status, msg,  flight_states->time_usec, flight_states->phase);
}

/**
 * @brief Send a flight_states message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us]  Timestamp(micro seconds since boot) 
 * @param phase  Current Flight State: 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_flight_states_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t phase)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FLIGHT_STATES_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 8, phase);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLIGHT_STATES, buf, MAVLINK_MSG_ID_FLIGHT_STATES_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_STATES_LEN, MAVLINK_MSG_ID_FLIGHT_STATES_CRC);
#else
    mavlink_flight_states_t packet;
    packet.time_usec = time_usec;
    packet.phase = phase;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLIGHT_STATES, (const char *)&packet, MAVLINK_MSG_ID_FLIGHT_STATES_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_STATES_LEN, MAVLINK_MSG_ID_FLIGHT_STATES_CRC);
#endif
}

/**
 * @brief Send a flight_states message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_flight_states_send_struct(mavlink_channel_t chan, const mavlink_flight_states_t* flight_states)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_flight_states_send(chan, flight_states->time_usec, flight_states->phase);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLIGHT_STATES, (const char *)flight_states, MAVLINK_MSG_ID_FLIGHT_STATES_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_STATES_LEN, MAVLINK_MSG_ID_FLIGHT_STATES_CRC);
#endif
}

#if MAVLINK_MSG_ID_FLIGHT_STATES_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_flight_states_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t phase)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 8, phase);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLIGHT_STATES, buf, MAVLINK_MSG_ID_FLIGHT_STATES_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_STATES_LEN, MAVLINK_MSG_ID_FLIGHT_STATES_CRC);
#else
    mavlink_flight_states_t *packet = (mavlink_flight_states_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->phase = phase;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLIGHT_STATES, (const char *)packet, MAVLINK_MSG_ID_FLIGHT_STATES_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_STATES_LEN, MAVLINK_MSG_ID_FLIGHT_STATES_CRC);
#endif
}
#endif

#endif

// MESSAGE FLIGHT_STATES UNPACKING


/**
 * @brief Get field time_usec from flight_states message
 *
 * @return [us]  Timestamp(micro seconds since boot) 
 */
static inline uint64_t mavlink_msg_flight_states_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field phase from flight_states message
 *
 * @return  Current Flight State: 
 */
static inline uint8_t mavlink_msg_flight_states_get_phase(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Decode a flight_states message into a struct
 *
 * @param msg The message to decode
 * @param flight_states C-struct to decode the message contents into
 */
static inline void mavlink_msg_flight_states_decode(const mavlink_message_t* msg, mavlink_flight_states_t* flight_states)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    flight_states->time_usec = mavlink_msg_flight_states_get_time_usec(msg);
    flight_states->phase = mavlink_msg_flight_states_get_phase(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FLIGHT_STATES_LEN? msg->len : MAVLINK_MSG_ID_FLIGHT_STATES_LEN;
        memset(flight_states, 0, MAVLINK_MSG_ID_FLIGHT_STATES_LEN);
    memcpy(flight_states, _MAV_PAYLOAD(msg), len);
#endif
}
