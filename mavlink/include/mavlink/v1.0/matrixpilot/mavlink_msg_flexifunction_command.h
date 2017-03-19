#pragma once
// MESSAGE FLEXIFUNCTION_COMMAND PACKING

#define MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND 157

MAVPACKED(
typedef struct __mavlink_flexifunction_command_t {
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
 uint8_t command_type; /*< Flexifunction command type*/
}) mavlink_flexifunction_command_t;

#define MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN 3
#define MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_MIN_LEN 3
#define MAVLINK_MSG_ID_157_LEN 3
#define MAVLINK_MSG_ID_157_MIN_LEN 3

#define MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_CRC 133
#define MAVLINK_MSG_ID_157_CRC 133



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FLEXIFUNCTION_COMMAND { \
    157, \
    "FLEXIFUNCTION_COMMAND", \
    3, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_flexifunction_command_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_flexifunction_command_t, target_component) }, \
         { "command_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_flexifunction_command_t, command_type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FLEXIFUNCTION_COMMAND { \
    "FLEXIFUNCTION_COMMAND", \
    3, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_flexifunction_command_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_flexifunction_command_t, target_component) }, \
         { "command_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_flexifunction_command_t, command_type) }, \
         } \
}
#endif

/**
 * @brief Pack a flexifunction_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param command_type Flexifunction command type
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flexifunction_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t command_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, command_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN);
#else
    mavlink_flexifunction_command_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.command_type = command_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_MIN_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_CRC);
}

/**
 * @brief Pack a flexifunction_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param command_type Flexifunction command type
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flexifunction_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t command_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, command_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN);
#else
    mavlink_flexifunction_command_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.command_type = command_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_MIN_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_CRC);
}

/**
 * @brief Encode a flexifunction_command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param flexifunction_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flexifunction_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_flexifunction_command_t* flexifunction_command)
{
    return mavlink_msg_flexifunction_command_pack(system_id, component_id, msg, flexifunction_command->target_system, flexifunction_command->target_component, flexifunction_command->command_type);
}

/**
 * @brief Encode a flexifunction_command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param flexifunction_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flexifunction_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_flexifunction_command_t* flexifunction_command)
{
    return mavlink_msg_flexifunction_command_pack_chan(system_id, component_id, chan, msg, flexifunction_command->target_system, flexifunction_command->target_component, flexifunction_command->command_type);
}

/**
 * @brief Send a flexifunction_command message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param command_type Flexifunction command type
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_flexifunction_command_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t command_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, command_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND, buf, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_MIN_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_CRC);
#else
    mavlink_flexifunction_command_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.command_type = command_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_MIN_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_CRC);
#endif
}

/**
 * @brief Send a flexifunction_command message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_flexifunction_command_send_struct(mavlink_channel_t chan, const mavlink_flexifunction_command_t* flexifunction_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_flexifunction_command_send(chan, flexifunction_command->target_system, flexifunction_command->target_component, flexifunction_command->command_type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND, (const char *)flexifunction_command, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_MIN_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_CRC);
#endif
}

#if MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_flexifunction_command_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t command_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, target_component);
    _mav_put_uint8_t(buf, 2, command_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND, buf, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_MIN_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_CRC);
#else
    mavlink_flexifunction_command_t *packet = (mavlink_flexifunction_command_t *)msgbuf;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->command_type = command_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND, (const char *)packet, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_MIN_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_CRC);
#endif
}
#endif

#endif

// MESSAGE FLEXIFUNCTION_COMMAND UNPACKING


/**
 * @brief Get field target_system from flexifunction_command message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_flexifunction_command_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from flexifunction_command message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_flexifunction_command_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field command_type from flexifunction_command message
 *
 * @return Flexifunction command type
 */
static inline uint8_t mavlink_msg_flexifunction_command_get_command_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a flexifunction_command message into a struct
 *
 * @param msg The message to decode
 * @param flexifunction_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_flexifunction_command_decode(const mavlink_message_t* msg, mavlink_flexifunction_command_t* flexifunction_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    flexifunction_command->target_system = mavlink_msg_flexifunction_command_get_target_system(msg);
    flexifunction_command->target_component = mavlink_msg_flexifunction_command_get_target_component(msg);
    flexifunction_command->command_type = mavlink_msg_flexifunction_command_get_command_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN? msg->len : MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN;
        memset(flexifunction_command, 0, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN);
    memcpy(flexifunction_command, _MAV_PAYLOAD(msg), len);
#endif
}
