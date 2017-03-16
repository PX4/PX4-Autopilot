#pragma once
// MESSAGE MID_LVL_CMDS PACKING

#define MAVLINK_MSG_ID_MID_LVL_CMDS 180

MAVPACKED(
typedef struct __mavlink_mid_lvl_cmds_t {
 float hCommand; /*< Commanded Altitude in meters*/
 float uCommand; /*< Commanded Airspeed in m/s*/
 float rCommand; /*< Commanded Turnrate in rad/s*/
 uint8_t target; /*< The system setting the commands*/
}) mavlink_mid_lvl_cmds_t;

#define MAVLINK_MSG_ID_MID_LVL_CMDS_LEN 13
#define MAVLINK_MSG_ID_MID_LVL_CMDS_MIN_LEN 13
#define MAVLINK_MSG_ID_180_LEN 13
#define MAVLINK_MSG_ID_180_MIN_LEN 13

#define MAVLINK_MSG_ID_MID_LVL_CMDS_CRC 146
#define MAVLINK_MSG_ID_180_CRC 146



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MID_LVL_CMDS { \
    180, \
    "MID_LVL_CMDS", \
    4, \
    {  { "hCommand", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_mid_lvl_cmds_t, hCommand) }, \
         { "uCommand", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_mid_lvl_cmds_t, uCommand) }, \
         { "rCommand", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mid_lvl_cmds_t, rCommand) }, \
         { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_mid_lvl_cmds_t, target) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MID_LVL_CMDS { \
    "MID_LVL_CMDS", \
    4, \
    {  { "hCommand", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_mid_lvl_cmds_t, hCommand) }, \
         { "uCommand", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_mid_lvl_cmds_t, uCommand) }, \
         { "rCommand", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mid_lvl_cmds_t, rCommand) }, \
         { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_mid_lvl_cmds_t, target) }, \
         } \
}
#endif

/**
 * @brief Pack a mid_lvl_cmds message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The system setting the commands
 * @param hCommand Commanded Altitude in meters
 * @param uCommand Commanded Airspeed in m/s
 * @param rCommand Commanded Turnrate in rad/s
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mid_lvl_cmds_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target, float hCommand, float uCommand, float rCommand)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MID_LVL_CMDS_LEN];
    _mav_put_float(buf, 0, hCommand);
    _mav_put_float(buf, 4, uCommand);
    _mav_put_float(buf, 8, rCommand);
    _mav_put_uint8_t(buf, 12, target);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MID_LVL_CMDS_LEN);
#else
    mavlink_mid_lvl_cmds_t packet;
    packet.hCommand = hCommand;
    packet.uCommand = uCommand;
    packet.rCommand = rCommand;
    packet.target = target;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MID_LVL_CMDS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MID_LVL_CMDS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MID_LVL_CMDS_MIN_LEN, MAVLINK_MSG_ID_MID_LVL_CMDS_LEN, MAVLINK_MSG_ID_MID_LVL_CMDS_CRC);
}

/**
 * @brief Pack a mid_lvl_cmds message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system setting the commands
 * @param hCommand Commanded Altitude in meters
 * @param uCommand Commanded Airspeed in m/s
 * @param rCommand Commanded Turnrate in rad/s
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mid_lvl_cmds_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target,float hCommand,float uCommand,float rCommand)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MID_LVL_CMDS_LEN];
    _mav_put_float(buf, 0, hCommand);
    _mav_put_float(buf, 4, uCommand);
    _mav_put_float(buf, 8, rCommand);
    _mav_put_uint8_t(buf, 12, target);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MID_LVL_CMDS_LEN);
#else
    mavlink_mid_lvl_cmds_t packet;
    packet.hCommand = hCommand;
    packet.uCommand = uCommand;
    packet.rCommand = rCommand;
    packet.target = target;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MID_LVL_CMDS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MID_LVL_CMDS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MID_LVL_CMDS_MIN_LEN, MAVLINK_MSG_ID_MID_LVL_CMDS_LEN, MAVLINK_MSG_ID_MID_LVL_CMDS_CRC);
}

/**
 * @brief Encode a mid_lvl_cmds struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mid_lvl_cmds C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mid_lvl_cmds_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mid_lvl_cmds_t* mid_lvl_cmds)
{
    return mavlink_msg_mid_lvl_cmds_pack(system_id, component_id, msg, mid_lvl_cmds->target, mid_lvl_cmds->hCommand, mid_lvl_cmds->uCommand, mid_lvl_cmds->rCommand);
}

/**
 * @brief Encode a mid_lvl_cmds struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mid_lvl_cmds C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mid_lvl_cmds_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mid_lvl_cmds_t* mid_lvl_cmds)
{
    return mavlink_msg_mid_lvl_cmds_pack_chan(system_id, component_id, chan, msg, mid_lvl_cmds->target, mid_lvl_cmds->hCommand, mid_lvl_cmds->uCommand, mid_lvl_cmds->rCommand);
}

/**
 * @brief Send a mid_lvl_cmds message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system setting the commands
 * @param hCommand Commanded Altitude in meters
 * @param uCommand Commanded Airspeed in m/s
 * @param rCommand Commanded Turnrate in rad/s
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mid_lvl_cmds_send(mavlink_channel_t chan, uint8_t target, float hCommand, float uCommand, float rCommand)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MID_LVL_CMDS_LEN];
    _mav_put_float(buf, 0, hCommand);
    _mav_put_float(buf, 4, uCommand);
    _mav_put_float(buf, 8, rCommand);
    _mav_put_uint8_t(buf, 12, target);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MID_LVL_CMDS, buf, MAVLINK_MSG_ID_MID_LVL_CMDS_MIN_LEN, MAVLINK_MSG_ID_MID_LVL_CMDS_LEN, MAVLINK_MSG_ID_MID_LVL_CMDS_CRC);
#else
    mavlink_mid_lvl_cmds_t packet;
    packet.hCommand = hCommand;
    packet.uCommand = uCommand;
    packet.rCommand = rCommand;
    packet.target = target;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MID_LVL_CMDS, (const char *)&packet, MAVLINK_MSG_ID_MID_LVL_CMDS_MIN_LEN, MAVLINK_MSG_ID_MID_LVL_CMDS_LEN, MAVLINK_MSG_ID_MID_LVL_CMDS_CRC);
#endif
}

/**
 * @brief Send a mid_lvl_cmds message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mid_lvl_cmds_send_struct(mavlink_channel_t chan, const mavlink_mid_lvl_cmds_t* mid_lvl_cmds)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mid_lvl_cmds_send(chan, mid_lvl_cmds->target, mid_lvl_cmds->hCommand, mid_lvl_cmds->uCommand, mid_lvl_cmds->rCommand);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MID_LVL_CMDS, (const char *)mid_lvl_cmds, MAVLINK_MSG_ID_MID_LVL_CMDS_MIN_LEN, MAVLINK_MSG_ID_MID_LVL_CMDS_LEN, MAVLINK_MSG_ID_MID_LVL_CMDS_CRC);
#endif
}

#if MAVLINK_MSG_ID_MID_LVL_CMDS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mid_lvl_cmds_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target, float hCommand, float uCommand, float rCommand)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, hCommand);
    _mav_put_float(buf, 4, uCommand);
    _mav_put_float(buf, 8, rCommand);
    _mav_put_uint8_t(buf, 12, target);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MID_LVL_CMDS, buf, MAVLINK_MSG_ID_MID_LVL_CMDS_MIN_LEN, MAVLINK_MSG_ID_MID_LVL_CMDS_LEN, MAVLINK_MSG_ID_MID_LVL_CMDS_CRC);
#else
    mavlink_mid_lvl_cmds_t *packet = (mavlink_mid_lvl_cmds_t *)msgbuf;
    packet->hCommand = hCommand;
    packet->uCommand = uCommand;
    packet->rCommand = rCommand;
    packet->target = target;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MID_LVL_CMDS, (const char *)packet, MAVLINK_MSG_ID_MID_LVL_CMDS_MIN_LEN, MAVLINK_MSG_ID_MID_LVL_CMDS_LEN, MAVLINK_MSG_ID_MID_LVL_CMDS_CRC);
#endif
}
#endif

#endif

// MESSAGE MID_LVL_CMDS UNPACKING


/**
 * @brief Get field target from mid_lvl_cmds message
 *
 * @return The system setting the commands
 */
static inline uint8_t mavlink_msg_mid_lvl_cmds_get_target(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field hCommand from mid_lvl_cmds message
 *
 * @return Commanded Altitude in meters
 */
static inline float mavlink_msg_mid_lvl_cmds_get_hCommand(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field uCommand from mid_lvl_cmds message
 *
 * @return Commanded Airspeed in m/s
 */
static inline float mavlink_msg_mid_lvl_cmds_get_uCommand(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field rCommand from mid_lvl_cmds message
 *
 * @return Commanded Turnrate in rad/s
 */
static inline float mavlink_msg_mid_lvl_cmds_get_rCommand(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a mid_lvl_cmds message into a struct
 *
 * @param msg The message to decode
 * @param mid_lvl_cmds C-struct to decode the message contents into
 */
static inline void mavlink_msg_mid_lvl_cmds_decode(const mavlink_message_t* msg, mavlink_mid_lvl_cmds_t* mid_lvl_cmds)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mid_lvl_cmds->hCommand = mavlink_msg_mid_lvl_cmds_get_hCommand(msg);
    mid_lvl_cmds->uCommand = mavlink_msg_mid_lvl_cmds_get_uCommand(msg);
    mid_lvl_cmds->rCommand = mavlink_msg_mid_lvl_cmds_get_rCommand(msg);
    mid_lvl_cmds->target = mavlink_msg_mid_lvl_cmds_get_target(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MID_LVL_CMDS_LEN? msg->len : MAVLINK_MSG_ID_MID_LVL_CMDS_LEN;
        memset(mid_lvl_cmds, 0, MAVLINK_MSG_ID_MID_LVL_CMDS_LEN);
    memcpy(mid_lvl_cmds, _MAV_PAYLOAD(msg), len);
#endif
}
