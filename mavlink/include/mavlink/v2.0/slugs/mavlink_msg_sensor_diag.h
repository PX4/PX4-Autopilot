#pragma once
// MESSAGE SENSOR_DIAG PACKING

#define MAVLINK_MSG_ID_SENSOR_DIAG 196

MAVPACKED(
typedef struct __mavlink_sensor_diag_t {
 float float1; /*< Float field 1*/
 float float2; /*< Float field 2*/
 int16_t int1; /*< Int 16 field 1*/
 int8_t char1; /*< Int 8 field 1*/
}) mavlink_sensor_diag_t;

#define MAVLINK_MSG_ID_SENSOR_DIAG_LEN 11
#define MAVLINK_MSG_ID_SENSOR_DIAG_MIN_LEN 11
#define MAVLINK_MSG_ID_196_LEN 11
#define MAVLINK_MSG_ID_196_MIN_LEN 11

#define MAVLINK_MSG_ID_SENSOR_DIAG_CRC 129
#define MAVLINK_MSG_ID_196_CRC 129



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SENSOR_DIAG { \
    196, \
    "SENSOR_DIAG", \
    4, \
    {  { "float1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sensor_diag_t, float1) }, \
         { "float2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_sensor_diag_t, float2) }, \
         { "int1", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_sensor_diag_t, int1) }, \
         { "char1", NULL, MAVLINK_TYPE_INT8_T, 0, 10, offsetof(mavlink_sensor_diag_t, char1) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SENSOR_DIAG { \
    "SENSOR_DIAG", \
    4, \
    {  { "float1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sensor_diag_t, float1) }, \
         { "float2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_sensor_diag_t, float2) }, \
         { "int1", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_sensor_diag_t, int1) }, \
         { "char1", NULL, MAVLINK_TYPE_INT8_T, 0, 10, offsetof(mavlink_sensor_diag_t, char1) }, \
         } \
}
#endif

/**
 * @brief Pack a sensor_diag message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param float1 Float field 1
 * @param float2 Float field 2
 * @param int1 Int 16 field 1
 * @param char1 Int 8 field 1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_diag_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float float1, float float2, int16_t int1, int8_t char1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_DIAG_LEN];
    _mav_put_float(buf, 0, float1);
    _mav_put_float(buf, 4, float2);
    _mav_put_int16_t(buf, 8, int1);
    _mav_put_int8_t(buf, 10, char1);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSOR_DIAG_LEN);
#else
    mavlink_sensor_diag_t packet;
    packet.float1 = float1;
    packet.float2 = float2;
    packet.int1 = int1;
    packet.char1 = char1;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSOR_DIAG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSOR_DIAG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENSOR_DIAG_MIN_LEN, MAVLINK_MSG_ID_SENSOR_DIAG_LEN, MAVLINK_MSG_ID_SENSOR_DIAG_CRC);
}

/**
 * @brief Pack a sensor_diag message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param float1 Float field 1
 * @param float2 Float field 2
 * @param int1 Int 16 field 1
 * @param char1 Int 8 field 1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_diag_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float float1,float float2,int16_t int1,int8_t char1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_DIAG_LEN];
    _mav_put_float(buf, 0, float1);
    _mav_put_float(buf, 4, float2);
    _mav_put_int16_t(buf, 8, int1);
    _mav_put_int8_t(buf, 10, char1);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSOR_DIAG_LEN);
#else
    mavlink_sensor_diag_t packet;
    packet.float1 = float1;
    packet.float2 = float2;
    packet.int1 = int1;
    packet.char1 = char1;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSOR_DIAG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSOR_DIAG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENSOR_DIAG_MIN_LEN, MAVLINK_MSG_ID_SENSOR_DIAG_LEN, MAVLINK_MSG_ID_SENSOR_DIAG_CRC);
}

/**
 * @brief Encode a sensor_diag struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sensor_diag C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_diag_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sensor_diag_t* sensor_diag)
{
    return mavlink_msg_sensor_diag_pack(system_id, component_id, msg, sensor_diag->float1, sensor_diag->float2, sensor_diag->int1, sensor_diag->char1);
}

/**
 * @brief Encode a sensor_diag struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sensor_diag C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_diag_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sensor_diag_t* sensor_diag)
{
    return mavlink_msg_sensor_diag_pack_chan(system_id, component_id, chan, msg, sensor_diag->float1, sensor_diag->float2, sensor_diag->int1, sensor_diag->char1);
}

/**
 * @brief Send a sensor_diag message
 * @param chan MAVLink channel to send the message
 *
 * @param float1 Float field 1
 * @param float2 Float field 2
 * @param int1 Int 16 field 1
 * @param char1 Int 8 field 1
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sensor_diag_send(mavlink_channel_t chan, float float1, float float2, int16_t int1, int8_t char1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_DIAG_LEN];
    _mav_put_float(buf, 0, float1);
    _mav_put_float(buf, 4, float2);
    _mav_put_int16_t(buf, 8, int1);
    _mav_put_int8_t(buf, 10, char1);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_DIAG, buf, MAVLINK_MSG_ID_SENSOR_DIAG_MIN_LEN, MAVLINK_MSG_ID_SENSOR_DIAG_LEN, MAVLINK_MSG_ID_SENSOR_DIAG_CRC);
#else
    mavlink_sensor_diag_t packet;
    packet.float1 = float1;
    packet.float2 = float2;
    packet.int1 = int1;
    packet.char1 = char1;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_DIAG, (const char *)&packet, MAVLINK_MSG_ID_SENSOR_DIAG_MIN_LEN, MAVLINK_MSG_ID_SENSOR_DIAG_LEN, MAVLINK_MSG_ID_SENSOR_DIAG_CRC);
#endif
}

/**
 * @brief Send a sensor_diag message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sensor_diag_send_struct(mavlink_channel_t chan, const mavlink_sensor_diag_t* sensor_diag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sensor_diag_send(chan, sensor_diag->float1, sensor_diag->float2, sensor_diag->int1, sensor_diag->char1);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_DIAG, (const char *)sensor_diag, MAVLINK_MSG_ID_SENSOR_DIAG_MIN_LEN, MAVLINK_MSG_ID_SENSOR_DIAG_LEN, MAVLINK_MSG_ID_SENSOR_DIAG_CRC);
#endif
}

#if MAVLINK_MSG_ID_SENSOR_DIAG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sensor_diag_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float float1, float float2, int16_t int1, int8_t char1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, float1);
    _mav_put_float(buf, 4, float2);
    _mav_put_int16_t(buf, 8, int1);
    _mav_put_int8_t(buf, 10, char1);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_DIAG, buf, MAVLINK_MSG_ID_SENSOR_DIAG_MIN_LEN, MAVLINK_MSG_ID_SENSOR_DIAG_LEN, MAVLINK_MSG_ID_SENSOR_DIAG_CRC);
#else
    mavlink_sensor_diag_t *packet = (mavlink_sensor_diag_t *)msgbuf;
    packet->float1 = float1;
    packet->float2 = float2;
    packet->int1 = int1;
    packet->char1 = char1;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_DIAG, (const char *)packet, MAVLINK_MSG_ID_SENSOR_DIAG_MIN_LEN, MAVLINK_MSG_ID_SENSOR_DIAG_LEN, MAVLINK_MSG_ID_SENSOR_DIAG_CRC);
#endif
}
#endif

#endif

// MESSAGE SENSOR_DIAG UNPACKING


/**
 * @brief Get field float1 from sensor_diag message
 *
 * @return Float field 1
 */
static inline float mavlink_msg_sensor_diag_get_float1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field float2 from sensor_diag message
 *
 * @return Float field 2
 */
static inline float mavlink_msg_sensor_diag_get_float2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field int1 from sensor_diag message
 *
 * @return Int 16 field 1
 */
static inline int16_t mavlink_msg_sensor_diag_get_int1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field char1 from sensor_diag message
 *
 * @return Int 8 field 1
 */
static inline int8_t mavlink_msg_sensor_diag_get_char1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  10);
}

/**
 * @brief Decode a sensor_diag message into a struct
 *
 * @param msg The message to decode
 * @param sensor_diag C-struct to decode the message contents into
 */
static inline void mavlink_msg_sensor_diag_decode(const mavlink_message_t* msg, mavlink_sensor_diag_t* sensor_diag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sensor_diag->float1 = mavlink_msg_sensor_diag_get_float1(msg);
    sensor_diag->float2 = mavlink_msg_sensor_diag_get_float2(msg);
    sensor_diag->int1 = mavlink_msg_sensor_diag_get_int1(msg);
    sensor_diag->char1 = mavlink_msg_sensor_diag_get_char1(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SENSOR_DIAG_LEN? msg->len : MAVLINK_MSG_ID_SENSOR_DIAG_LEN;
        memset(sensor_diag, 0, MAVLINK_MSG_ID_SENSOR_DIAG_LEN);
    memcpy(sensor_diag, _MAV_PAYLOAD(msg), len);
#endif
}
