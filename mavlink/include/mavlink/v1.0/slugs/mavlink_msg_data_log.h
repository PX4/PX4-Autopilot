#pragma once
// MESSAGE DATA_LOG PACKING

#define MAVLINK_MSG_ID_DATA_LOG 177

MAVPACKED(
typedef struct __mavlink_data_log_t {
 float fl_1; /*< Log value 1 */
 float fl_2; /*< Log value 2 */
 float fl_3; /*< Log value 3 */
 float fl_4; /*< Log value 4 */
 float fl_5; /*< Log value 5 */
 float fl_6; /*< Log value 6 */
}) mavlink_data_log_t;

#define MAVLINK_MSG_ID_DATA_LOG_LEN 24
#define MAVLINK_MSG_ID_DATA_LOG_MIN_LEN 24
#define MAVLINK_MSG_ID_177_LEN 24
#define MAVLINK_MSG_ID_177_MIN_LEN 24

#define MAVLINK_MSG_ID_DATA_LOG_CRC 167
#define MAVLINK_MSG_ID_177_CRC 167



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DATA_LOG { \
    177, \
    "DATA_LOG", \
    6, \
    {  { "fl_1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_data_log_t, fl_1) }, \
         { "fl_2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_data_log_t, fl_2) }, \
         { "fl_3", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_data_log_t, fl_3) }, \
         { "fl_4", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_data_log_t, fl_4) }, \
         { "fl_5", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_data_log_t, fl_5) }, \
         { "fl_6", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_data_log_t, fl_6) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DATA_LOG { \
    "DATA_LOG", \
    6, \
    {  { "fl_1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_data_log_t, fl_1) }, \
         { "fl_2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_data_log_t, fl_2) }, \
         { "fl_3", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_data_log_t, fl_3) }, \
         { "fl_4", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_data_log_t, fl_4) }, \
         { "fl_5", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_data_log_t, fl_5) }, \
         { "fl_6", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_data_log_t, fl_6) }, \
         } \
}
#endif

/**
 * @brief Pack a data_log message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param fl_1 Log value 1 
 * @param fl_2 Log value 2 
 * @param fl_3 Log value 3 
 * @param fl_4 Log value 4 
 * @param fl_5 Log value 5 
 * @param fl_6 Log value 6 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_data_log_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float fl_1, float fl_2, float fl_3, float fl_4, float fl_5, float fl_6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DATA_LOG_LEN];
    _mav_put_float(buf, 0, fl_1);
    _mav_put_float(buf, 4, fl_2);
    _mav_put_float(buf, 8, fl_3);
    _mav_put_float(buf, 12, fl_4);
    _mav_put_float(buf, 16, fl_5);
    _mav_put_float(buf, 20, fl_6);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DATA_LOG_LEN);
#else
    mavlink_data_log_t packet;
    packet.fl_1 = fl_1;
    packet.fl_2 = fl_2;
    packet.fl_3 = fl_3;
    packet.fl_4 = fl_4;
    packet.fl_5 = fl_5;
    packet.fl_6 = fl_6;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DATA_LOG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DATA_LOG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DATA_LOG_MIN_LEN, MAVLINK_MSG_ID_DATA_LOG_LEN, MAVLINK_MSG_ID_DATA_LOG_CRC);
}

/**
 * @brief Pack a data_log message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param fl_1 Log value 1 
 * @param fl_2 Log value 2 
 * @param fl_3 Log value 3 
 * @param fl_4 Log value 4 
 * @param fl_5 Log value 5 
 * @param fl_6 Log value 6 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_data_log_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float fl_1,float fl_2,float fl_3,float fl_4,float fl_5,float fl_6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DATA_LOG_LEN];
    _mav_put_float(buf, 0, fl_1);
    _mav_put_float(buf, 4, fl_2);
    _mav_put_float(buf, 8, fl_3);
    _mav_put_float(buf, 12, fl_4);
    _mav_put_float(buf, 16, fl_5);
    _mav_put_float(buf, 20, fl_6);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DATA_LOG_LEN);
#else
    mavlink_data_log_t packet;
    packet.fl_1 = fl_1;
    packet.fl_2 = fl_2;
    packet.fl_3 = fl_3;
    packet.fl_4 = fl_4;
    packet.fl_5 = fl_5;
    packet.fl_6 = fl_6;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DATA_LOG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DATA_LOG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DATA_LOG_MIN_LEN, MAVLINK_MSG_ID_DATA_LOG_LEN, MAVLINK_MSG_ID_DATA_LOG_CRC);
}

/**
 * @brief Encode a data_log struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param data_log C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_data_log_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_data_log_t* data_log)
{
    return mavlink_msg_data_log_pack(system_id, component_id, msg, data_log->fl_1, data_log->fl_2, data_log->fl_3, data_log->fl_4, data_log->fl_5, data_log->fl_6);
}

/**
 * @brief Encode a data_log struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param data_log C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_data_log_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_data_log_t* data_log)
{
    return mavlink_msg_data_log_pack_chan(system_id, component_id, chan, msg, data_log->fl_1, data_log->fl_2, data_log->fl_3, data_log->fl_4, data_log->fl_5, data_log->fl_6);
}

/**
 * @brief Send a data_log message
 * @param chan MAVLink channel to send the message
 *
 * @param fl_1 Log value 1 
 * @param fl_2 Log value 2 
 * @param fl_3 Log value 3 
 * @param fl_4 Log value 4 
 * @param fl_5 Log value 5 
 * @param fl_6 Log value 6 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_data_log_send(mavlink_channel_t chan, float fl_1, float fl_2, float fl_3, float fl_4, float fl_5, float fl_6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DATA_LOG_LEN];
    _mav_put_float(buf, 0, fl_1);
    _mav_put_float(buf, 4, fl_2);
    _mav_put_float(buf, 8, fl_3);
    _mav_put_float(buf, 12, fl_4);
    _mav_put_float(buf, 16, fl_5);
    _mav_put_float(buf, 20, fl_6);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_LOG, buf, MAVLINK_MSG_ID_DATA_LOG_MIN_LEN, MAVLINK_MSG_ID_DATA_LOG_LEN, MAVLINK_MSG_ID_DATA_LOG_CRC);
#else
    mavlink_data_log_t packet;
    packet.fl_1 = fl_1;
    packet.fl_2 = fl_2;
    packet.fl_3 = fl_3;
    packet.fl_4 = fl_4;
    packet.fl_5 = fl_5;
    packet.fl_6 = fl_6;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_LOG, (const char *)&packet, MAVLINK_MSG_ID_DATA_LOG_MIN_LEN, MAVLINK_MSG_ID_DATA_LOG_LEN, MAVLINK_MSG_ID_DATA_LOG_CRC);
#endif
}

/**
 * @brief Send a data_log message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_data_log_send_struct(mavlink_channel_t chan, const mavlink_data_log_t* data_log)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_data_log_send(chan, data_log->fl_1, data_log->fl_2, data_log->fl_3, data_log->fl_4, data_log->fl_5, data_log->fl_6);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_LOG, (const char *)data_log, MAVLINK_MSG_ID_DATA_LOG_MIN_LEN, MAVLINK_MSG_ID_DATA_LOG_LEN, MAVLINK_MSG_ID_DATA_LOG_CRC);
#endif
}

#if MAVLINK_MSG_ID_DATA_LOG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_data_log_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float fl_1, float fl_2, float fl_3, float fl_4, float fl_5, float fl_6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, fl_1);
    _mav_put_float(buf, 4, fl_2);
    _mav_put_float(buf, 8, fl_3);
    _mav_put_float(buf, 12, fl_4);
    _mav_put_float(buf, 16, fl_5);
    _mav_put_float(buf, 20, fl_6);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_LOG, buf, MAVLINK_MSG_ID_DATA_LOG_MIN_LEN, MAVLINK_MSG_ID_DATA_LOG_LEN, MAVLINK_MSG_ID_DATA_LOG_CRC);
#else
    mavlink_data_log_t *packet = (mavlink_data_log_t *)msgbuf;
    packet->fl_1 = fl_1;
    packet->fl_2 = fl_2;
    packet->fl_3 = fl_3;
    packet->fl_4 = fl_4;
    packet->fl_5 = fl_5;
    packet->fl_6 = fl_6;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_LOG, (const char *)packet, MAVLINK_MSG_ID_DATA_LOG_MIN_LEN, MAVLINK_MSG_ID_DATA_LOG_LEN, MAVLINK_MSG_ID_DATA_LOG_CRC);
#endif
}
#endif

#endif

// MESSAGE DATA_LOG UNPACKING


/**
 * @brief Get field fl_1 from data_log message
 *
 * @return Log value 1 
 */
static inline float mavlink_msg_data_log_get_fl_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field fl_2 from data_log message
 *
 * @return Log value 2 
 */
static inline float mavlink_msg_data_log_get_fl_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field fl_3 from data_log message
 *
 * @return Log value 3 
 */
static inline float mavlink_msg_data_log_get_fl_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field fl_4 from data_log message
 *
 * @return Log value 4 
 */
static inline float mavlink_msg_data_log_get_fl_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field fl_5 from data_log message
 *
 * @return Log value 5 
 */
static inline float mavlink_msg_data_log_get_fl_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field fl_6 from data_log message
 *
 * @return Log value 6 
 */
static inline float mavlink_msg_data_log_get_fl_6(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a data_log message into a struct
 *
 * @param msg The message to decode
 * @param data_log C-struct to decode the message contents into
 */
static inline void mavlink_msg_data_log_decode(const mavlink_message_t* msg, mavlink_data_log_t* data_log)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    data_log->fl_1 = mavlink_msg_data_log_get_fl_1(msg);
    data_log->fl_2 = mavlink_msg_data_log_get_fl_2(msg);
    data_log->fl_3 = mavlink_msg_data_log_get_fl_3(msg);
    data_log->fl_4 = mavlink_msg_data_log_get_fl_4(msg);
    data_log->fl_5 = mavlink_msg_data_log_get_fl_5(msg);
    data_log->fl_6 = mavlink_msg_data_log_get_fl_6(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DATA_LOG_LEN? msg->len : MAVLINK_MSG_ID_DATA_LOG_LEN;
        memset(data_log, 0, MAVLINK_MSG_ID_DATA_LOG_LEN);
    memcpy(data_log, _MAV_PAYLOAD(msg), len);
#endif
}
