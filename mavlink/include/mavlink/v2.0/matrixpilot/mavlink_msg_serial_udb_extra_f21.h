#pragma once
// MESSAGE SERIAL_UDB_EXTRA_F21 PACKING

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21 187

MAVPACKED(
typedef struct __mavlink_serial_udb_extra_f21_t {
 int16_t sue_accel_x_offset; /*< SUE X accelerometer offset*/
 int16_t sue_accel_y_offset; /*< SUE Y accelerometer offset*/
 int16_t sue_accel_z_offset; /*< SUE Z accelerometer offset*/
 int16_t sue_gyro_x_offset; /*< SUE X gyro offset*/
 int16_t sue_gyro_y_offset; /*< SUE Y gyro offset*/
 int16_t sue_gyro_z_offset; /*< SUE Z gyro offset*/
}) mavlink_serial_udb_extra_f21_t;

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_LEN 12
#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_MIN_LEN 12
#define MAVLINK_MSG_ID_187_LEN 12
#define MAVLINK_MSG_ID_187_MIN_LEN 12

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_CRC 134
#define MAVLINK_MSG_ID_187_CRC 134



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F21 { \
    187, \
    "SERIAL_UDB_EXTRA_F21", \
    6, \
    {  { "sue_accel_x_offset", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_serial_udb_extra_f21_t, sue_accel_x_offset) }, \
         { "sue_accel_y_offset", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_serial_udb_extra_f21_t, sue_accel_y_offset) }, \
         { "sue_accel_z_offset", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_serial_udb_extra_f21_t, sue_accel_z_offset) }, \
         { "sue_gyro_x_offset", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_serial_udb_extra_f21_t, sue_gyro_x_offset) }, \
         { "sue_gyro_y_offset", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_serial_udb_extra_f21_t, sue_gyro_y_offset) }, \
         { "sue_gyro_z_offset", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_serial_udb_extra_f21_t, sue_gyro_z_offset) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F21 { \
    "SERIAL_UDB_EXTRA_F21", \
    6, \
    {  { "sue_accel_x_offset", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_serial_udb_extra_f21_t, sue_accel_x_offset) }, \
         { "sue_accel_y_offset", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_serial_udb_extra_f21_t, sue_accel_y_offset) }, \
         { "sue_accel_z_offset", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_serial_udb_extra_f21_t, sue_accel_z_offset) }, \
         { "sue_gyro_x_offset", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_serial_udb_extra_f21_t, sue_gyro_x_offset) }, \
         { "sue_gyro_y_offset", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_serial_udb_extra_f21_t, sue_gyro_y_offset) }, \
         { "sue_gyro_z_offset", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_serial_udb_extra_f21_t, sue_gyro_z_offset) }, \
         } \
}
#endif

/**
 * @brief Pack a serial_udb_extra_f21 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sue_accel_x_offset SUE X accelerometer offset
 * @param sue_accel_y_offset SUE Y accelerometer offset
 * @param sue_accel_z_offset SUE Z accelerometer offset
 * @param sue_gyro_x_offset SUE X gyro offset
 * @param sue_gyro_y_offset SUE Y gyro offset
 * @param sue_gyro_z_offset SUE Z gyro offset
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f21_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int16_t sue_accel_x_offset, int16_t sue_accel_y_offset, int16_t sue_accel_z_offset, int16_t sue_gyro_x_offset, int16_t sue_gyro_y_offset, int16_t sue_gyro_z_offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_LEN];
    _mav_put_int16_t(buf, 0, sue_accel_x_offset);
    _mav_put_int16_t(buf, 2, sue_accel_y_offset);
    _mav_put_int16_t(buf, 4, sue_accel_z_offset);
    _mav_put_int16_t(buf, 6, sue_gyro_x_offset);
    _mav_put_int16_t(buf, 8, sue_gyro_y_offset);
    _mav_put_int16_t(buf, 10, sue_gyro_z_offset);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_LEN);
#else
    mavlink_serial_udb_extra_f21_t packet;
    packet.sue_accel_x_offset = sue_accel_x_offset;
    packet.sue_accel_y_offset = sue_accel_y_offset;
    packet.sue_accel_z_offset = sue_accel_z_offset;
    packet.sue_gyro_x_offset = sue_gyro_x_offset;
    packet.sue_gyro_y_offset = sue_gyro_y_offset;
    packet.sue_gyro_z_offset = sue_gyro_z_offset;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_CRC);
}

/**
 * @brief Pack a serial_udb_extra_f21 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sue_accel_x_offset SUE X accelerometer offset
 * @param sue_accel_y_offset SUE Y accelerometer offset
 * @param sue_accel_z_offset SUE Z accelerometer offset
 * @param sue_gyro_x_offset SUE X gyro offset
 * @param sue_gyro_y_offset SUE Y gyro offset
 * @param sue_gyro_z_offset SUE Z gyro offset
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f21_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int16_t sue_accel_x_offset,int16_t sue_accel_y_offset,int16_t sue_accel_z_offset,int16_t sue_gyro_x_offset,int16_t sue_gyro_y_offset,int16_t sue_gyro_z_offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_LEN];
    _mav_put_int16_t(buf, 0, sue_accel_x_offset);
    _mav_put_int16_t(buf, 2, sue_accel_y_offset);
    _mav_put_int16_t(buf, 4, sue_accel_z_offset);
    _mav_put_int16_t(buf, 6, sue_gyro_x_offset);
    _mav_put_int16_t(buf, 8, sue_gyro_y_offset);
    _mav_put_int16_t(buf, 10, sue_gyro_z_offset);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_LEN);
#else
    mavlink_serial_udb_extra_f21_t packet;
    packet.sue_accel_x_offset = sue_accel_x_offset;
    packet.sue_accel_y_offset = sue_accel_y_offset;
    packet.sue_accel_z_offset = sue_accel_z_offset;
    packet.sue_gyro_x_offset = sue_gyro_x_offset;
    packet.sue_gyro_y_offset = sue_gyro_y_offset;
    packet.sue_gyro_z_offset = sue_gyro_z_offset;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_CRC);
}

/**
 * @brief Encode a serial_udb_extra_f21 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f21 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f21_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_serial_udb_extra_f21_t* serial_udb_extra_f21)
{
    return mavlink_msg_serial_udb_extra_f21_pack(system_id, component_id, msg, serial_udb_extra_f21->sue_accel_x_offset, serial_udb_extra_f21->sue_accel_y_offset, serial_udb_extra_f21->sue_accel_z_offset, serial_udb_extra_f21->sue_gyro_x_offset, serial_udb_extra_f21->sue_gyro_y_offset, serial_udb_extra_f21->sue_gyro_z_offset);
}

/**
 * @brief Encode a serial_udb_extra_f21 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f21 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f21_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_serial_udb_extra_f21_t* serial_udb_extra_f21)
{
    return mavlink_msg_serial_udb_extra_f21_pack_chan(system_id, component_id, chan, msg, serial_udb_extra_f21->sue_accel_x_offset, serial_udb_extra_f21->sue_accel_y_offset, serial_udb_extra_f21->sue_accel_z_offset, serial_udb_extra_f21->sue_gyro_x_offset, serial_udb_extra_f21->sue_gyro_y_offset, serial_udb_extra_f21->sue_gyro_z_offset);
}

/**
 * @brief Send a serial_udb_extra_f21 message
 * @param chan MAVLink channel to send the message
 *
 * @param sue_accel_x_offset SUE X accelerometer offset
 * @param sue_accel_y_offset SUE Y accelerometer offset
 * @param sue_accel_z_offset SUE Z accelerometer offset
 * @param sue_gyro_x_offset SUE X gyro offset
 * @param sue_gyro_y_offset SUE Y gyro offset
 * @param sue_gyro_z_offset SUE Z gyro offset
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_serial_udb_extra_f21_send(mavlink_channel_t chan, int16_t sue_accel_x_offset, int16_t sue_accel_y_offset, int16_t sue_accel_z_offset, int16_t sue_gyro_x_offset, int16_t sue_gyro_y_offset, int16_t sue_gyro_z_offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_LEN];
    _mav_put_int16_t(buf, 0, sue_accel_x_offset);
    _mav_put_int16_t(buf, 2, sue_accel_y_offset);
    _mav_put_int16_t(buf, 4, sue_accel_z_offset);
    _mav_put_int16_t(buf, 6, sue_gyro_x_offset);
    _mav_put_int16_t(buf, 8, sue_gyro_y_offset);
    _mav_put_int16_t(buf, 10, sue_gyro_z_offset);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_CRC);
#else
    mavlink_serial_udb_extra_f21_t packet;
    packet.sue_accel_x_offset = sue_accel_x_offset;
    packet.sue_accel_y_offset = sue_accel_y_offset;
    packet.sue_accel_z_offset = sue_accel_z_offset;
    packet.sue_gyro_x_offset = sue_gyro_x_offset;
    packet.sue_gyro_y_offset = sue_gyro_y_offset;
    packet.sue_gyro_z_offset = sue_gyro_z_offset;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_CRC);
#endif
}

/**
 * @brief Send a serial_udb_extra_f21 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_serial_udb_extra_f21_send_struct(mavlink_channel_t chan, const mavlink_serial_udb_extra_f21_t* serial_udb_extra_f21)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_serial_udb_extra_f21_send(chan, serial_udb_extra_f21->sue_accel_x_offset, serial_udb_extra_f21->sue_accel_y_offset, serial_udb_extra_f21->sue_accel_z_offset, serial_udb_extra_f21->sue_gyro_x_offset, serial_udb_extra_f21->sue_gyro_y_offset, serial_udb_extra_f21->sue_gyro_z_offset);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21, (const char *)serial_udb_extra_f21, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_CRC);
#endif
}

#if MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_serial_udb_extra_f21_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t sue_accel_x_offset, int16_t sue_accel_y_offset, int16_t sue_accel_z_offset, int16_t sue_gyro_x_offset, int16_t sue_gyro_y_offset, int16_t sue_gyro_z_offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, sue_accel_x_offset);
    _mav_put_int16_t(buf, 2, sue_accel_y_offset);
    _mav_put_int16_t(buf, 4, sue_accel_z_offset);
    _mav_put_int16_t(buf, 6, sue_gyro_x_offset);
    _mav_put_int16_t(buf, 8, sue_gyro_y_offset);
    _mav_put_int16_t(buf, 10, sue_gyro_z_offset);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_CRC);
#else
    mavlink_serial_udb_extra_f21_t *packet = (mavlink_serial_udb_extra_f21_t *)msgbuf;
    packet->sue_accel_x_offset = sue_accel_x_offset;
    packet->sue_accel_y_offset = sue_accel_y_offset;
    packet->sue_accel_z_offset = sue_accel_z_offset;
    packet->sue_gyro_x_offset = sue_gyro_x_offset;
    packet->sue_gyro_y_offset = sue_gyro_y_offset;
    packet->sue_gyro_z_offset = sue_gyro_z_offset;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21, (const char *)packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_CRC);
#endif
}
#endif

#endif

// MESSAGE SERIAL_UDB_EXTRA_F21 UNPACKING


/**
 * @brief Get field sue_accel_x_offset from serial_udb_extra_f21 message
 *
 * @return SUE X accelerometer offset
 */
static inline int16_t mavlink_msg_serial_udb_extra_f21_get_sue_accel_x_offset(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field sue_accel_y_offset from serial_udb_extra_f21 message
 *
 * @return SUE Y accelerometer offset
 */
static inline int16_t mavlink_msg_serial_udb_extra_f21_get_sue_accel_y_offset(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field sue_accel_z_offset from serial_udb_extra_f21 message
 *
 * @return SUE Z accelerometer offset
 */
static inline int16_t mavlink_msg_serial_udb_extra_f21_get_sue_accel_z_offset(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field sue_gyro_x_offset from serial_udb_extra_f21 message
 *
 * @return SUE X gyro offset
 */
static inline int16_t mavlink_msg_serial_udb_extra_f21_get_sue_gyro_x_offset(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field sue_gyro_y_offset from serial_udb_extra_f21 message
 *
 * @return SUE Y gyro offset
 */
static inline int16_t mavlink_msg_serial_udb_extra_f21_get_sue_gyro_y_offset(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field sue_gyro_z_offset from serial_udb_extra_f21 message
 *
 * @return SUE Z gyro offset
 */
static inline int16_t mavlink_msg_serial_udb_extra_f21_get_sue_gyro_z_offset(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Decode a serial_udb_extra_f21 message into a struct
 *
 * @param msg The message to decode
 * @param serial_udb_extra_f21 C-struct to decode the message contents into
 */
static inline void mavlink_msg_serial_udb_extra_f21_decode(const mavlink_message_t* msg, mavlink_serial_udb_extra_f21_t* serial_udb_extra_f21)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    serial_udb_extra_f21->sue_accel_x_offset = mavlink_msg_serial_udb_extra_f21_get_sue_accel_x_offset(msg);
    serial_udb_extra_f21->sue_accel_y_offset = mavlink_msg_serial_udb_extra_f21_get_sue_accel_y_offset(msg);
    serial_udb_extra_f21->sue_accel_z_offset = mavlink_msg_serial_udb_extra_f21_get_sue_accel_z_offset(msg);
    serial_udb_extra_f21->sue_gyro_x_offset = mavlink_msg_serial_udb_extra_f21_get_sue_gyro_x_offset(msg);
    serial_udb_extra_f21->sue_gyro_y_offset = mavlink_msg_serial_udb_extra_f21_get_sue_gyro_y_offset(msg);
    serial_udb_extra_f21->sue_gyro_z_offset = mavlink_msg_serial_udb_extra_f21_get_sue_gyro_z_offset(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_LEN? msg->len : MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_LEN;
        memset(serial_udb_extra_f21, 0, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F21_LEN);
    memcpy(serial_udb_extra_f21, _MAV_PAYLOAD(msg), len);
#endif
}
