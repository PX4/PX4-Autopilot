#pragma once
// MESSAGE SERIAL_UDB_EXTRA_F22 PACKING

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22 188

MAVPACKED(
typedef struct __mavlink_serial_udb_extra_f22_t {
 int16_t sue_accel_x_at_calibration; /*< SUE X accelerometer at calibration time*/
 int16_t sue_accel_y_at_calibration; /*< SUE Y accelerometer at calibration time*/
 int16_t sue_accel_z_at_calibration; /*< SUE Z accelerometer at calibration time*/
 int16_t sue_gyro_x_at_calibration; /*< SUE X gyro at calibration time*/
 int16_t sue_gyro_y_at_calibration; /*< SUE Y gyro at calibration time*/
 int16_t sue_gyro_z_at_calibration; /*< SUE Z gyro at calibration time*/
}) mavlink_serial_udb_extra_f22_t;

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_LEN 12
#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_MIN_LEN 12
#define MAVLINK_MSG_ID_188_LEN 12
#define MAVLINK_MSG_ID_188_MIN_LEN 12

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_CRC 91
#define MAVLINK_MSG_ID_188_CRC 91



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F22 { \
    188, \
    "SERIAL_UDB_EXTRA_F22", \
    6, \
    {  { "sue_accel_x_at_calibration", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_serial_udb_extra_f22_t, sue_accel_x_at_calibration) }, \
         { "sue_accel_y_at_calibration", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_serial_udb_extra_f22_t, sue_accel_y_at_calibration) }, \
         { "sue_accel_z_at_calibration", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_serial_udb_extra_f22_t, sue_accel_z_at_calibration) }, \
         { "sue_gyro_x_at_calibration", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_serial_udb_extra_f22_t, sue_gyro_x_at_calibration) }, \
         { "sue_gyro_y_at_calibration", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_serial_udb_extra_f22_t, sue_gyro_y_at_calibration) }, \
         { "sue_gyro_z_at_calibration", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_serial_udb_extra_f22_t, sue_gyro_z_at_calibration) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F22 { \
    "SERIAL_UDB_EXTRA_F22", \
    6, \
    {  { "sue_accel_x_at_calibration", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_serial_udb_extra_f22_t, sue_accel_x_at_calibration) }, \
         { "sue_accel_y_at_calibration", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_serial_udb_extra_f22_t, sue_accel_y_at_calibration) }, \
         { "sue_accel_z_at_calibration", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_serial_udb_extra_f22_t, sue_accel_z_at_calibration) }, \
         { "sue_gyro_x_at_calibration", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_serial_udb_extra_f22_t, sue_gyro_x_at_calibration) }, \
         { "sue_gyro_y_at_calibration", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_serial_udb_extra_f22_t, sue_gyro_y_at_calibration) }, \
         { "sue_gyro_z_at_calibration", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_serial_udb_extra_f22_t, sue_gyro_z_at_calibration) }, \
         } \
}
#endif

/**
 * @brief Pack a serial_udb_extra_f22 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sue_accel_x_at_calibration SUE X accelerometer at calibration time
 * @param sue_accel_y_at_calibration SUE Y accelerometer at calibration time
 * @param sue_accel_z_at_calibration SUE Z accelerometer at calibration time
 * @param sue_gyro_x_at_calibration SUE X gyro at calibration time
 * @param sue_gyro_y_at_calibration SUE Y gyro at calibration time
 * @param sue_gyro_z_at_calibration SUE Z gyro at calibration time
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f22_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int16_t sue_accel_x_at_calibration, int16_t sue_accel_y_at_calibration, int16_t sue_accel_z_at_calibration, int16_t sue_gyro_x_at_calibration, int16_t sue_gyro_y_at_calibration, int16_t sue_gyro_z_at_calibration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_LEN];
    _mav_put_int16_t(buf, 0, sue_accel_x_at_calibration);
    _mav_put_int16_t(buf, 2, sue_accel_y_at_calibration);
    _mav_put_int16_t(buf, 4, sue_accel_z_at_calibration);
    _mav_put_int16_t(buf, 6, sue_gyro_x_at_calibration);
    _mav_put_int16_t(buf, 8, sue_gyro_y_at_calibration);
    _mav_put_int16_t(buf, 10, sue_gyro_z_at_calibration);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_LEN);
#else
    mavlink_serial_udb_extra_f22_t packet;
    packet.sue_accel_x_at_calibration = sue_accel_x_at_calibration;
    packet.sue_accel_y_at_calibration = sue_accel_y_at_calibration;
    packet.sue_accel_z_at_calibration = sue_accel_z_at_calibration;
    packet.sue_gyro_x_at_calibration = sue_gyro_x_at_calibration;
    packet.sue_gyro_y_at_calibration = sue_gyro_y_at_calibration;
    packet.sue_gyro_z_at_calibration = sue_gyro_z_at_calibration;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_CRC);
}

/**
 * @brief Pack a serial_udb_extra_f22 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sue_accel_x_at_calibration SUE X accelerometer at calibration time
 * @param sue_accel_y_at_calibration SUE Y accelerometer at calibration time
 * @param sue_accel_z_at_calibration SUE Z accelerometer at calibration time
 * @param sue_gyro_x_at_calibration SUE X gyro at calibration time
 * @param sue_gyro_y_at_calibration SUE Y gyro at calibration time
 * @param sue_gyro_z_at_calibration SUE Z gyro at calibration time
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f22_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int16_t sue_accel_x_at_calibration,int16_t sue_accel_y_at_calibration,int16_t sue_accel_z_at_calibration,int16_t sue_gyro_x_at_calibration,int16_t sue_gyro_y_at_calibration,int16_t sue_gyro_z_at_calibration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_LEN];
    _mav_put_int16_t(buf, 0, sue_accel_x_at_calibration);
    _mav_put_int16_t(buf, 2, sue_accel_y_at_calibration);
    _mav_put_int16_t(buf, 4, sue_accel_z_at_calibration);
    _mav_put_int16_t(buf, 6, sue_gyro_x_at_calibration);
    _mav_put_int16_t(buf, 8, sue_gyro_y_at_calibration);
    _mav_put_int16_t(buf, 10, sue_gyro_z_at_calibration);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_LEN);
#else
    mavlink_serial_udb_extra_f22_t packet;
    packet.sue_accel_x_at_calibration = sue_accel_x_at_calibration;
    packet.sue_accel_y_at_calibration = sue_accel_y_at_calibration;
    packet.sue_accel_z_at_calibration = sue_accel_z_at_calibration;
    packet.sue_gyro_x_at_calibration = sue_gyro_x_at_calibration;
    packet.sue_gyro_y_at_calibration = sue_gyro_y_at_calibration;
    packet.sue_gyro_z_at_calibration = sue_gyro_z_at_calibration;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_CRC);
}

/**
 * @brief Encode a serial_udb_extra_f22 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f22 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f22_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_serial_udb_extra_f22_t* serial_udb_extra_f22)
{
    return mavlink_msg_serial_udb_extra_f22_pack(system_id, component_id, msg, serial_udb_extra_f22->sue_accel_x_at_calibration, serial_udb_extra_f22->sue_accel_y_at_calibration, serial_udb_extra_f22->sue_accel_z_at_calibration, serial_udb_extra_f22->sue_gyro_x_at_calibration, serial_udb_extra_f22->sue_gyro_y_at_calibration, serial_udb_extra_f22->sue_gyro_z_at_calibration);
}

/**
 * @brief Encode a serial_udb_extra_f22 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f22 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f22_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_serial_udb_extra_f22_t* serial_udb_extra_f22)
{
    return mavlink_msg_serial_udb_extra_f22_pack_chan(system_id, component_id, chan, msg, serial_udb_extra_f22->sue_accel_x_at_calibration, serial_udb_extra_f22->sue_accel_y_at_calibration, serial_udb_extra_f22->sue_accel_z_at_calibration, serial_udb_extra_f22->sue_gyro_x_at_calibration, serial_udb_extra_f22->sue_gyro_y_at_calibration, serial_udb_extra_f22->sue_gyro_z_at_calibration);
}

/**
 * @brief Send a serial_udb_extra_f22 message
 * @param chan MAVLink channel to send the message
 *
 * @param sue_accel_x_at_calibration SUE X accelerometer at calibration time
 * @param sue_accel_y_at_calibration SUE Y accelerometer at calibration time
 * @param sue_accel_z_at_calibration SUE Z accelerometer at calibration time
 * @param sue_gyro_x_at_calibration SUE X gyro at calibration time
 * @param sue_gyro_y_at_calibration SUE Y gyro at calibration time
 * @param sue_gyro_z_at_calibration SUE Z gyro at calibration time
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_serial_udb_extra_f22_send(mavlink_channel_t chan, int16_t sue_accel_x_at_calibration, int16_t sue_accel_y_at_calibration, int16_t sue_accel_z_at_calibration, int16_t sue_gyro_x_at_calibration, int16_t sue_gyro_y_at_calibration, int16_t sue_gyro_z_at_calibration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_LEN];
    _mav_put_int16_t(buf, 0, sue_accel_x_at_calibration);
    _mav_put_int16_t(buf, 2, sue_accel_y_at_calibration);
    _mav_put_int16_t(buf, 4, sue_accel_z_at_calibration);
    _mav_put_int16_t(buf, 6, sue_gyro_x_at_calibration);
    _mav_put_int16_t(buf, 8, sue_gyro_y_at_calibration);
    _mav_put_int16_t(buf, 10, sue_gyro_z_at_calibration);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_CRC);
#else
    mavlink_serial_udb_extra_f22_t packet;
    packet.sue_accel_x_at_calibration = sue_accel_x_at_calibration;
    packet.sue_accel_y_at_calibration = sue_accel_y_at_calibration;
    packet.sue_accel_z_at_calibration = sue_accel_z_at_calibration;
    packet.sue_gyro_x_at_calibration = sue_gyro_x_at_calibration;
    packet.sue_gyro_y_at_calibration = sue_gyro_y_at_calibration;
    packet.sue_gyro_z_at_calibration = sue_gyro_z_at_calibration;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_CRC);
#endif
}

/**
 * @brief Send a serial_udb_extra_f22 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_serial_udb_extra_f22_send_struct(mavlink_channel_t chan, const mavlink_serial_udb_extra_f22_t* serial_udb_extra_f22)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_serial_udb_extra_f22_send(chan, serial_udb_extra_f22->sue_accel_x_at_calibration, serial_udb_extra_f22->sue_accel_y_at_calibration, serial_udb_extra_f22->sue_accel_z_at_calibration, serial_udb_extra_f22->sue_gyro_x_at_calibration, serial_udb_extra_f22->sue_gyro_y_at_calibration, serial_udb_extra_f22->sue_gyro_z_at_calibration);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22, (const char *)serial_udb_extra_f22, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_CRC);
#endif
}

#if MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_serial_udb_extra_f22_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t sue_accel_x_at_calibration, int16_t sue_accel_y_at_calibration, int16_t sue_accel_z_at_calibration, int16_t sue_gyro_x_at_calibration, int16_t sue_gyro_y_at_calibration, int16_t sue_gyro_z_at_calibration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, sue_accel_x_at_calibration);
    _mav_put_int16_t(buf, 2, sue_accel_y_at_calibration);
    _mav_put_int16_t(buf, 4, sue_accel_z_at_calibration);
    _mav_put_int16_t(buf, 6, sue_gyro_x_at_calibration);
    _mav_put_int16_t(buf, 8, sue_gyro_y_at_calibration);
    _mav_put_int16_t(buf, 10, sue_gyro_z_at_calibration);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_CRC);
#else
    mavlink_serial_udb_extra_f22_t *packet = (mavlink_serial_udb_extra_f22_t *)msgbuf;
    packet->sue_accel_x_at_calibration = sue_accel_x_at_calibration;
    packet->sue_accel_y_at_calibration = sue_accel_y_at_calibration;
    packet->sue_accel_z_at_calibration = sue_accel_z_at_calibration;
    packet->sue_gyro_x_at_calibration = sue_gyro_x_at_calibration;
    packet->sue_gyro_y_at_calibration = sue_gyro_y_at_calibration;
    packet->sue_gyro_z_at_calibration = sue_gyro_z_at_calibration;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22, (const char *)packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_CRC);
#endif
}
#endif

#endif

// MESSAGE SERIAL_UDB_EXTRA_F22 UNPACKING


/**
 * @brief Get field sue_accel_x_at_calibration from serial_udb_extra_f22 message
 *
 * @return SUE X accelerometer at calibration time
 */
static inline int16_t mavlink_msg_serial_udb_extra_f22_get_sue_accel_x_at_calibration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field sue_accel_y_at_calibration from serial_udb_extra_f22 message
 *
 * @return SUE Y accelerometer at calibration time
 */
static inline int16_t mavlink_msg_serial_udb_extra_f22_get_sue_accel_y_at_calibration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field sue_accel_z_at_calibration from serial_udb_extra_f22 message
 *
 * @return SUE Z accelerometer at calibration time
 */
static inline int16_t mavlink_msg_serial_udb_extra_f22_get_sue_accel_z_at_calibration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field sue_gyro_x_at_calibration from serial_udb_extra_f22 message
 *
 * @return SUE X gyro at calibration time
 */
static inline int16_t mavlink_msg_serial_udb_extra_f22_get_sue_gyro_x_at_calibration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field sue_gyro_y_at_calibration from serial_udb_extra_f22 message
 *
 * @return SUE Y gyro at calibration time
 */
static inline int16_t mavlink_msg_serial_udb_extra_f22_get_sue_gyro_y_at_calibration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field sue_gyro_z_at_calibration from serial_udb_extra_f22 message
 *
 * @return SUE Z gyro at calibration time
 */
static inline int16_t mavlink_msg_serial_udb_extra_f22_get_sue_gyro_z_at_calibration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Decode a serial_udb_extra_f22 message into a struct
 *
 * @param msg The message to decode
 * @param serial_udb_extra_f22 C-struct to decode the message contents into
 */
static inline void mavlink_msg_serial_udb_extra_f22_decode(const mavlink_message_t* msg, mavlink_serial_udb_extra_f22_t* serial_udb_extra_f22)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    serial_udb_extra_f22->sue_accel_x_at_calibration = mavlink_msg_serial_udb_extra_f22_get_sue_accel_x_at_calibration(msg);
    serial_udb_extra_f22->sue_accel_y_at_calibration = mavlink_msg_serial_udb_extra_f22_get_sue_accel_y_at_calibration(msg);
    serial_udb_extra_f22->sue_accel_z_at_calibration = mavlink_msg_serial_udb_extra_f22_get_sue_accel_z_at_calibration(msg);
    serial_udb_extra_f22->sue_gyro_x_at_calibration = mavlink_msg_serial_udb_extra_f22_get_sue_gyro_x_at_calibration(msg);
    serial_udb_extra_f22->sue_gyro_y_at_calibration = mavlink_msg_serial_udb_extra_f22_get_sue_gyro_y_at_calibration(msg);
    serial_udb_extra_f22->sue_gyro_z_at_calibration = mavlink_msg_serial_udb_extra_f22_get_sue_gyro_z_at_calibration(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_LEN? msg->len : MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_LEN;
        memset(serial_udb_extra_f22, 0, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F22_LEN);
    memcpy(serial_udb_extra_f22, _MAV_PAYLOAD(msg), len);
#endif
}
