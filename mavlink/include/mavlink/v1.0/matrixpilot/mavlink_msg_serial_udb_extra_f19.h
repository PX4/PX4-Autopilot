#pragma once
// MESSAGE SERIAL_UDB_EXTRA_F19 PACKING

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19 185

MAVPACKED(
typedef struct __mavlink_serial_udb_extra_f19_t {
 uint8_t sue_aileron_output_channel; /*< SUE aileron output channel*/
 uint8_t sue_aileron_reversed; /*< SUE aileron reversed*/
 uint8_t sue_elevator_output_channel; /*< SUE elevator output channel*/
 uint8_t sue_elevator_reversed; /*< SUE elevator reversed*/
 uint8_t sue_throttle_output_channel; /*< SUE throttle output channel*/
 uint8_t sue_throttle_reversed; /*< SUE throttle reversed*/
 uint8_t sue_rudder_output_channel; /*< SUE rudder output channel*/
 uint8_t sue_rudder_reversed; /*< SUE rudder reversed*/
}) mavlink_serial_udb_extra_f19_t;

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_LEN 8
#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_MIN_LEN 8
#define MAVLINK_MSG_ID_185_LEN 8
#define MAVLINK_MSG_ID_185_MIN_LEN 8

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_CRC 87
#define MAVLINK_MSG_ID_185_CRC 87



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F19 { \
    185, \
    "SERIAL_UDB_EXTRA_F19", \
    8, \
    {  { "sue_aileron_output_channel", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_serial_udb_extra_f19_t, sue_aileron_output_channel) }, \
         { "sue_aileron_reversed", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_serial_udb_extra_f19_t, sue_aileron_reversed) }, \
         { "sue_elevator_output_channel", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_serial_udb_extra_f19_t, sue_elevator_output_channel) }, \
         { "sue_elevator_reversed", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_serial_udb_extra_f19_t, sue_elevator_reversed) }, \
         { "sue_throttle_output_channel", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_serial_udb_extra_f19_t, sue_throttle_output_channel) }, \
         { "sue_throttle_reversed", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_serial_udb_extra_f19_t, sue_throttle_reversed) }, \
         { "sue_rudder_output_channel", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_serial_udb_extra_f19_t, sue_rudder_output_channel) }, \
         { "sue_rudder_reversed", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_serial_udb_extra_f19_t, sue_rudder_reversed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F19 { \
    "SERIAL_UDB_EXTRA_F19", \
    8, \
    {  { "sue_aileron_output_channel", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_serial_udb_extra_f19_t, sue_aileron_output_channel) }, \
         { "sue_aileron_reversed", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_serial_udb_extra_f19_t, sue_aileron_reversed) }, \
         { "sue_elevator_output_channel", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_serial_udb_extra_f19_t, sue_elevator_output_channel) }, \
         { "sue_elevator_reversed", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_serial_udb_extra_f19_t, sue_elevator_reversed) }, \
         { "sue_throttle_output_channel", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_serial_udb_extra_f19_t, sue_throttle_output_channel) }, \
         { "sue_throttle_reversed", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_serial_udb_extra_f19_t, sue_throttle_reversed) }, \
         { "sue_rudder_output_channel", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_serial_udb_extra_f19_t, sue_rudder_output_channel) }, \
         { "sue_rudder_reversed", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_serial_udb_extra_f19_t, sue_rudder_reversed) }, \
         } \
}
#endif

/**
 * @brief Pack a serial_udb_extra_f19 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sue_aileron_output_channel SUE aileron output channel
 * @param sue_aileron_reversed SUE aileron reversed
 * @param sue_elevator_output_channel SUE elevator output channel
 * @param sue_elevator_reversed SUE elevator reversed
 * @param sue_throttle_output_channel SUE throttle output channel
 * @param sue_throttle_reversed SUE throttle reversed
 * @param sue_rudder_output_channel SUE rudder output channel
 * @param sue_rudder_reversed SUE rudder reversed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f19_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t sue_aileron_output_channel, uint8_t sue_aileron_reversed, uint8_t sue_elevator_output_channel, uint8_t sue_elevator_reversed, uint8_t sue_throttle_output_channel, uint8_t sue_throttle_reversed, uint8_t sue_rudder_output_channel, uint8_t sue_rudder_reversed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_LEN];
    _mav_put_uint8_t(buf, 0, sue_aileron_output_channel);
    _mav_put_uint8_t(buf, 1, sue_aileron_reversed);
    _mav_put_uint8_t(buf, 2, sue_elevator_output_channel);
    _mav_put_uint8_t(buf, 3, sue_elevator_reversed);
    _mav_put_uint8_t(buf, 4, sue_throttle_output_channel);
    _mav_put_uint8_t(buf, 5, sue_throttle_reversed);
    _mav_put_uint8_t(buf, 6, sue_rudder_output_channel);
    _mav_put_uint8_t(buf, 7, sue_rudder_reversed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_LEN);
#else
    mavlink_serial_udb_extra_f19_t packet;
    packet.sue_aileron_output_channel = sue_aileron_output_channel;
    packet.sue_aileron_reversed = sue_aileron_reversed;
    packet.sue_elevator_output_channel = sue_elevator_output_channel;
    packet.sue_elevator_reversed = sue_elevator_reversed;
    packet.sue_throttle_output_channel = sue_throttle_output_channel;
    packet.sue_throttle_reversed = sue_throttle_reversed;
    packet.sue_rudder_output_channel = sue_rudder_output_channel;
    packet.sue_rudder_reversed = sue_rudder_reversed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_CRC);
}

/**
 * @brief Pack a serial_udb_extra_f19 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sue_aileron_output_channel SUE aileron output channel
 * @param sue_aileron_reversed SUE aileron reversed
 * @param sue_elevator_output_channel SUE elevator output channel
 * @param sue_elevator_reversed SUE elevator reversed
 * @param sue_throttle_output_channel SUE throttle output channel
 * @param sue_throttle_reversed SUE throttle reversed
 * @param sue_rudder_output_channel SUE rudder output channel
 * @param sue_rudder_reversed SUE rudder reversed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f19_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t sue_aileron_output_channel,uint8_t sue_aileron_reversed,uint8_t sue_elevator_output_channel,uint8_t sue_elevator_reversed,uint8_t sue_throttle_output_channel,uint8_t sue_throttle_reversed,uint8_t sue_rudder_output_channel,uint8_t sue_rudder_reversed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_LEN];
    _mav_put_uint8_t(buf, 0, sue_aileron_output_channel);
    _mav_put_uint8_t(buf, 1, sue_aileron_reversed);
    _mav_put_uint8_t(buf, 2, sue_elevator_output_channel);
    _mav_put_uint8_t(buf, 3, sue_elevator_reversed);
    _mav_put_uint8_t(buf, 4, sue_throttle_output_channel);
    _mav_put_uint8_t(buf, 5, sue_throttle_reversed);
    _mav_put_uint8_t(buf, 6, sue_rudder_output_channel);
    _mav_put_uint8_t(buf, 7, sue_rudder_reversed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_LEN);
#else
    mavlink_serial_udb_extra_f19_t packet;
    packet.sue_aileron_output_channel = sue_aileron_output_channel;
    packet.sue_aileron_reversed = sue_aileron_reversed;
    packet.sue_elevator_output_channel = sue_elevator_output_channel;
    packet.sue_elevator_reversed = sue_elevator_reversed;
    packet.sue_throttle_output_channel = sue_throttle_output_channel;
    packet.sue_throttle_reversed = sue_throttle_reversed;
    packet.sue_rudder_output_channel = sue_rudder_output_channel;
    packet.sue_rudder_reversed = sue_rudder_reversed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_CRC);
}

/**
 * @brief Encode a serial_udb_extra_f19 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f19 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f19_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_serial_udb_extra_f19_t* serial_udb_extra_f19)
{
    return mavlink_msg_serial_udb_extra_f19_pack(system_id, component_id, msg, serial_udb_extra_f19->sue_aileron_output_channel, serial_udb_extra_f19->sue_aileron_reversed, serial_udb_extra_f19->sue_elevator_output_channel, serial_udb_extra_f19->sue_elevator_reversed, serial_udb_extra_f19->sue_throttle_output_channel, serial_udb_extra_f19->sue_throttle_reversed, serial_udb_extra_f19->sue_rudder_output_channel, serial_udb_extra_f19->sue_rudder_reversed);
}

/**
 * @brief Encode a serial_udb_extra_f19 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f19 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f19_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_serial_udb_extra_f19_t* serial_udb_extra_f19)
{
    return mavlink_msg_serial_udb_extra_f19_pack_chan(system_id, component_id, chan, msg, serial_udb_extra_f19->sue_aileron_output_channel, serial_udb_extra_f19->sue_aileron_reversed, serial_udb_extra_f19->sue_elevator_output_channel, serial_udb_extra_f19->sue_elevator_reversed, serial_udb_extra_f19->sue_throttle_output_channel, serial_udb_extra_f19->sue_throttle_reversed, serial_udb_extra_f19->sue_rudder_output_channel, serial_udb_extra_f19->sue_rudder_reversed);
}

/**
 * @brief Send a serial_udb_extra_f19 message
 * @param chan MAVLink channel to send the message
 *
 * @param sue_aileron_output_channel SUE aileron output channel
 * @param sue_aileron_reversed SUE aileron reversed
 * @param sue_elevator_output_channel SUE elevator output channel
 * @param sue_elevator_reversed SUE elevator reversed
 * @param sue_throttle_output_channel SUE throttle output channel
 * @param sue_throttle_reversed SUE throttle reversed
 * @param sue_rudder_output_channel SUE rudder output channel
 * @param sue_rudder_reversed SUE rudder reversed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_serial_udb_extra_f19_send(mavlink_channel_t chan, uint8_t sue_aileron_output_channel, uint8_t sue_aileron_reversed, uint8_t sue_elevator_output_channel, uint8_t sue_elevator_reversed, uint8_t sue_throttle_output_channel, uint8_t sue_throttle_reversed, uint8_t sue_rudder_output_channel, uint8_t sue_rudder_reversed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_LEN];
    _mav_put_uint8_t(buf, 0, sue_aileron_output_channel);
    _mav_put_uint8_t(buf, 1, sue_aileron_reversed);
    _mav_put_uint8_t(buf, 2, sue_elevator_output_channel);
    _mav_put_uint8_t(buf, 3, sue_elevator_reversed);
    _mav_put_uint8_t(buf, 4, sue_throttle_output_channel);
    _mav_put_uint8_t(buf, 5, sue_throttle_reversed);
    _mav_put_uint8_t(buf, 6, sue_rudder_output_channel);
    _mav_put_uint8_t(buf, 7, sue_rudder_reversed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_CRC);
#else
    mavlink_serial_udb_extra_f19_t packet;
    packet.sue_aileron_output_channel = sue_aileron_output_channel;
    packet.sue_aileron_reversed = sue_aileron_reversed;
    packet.sue_elevator_output_channel = sue_elevator_output_channel;
    packet.sue_elevator_reversed = sue_elevator_reversed;
    packet.sue_throttle_output_channel = sue_throttle_output_channel;
    packet.sue_throttle_reversed = sue_throttle_reversed;
    packet.sue_rudder_output_channel = sue_rudder_output_channel;
    packet.sue_rudder_reversed = sue_rudder_reversed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_CRC);
#endif
}

/**
 * @brief Send a serial_udb_extra_f19 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_serial_udb_extra_f19_send_struct(mavlink_channel_t chan, const mavlink_serial_udb_extra_f19_t* serial_udb_extra_f19)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_serial_udb_extra_f19_send(chan, serial_udb_extra_f19->sue_aileron_output_channel, serial_udb_extra_f19->sue_aileron_reversed, serial_udb_extra_f19->sue_elevator_output_channel, serial_udb_extra_f19->sue_elevator_reversed, serial_udb_extra_f19->sue_throttle_output_channel, serial_udb_extra_f19->sue_throttle_reversed, serial_udb_extra_f19->sue_rudder_output_channel, serial_udb_extra_f19->sue_rudder_reversed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19, (const char *)serial_udb_extra_f19, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_CRC);
#endif
}

#if MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_serial_udb_extra_f19_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t sue_aileron_output_channel, uint8_t sue_aileron_reversed, uint8_t sue_elevator_output_channel, uint8_t sue_elevator_reversed, uint8_t sue_throttle_output_channel, uint8_t sue_throttle_reversed, uint8_t sue_rudder_output_channel, uint8_t sue_rudder_reversed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, sue_aileron_output_channel);
    _mav_put_uint8_t(buf, 1, sue_aileron_reversed);
    _mav_put_uint8_t(buf, 2, sue_elevator_output_channel);
    _mav_put_uint8_t(buf, 3, sue_elevator_reversed);
    _mav_put_uint8_t(buf, 4, sue_throttle_output_channel);
    _mav_put_uint8_t(buf, 5, sue_throttle_reversed);
    _mav_put_uint8_t(buf, 6, sue_rudder_output_channel);
    _mav_put_uint8_t(buf, 7, sue_rudder_reversed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_CRC);
#else
    mavlink_serial_udb_extra_f19_t *packet = (mavlink_serial_udb_extra_f19_t *)msgbuf;
    packet->sue_aileron_output_channel = sue_aileron_output_channel;
    packet->sue_aileron_reversed = sue_aileron_reversed;
    packet->sue_elevator_output_channel = sue_elevator_output_channel;
    packet->sue_elevator_reversed = sue_elevator_reversed;
    packet->sue_throttle_output_channel = sue_throttle_output_channel;
    packet->sue_throttle_reversed = sue_throttle_reversed;
    packet->sue_rudder_output_channel = sue_rudder_output_channel;
    packet->sue_rudder_reversed = sue_rudder_reversed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19, (const char *)packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_CRC);
#endif
}
#endif

#endif

// MESSAGE SERIAL_UDB_EXTRA_F19 UNPACKING


/**
 * @brief Get field sue_aileron_output_channel from serial_udb_extra_f19 message
 *
 * @return SUE aileron output channel
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f19_get_sue_aileron_output_channel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field sue_aileron_reversed from serial_udb_extra_f19 message
 *
 * @return SUE aileron reversed
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f19_get_sue_aileron_reversed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field sue_elevator_output_channel from serial_udb_extra_f19 message
 *
 * @return SUE elevator output channel
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f19_get_sue_elevator_output_channel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field sue_elevator_reversed from serial_udb_extra_f19 message
 *
 * @return SUE elevator reversed
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f19_get_sue_elevator_reversed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field sue_throttle_output_channel from serial_udb_extra_f19 message
 *
 * @return SUE throttle output channel
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f19_get_sue_throttle_output_channel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field sue_throttle_reversed from serial_udb_extra_f19 message
 *
 * @return SUE throttle reversed
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f19_get_sue_throttle_reversed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field sue_rudder_output_channel from serial_udb_extra_f19 message
 *
 * @return SUE rudder output channel
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f19_get_sue_rudder_output_channel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field sue_rudder_reversed from serial_udb_extra_f19 message
 *
 * @return SUE rudder reversed
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f19_get_sue_rudder_reversed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Decode a serial_udb_extra_f19 message into a struct
 *
 * @param msg The message to decode
 * @param serial_udb_extra_f19 C-struct to decode the message contents into
 */
static inline void mavlink_msg_serial_udb_extra_f19_decode(const mavlink_message_t* msg, mavlink_serial_udb_extra_f19_t* serial_udb_extra_f19)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    serial_udb_extra_f19->sue_aileron_output_channel = mavlink_msg_serial_udb_extra_f19_get_sue_aileron_output_channel(msg);
    serial_udb_extra_f19->sue_aileron_reversed = mavlink_msg_serial_udb_extra_f19_get_sue_aileron_reversed(msg);
    serial_udb_extra_f19->sue_elevator_output_channel = mavlink_msg_serial_udb_extra_f19_get_sue_elevator_output_channel(msg);
    serial_udb_extra_f19->sue_elevator_reversed = mavlink_msg_serial_udb_extra_f19_get_sue_elevator_reversed(msg);
    serial_udb_extra_f19->sue_throttle_output_channel = mavlink_msg_serial_udb_extra_f19_get_sue_throttle_output_channel(msg);
    serial_udb_extra_f19->sue_throttle_reversed = mavlink_msg_serial_udb_extra_f19_get_sue_throttle_reversed(msg);
    serial_udb_extra_f19->sue_rudder_output_channel = mavlink_msg_serial_udb_extra_f19_get_sue_rudder_output_channel(msg);
    serial_udb_extra_f19->sue_rudder_reversed = mavlink_msg_serial_udb_extra_f19_get_sue_rudder_reversed(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_LEN? msg->len : MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_LEN;
        memset(serial_udb_extra_f19, 0, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F19_LEN);
    memcpy(serial_udb_extra_f19, _MAV_PAYLOAD(msg), len);
#endif
}
