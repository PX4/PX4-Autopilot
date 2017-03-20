#pragma once
// MESSAGE SERIAL_UDB_EXTRA_F17 PACKING

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17 183

MAVPACKED(
typedef struct __mavlink_serial_udb_extra_f17_t {
 float sue_feed_forward; /*< SUE Feed Forward Gain*/
 float sue_turn_rate_nav; /*< SUE Max Turn Rate when Navigating*/
 float sue_turn_rate_fbw; /*< SUE Max Turn Rate in Fly By Wire Mode*/
}) mavlink_serial_udb_extra_f17_t;

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_LEN 12
#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_MIN_LEN 12
#define MAVLINK_MSG_ID_183_LEN 12
#define MAVLINK_MSG_ID_183_MIN_LEN 12

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_CRC 175
#define MAVLINK_MSG_ID_183_CRC 175



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F17 { \
    183, \
    "SERIAL_UDB_EXTRA_F17", \
    3, \
    {  { "sue_feed_forward", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_serial_udb_extra_f17_t, sue_feed_forward) }, \
         { "sue_turn_rate_nav", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_serial_udb_extra_f17_t, sue_turn_rate_nav) }, \
         { "sue_turn_rate_fbw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_serial_udb_extra_f17_t, sue_turn_rate_fbw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F17 { \
    "SERIAL_UDB_EXTRA_F17", \
    3, \
    {  { "sue_feed_forward", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_serial_udb_extra_f17_t, sue_feed_forward) }, \
         { "sue_turn_rate_nav", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_serial_udb_extra_f17_t, sue_turn_rate_nav) }, \
         { "sue_turn_rate_fbw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_serial_udb_extra_f17_t, sue_turn_rate_fbw) }, \
         } \
}
#endif

/**
 * @brief Pack a serial_udb_extra_f17 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sue_feed_forward SUE Feed Forward Gain
 * @param sue_turn_rate_nav SUE Max Turn Rate when Navigating
 * @param sue_turn_rate_fbw SUE Max Turn Rate in Fly By Wire Mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f17_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float sue_feed_forward, float sue_turn_rate_nav, float sue_turn_rate_fbw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_LEN];
    _mav_put_float(buf, 0, sue_feed_forward);
    _mav_put_float(buf, 4, sue_turn_rate_nav);
    _mav_put_float(buf, 8, sue_turn_rate_fbw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_LEN);
#else
    mavlink_serial_udb_extra_f17_t packet;
    packet.sue_feed_forward = sue_feed_forward;
    packet.sue_turn_rate_nav = sue_turn_rate_nav;
    packet.sue_turn_rate_fbw = sue_turn_rate_fbw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_CRC);
}

/**
 * @brief Pack a serial_udb_extra_f17 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sue_feed_forward SUE Feed Forward Gain
 * @param sue_turn_rate_nav SUE Max Turn Rate when Navigating
 * @param sue_turn_rate_fbw SUE Max Turn Rate in Fly By Wire Mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f17_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float sue_feed_forward,float sue_turn_rate_nav,float sue_turn_rate_fbw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_LEN];
    _mav_put_float(buf, 0, sue_feed_forward);
    _mav_put_float(buf, 4, sue_turn_rate_nav);
    _mav_put_float(buf, 8, sue_turn_rate_fbw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_LEN);
#else
    mavlink_serial_udb_extra_f17_t packet;
    packet.sue_feed_forward = sue_feed_forward;
    packet.sue_turn_rate_nav = sue_turn_rate_nav;
    packet.sue_turn_rate_fbw = sue_turn_rate_fbw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_CRC);
}

/**
 * @brief Encode a serial_udb_extra_f17 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f17 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f17_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_serial_udb_extra_f17_t* serial_udb_extra_f17)
{
    return mavlink_msg_serial_udb_extra_f17_pack(system_id, component_id, msg, serial_udb_extra_f17->sue_feed_forward, serial_udb_extra_f17->sue_turn_rate_nav, serial_udb_extra_f17->sue_turn_rate_fbw);
}

/**
 * @brief Encode a serial_udb_extra_f17 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f17 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f17_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_serial_udb_extra_f17_t* serial_udb_extra_f17)
{
    return mavlink_msg_serial_udb_extra_f17_pack_chan(system_id, component_id, chan, msg, serial_udb_extra_f17->sue_feed_forward, serial_udb_extra_f17->sue_turn_rate_nav, serial_udb_extra_f17->sue_turn_rate_fbw);
}

/**
 * @brief Send a serial_udb_extra_f17 message
 * @param chan MAVLink channel to send the message
 *
 * @param sue_feed_forward SUE Feed Forward Gain
 * @param sue_turn_rate_nav SUE Max Turn Rate when Navigating
 * @param sue_turn_rate_fbw SUE Max Turn Rate in Fly By Wire Mode
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_serial_udb_extra_f17_send(mavlink_channel_t chan, float sue_feed_forward, float sue_turn_rate_nav, float sue_turn_rate_fbw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_LEN];
    _mav_put_float(buf, 0, sue_feed_forward);
    _mav_put_float(buf, 4, sue_turn_rate_nav);
    _mav_put_float(buf, 8, sue_turn_rate_fbw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_CRC);
#else
    mavlink_serial_udb_extra_f17_t packet;
    packet.sue_feed_forward = sue_feed_forward;
    packet.sue_turn_rate_nav = sue_turn_rate_nav;
    packet.sue_turn_rate_fbw = sue_turn_rate_fbw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_CRC);
#endif
}

/**
 * @brief Send a serial_udb_extra_f17 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_serial_udb_extra_f17_send_struct(mavlink_channel_t chan, const mavlink_serial_udb_extra_f17_t* serial_udb_extra_f17)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_serial_udb_extra_f17_send(chan, serial_udb_extra_f17->sue_feed_forward, serial_udb_extra_f17->sue_turn_rate_nav, serial_udb_extra_f17->sue_turn_rate_fbw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17, (const char *)serial_udb_extra_f17, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_CRC);
#endif
}

#if MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_serial_udb_extra_f17_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float sue_feed_forward, float sue_turn_rate_nav, float sue_turn_rate_fbw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, sue_feed_forward);
    _mav_put_float(buf, 4, sue_turn_rate_nav);
    _mav_put_float(buf, 8, sue_turn_rate_fbw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_CRC);
#else
    mavlink_serial_udb_extra_f17_t *packet = (mavlink_serial_udb_extra_f17_t *)msgbuf;
    packet->sue_feed_forward = sue_feed_forward;
    packet->sue_turn_rate_nav = sue_turn_rate_nav;
    packet->sue_turn_rate_fbw = sue_turn_rate_fbw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17, (const char *)packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_CRC);
#endif
}
#endif

#endif

// MESSAGE SERIAL_UDB_EXTRA_F17 UNPACKING


/**
 * @brief Get field sue_feed_forward from serial_udb_extra_f17 message
 *
 * @return SUE Feed Forward Gain
 */
static inline float mavlink_msg_serial_udb_extra_f17_get_sue_feed_forward(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field sue_turn_rate_nav from serial_udb_extra_f17 message
 *
 * @return SUE Max Turn Rate when Navigating
 */
static inline float mavlink_msg_serial_udb_extra_f17_get_sue_turn_rate_nav(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field sue_turn_rate_fbw from serial_udb_extra_f17 message
 *
 * @return SUE Max Turn Rate in Fly By Wire Mode
 */
static inline float mavlink_msg_serial_udb_extra_f17_get_sue_turn_rate_fbw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a serial_udb_extra_f17 message into a struct
 *
 * @param msg The message to decode
 * @param serial_udb_extra_f17 C-struct to decode the message contents into
 */
static inline void mavlink_msg_serial_udb_extra_f17_decode(const mavlink_message_t* msg, mavlink_serial_udb_extra_f17_t* serial_udb_extra_f17)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    serial_udb_extra_f17->sue_feed_forward = mavlink_msg_serial_udb_extra_f17_get_sue_feed_forward(msg);
    serial_udb_extra_f17->sue_turn_rate_nav = mavlink_msg_serial_udb_extra_f17_get_sue_turn_rate_nav(msg);
    serial_udb_extra_f17->sue_turn_rate_fbw = mavlink_msg_serial_udb_extra_f17_get_sue_turn_rate_fbw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_LEN? msg->len : MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_LEN;
        memset(serial_udb_extra_f17, 0, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F17_LEN);
    memcpy(serial_udb_extra_f17, _MAV_PAYLOAD(msg), len);
#endif
}
