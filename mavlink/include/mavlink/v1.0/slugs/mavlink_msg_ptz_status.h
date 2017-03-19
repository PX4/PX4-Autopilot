#pragma once
// MESSAGE PTZ_STATUS PACKING

#define MAVLINK_MSG_ID_PTZ_STATUS 192

MAVPACKED(
typedef struct __mavlink_ptz_status_t {
 int16_t pan; /*< The Pan value in 10ths of degree*/
 int16_t tilt; /*< The Tilt value in 10ths of degree*/
 uint8_t zoom; /*< The actual Zoom Value*/
}) mavlink_ptz_status_t;

#define MAVLINK_MSG_ID_PTZ_STATUS_LEN 5
#define MAVLINK_MSG_ID_PTZ_STATUS_MIN_LEN 5
#define MAVLINK_MSG_ID_192_LEN 5
#define MAVLINK_MSG_ID_192_MIN_LEN 5

#define MAVLINK_MSG_ID_PTZ_STATUS_CRC 187
#define MAVLINK_MSG_ID_192_CRC 187



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PTZ_STATUS { \
    192, \
    "PTZ_STATUS", \
    3, \
    {  { "pan", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_ptz_status_t, pan) }, \
         { "tilt", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_ptz_status_t, tilt) }, \
         { "zoom", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_ptz_status_t, zoom) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PTZ_STATUS { \
    "PTZ_STATUS", \
    3, \
    {  { "pan", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_ptz_status_t, pan) }, \
         { "tilt", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_ptz_status_t, tilt) }, \
         { "zoom", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_ptz_status_t, zoom) }, \
         } \
}
#endif

/**
 * @brief Pack a ptz_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param zoom The actual Zoom Value
 * @param pan The Pan value in 10ths of degree
 * @param tilt The Tilt value in 10ths of degree
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ptz_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t zoom, int16_t pan, int16_t tilt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PTZ_STATUS_LEN];
    _mav_put_int16_t(buf, 0, pan);
    _mav_put_int16_t(buf, 2, tilt);
    _mav_put_uint8_t(buf, 4, zoom);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PTZ_STATUS_LEN);
#else
    mavlink_ptz_status_t packet;
    packet.pan = pan;
    packet.tilt = tilt;
    packet.zoom = zoom;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PTZ_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PTZ_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PTZ_STATUS_MIN_LEN, MAVLINK_MSG_ID_PTZ_STATUS_LEN, MAVLINK_MSG_ID_PTZ_STATUS_CRC);
}

/**
 * @brief Pack a ptz_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param zoom The actual Zoom Value
 * @param pan The Pan value in 10ths of degree
 * @param tilt The Tilt value in 10ths of degree
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ptz_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t zoom,int16_t pan,int16_t tilt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PTZ_STATUS_LEN];
    _mav_put_int16_t(buf, 0, pan);
    _mav_put_int16_t(buf, 2, tilt);
    _mav_put_uint8_t(buf, 4, zoom);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PTZ_STATUS_LEN);
#else
    mavlink_ptz_status_t packet;
    packet.pan = pan;
    packet.tilt = tilt;
    packet.zoom = zoom;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PTZ_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PTZ_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PTZ_STATUS_MIN_LEN, MAVLINK_MSG_ID_PTZ_STATUS_LEN, MAVLINK_MSG_ID_PTZ_STATUS_CRC);
}

/**
 * @brief Encode a ptz_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ptz_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ptz_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ptz_status_t* ptz_status)
{
    return mavlink_msg_ptz_status_pack(system_id, component_id, msg, ptz_status->zoom, ptz_status->pan, ptz_status->tilt);
}

/**
 * @brief Encode a ptz_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ptz_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ptz_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ptz_status_t* ptz_status)
{
    return mavlink_msg_ptz_status_pack_chan(system_id, component_id, chan, msg, ptz_status->zoom, ptz_status->pan, ptz_status->tilt);
}

/**
 * @brief Send a ptz_status message
 * @param chan MAVLink channel to send the message
 *
 * @param zoom The actual Zoom Value
 * @param pan The Pan value in 10ths of degree
 * @param tilt The Tilt value in 10ths of degree
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ptz_status_send(mavlink_channel_t chan, uint8_t zoom, int16_t pan, int16_t tilt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PTZ_STATUS_LEN];
    _mav_put_int16_t(buf, 0, pan);
    _mav_put_int16_t(buf, 2, tilt);
    _mav_put_uint8_t(buf, 4, zoom);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PTZ_STATUS, buf, MAVLINK_MSG_ID_PTZ_STATUS_MIN_LEN, MAVLINK_MSG_ID_PTZ_STATUS_LEN, MAVLINK_MSG_ID_PTZ_STATUS_CRC);
#else
    mavlink_ptz_status_t packet;
    packet.pan = pan;
    packet.tilt = tilt;
    packet.zoom = zoom;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PTZ_STATUS, (const char *)&packet, MAVLINK_MSG_ID_PTZ_STATUS_MIN_LEN, MAVLINK_MSG_ID_PTZ_STATUS_LEN, MAVLINK_MSG_ID_PTZ_STATUS_CRC);
#endif
}

/**
 * @brief Send a ptz_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ptz_status_send_struct(mavlink_channel_t chan, const mavlink_ptz_status_t* ptz_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ptz_status_send(chan, ptz_status->zoom, ptz_status->pan, ptz_status->tilt);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PTZ_STATUS, (const char *)ptz_status, MAVLINK_MSG_ID_PTZ_STATUS_MIN_LEN, MAVLINK_MSG_ID_PTZ_STATUS_LEN, MAVLINK_MSG_ID_PTZ_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_PTZ_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ptz_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t zoom, int16_t pan, int16_t tilt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, pan);
    _mav_put_int16_t(buf, 2, tilt);
    _mav_put_uint8_t(buf, 4, zoom);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PTZ_STATUS, buf, MAVLINK_MSG_ID_PTZ_STATUS_MIN_LEN, MAVLINK_MSG_ID_PTZ_STATUS_LEN, MAVLINK_MSG_ID_PTZ_STATUS_CRC);
#else
    mavlink_ptz_status_t *packet = (mavlink_ptz_status_t *)msgbuf;
    packet->pan = pan;
    packet->tilt = tilt;
    packet->zoom = zoom;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PTZ_STATUS, (const char *)packet, MAVLINK_MSG_ID_PTZ_STATUS_MIN_LEN, MAVLINK_MSG_ID_PTZ_STATUS_LEN, MAVLINK_MSG_ID_PTZ_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE PTZ_STATUS UNPACKING


/**
 * @brief Get field zoom from ptz_status message
 *
 * @return The actual Zoom Value
 */
static inline uint8_t mavlink_msg_ptz_status_get_zoom(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field pan from ptz_status message
 *
 * @return The Pan value in 10ths of degree
 */
static inline int16_t mavlink_msg_ptz_status_get_pan(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field tilt from ptz_status message
 *
 * @return The Tilt value in 10ths of degree
 */
static inline int16_t mavlink_msg_ptz_status_get_tilt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Decode a ptz_status message into a struct
 *
 * @param msg The message to decode
 * @param ptz_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_ptz_status_decode(const mavlink_message_t* msg, mavlink_ptz_status_t* ptz_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ptz_status->pan = mavlink_msg_ptz_status_get_pan(msg);
    ptz_status->tilt = mavlink_msg_ptz_status_get_tilt(msg);
    ptz_status->zoom = mavlink_msg_ptz_status_get_zoom(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PTZ_STATUS_LEN? msg->len : MAVLINK_MSG_ID_PTZ_STATUS_LEN;
        memset(ptz_status, 0, MAVLINK_MSG_ID_PTZ_STATUS_LEN);
    memcpy(ptz_status, _MAV_PAYLOAD(msg), len);
#endif
}
