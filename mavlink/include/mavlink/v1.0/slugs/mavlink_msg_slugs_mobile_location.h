#pragma once
// MESSAGE SLUGS_MOBILE_LOCATION PACKING

#define MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION 186

MAVPACKED(
typedef struct __mavlink_slugs_mobile_location_t {
 float latitude; /*< Mobile Latitude*/
 float longitude; /*< Mobile Longitude*/
 uint8_t target; /*< The system reporting the action*/
}) mavlink_slugs_mobile_location_t;

#define MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_LEN 9
#define MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_MIN_LEN 9
#define MAVLINK_MSG_ID_186_LEN 9
#define MAVLINK_MSG_ID_186_MIN_LEN 9

#define MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_CRC 101
#define MAVLINK_MSG_ID_186_CRC 101



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SLUGS_MOBILE_LOCATION { \
    186, \
    "SLUGS_MOBILE_LOCATION", \
    3, \
    {  { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_slugs_mobile_location_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_slugs_mobile_location_t, longitude) }, \
         { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_slugs_mobile_location_t, target) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SLUGS_MOBILE_LOCATION { \
    "SLUGS_MOBILE_LOCATION", \
    3, \
    {  { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_slugs_mobile_location_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_slugs_mobile_location_t, longitude) }, \
         { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_slugs_mobile_location_t, target) }, \
         } \
}
#endif

/**
 * @brief Pack a slugs_mobile_location message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The system reporting the action
 * @param latitude Mobile Latitude
 * @param longitude Mobile Longitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_slugs_mobile_location_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target, float latitude, float longitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_uint8_t(buf, 8, target);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_LEN);
#else
    mavlink_slugs_mobile_location_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.target = target;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_MIN_LEN, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_LEN, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_CRC);
}

/**
 * @brief Pack a slugs_mobile_location message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system reporting the action
 * @param latitude Mobile Latitude
 * @param longitude Mobile Longitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_slugs_mobile_location_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target,float latitude,float longitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_uint8_t(buf, 8, target);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_LEN);
#else
    mavlink_slugs_mobile_location_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.target = target;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_MIN_LEN, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_LEN, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_CRC);
}

/**
 * @brief Encode a slugs_mobile_location struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param slugs_mobile_location C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_slugs_mobile_location_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_slugs_mobile_location_t* slugs_mobile_location)
{
    return mavlink_msg_slugs_mobile_location_pack(system_id, component_id, msg, slugs_mobile_location->target, slugs_mobile_location->latitude, slugs_mobile_location->longitude);
}

/**
 * @brief Encode a slugs_mobile_location struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param slugs_mobile_location C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_slugs_mobile_location_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_slugs_mobile_location_t* slugs_mobile_location)
{
    return mavlink_msg_slugs_mobile_location_pack_chan(system_id, component_id, chan, msg, slugs_mobile_location->target, slugs_mobile_location->latitude, slugs_mobile_location->longitude);
}

/**
 * @brief Send a slugs_mobile_location message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system reporting the action
 * @param latitude Mobile Latitude
 * @param longitude Mobile Longitude
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_slugs_mobile_location_send(mavlink_channel_t chan, uint8_t target, float latitude, float longitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_uint8_t(buf, 8, target);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION, buf, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_MIN_LEN, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_LEN, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_CRC);
#else
    mavlink_slugs_mobile_location_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.target = target;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION, (const char *)&packet, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_MIN_LEN, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_LEN, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_CRC);
#endif
}

/**
 * @brief Send a slugs_mobile_location message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_slugs_mobile_location_send_struct(mavlink_channel_t chan, const mavlink_slugs_mobile_location_t* slugs_mobile_location)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_slugs_mobile_location_send(chan, slugs_mobile_location->target, slugs_mobile_location->latitude, slugs_mobile_location->longitude);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION, (const char *)slugs_mobile_location, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_MIN_LEN, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_LEN, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_slugs_mobile_location_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target, float latitude, float longitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_uint8_t(buf, 8, target);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION, buf, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_MIN_LEN, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_LEN, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_CRC);
#else
    mavlink_slugs_mobile_location_t *packet = (mavlink_slugs_mobile_location_t *)msgbuf;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->target = target;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION, (const char *)packet, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_MIN_LEN, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_LEN, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_CRC);
#endif
}
#endif

#endif

// MESSAGE SLUGS_MOBILE_LOCATION UNPACKING


/**
 * @brief Get field target from slugs_mobile_location message
 *
 * @return The system reporting the action
 */
static inline uint8_t mavlink_msg_slugs_mobile_location_get_target(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field latitude from slugs_mobile_location message
 *
 * @return Mobile Latitude
 */
static inline float mavlink_msg_slugs_mobile_location_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field longitude from slugs_mobile_location message
 *
 * @return Mobile Longitude
 */
static inline float mavlink_msg_slugs_mobile_location_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a slugs_mobile_location message into a struct
 *
 * @param msg The message to decode
 * @param slugs_mobile_location C-struct to decode the message contents into
 */
static inline void mavlink_msg_slugs_mobile_location_decode(const mavlink_message_t* msg, mavlink_slugs_mobile_location_t* slugs_mobile_location)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    slugs_mobile_location->latitude = mavlink_msg_slugs_mobile_location_get_latitude(msg);
    slugs_mobile_location->longitude = mavlink_msg_slugs_mobile_location_get_longitude(msg);
    slugs_mobile_location->target = mavlink_msg_slugs_mobile_location_get_target(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_LEN? msg->len : MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_LEN;
        memset(slugs_mobile_location, 0, MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_LEN);
    memcpy(slugs_mobile_location, _MAV_PAYLOAD(msg), len);
#endif
}
