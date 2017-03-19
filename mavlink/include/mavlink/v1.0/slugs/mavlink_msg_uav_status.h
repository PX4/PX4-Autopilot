#pragma once
// MESSAGE UAV_STATUS PACKING

#define MAVLINK_MSG_ID_UAV_STATUS 193

MAVPACKED(
typedef struct __mavlink_uav_status_t {
 float latitude; /*< Latitude UAV*/
 float longitude; /*< Longitude UAV*/
 float altitude; /*< Altitude UAV*/
 float speed; /*< Speed UAV*/
 float course; /*< Course UAV*/
 uint8_t target; /*< The ID system reporting the action*/
}) mavlink_uav_status_t;

#define MAVLINK_MSG_ID_UAV_STATUS_LEN 21
#define MAVLINK_MSG_ID_UAV_STATUS_MIN_LEN 21
#define MAVLINK_MSG_ID_193_LEN 21
#define MAVLINK_MSG_ID_193_MIN_LEN 21

#define MAVLINK_MSG_ID_UAV_STATUS_CRC 160
#define MAVLINK_MSG_ID_193_CRC 160



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UAV_STATUS { \
    193, \
    "UAV_STATUS", \
    6, \
    {  { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_uav_status_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_uav_status_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_uav_status_t, altitude) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_uav_status_t, speed) }, \
         { "course", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_uav_status_t, course) }, \
         { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_uav_status_t, target) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UAV_STATUS { \
    "UAV_STATUS", \
    6, \
    {  { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_uav_status_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_uav_status_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_uav_status_t, altitude) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_uav_status_t, speed) }, \
         { "course", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_uav_status_t, course) }, \
         { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_uav_status_t, target) }, \
         } \
}
#endif

/**
 * @brief Pack a uav_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The ID system reporting the action
 * @param latitude Latitude UAV
 * @param longitude Longitude UAV
 * @param altitude Altitude UAV
 * @param speed Speed UAV
 * @param course Course UAV
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uav_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target, float latitude, float longitude, float altitude, float speed, float course)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_STATUS_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, altitude);
    _mav_put_float(buf, 12, speed);
    _mav_put_float(buf, 16, course);
    _mav_put_uint8_t(buf, 20, target);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAV_STATUS_LEN);
#else
    mavlink_uav_status_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.speed = speed;
    packet.course = course;
    packet.target = target;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAV_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAV_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAV_STATUS_LEN, MAVLINK_MSG_ID_UAV_STATUS_CRC);
}

/**
 * @brief Pack a uav_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The ID system reporting the action
 * @param latitude Latitude UAV
 * @param longitude Longitude UAV
 * @param altitude Altitude UAV
 * @param speed Speed UAV
 * @param course Course UAV
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uav_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target,float latitude,float longitude,float altitude,float speed,float course)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_STATUS_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, altitude);
    _mav_put_float(buf, 12, speed);
    _mav_put_float(buf, 16, course);
    _mav_put_uint8_t(buf, 20, target);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAV_STATUS_LEN);
#else
    mavlink_uav_status_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.speed = speed;
    packet.course = course;
    packet.target = target;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAV_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAV_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAV_STATUS_LEN, MAVLINK_MSG_ID_UAV_STATUS_CRC);
}

/**
 * @brief Encode a uav_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uav_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uav_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uav_status_t* uav_status)
{
    return mavlink_msg_uav_status_pack(system_id, component_id, msg, uav_status->target, uav_status->latitude, uav_status->longitude, uav_status->altitude, uav_status->speed, uav_status->course);
}

/**
 * @brief Encode a uav_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uav_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uav_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uav_status_t* uav_status)
{
    return mavlink_msg_uav_status_pack_chan(system_id, component_id, chan, msg, uav_status->target, uav_status->latitude, uav_status->longitude, uav_status->altitude, uav_status->speed, uav_status->course);
}

/**
 * @brief Send a uav_status message
 * @param chan MAVLink channel to send the message
 *
 * @param target The ID system reporting the action
 * @param latitude Latitude UAV
 * @param longitude Longitude UAV
 * @param altitude Altitude UAV
 * @param speed Speed UAV
 * @param course Course UAV
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uav_status_send(mavlink_channel_t chan, uint8_t target, float latitude, float longitude, float altitude, float speed, float course)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAV_STATUS_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, altitude);
    _mav_put_float(buf, 12, speed);
    _mav_put_float(buf, 16, course);
    _mav_put_uint8_t(buf, 20, target);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_STATUS, buf, MAVLINK_MSG_ID_UAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAV_STATUS_LEN, MAVLINK_MSG_ID_UAV_STATUS_CRC);
#else
    mavlink_uav_status_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.speed = speed;
    packet.course = course;
    packet.target = target;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_STATUS, (const char *)&packet, MAVLINK_MSG_ID_UAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAV_STATUS_LEN, MAVLINK_MSG_ID_UAV_STATUS_CRC);
#endif
}

/**
 * @brief Send a uav_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uav_status_send_struct(mavlink_channel_t chan, const mavlink_uav_status_t* uav_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uav_status_send(chan, uav_status->target, uav_status->latitude, uav_status->longitude, uav_status->altitude, uav_status->speed, uav_status->course);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_STATUS, (const char *)uav_status, MAVLINK_MSG_ID_UAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAV_STATUS_LEN, MAVLINK_MSG_ID_UAV_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_UAV_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uav_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target, float latitude, float longitude, float altitude, float speed, float course)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, altitude);
    _mav_put_float(buf, 12, speed);
    _mav_put_float(buf, 16, course);
    _mav_put_uint8_t(buf, 20, target);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_STATUS, buf, MAVLINK_MSG_ID_UAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAV_STATUS_LEN, MAVLINK_MSG_ID_UAV_STATUS_CRC);
#else
    mavlink_uav_status_t *packet = (mavlink_uav_status_t *)msgbuf;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->altitude = altitude;
    packet->speed = speed;
    packet->course = course;
    packet->target = target;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_STATUS, (const char *)packet, MAVLINK_MSG_ID_UAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAV_STATUS_LEN, MAVLINK_MSG_ID_UAV_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE UAV_STATUS UNPACKING


/**
 * @brief Get field target from uav_status message
 *
 * @return The ID system reporting the action
 */
static inline uint8_t mavlink_msg_uav_status_get_target(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field latitude from uav_status message
 *
 * @return Latitude UAV
 */
static inline float mavlink_msg_uav_status_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field longitude from uav_status message
 *
 * @return Longitude UAV
 */
static inline float mavlink_msg_uav_status_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field altitude from uav_status message
 *
 * @return Altitude UAV
 */
static inline float mavlink_msg_uav_status_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field speed from uav_status message
 *
 * @return Speed UAV
 */
static inline float mavlink_msg_uav_status_get_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field course from uav_status message
 *
 * @return Course UAV
 */
static inline float mavlink_msg_uav_status_get_course(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a uav_status message into a struct
 *
 * @param msg The message to decode
 * @param uav_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_uav_status_decode(const mavlink_message_t* msg, mavlink_uav_status_t* uav_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    uav_status->latitude = mavlink_msg_uav_status_get_latitude(msg);
    uav_status->longitude = mavlink_msg_uav_status_get_longitude(msg);
    uav_status->altitude = mavlink_msg_uav_status_get_altitude(msg);
    uav_status->speed = mavlink_msg_uav_status_get_speed(msg);
    uav_status->course = mavlink_msg_uav_status_get_course(msg);
    uav_status->target = mavlink_msg_uav_status_get_target(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UAV_STATUS_LEN? msg->len : MAVLINK_MSG_ID_UAV_STATUS_LEN;
        memset(uav_status, 0, MAVLINK_MSG_ID_UAV_STATUS_LEN);
    memcpy(uav_status, _MAV_PAYLOAD(msg), len);
#endif
}
