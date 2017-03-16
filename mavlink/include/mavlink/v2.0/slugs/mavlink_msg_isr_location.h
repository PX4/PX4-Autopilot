#pragma once
// MESSAGE ISR_LOCATION PACKING

#define MAVLINK_MSG_ID_ISR_LOCATION 189

MAVPACKED(
typedef struct __mavlink_isr_location_t {
 float latitude; /*< ISR Latitude*/
 float longitude; /*< ISR Longitude*/
 float height; /*< ISR Height*/
 uint8_t target; /*< The system reporting the action*/
 uint8_t option1; /*< Option 1*/
 uint8_t option2; /*< Option 2*/
 uint8_t option3; /*< Option 3*/
}) mavlink_isr_location_t;

#define MAVLINK_MSG_ID_ISR_LOCATION_LEN 16
#define MAVLINK_MSG_ID_ISR_LOCATION_MIN_LEN 16
#define MAVLINK_MSG_ID_189_LEN 16
#define MAVLINK_MSG_ID_189_MIN_LEN 16

#define MAVLINK_MSG_ID_ISR_LOCATION_CRC 246
#define MAVLINK_MSG_ID_189_CRC 246



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ISR_LOCATION { \
    189, \
    "ISR_LOCATION", \
    7, \
    {  { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_isr_location_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_isr_location_t, longitude) }, \
         { "height", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_isr_location_t, height) }, \
         { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_isr_location_t, target) }, \
         { "option1", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_isr_location_t, option1) }, \
         { "option2", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_isr_location_t, option2) }, \
         { "option3", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_isr_location_t, option3) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ISR_LOCATION { \
    "ISR_LOCATION", \
    7, \
    {  { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_isr_location_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_isr_location_t, longitude) }, \
         { "height", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_isr_location_t, height) }, \
         { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_isr_location_t, target) }, \
         { "option1", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_isr_location_t, option1) }, \
         { "option2", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_isr_location_t, option2) }, \
         { "option3", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_isr_location_t, option3) }, \
         } \
}
#endif

/**
 * @brief Pack a isr_location message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The system reporting the action
 * @param latitude ISR Latitude
 * @param longitude ISR Longitude
 * @param height ISR Height
 * @param option1 Option 1
 * @param option2 Option 2
 * @param option3 Option 3
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_isr_location_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target, float latitude, float longitude, float height, uint8_t option1, uint8_t option2, uint8_t option3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ISR_LOCATION_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, height);
    _mav_put_uint8_t(buf, 12, target);
    _mav_put_uint8_t(buf, 13, option1);
    _mav_put_uint8_t(buf, 14, option2);
    _mav_put_uint8_t(buf, 15, option3);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ISR_LOCATION_LEN);
#else
    mavlink_isr_location_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.height = height;
    packet.target = target;
    packet.option1 = option1;
    packet.option2 = option2;
    packet.option3 = option3;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ISR_LOCATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ISR_LOCATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ISR_LOCATION_MIN_LEN, MAVLINK_MSG_ID_ISR_LOCATION_LEN, MAVLINK_MSG_ID_ISR_LOCATION_CRC);
}

/**
 * @brief Pack a isr_location message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system reporting the action
 * @param latitude ISR Latitude
 * @param longitude ISR Longitude
 * @param height ISR Height
 * @param option1 Option 1
 * @param option2 Option 2
 * @param option3 Option 3
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_isr_location_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target,float latitude,float longitude,float height,uint8_t option1,uint8_t option2,uint8_t option3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ISR_LOCATION_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, height);
    _mav_put_uint8_t(buf, 12, target);
    _mav_put_uint8_t(buf, 13, option1);
    _mav_put_uint8_t(buf, 14, option2);
    _mav_put_uint8_t(buf, 15, option3);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ISR_LOCATION_LEN);
#else
    mavlink_isr_location_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.height = height;
    packet.target = target;
    packet.option1 = option1;
    packet.option2 = option2;
    packet.option3 = option3;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ISR_LOCATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ISR_LOCATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ISR_LOCATION_MIN_LEN, MAVLINK_MSG_ID_ISR_LOCATION_LEN, MAVLINK_MSG_ID_ISR_LOCATION_CRC);
}

/**
 * @brief Encode a isr_location struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param isr_location C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_isr_location_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_isr_location_t* isr_location)
{
    return mavlink_msg_isr_location_pack(system_id, component_id, msg, isr_location->target, isr_location->latitude, isr_location->longitude, isr_location->height, isr_location->option1, isr_location->option2, isr_location->option3);
}

/**
 * @brief Encode a isr_location struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param isr_location C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_isr_location_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_isr_location_t* isr_location)
{
    return mavlink_msg_isr_location_pack_chan(system_id, component_id, chan, msg, isr_location->target, isr_location->latitude, isr_location->longitude, isr_location->height, isr_location->option1, isr_location->option2, isr_location->option3);
}

/**
 * @brief Send a isr_location message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system reporting the action
 * @param latitude ISR Latitude
 * @param longitude ISR Longitude
 * @param height ISR Height
 * @param option1 Option 1
 * @param option2 Option 2
 * @param option3 Option 3
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_isr_location_send(mavlink_channel_t chan, uint8_t target, float latitude, float longitude, float height, uint8_t option1, uint8_t option2, uint8_t option3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ISR_LOCATION_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, height);
    _mav_put_uint8_t(buf, 12, target);
    _mav_put_uint8_t(buf, 13, option1);
    _mav_put_uint8_t(buf, 14, option2);
    _mav_put_uint8_t(buf, 15, option3);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ISR_LOCATION, buf, MAVLINK_MSG_ID_ISR_LOCATION_MIN_LEN, MAVLINK_MSG_ID_ISR_LOCATION_LEN, MAVLINK_MSG_ID_ISR_LOCATION_CRC);
#else
    mavlink_isr_location_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.height = height;
    packet.target = target;
    packet.option1 = option1;
    packet.option2 = option2;
    packet.option3 = option3;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ISR_LOCATION, (const char *)&packet, MAVLINK_MSG_ID_ISR_LOCATION_MIN_LEN, MAVLINK_MSG_ID_ISR_LOCATION_LEN, MAVLINK_MSG_ID_ISR_LOCATION_CRC);
#endif
}

/**
 * @brief Send a isr_location message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_isr_location_send_struct(mavlink_channel_t chan, const mavlink_isr_location_t* isr_location)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_isr_location_send(chan, isr_location->target, isr_location->latitude, isr_location->longitude, isr_location->height, isr_location->option1, isr_location->option2, isr_location->option3);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ISR_LOCATION, (const char *)isr_location, MAVLINK_MSG_ID_ISR_LOCATION_MIN_LEN, MAVLINK_MSG_ID_ISR_LOCATION_LEN, MAVLINK_MSG_ID_ISR_LOCATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_ISR_LOCATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_isr_location_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target, float latitude, float longitude, float height, uint8_t option1, uint8_t option2, uint8_t option3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, height);
    _mav_put_uint8_t(buf, 12, target);
    _mav_put_uint8_t(buf, 13, option1);
    _mav_put_uint8_t(buf, 14, option2);
    _mav_put_uint8_t(buf, 15, option3);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ISR_LOCATION, buf, MAVLINK_MSG_ID_ISR_LOCATION_MIN_LEN, MAVLINK_MSG_ID_ISR_LOCATION_LEN, MAVLINK_MSG_ID_ISR_LOCATION_CRC);
#else
    mavlink_isr_location_t *packet = (mavlink_isr_location_t *)msgbuf;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->height = height;
    packet->target = target;
    packet->option1 = option1;
    packet->option2 = option2;
    packet->option3 = option3;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ISR_LOCATION, (const char *)packet, MAVLINK_MSG_ID_ISR_LOCATION_MIN_LEN, MAVLINK_MSG_ID_ISR_LOCATION_LEN, MAVLINK_MSG_ID_ISR_LOCATION_CRC);
#endif
}
#endif

#endif

// MESSAGE ISR_LOCATION UNPACKING


/**
 * @brief Get field target from isr_location message
 *
 * @return The system reporting the action
 */
static inline uint8_t mavlink_msg_isr_location_get_target(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field latitude from isr_location message
 *
 * @return ISR Latitude
 */
static inline float mavlink_msg_isr_location_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field longitude from isr_location message
 *
 * @return ISR Longitude
 */
static inline float mavlink_msg_isr_location_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field height from isr_location message
 *
 * @return ISR Height
 */
static inline float mavlink_msg_isr_location_get_height(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field option1 from isr_location message
 *
 * @return Option 1
 */
static inline uint8_t mavlink_msg_isr_location_get_option1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field option2 from isr_location message
 *
 * @return Option 2
 */
static inline uint8_t mavlink_msg_isr_location_get_option2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field option3 from isr_location message
 *
 * @return Option 3
 */
static inline uint8_t mavlink_msg_isr_location_get_option3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Decode a isr_location message into a struct
 *
 * @param msg The message to decode
 * @param isr_location C-struct to decode the message contents into
 */
static inline void mavlink_msg_isr_location_decode(const mavlink_message_t* msg, mavlink_isr_location_t* isr_location)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    isr_location->latitude = mavlink_msg_isr_location_get_latitude(msg);
    isr_location->longitude = mavlink_msg_isr_location_get_longitude(msg);
    isr_location->height = mavlink_msg_isr_location_get_height(msg);
    isr_location->target = mavlink_msg_isr_location_get_target(msg);
    isr_location->option1 = mavlink_msg_isr_location_get_option1(msg);
    isr_location->option2 = mavlink_msg_isr_location_get_option2(msg);
    isr_location->option3 = mavlink_msg_isr_location_get_option3(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ISR_LOCATION_LEN? msg->len : MAVLINK_MSG_ID_ISR_LOCATION_LEN;
        memset(isr_location, 0, MAVLINK_MSG_ID_ISR_LOCATION_LEN);
    memcpy(isr_location, _MAV_PAYLOAD(msg), len);
#endif
}
