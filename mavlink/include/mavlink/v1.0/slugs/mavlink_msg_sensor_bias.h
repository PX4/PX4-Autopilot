#pragma once
// MESSAGE SENSOR_BIAS PACKING

#define MAVLINK_MSG_ID_SENSOR_BIAS 172

MAVPACKED(
typedef struct __mavlink_sensor_bias_t {
 float axBias; /*< Accelerometer X bias (m/s)*/
 float ayBias; /*< Accelerometer Y bias (m/s)*/
 float azBias; /*< Accelerometer Z bias (m/s)*/
 float gxBias; /*< Gyro X bias (rad/s)*/
 float gyBias; /*< Gyro Y bias (rad/s)*/
 float gzBias; /*< Gyro Z bias (rad/s)*/
}) mavlink_sensor_bias_t;

#define MAVLINK_MSG_ID_SENSOR_BIAS_LEN 24
#define MAVLINK_MSG_ID_SENSOR_BIAS_MIN_LEN 24
#define MAVLINK_MSG_ID_172_LEN 24
#define MAVLINK_MSG_ID_172_MIN_LEN 24

#define MAVLINK_MSG_ID_SENSOR_BIAS_CRC 168
#define MAVLINK_MSG_ID_172_CRC 168



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SENSOR_BIAS { \
    172, \
    "SENSOR_BIAS", \
    6, \
    {  { "axBias", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sensor_bias_t, axBias) }, \
         { "ayBias", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_sensor_bias_t, ayBias) }, \
         { "azBias", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sensor_bias_t, azBias) }, \
         { "gxBias", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sensor_bias_t, gxBias) }, \
         { "gyBias", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sensor_bias_t, gyBias) }, \
         { "gzBias", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sensor_bias_t, gzBias) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SENSOR_BIAS { \
    "SENSOR_BIAS", \
    6, \
    {  { "axBias", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sensor_bias_t, axBias) }, \
         { "ayBias", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_sensor_bias_t, ayBias) }, \
         { "azBias", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sensor_bias_t, azBias) }, \
         { "gxBias", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sensor_bias_t, gxBias) }, \
         { "gyBias", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sensor_bias_t, gyBias) }, \
         { "gzBias", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sensor_bias_t, gzBias) }, \
         } \
}
#endif

/**
 * @brief Pack a sensor_bias message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param axBias Accelerometer X bias (m/s)
 * @param ayBias Accelerometer Y bias (m/s)
 * @param azBias Accelerometer Z bias (m/s)
 * @param gxBias Gyro X bias (rad/s)
 * @param gyBias Gyro Y bias (rad/s)
 * @param gzBias Gyro Z bias (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_bias_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float axBias, float ayBias, float azBias, float gxBias, float gyBias, float gzBias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_BIAS_LEN];
    _mav_put_float(buf, 0, axBias);
    _mav_put_float(buf, 4, ayBias);
    _mav_put_float(buf, 8, azBias);
    _mav_put_float(buf, 12, gxBias);
    _mav_put_float(buf, 16, gyBias);
    _mav_put_float(buf, 20, gzBias);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSOR_BIAS_LEN);
#else
    mavlink_sensor_bias_t packet;
    packet.axBias = axBias;
    packet.ayBias = ayBias;
    packet.azBias = azBias;
    packet.gxBias = gxBias;
    packet.gyBias = gyBias;
    packet.gzBias = gzBias;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSOR_BIAS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSOR_BIAS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENSOR_BIAS_MIN_LEN, MAVLINK_MSG_ID_SENSOR_BIAS_LEN, MAVLINK_MSG_ID_SENSOR_BIAS_CRC);
}

/**
 * @brief Pack a sensor_bias message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param axBias Accelerometer X bias (m/s)
 * @param ayBias Accelerometer Y bias (m/s)
 * @param azBias Accelerometer Z bias (m/s)
 * @param gxBias Gyro X bias (rad/s)
 * @param gyBias Gyro Y bias (rad/s)
 * @param gzBias Gyro Z bias (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_bias_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float axBias,float ayBias,float azBias,float gxBias,float gyBias,float gzBias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_BIAS_LEN];
    _mav_put_float(buf, 0, axBias);
    _mav_put_float(buf, 4, ayBias);
    _mav_put_float(buf, 8, azBias);
    _mav_put_float(buf, 12, gxBias);
    _mav_put_float(buf, 16, gyBias);
    _mav_put_float(buf, 20, gzBias);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSOR_BIAS_LEN);
#else
    mavlink_sensor_bias_t packet;
    packet.axBias = axBias;
    packet.ayBias = ayBias;
    packet.azBias = azBias;
    packet.gxBias = gxBias;
    packet.gyBias = gyBias;
    packet.gzBias = gzBias;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSOR_BIAS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSOR_BIAS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENSOR_BIAS_MIN_LEN, MAVLINK_MSG_ID_SENSOR_BIAS_LEN, MAVLINK_MSG_ID_SENSOR_BIAS_CRC);
}

/**
 * @brief Encode a sensor_bias struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sensor_bias C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_bias_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sensor_bias_t* sensor_bias)
{
    return mavlink_msg_sensor_bias_pack(system_id, component_id, msg, sensor_bias->axBias, sensor_bias->ayBias, sensor_bias->azBias, sensor_bias->gxBias, sensor_bias->gyBias, sensor_bias->gzBias);
}

/**
 * @brief Encode a sensor_bias struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sensor_bias C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_bias_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sensor_bias_t* sensor_bias)
{
    return mavlink_msg_sensor_bias_pack_chan(system_id, component_id, chan, msg, sensor_bias->axBias, sensor_bias->ayBias, sensor_bias->azBias, sensor_bias->gxBias, sensor_bias->gyBias, sensor_bias->gzBias);
}

/**
 * @brief Send a sensor_bias message
 * @param chan MAVLink channel to send the message
 *
 * @param axBias Accelerometer X bias (m/s)
 * @param ayBias Accelerometer Y bias (m/s)
 * @param azBias Accelerometer Z bias (m/s)
 * @param gxBias Gyro X bias (rad/s)
 * @param gyBias Gyro Y bias (rad/s)
 * @param gzBias Gyro Z bias (rad/s)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sensor_bias_send(mavlink_channel_t chan, float axBias, float ayBias, float azBias, float gxBias, float gyBias, float gzBias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_BIAS_LEN];
    _mav_put_float(buf, 0, axBias);
    _mav_put_float(buf, 4, ayBias);
    _mav_put_float(buf, 8, azBias);
    _mav_put_float(buf, 12, gxBias);
    _mav_put_float(buf, 16, gyBias);
    _mav_put_float(buf, 20, gzBias);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_BIAS, buf, MAVLINK_MSG_ID_SENSOR_BIAS_MIN_LEN, MAVLINK_MSG_ID_SENSOR_BIAS_LEN, MAVLINK_MSG_ID_SENSOR_BIAS_CRC);
#else
    mavlink_sensor_bias_t packet;
    packet.axBias = axBias;
    packet.ayBias = ayBias;
    packet.azBias = azBias;
    packet.gxBias = gxBias;
    packet.gyBias = gyBias;
    packet.gzBias = gzBias;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_BIAS, (const char *)&packet, MAVLINK_MSG_ID_SENSOR_BIAS_MIN_LEN, MAVLINK_MSG_ID_SENSOR_BIAS_LEN, MAVLINK_MSG_ID_SENSOR_BIAS_CRC);
#endif
}

/**
 * @brief Send a sensor_bias message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sensor_bias_send_struct(mavlink_channel_t chan, const mavlink_sensor_bias_t* sensor_bias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sensor_bias_send(chan, sensor_bias->axBias, sensor_bias->ayBias, sensor_bias->azBias, sensor_bias->gxBias, sensor_bias->gyBias, sensor_bias->gzBias);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_BIAS, (const char *)sensor_bias, MAVLINK_MSG_ID_SENSOR_BIAS_MIN_LEN, MAVLINK_MSG_ID_SENSOR_BIAS_LEN, MAVLINK_MSG_ID_SENSOR_BIAS_CRC);
#endif
}

#if MAVLINK_MSG_ID_SENSOR_BIAS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sensor_bias_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float axBias, float ayBias, float azBias, float gxBias, float gyBias, float gzBias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, axBias);
    _mav_put_float(buf, 4, ayBias);
    _mav_put_float(buf, 8, azBias);
    _mav_put_float(buf, 12, gxBias);
    _mav_put_float(buf, 16, gyBias);
    _mav_put_float(buf, 20, gzBias);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_BIAS, buf, MAVLINK_MSG_ID_SENSOR_BIAS_MIN_LEN, MAVLINK_MSG_ID_SENSOR_BIAS_LEN, MAVLINK_MSG_ID_SENSOR_BIAS_CRC);
#else
    mavlink_sensor_bias_t *packet = (mavlink_sensor_bias_t *)msgbuf;
    packet->axBias = axBias;
    packet->ayBias = ayBias;
    packet->azBias = azBias;
    packet->gxBias = gxBias;
    packet->gyBias = gyBias;
    packet->gzBias = gzBias;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_BIAS, (const char *)packet, MAVLINK_MSG_ID_SENSOR_BIAS_MIN_LEN, MAVLINK_MSG_ID_SENSOR_BIAS_LEN, MAVLINK_MSG_ID_SENSOR_BIAS_CRC);
#endif
}
#endif

#endif

// MESSAGE SENSOR_BIAS UNPACKING


/**
 * @brief Get field axBias from sensor_bias message
 *
 * @return Accelerometer X bias (m/s)
 */
static inline float mavlink_msg_sensor_bias_get_axBias(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field ayBias from sensor_bias message
 *
 * @return Accelerometer Y bias (m/s)
 */
static inline float mavlink_msg_sensor_bias_get_ayBias(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field azBias from sensor_bias message
 *
 * @return Accelerometer Z bias (m/s)
 */
static inline float mavlink_msg_sensor_bias_get_azBias(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field gxBias from sensor_bias message
 *
 * @return Gyro X bias (rad/s)
 */
static inline float mavlink_msg_sensor_bias_get_gxBias(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field gyBias from sensor_bias message
 *
 * @return Gyro Y bias (rad/s)
 */
static inline float mavlink_msg_sensor_bias_get_gyBias(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field gzBias from sensor_bias message
 *
 * @return Gyro Z bias (rad/s)
 */
static inline float mavlink_msg_sensor_bias_get_gzBias(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a sensor_bias message into a struct
 *
 * @param msg The message to decode
 * @param sensor_bias C-struct to decode the message contents into
 */
static inline void mavlink_msg_sensor_bias_decode(const mavlink_message_t* msg, mavlink_sensor_bias_t* sensor_bias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sensor_bias->axBias = mavlink_msg_sensor_bias_get_axBias(msg);
    sensor_bias->ayBias = mavlink_msg_sensor_bias_get_ayBias(msg);
    sensor_bias->azBias = mavlink_msg_sensor_bias_get_azBias(msg);
    sensor_bias->gxBias = mavlink_msg_sensor_bias_get_gxBias(msg);
    sensor_bias->gyBias = mavlink_msg_sensor_bias_get_gyBias(msg);
    sensor_bias->gzBias = mavlink_msg_sensor_bias_get_gzBias(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SENSOR_BIAS_LEN? msg->len : MAVLINK_MSG_ID_SENSOR_BIAS_LEN;
        memset(sensor_bias, 0, MAVLINK_MSG_ID_SENSOR_BIAS_LEN);
    memcpy(sensor_bias, _MAV_PAYLOAD(msg), len);
#endif
}
