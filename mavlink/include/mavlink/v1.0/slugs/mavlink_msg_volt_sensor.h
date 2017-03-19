#pragma once
// MESSAGE VOLT_SENSOR PACKING

#define MAVLINK_MSG_ID_VOLT_SENSOR 191

MAVPACKED(
typedef struct __mavlink_volt_sensor_t {
 uint16_t voltage; /*< Voltage in uS of PWM. 0 uS = 0V, 20 uS = 21.5V */
 uint16_t reading2; /*< Depends on the value of r2Type (0) Current consumption in uS of PWM, 20 uS = 90Amp (1) Distance in cm (2) Distance in cm (3) Absolute value*/
 uint8_t r2Type; /*< It is the value of reading 2: 0 - Current, 1 - Foreward Sonar, 2 - Back Sonar, 3 - RPM*/
}) mavlink_volt_sensor_t;

#define MAVLINK_MSG_ID_VOLT_SENSOR_LEN 5
#define MAVLINK_MSG_ID_VOLT_SENSOR_MIN_LEN 5
#define MAVLINK_MSG_ID_191_LEN 5
#define MAVLINK_MSG_ID_191_MIN_LEN 5

#define MAVLINK_MSG_ID_VOLT_SENSOR_CRC 17
#define MAVLINK_MSG_ID_191_CRC 17



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VOLT_SENSOR { \
    191, \
    "VOLT_SENSOR", \
    3, \
    {  { "voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_volt_sensor_t, voltage) }, \
         { "reading2", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_volt_sensor_t, reading2) }, \
         { "r2Type", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_volt_sensor_t, r2Type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VOLT_SENSOR { \
    "VOLT_SENSOR", \
    3, \
    {  { "voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_volt_sensor_t, voltage) }, \
         { "reading2", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_volt_sensor_t, reading2) }, \
         { "r2Type", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_volt_sensor_t, r2Type) }, \
         } \
}
#endif

/**
 * @brief Pack a volt_sensor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param r2Type It is the value of reading 2: 0 - Current, 1 - Foreward Sonar, 2 - Back Sonar, 3 - RPM
 * @param voltage Voltage in uS of PWM. 0 uS = 0V, 20 uS = 21.5V 
 * @param reading2 Depends on the value of r2Type (0) Current consumption in uS of PWM, 20 uS = 90Amp (1) Distance in cm (2) Distance in cm (3) Absolute value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_volt_sensor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t r2Type, uint16_t voltage, uint16_t reading2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VOLT_SENSOR_LEN];
    _mav_put_uint16_t(buf, 0, voltage);
    _mav_put_uint16_t(buf, 2, reading2);
    _mav_put_uint8_t(buf, 4, r2Type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VOLT_SENSOR_LEN);
#else
    mavlink_volt_sensor_t packet;
    packet.voltage = voltage;
    packet.reading2 = reading2;
    packet.r2Type = r2Type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VOLT_SENSOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VOLT_SENSOR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VOLT_SENSOR_MIN_LEN, MAVLINK_MSG_ID_VOLT_SENSOR_LEN, MAVLINK_MSG_ID_VOLT_SENSOR_CRC);
}

/**
 * @brief Pack a volt_sensor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param r2Type It is the value of reading 2: 0 - Current, 1 - Foreward Sonar, 2 - Back Sonar, 3 - RPM
 * @param voltage Voltage in uS of PWM. 0 uS = 0V, 20 uS = 21.5V 
 * @param reading2 Depends on the value of r2Type (0) Current consumption in uS of PWM, 20 uS = 90Amp (1) Distance in cm (2) Distance in cm (3) Absolute value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_volt_sensor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t r2Type,uint16_t voltage,uint16_t reading2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VOLT_SENSOR_LEN];
    _mav_put_uint16_t(buf, 0, voltage);
    _mav_put_uint16_t(buf, 2, reading2);
    _mav_put_uint8_t(buf, 4, r2Type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VOLT_SENSOR_LEN);
#else
    mavlink_volt_sensor_t packet;
    packet.voltage = voltage;
    packet.reading2 = reading2;
    packet.r2Type = r2Type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VOLT_SENSOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VOLT_SENSOR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VOLT_SENSOR_MIN_LEN, MAVLINK_MSG_ID_VOLT_SENSOR_LEN, MAVLINK_MSG_ID_VOLT_SENSOR_CRC);
}

/**
 * @brief Encode a volt_sensor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param volt_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_volt_sensor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_volt_sensor_t* volt_sensor)
{
    return mavlink_msg_volt_sensor_pack(system_id, component_id, msg, volt_sensor->r2Type, volt_sensor->voltage, volt_sensor->reading2);
}

/**
 * @brief Encode a volt_sensor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param volt_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_volt_sensor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_volt_sensor_t* volt_sensor)
{
    return mavlink_msg_volt_sensor_pack_chan(system_id, component_id, chan, msg, volt_sensor->r2Type, volt_sensor->voltage, volt_sensor->reading2);
}

/**
 * @brief Send a volt_sensor message
 * @param chan MAVLink channel to send the message
 *
 * @param r2Type It is the value of reading 2: 0 - Current, 1 - Foreward Sonar, 2 - Back Sonar, 3 - RPM
 * @param voltage Voltage in uS of PWM. 0 uS = 0V, 20 uS = 21.5V 
 * @param reading2 Depends on the value of r2Type (0) Current consumption in uS of PWM, 20 uS = 90Amp (1) Distance in cm (2) Distance in cm (3) Absolute value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_volt_sensor_send(mavlink_channel_t chan, uint8_t r2Type, uint16_t voltage, uint16_t reading2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VOLT_SENSOR_LEN];
    _mav_put_uint16_t(buf, 0, voltage);
    _mav_put_uint16_t(buf, 2, reading2);
    _mav_put_uint8_t(buf, 4, r2Type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VOLT_SENSOR, buf, MAVLINK_MSG_ID_VOLT_SENSOR_MIN_LEN, MAVLINK_MSG_ID_VOLT_SENSOR_LEN, MAVLINK_MSG_ID_VOLT_SENSOR_CRC);
#else
    mavlink_volt_sensor_t packet;
    packet.voltage = voltage;
    packet.reading2 = reading2;
    packet.r2Type = r2Type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VOLT_SENSOR, (const char *)&packet, MAVLINK_MSG_ID_VOLT_SENSOR_MIN_LEN, MAVLINK_MSG_ID_VOLT_SENSOR_LEN, MAVLINK_MSG_ID_VOLT_SENSOR_CRC);
#endif
}

/**
 * @brief Send a volt_sensor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_volt_sensor_send_struct(mavlink_channel_t chan, const mavlink_volt_sensor_t* volt_sensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_volt_sensor_send(chan, volt_sensor->r2Type, volt_sensor->voltage, volt_sensor->reading2);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VOLT_SENSOR, (const char *)volt_sensor, MAVLINK_MSG_ID_VOLT_SENSOR_MIN_LEN, MAVLINK_MSG_ID_VOLT_SENSOR_LEN, MAVLINK_MSG_ID_VOLT_SENSOR_CRC);
#endif
}

#if MAVLINK_MSG_ID_VOLT_SENSOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_volt_sensor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t r2Type, uint16_t voltage, uint16_t reading2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, voltage);
    _mav_put_uint16_t(buf, 2, reading2);
    _mav_put_uint8_t(buf, 4, r2Type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VOLT_SENSOR, buf, MAVLINK_MSG_ID_VOLT_SENSOR_MIN_LEN, MAVLINK_MSG_ID_VOLT_SENSOR_LEN, MAVLINK_MSG_ID_VOLT_SENSOR_CRC);
#else
    mavlink_volt_sensor_t *packet = (mavlink_volt_sensor_t *)msgbuf;
    packet->voltage = voltage;
    packet->reading2 = reading2;
    packet->r2Type = r2Type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VOLT_SENSOR, (const char *)packet, MAVLINK_MSG_ID_VOLT_SENSOR_MIN_LEN, MAVLINK_MSG_ID_VOLT_SENSOR_LEN, MAVLINK_MSG_ID_VOLT_SENSOR_CRC);
#endif
}
#endif

#endif

// MESSAGE VOLT_SENSOR UNPACKING


/**
 * @brief Get field r2Type from volt_sensor message
 *
 * @return It is the value of reading 2: 0 - Current, 1 - Foreward Sonar, 2 - Back Sonar, 3 - RPM
 */
static inline uint8_t mavlink_msg_volt_sensor_get_r2Type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field voltage from volt_sensor message
 *
 * @return Voltage in uS of PWM. 0 uS = 0V, 20 uS = 21.5V 
 */
static inline uint16_t mavlink_msg_volt_sensor_get_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field reading2 from volt_sensor message
 *
 * @return Depends on the value of r2Type (0) Current consumption in uS of PWM, 20 uS = 90Amp (1) Distance in cm (2) Distance in cm (3) Absolute value
 */
static inline uint16_t mavlink_msg_volt_sensor_get_reading2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Decode a volt_sensor message into a struct
 *
 * @param msg The message to decode
 * @param volt_sensor C-struct to decode the message contents into
 */
static inline void mavlink_msg_volt_sensor_decode(const mavlink_message_t* msg, mavlink_volt_sensor_t* volt_sensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    volt_sensor->voltage = mavlink_msg_volt_sensor_get_voltage(msg);
    volt_sensor->reading2 = mavlink_msg_volt_sensor_get_reading2(msg);
    volt_sensor->r2Type = mavlink_msg_volt_sensor_get_r2Type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VOLT_SENSOR_LEN? msg->len : MAVLINK_MSG_ID_VOLT_SENSOR_LEN;
        memset(volt_sensor, 0, MAVLINK_MSG_ID_VOLT_SENSOR_LEN);
    memcpy(volt_sensor, _MAV_PAYLOAD(msg), len);
#endif
}
