#pragma once
// MESSAGE SENS_ATMOS PACKING

#define MAVLINK_MSG_ID_SENS_ATMOS 208

MAVPACKED(
typedef struct __mavlink_sens_atmos_t {
 float TempAmbient; /*<  Ambient temperature [degrees Celsius]*/
 float Humidity; /*<  Relative humidity [%]*/
}) mavlink_sens_atmos_t;

#define MAVLINK_MSG_ID_SENS_ATMOS_LEN 8
#define MAVLINK_MSG_ID_SENS_ATMOS_MIN_LEN 8
#define MAVLINK_MSG_ID_208_LEN 8
#define MAVLINK_MSG_ID_208_MIN_LEN 8

#define MAVLINK_MSG_ID_SENS_ATMOS_CRC 175
#define MAVLINK_MSG_ID_208_CRC 175



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SENS_ATMOS { \
    208, \
    "SENS_ATMOS", \
    2, \
    {  { "TempAmbient", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sens_atmos_t, TempAmbient) }, \
         { "Humidity", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_sens_atmos_t, Humidity) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SENS_ATMOS { \
    "SENS_ATMOS", \
    2, \
    {  { "TempAmbient", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sens_atmos_t, TempAmbient) }, \
         { "Humidity", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_sens_atmos_t, Humidity) }, \
         } \
}
#endif

/**
 * @brief Pack a sens_atmos message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param TempAmbient  Ambient temperature [degrees Celsius]
 * @param Humidity  Relative humidity [%]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sens_atmos_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float TempAmbient, float Humidity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_ATMOS_LEN];
    _mav_put_float(buf, 0, TempAmbient);
    _mav_put_float(buf, 4, Humidity);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENS_ATMOS_LEN);
#else
    mavlink_sens_atmos_t packet;
    packet.TempAmbient = TempAmbient;
    packet.Humidity = Humidity;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENS_ATMOS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENS_ATMOS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENS_ATMOS_MIN_LEN, MAVLINK_MSG_ID_SENS_ATMOS_LEN, MAVLINK_MSG_ID_SENS_ATMOS_CRC);
}

/**
 * @brief Pack a sens_atmos message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param TempAmbient  Ambient temperature [degrees Celsius]
 * @param Humidity  Relative humidity [%]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sens_atmos_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float TempAmbient,float Humidity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_ATMOS_LEN];
    _mav_put_float(buf, 0, TempAmbient);
    _mav_put_float(buf, 4, Humidity);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENS_ATMOS_LEN);
#else
    mavlink_sens_atmos_t packet;
    packet.TempAmbient = TempAmbient;
    packet.Humidity = Humidity;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENS_ATMOS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENS_ATMOS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENS_ATMOS_MIN_LEN, MAVLINK_MSG_ID_SENS_ATMOS_LEN, MAVLINK_MSG_ID_SENS_ATMOS_CRC);
}

/**
 * @brief Encode a sens_atmos struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sens_atmos C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sens_atmos_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sens_atmos_t* sens_atmos)
{
    return mavlink_msg_sens_atmos_pack(system_id, component_id, msg, sens_atmos->TempAmbient, sens_atmos->Humidity);
}

/**
 * @brief Encode a sens_atmos struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sens_atmos C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sens_atmos_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sens_atmos_t* sens_atmos)
{
    return mavlink_msg_sens_atmos_pack_chan(system_id, component_id, chan, msg, sens_atmos->TempAmbient, sens_atmos->Humidity);
}

/**
 * @brief Send a sens_atmos message
 * @param chan MAVLink channel to send the message
 *
 * @param TempAmbient  Ambient temperature [degrees Celsius]
 * @param Humidity  Relative humidity [%]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sens_atmos_send(mavlink_channel_t chan, float TempAmbient, float Humidity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_ATMOS_LEN];
    _mav_put_float(buf, 0, TempAmbient);
    _mav_put_float(buf, 4, Humidity);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_ATMOS, buf, MAVLINK_MSG_ID_SENS_ATMOS_MIN_LEN, MAVLINK_MSG_ID_SENS_ATMOS_LEN, MAVLINK_MSG_ID_SENS_ATMOS_CRC);
#else
    mavlink_sens_atmos_t packet;
    packet.TempAmbient = TempAmbient;
    packet.Humidity = Humidity;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_ATMOS, (const char *)&packet, MAVLINK_MSG_ID_SENS_ATMOS_MIN_LEN, MAVLINK_MSG_ID_SENS_ATMOS_LEN, MAVLINK_MSG_ID_SENS_ATMOS_CRC);
#endif
}

/**
 * @brief Send a sens_atmos message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sens_atmos_send_struct(mavlink_channel_t chan, const mavlink_sens_atmos_t* sens_atmos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sens_atmos_send(chan, sens_atmos->TempAmbient, sens_atmos->Humidity);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_ATMOS, (const char *)sens_atmos, MAVLINK_MSG_ID_SENS_ATMOS_MIN_LEN, MAVLINK_MSG_ID_SENS_ATMOS_LEN, MAVLINK_MSG_ID_SENS_ATMOS_CRC);
#endif
}

#if MAVLINK_MSG_ID_SENS_ATMOS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sens_atmos_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float TempAmbient, float Humidity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, TempAmbient);
    _mav_put_float(buf, 4, Humidity);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_ATMOS, buf, MAVLINK_MSG_ID_SENS_ATMOS_MIN_LEN, MAVLINK_MSG_ID_SENS_ATMOS_LEN, MAVLINK_MSG_ID_SENS_ATMOS_CRC);
#else
    mavlink_sens_atmos_t *packet = (mavlink_sens_atmos_t *)msgbuf;
    packet->TempAmbient = TempAmbient;
    packet->Humidity = Humidity;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_ATMOS, (const char *)packet, MAVLINK_MSG_ID_SENS_ATMOS_MIN_LEN, MAVLINK_MSG_ID_SENS_ATMOS_LEN, MAVLINK_MSG_ID_SENS_ATMOS_CRC);
#endif
}
#endif

#endif

// MESSAGE SENS_ATMOS UNPACKING


/**
 * @brief Get field TempAmbient from sens_atmos message
 *
 * @return  Ambient temperature [degrees Celsius]
 */
static inline float mavlink_msg_sens_atmos_get_TempAmbient(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field Humidity from sens_atmos message
 *
 * @return  Relative humidity [%]
 */
static inline float mavlink_msg_sens_atmos_get_Humidity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a sens_atmos message into a struct
 *
 * @param msg The message to decode
 * @param sens_atmos C-struct to decode the message contents into
 */
static inline void mavlink_msg_sens_atmos_decode(const mavlink_message_t* msg, mavlink_sens_atmos_t* sens_atmos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sens_atmos->TempAmbient = mavlink_msg_sens_atmos_get_TempAmbient(msg);
    sens_atmos->Humidity = mavlink_msg_sens_atmos_get_Humidity(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SENS_ATMOS_LEN? msg->len : MAVLINK_MSG_ID_SENS_ATMOS_LEN;
        memset(sens_atmos, 0, MAVLINK_MSG_ID_SENS_ATMOS_LEN);
    memcpy(sens_atmos, _MAV_PAYLOAD(msg), len);
#endif
}
