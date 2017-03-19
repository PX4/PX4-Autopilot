#pragma once
// MESSAGE SENS_POWER PACKING

#define MAVLINK_MSG_ID_SENS_POWER 201

MAVPACKED(
typedef struct __mavlink_sens_power_t {
 float adc121_vspb_volt; /*<  Power board voltage sensor reading in volts*/
 float adc121_cspb_amp; /*<  Power board current sensor reading in amps*/
 float adc121_cs1_amp; /*<  Board current sensor 1 reading in amps*/
 float adc121_cs2_amp; /*<  Board current sensor 2 reading in amps*/
}) mavlink_sens_power_t;

#define MAVLINK_MSG_ID_SENS_POWER_LEN 16
#define MAVLINK_MSG_ID_SENS_POWER_MIN_LEN 16
#define MAVLINK_MSG_ID_201_LEN 16
#define MAVLINK_MSG_ID_201_MIN_LEN 16

#define MAVLINK_MSG_ID_SENS_POWER_CRC 218
#define MAVLINK_MSG_ID_201_CRC 218



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SENS_POWER { \
    201, \
    "SENS_POWER", \
    4, \
    {  { "adc121_vspb_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sens_power_t, adc121_vspb_volt) }, \
         { "adc121_cspb_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_sens_power_t, adc121_cspb_amp) }, \
         { "adc121_cs1_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sens_power_t, adc121_cs1_amp) }, \
         { "adc121_cs2_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sens_power_t, adc121_cs2_amp) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SENS_POWER { \
    "SENS_POWER", \
    4, \
    {  { "adc121_vspb_volt", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sens_power_t, adc121_vspb_volt) }, \
         { "adc121_cspb_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_sens_power_t, adc121_cspb_amp) }, \
         { "adc121_cs1_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sens_power_t, adc121_cs1_amp) }, \
         { "adc121_cs2_amp", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sens_power_t, adc121_cs2_amp) }, \
         } \
}
#endif

/**
 * @brief Pack a sens_power message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param adc121_vspb_volt  Power board voltage sensor reading in volts
 * @param adc121_cspb_amp  Power board current sensor reading in amps
 * @param adc121_cs1_amp  Board current sensor 1 reading in amps
 * @param adc121_cs2_amp  Board current sensor 2 reading in amps
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sens_power_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float adc121_vspb_volt, float adc121_cspb_amp, float adc121_cs1_amp, float adc121_cs2_amp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_POWER_LEN];
    _mav_put_float(buf, 0, adc121_vspb_volt);
    _mav_put_float(buf, 4, adc121_cspb_amp);
    _mav_put_float(buf, 8, adc121_cs1_amp);
    _mav_put_float(buf, 12, adc121_cs2_amp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENS_POWER_LEN);
#else
    mavlink_sens_power_t packet;
    packet.adc121_vspb_volt = adc121_vspb_volt;
    packet.adc121_cspb_amp = adc121_cspb_amp;
    packet.adc121_cs1_amp = adc121_cs1_amp;
    packet.adc121_cs2_amp = adc121_cs2_amp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENS_POWER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENS_POWER;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENS_POWER_MIN_LEN, MAVLINK_MSG_ID_SENS_POWER_LEN, MAVLINK_MSG_ID_SENS_POWER_CRC);
}

/**
 * @brief Pack a sens_power message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param adc121_vspb_volt  Power board voltage sensor reading in volts
 * @param adc121_cspb_amp  Power board current sensor reading in amps
 * @param adc121_cs1_amp  Board current sensor 1 reading in amps
 * @param adc121_cs2_amp  Board current sensor 2 reading in amps
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sens_power_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float adc121_vspb_volt,float adc121_cspb_amp,float adc121_cs1_amp,float adc121_cs2_amp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_POWER_LEN];
    _mav_put_float(buf, 0, adc121_vspb_volt);
    _mav_put_float(buf, 4, adc121_cspb_amp);
    _mav_put_float(buf, 8, adc121_cs1_amp);
    _mav_put_float(buf, 12, adc121_cs2_amp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENS_POWER_LEN);
#else
    mavlink_sens_power_t packet;
    packet.adc121_vspb_volt = adc121_vspb_volt;
    packet.adc121_cspb_amp = adc121_cspb_amp;
    packet.adc121_cs1_amp = adc121_cs1_amp;
    packet.adc121_cs2_amp = adc121_cs2_amp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENS_POWER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENS_POWER;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENS_POWER_MIN_LEN, MAVLINK_MSG_ID_SENS_POWER_LEN, MAVLINK_MSG_ID_SENS_POWER_CRC);
}

/**
 * @brief Encode a sens_power struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sens_power C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sens_power_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sens_power_t* sens_power)
{
    return mavlink_msg_sens_power_pack(system_id, component_id, msg, sens_power->adc121_vspb_volt, sens_power->adc121_cspb_amp, sens_power->adc121_cs1_amp, sens_power->adc121_cs2_amp);
}

/**
 * @brief Encode a sens_power struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sens_power C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sens_power_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sens_power_t* sens_power)
{
    return mavlink_msg_sens_power_pack_chan(system_id, component_id, chan, msg, sens_power->adc121_vspb_volt, sens_power->adc121_cspb_amp, sens_power->adc121_cs1_amp, sens_power->adc121_cs2_amp);
}

/**
 * @brief Send a sens_power message
 * @param chan MAVLink channel to send the message
 *
 * @param adc121_vspb_volt  Power board voltage sensor reading in volts
 * @param adc121_cspb_amp  Power board current sensor reading in amps
 * @param adc121_cs1_amp  Board current sensor 1 reading in amps
 * @param adc121_cs2_amp  Board current sensor 2 reading in amps
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sens_power_send(mavlink_channel_t chan, float adc121_vspb_volt, float adc121_cspb_amp, float adc121_cs1_amp, float adc121_cs2_amp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_POWER_LEN];
    _mav_put_float(buf, 0, adc121_vspb_volt);
    _mav_put_float(buf, 4, adc121_cspb_amp);
    _mav_put_float(buf, 8, adc121_cs1_amp);
    _mav_put_float(buf, 12, adc121_cs2_amp);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_POWER, buf, MAVLINK_MSG_ID_SENS_POWER_MIN_LEN, MAVLINK_MSG_ID_SENS_POWER_LEN, MAVLINK_MSG_ID_SENS_POWER_CRC);
#else
    mavlink_sens_power_t packet;
    packet.adc121_vspb_volt = adc121_vspb_volt;
    packet.adc121_cspb_amp = adc121_cspb_amp;
    packet.adc121_cs1_amp = adc121_cs1_amp;
    packet.adc121_cs2_amp = adc121_cs2_amp;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_POWER, (const char *)&packet, MAVLINK_MSG_ID_SENS_POWER_MIN_LEN, MAVLINK_MSG_ID_SENS_POWER_LEN, MAVLINK_MSG_ID_SENS_POWER_CRC);
#endif
}

/**
 * @brief Send a sens_power message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sens_power_send_struct(mavlink_channel_t chan, const mavlink_sens_power_t* sens_power)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sens_power_send(chan, sens_power->adc121_vspb_volt, sens_power->adc121_cspb_amp, sens_power->adc121_cs1_amp, sens_power->adc121_cs2_amp);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_POWER, (const char *)sens_power, MAVLINK_MSG_ID_SENS_POWER_MIN_LEN, MAVLINK_MSG_ID_SENS_POWER_LEN, MAVLINK_MSG_ID_SENS_POWER_CRC);
#endif
}

#if MAVLINK_MSG_ID_SENS_POWER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sens_power_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float adc121_vspb_volt, float adc121_cspb_amp, float adc121_cs1_amp, float adc121_cs2_amp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, adc121_vspb_volt);
    _mav_put_float(buf, 4, adc121_cspb_amp);
    _mav_put_float(buf, 8, adc121_cs1_amp);
    _mav_put_float(buf, 12, adc121_cs2_amp);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_POWER, buf, MAVLINK_MSG_ID_SENS_POWER_MIN_LEN, MAVLINK_MSG_ID_SENS_POWER_LEN, MAVLINK_MSG_ID_SENS_POWER_CRC);
#else
    mavlink_sens_power_t *packet = (mavlink_sens_power_t *)msgbuf;
    packet->adc121_vspb_volt = adc121_vspb_volt;
    packet->adc121_cspb_amp = adc121_cspb_amp;
    packet->adc121_cs1_amp = adc121_cs1_amp;
    packet->adc121_cs2_amp = adc121_cs2_amp;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_POWER, (const char *)packet, MAVLINK_MSG_ID_SENS_POWER_MIN_LEN, MAVLINK_MSG_ID_SENS_POWER_LEN, MAVLINK_MSG_ID_SENS_POWER_CRC);
#endif
}
#endif

#endif

// MESSAGE SENS_POWER UNPACKING


/**
 * @brief Get field adc121_vspb_volt from sens_power message
 *
 * @return  Power board voltage sensor reading in volts
 */
static inline float mavlink_msg_sens_power_get_adc121_vspb_volt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field adc121_cspb_amp from sens_power message
 *
 * @return  Power board current sensor reading in amps
 */
static inline float mavlink_msg_sens_power_get_adc121_cspb_amp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field adc121_cs1_amp from sens_power message
 *
 * @return  Board current sensor 1 reading in amps
 */
static inline float mavlink_msg_sens_power_get_adc121_cs1_amp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field adc121_cs2_amp from sens_power message
 *
 * @return  Board current sensor 2 reading in amps
 */
static inline float mavlink_msg_sens_power_get_adc121_cs2_amp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a sens_power message into a struct
 *
 * @param msg The message to decode
 * @param sens_power C-struct to decode the message contents into
 */
static inline void mavlink_msg_sens_power_decode(const mavlink_message_t* msg, mavlink_sens_power_t* sens_power)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sens_power->adc121_vspb_volt = mavlink_msg_sens_power_get_adc121_vspb_volt(msg);
    sens_power->adc121_cspb_amp = mavlink_msg_sens_power_get_adc121_cspb_amp(msg);
    sens_power->adc121_cs1_amp = mavlink_msg_sens_power_get_adc121_cs1_amp(msg);
    sens_power->adc121_cs2_amp = mavlink_msg_sens_power_get_adc121_cs2_amp(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SENS_POWER_LEN? msg->len : MAVLINK_MSG_ID_SENS_POWER_LEN;
        memset(sens_power, 0, MAVLINK_MSG_ID_SENS_POWER_LEN);
    memcpy(sens_power, _MAV_PAYLOAD(msg), len);
#endif
}
