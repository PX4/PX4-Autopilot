#pragma once
// MESSAGE VOLIRO_AO PACKING

#define MAVLINK_MSG_ID_VOLIRO_AO 269

MAVPACKED(
typedef struct __mavlink_voliro_ao_t {
 float alpha[6]; /*< alpha*/
 float omega[6]; /*< omega*/
}) mavlink_voliro_ao_t;

#define MAVLINK_MSG_ID_VOLIRO_AO_LEN 48
#define MAVLINK_MSG_ID_VOLIRO_AO_MIN_LEN 48
#define MAVLINK_MSG_ID_269_LEN 48
#define MAVLINK_MSG_ID_269_MIN_LEN 48

#define MAVLINK_MSG_ID_VOLIRO_AO_CRC 0
#define MAVLINK_MSG_ID_269_CRC 0

#define MAVLINK_MSG_VOLIRO_AO_FIELD_ALPHA_LEN 6
#define MAVLINK_MSG_VOLIRO_AO_FIELD_OMEGA_LEN 6

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VOLIRO_AO { \
    269, \
    "VOLIRO_AO", \
    2, \
    {  { "alpha", NULL, MAVLINK_TYPE_FLOAT, 6, 0, offsetof(mavlink_voliro_ao_t, alpha) }, \
         { "omega", NULL, MAVLINK_TYPE_FLOAT, 6, 24, offsetof(mavlink_voliro_ao_t, omega) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VOLIRO_AO { \
    "VOLIRO_AO", \
    2, \
    {  { "alpha", NULL, MAVLINK_TYPE_FLOAT, 6, 0, offsetof(mavlink_voliro_ao_t, alpha) }, \
         { "omega", NULL, MAVLINK_TYPE_FLOAT, 6, 24, offsetof(mavlink_voliro_ao_t, omega) }, \
         } \
}
#endif

/**
 * @brief Pack a voliro_ao message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param alpha alpha
 * @param omega omega
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_voliro_ao_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const float *alpha, const float *omega)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VOLIRO_AO_LEN];

    _mav_put_float_array(buf, 0, alpha, 6);
    _mav_put_float_array(buf, 24, omega, 6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VOLIRO_AO_LEN);
#else
    mavlink_voliro_ao_t packet;

    mav_array_memcpy(packet.alpha, alpha, sizeof(float)*6);
    mav_array_memcpy(packet.omega, omega, sizeof(float)*6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VOLIRO_AO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VOLIRO_AO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VOLIRO_AO_MIN_LEN, MAVLINK_MSG_ID_VOLIRO_AO_LEN, MAVLINK_MSG_ID_VOLIRO_AO_CRC);
}

/**
 * @brief Pack a voliro_ao message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param alpha alpha
 * @param omega omega
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_voliro_ao_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const float *alpha,const float *omega)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VOLIRO_AO_LEN];

    _mav_put_float_array(buf, 0, alpha, 6);
    _mav_put_float_array(buf, 24, omega, 6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VOLIRO_AO_LEN);
#else
    mavlink_voliro_ao_t packet;

    mav_array_memcpy(packet.alpha, alpha, sizeof(float)*6);
    mav_array_memcpy(packet.omega, omega, sizeof(float)*6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VOLIRO_AO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VOLIRO_AO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VOLIRO_AO_MIN_LEN, MAVLINK_MSG_ID_VOLIRO_AO_LEN, MAVLINK_MSG_ID_VOLIRO_AO_CRC);
}

/**
 * @brief Encode a voliro_ao struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param voliro_ao C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_voliro_ao_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_voliro_ao_t* voliro_ao)
{
    return mavlink_msg_voliro_ao_pack(system_id, component_id, msg, voliro_ao->alpha, voliro_ao->omega);
}

/**
 * @brief Encode a voliro_ao struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param voliro_ao C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_voliro_ao_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_voliro_ao_t* voliro_ao)
{
    return mavlink_msg_voliro_ao_pack_chan(system_id, component_id, chan, msg, voliro_ao->alpha, voliro_ao->omega);
}

/**
 * @brief Send a voliro_ao message
 * @param chan MAVLink channel to send the message
 *
 * @param alpha alpha
 * @param omega omega
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_voliro_ao_send(mavlink_channel_t chan, const float *alpha, const float *omega)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VOLIRO_AO_LEN];

    _mav_put_float_array(buf, 0, alpha, 6);
    _mav_put_float_array(buf, 24, omega, 6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VOLIRO_AO, buf, MAVLINK_MSG_ID_VOLIRO_AO_MIN_LEN, MAVLINK_MSG_ID_VOLIRO_AO_LEN, MAVLINK_MSG_ID_VOLIRO_AO_CRC);
#else
    mavlink_voliro_ao_t packet;

    mav_array_memcpy(packet.alpha, alpha, sizeof(float)*6);
    mav_array_memcpy(packet.omega, omega, sizeof(float)*6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VOLIRO_AO, (const char *)&packet, MAVLINK_MSG_ID_VOLIRO_AO_MIN_LEN, MAVLINK_MSG_ID_VOLIRO_AO_LEN, MAVLINK_MSG_ID_VOLIRO_AO_CRC);
#endif
}

/**
 * @brief Send a voliro_ao message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_voliro_ao_send_struct(mavlink_channel_t chan, const mavlink_voliro_ao_t* voliro_ao)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_voliro_ao_send(chan, voliro_ao->alpha, voliro_ao->omega);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VOLIRO_AO, (const char *)voliro_ao, MAVLINK_MSG_ID_VOLIRO_AO_MIN_LEN, MAVLINK_MSG_ID_VOLIRO_AO_LEN, MAVLINK_MSG_ID_VOLIRO_AO_CRC);
#endif
}

#if MAVLINK_MSG_ID_VOLIRO_AO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_voliro_ao_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const float *alpha, const float *omega)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_float_array(buf, 0, alpha, 6);
    _mav_put_float_array(buf, 24, omega, 6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VOLIRO_AO, buf, MAVLINK_MSG_ID_VOLIRO_AO_MIN_LEN, MAVLINK_MSG_ID_VOLIRO_AO_LEN, MAVLINK_MSG_ID_VOLIRO_AO_CRC);
#else
    mavlink_voliro_ao_t *packet = (mavlink_voliro_ao_t *)msgbuf;

    mav_array_memcpy(packet->alpha, alpha, sizeof(float)*6);
    mav_array_memcpy(packet->omega, omega, sizeof(float)*6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VOLIRO_AO, (const char *)packet, MAVLINK_MSG_ID_VOLIRO_AO_MIN_LEN, MAVLINK_MSG_ID_VOLIRO_AO_LEN, MAVLINK_MSG_ID_VOLIRO_AO_CRC);
#endif
}
#endif

#endif

// MESSAGE VOLIRO_AO UNPACKING


/**
 * @brief Get field alpha from voliro_ao message
 *
 * @return alpha
 */
static inline uint16_t mavlink_msg_voliro_ao_get_alpha(const mavlink_message_t* msg, float *alpha)
{
    return _MAV_RETURN_float_array(msg, alpha, 6,  0);
}

/**
 * @brief Get field omega from voliro_ao message
 *
 * @return omega
 */
static inline uint16_t mavlink_msg_voliro_ao_get_omega(const mavlink_message_t* msg, float *omega)
{
    return _MAV_RETURN_float_array(msg, omega, 6,  24);
}

/**
 * @brief Decode a voliro_ao message into a struct
 *
 * @param msg The message to decode
 * @param voliro_ao C-struct to decode the message contents into
 */
static inline void mavlink_msg_voliro_ao_decode(const mavlink_message_t* msg, mavlink_voliro_ao_t* voliro_ao)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_voliro_ao_get_alpha(msg, voliro_ao->alpha);
    mavlink_msg_voliro_ao_get_omega(msg, voliro_ao->omega);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VOLIRO_AO_LEN? msg->len : MAVLINK_MSG_ID_VOLIRO_AO_LEN;
        memset(voliro_ao, 0, MAVLINK_MSG_ID_VOLIRO_AO_LEN);
    memcpy(voliro_ao, _MAV_PAYLOAD(msg), len);
#endif
}
