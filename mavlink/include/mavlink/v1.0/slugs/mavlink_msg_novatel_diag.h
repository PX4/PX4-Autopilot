#pragma once
// MESSAGE NOVATEL_DIAG PACKING

#define MAVLINK_MSG_ID_NOVATEL_DIAG 195

MAVPACKED(
typedef struct __mavlink_novatel_diag_t {
 uint32_t receiverStatus; /*< Status Bitfield. See table 69 page 350 Novatel OEMstar Manual*/
 float posSolAge; /*< Age of the position solution in seconds*/
 uint16_t csFails; /*< Times the CRC has failed since boot*/
 uint8_t timeStatus; /*< The Time Status. See Table 8 page 27 Novatel OEMStar Manual*/
 uint8_t solStatus; /*< solution Status. See table 44 page 197*/
 uint8_t posType; /*< position type. See table 43 page 196*/
 uint8_t velType; /*< velocity type. See table 43 page 196*/
}) mavlink_novatel_diag_t;

#define MAVLINK_MSG_ID_NOVATEL_DIAG_LEN 14
#define MAVLINK_MSG_ID_NOVATEL_DIAG_MIN_LEN 14
#define MAVLINK_MSG_ID_195_LEN 14
#define MAVLINK_MSG_ID_195_MIN_LEN 14

#define MAVLINK_MSG_ID_NOVATEL_DIAG_CRC 59
#define MAVLINK_MSG_ID_195_CRC 59



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NOVATEL_DIAG { \
    195, \
    "NOVATEL_DIAG", \
    7, \
    {  { "receiverStatus", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_novatel_diag_t, receiverStatus) }, \
         { "posSolAge", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_novatel_diag_t, posSolAge) }, \
         { "csFails", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_novatel_diag_t, csFails) }, \
         { "timeStatus", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_novatel_diag_t, timeStatus) }, \
         { "solStatus", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_novatel_diag_t, solStatus) }, \
         { "posType", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_novatel_diag_t, posType) }, \
         { "velType", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_novatel_diag_t, velType) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NOVATEL_DIAG { \
    "NOVATEL_DIAG", \
    7, \
    {  { "receiverStatus", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_novatel_diag_t, receiverStatus) }, \
         { "posSolAge", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_novatel_diag_t, posSolAge) }, \
         { "csFails", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_novatel_diag_t, csFails) }, \
         { "timeStatus", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_novatel_diag_t, timeStatus) }, \
         { "solStatus", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_novatel_diag_t, solStatus) }, \
         { "posType", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_novatel_diag_t, posType) }, \
         { "velType", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_novatel_diag_t, velType) }, \
         } \
}
#endif

/**
 * @brief Pack a novatel_diag message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timeStatus The Time Status. See Table 8 page 27 Novatel OEMStar Manual
 * @param receiverStatus Status Bitfield. See table 69 page 350 Novatel OEMstar Manual
 * @param solStatus solution Status. See table 44 page 197
 * @param posType position type. See table 43 page 196
 * @param velType velocity type. See table 43 page 196
 * @param posSolAge Age of the position solution in seconds
 * @param csFails Times the CRC has failed since boot
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_novatel_diag_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t timeStatus, uint32_t receiverStatus, uint8_t solStatus, uint8_t posType, uint8_t velType, float posSolAge, uint16_t csFails)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NOVATEL_DIAG_LEN];
    _mav_put_uint32_t(buf, 0, receiverStatus);
    _mav_put_float(buf, 4, posSolAge);
    _mav_put_uint16_t(buf, 8, csFails);
    _mav_put_uint8_t(buf, 10, timeStatus);
    _mav_put_uint8_t(buf, 11, solStatus);
    _mav_put_uint8_t(buf, 12, posType);
    _mav_put_uint8_t(buf, 13, velType);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NOVATEL_DIAG_LEN);
#else
    mavlink_novatel_diag_t packet;
    packet.receiverStatus = receiverStatus;
    packet.posSolAge = posSolAge;
    packet.csFails = csFails;
    packet.timeStatus = timeStatus;
    packet.solStatus = solStatus;
    packet.posType = posType;
    packet.velType = velType;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NOVATEL_DIAG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NOVATEL_DIAG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NOVATEL_DIAG_MIN_LEN, MAVLINK_MSG_ID_NOVATEL_DIAG_LEN, MAVLINK_MSG_ID_NOVATEL_DIAG_CRC);
}

/**
 * @brief Pack a novatel_diag message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timeStatus The Time Status. See Table 8 page 27 Novatel OEMStar Manual
 * @param receiverStatus Status Bitfield. See table 69 page 350 Novatel OEMstar Manual
 * @param solStatus solution Status. See table 44 page 197
 * @param posType position type. See table 43 page 196
 * @param velType velocity type. See table 43 page 196
 * @param posSolAge Age of the position solution in seconds
 * @param csFails Times the CRC has failed since boot
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_novatel_diag_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t timeStatus,uint32_t receiverStatus,uint8_t solStatus,uint8_t posType,uint8_t velType,float posSolAge,uint16_t csFails)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NOVATEL_DIAG_LEN];
    _mav_put_uint32_t(buf, 0, receiverStatus);
    _mav_put_float(buf, 4, posSolAge);
    _mav_put_uint16_t(buf, 8, csFails);
    _mav_put_uint8_t(buf, 10, timeStatus);
    _mav_put_uint8_t(buf, 11, solStatus);
    _mav_put_uint8_t(buf, 12, posType);
    _mav_put_uint8_t(buf, 13, velType);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NOVATEL_DIAG_LEN);
#else
    mavlink_novatel_diag_t packet;
    packet.receiverStatus = receiverStatus;
    packet.posSolAge = posSolAge;
    packet.csFails = csFails;
    packet.timeStatus = timeStatus;
    packet.solStatus = solStatus;
    packet.posType = posType;
    packet.velType = velType;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NOVATEL_DIAG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NOVATEL_DIAG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NOVATEL_DIAG_MIN_LEN, MAVLINK_MSG_ID_NOVATEL_DIAG_LEN, MAVLINK_MSG_ID_NOVATEL_DIAG_CRC);
}

/**
 * @brief Encode a novatel_diag struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param novatel_diag C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_novatel_diag_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_novatel_diag_t* novatel_diag)
{
    return mavlink_msg_novatel_diag_pack(system_id, component_id, msg, novatel_diag->timeStatus, novatel_diag->receiverStatus, novatel_diag->solStatus, novatel_diag->posType, novatel_diag->velType, novatel_diag->posSolAge, novatel_diag->csFails);
}

/**
 * @brief Encode a novatel_diag struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param novatel_diag C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_novatel_diag_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_novatel_diag_t* novatel_diag)
{
    return mavlink_msg_novatel_diag_pack_chan(system_id, component_id, chan, msg, novatel_diag->timeStatus, novatel_diag->receiverStatus, novatel_diag->solStatus, novatel_diag->posType, novatel_diag->velType, novatel_diag->posSolAge, novatel_diag->csFails);
}

/**
 * @brief Send a novatel_diag message
 * @param chan MAVLink channel to send the message
 *
 * @param timeStatus The Time Status. See Table 8 page 27 Novatel OEMStar Manual
 * @param receiverStatus Status Bitfield. See table 69 page 350 Novatel OEMstar Manual
 * @param solStatus solution Status. See table 44 page 197
 * @param posType position type. See table 43 page 196
 * @param velType velocity type. See table 43 page 196
 * @param posSolAge Age of the position solution in seconds
 * @param csFails Times the CRC has failed since boot
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_novatel_diag_send(mavlink_channel_t chan, uint8_t timeStatus, uint32_t receiverStatus, uint8_t solStatus, uint8_t posType, uint8_t velType, float posSolAge, uint16_t csFails)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NOVATEL_DIAG_LEN];
    _mav_put_uint32_t(buf, 0, receiverStatus);
    _mav_put_float(buf, 4, posSolAge);
    _mav_put_uint16_t(buf, 8, csFails);
    _mav_put_uint8_t(buf, 10, timeStatus);
    _mav_put_uint8_t(buf, 11, solStatus);
    _mav_put_uint8_t(buf, 12, posType);
    _mav_put_uint8_t(buf, 13, velType);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NOVATEL_DIAG, buf, MAVLINK_MSG_ID_NOVATEL_DIAG_MIN_LEN, MAVLINK_MSG_ID_NOVATEL_DIAG_LEN, MAVLINK_MSG_ID_NOVATEL_DIAG_CRC);
#else
    mavlink_novatel_diag_t packet;
    packet.receiverStatus = receiverStatus;
    packet.posSolAge = posSolAge;
    packet.csFails = csFails;
    packet.timeStatus = timeStatus;
    packet.solStatus = solStatus;
    packet.posType = posType;
    packet.velType = velType;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NOVATEL_DIAG, (const char *)&packet, MAVLINK_MSG_ID_NOVATEL_DIAG_MIN_LEN, MAVLINK_MSG_ID_NOVATEL_DIAG_LEN, MAVLINK_MSG_ID_NOVATEL_DIAG_CRC);
#endif
}

/**
 * @brief Send a novatel_diag message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_novatel_diag_send_struct(mavlink_channel_t chan, const mavlink_novatel_diag_t* novatel_diag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_novatel_diag_send(chan, novatel_diag->timeStatus, novatel_diag->receiverStatus, novatel_diag->solStatus, novatel_diag->posType, novatel_diag->velType, novatel_diag->posSolAge, novatel_diag->csFails);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NOVATEL_DIAG, (const char *)novatel_diag, MAVLINK_MSG_ID_NOVATEL_DIAG_MIN_LEN, MAVLINK_MSG_ID_NOVATEL_DIAG_LEN, MAVLINK_MSG_ID_NOVATEL_DIAG_CRC);
#endif
}

#if MAVLINK_MSG_ID_NOVATEL_DIAG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_novatel_diag_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t timeStatus, uint32_t receiverStatus, uint8_t solStatus, uint8_t posType, uint8_t velType, float posSolAge, uint16_t csFails)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, receiverStatus);
    _mav_put_float(buf, 4, posSolAge);
    _mav_put_uint16_t(buf, 8, csFails);
    _mav_put_uint8_t(buf, 10, timeStatus);
    _mav_put_uint8_t(buf, 11, solStatus);
    _mav_put_uint8_t(buf, 12, posType);
    _mav_put_uint8_t(buf, 13, velType);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NOVATEL_DIAG, buf, MAVLINK_MSG_ID_NOVATEL_DIAG_MIN_LEN, MAVLINK_MSG_ID_NOVATEL_DIAG_LEN, MAVLINK_MSG_ID_NOVATEL_DIAG_CRC);
#else
    mavlink_novatel_diag_t *packet = (mavlink_novatel_diag_t *)msgbuf;
    packet->receiverStatus = receiverStatus;
    packet->posSolAge = posSolAge;
    packet->csFails = csFails;
    packet->timeStatus = timeStatus;
    packet->solStatus = solStatus;
    packet->posType = posType;
    packet->velType = velType;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NOVATEL_DIAG, (const char *)packet, MAVLINK_MSG_ID_NOVATEL_DIAG_MIN_LEN, MAVLINK_MSG_ID_NOVATEL_DIAG_LEN, MAVLINK_MSG_ID_NOVATEL_DIAG_CRC);
#endif
}
#endif

#endif

// MESSAGE NOVATEL_DIAG UNPACKING


/**
 * @brief Get field timeStatus from novatel_diag message
 *
 * @return The Time Status. See Table 8 page 27 Novatel OEMStar Manual
 */
static inline uint8_t mavlink_msg_novatel_diag_get_timeStatus(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field receiverStatus from novatel_diag message
 *
 * @return Status Bitfield. See table 69 page 350 Novatel OEMstar Manual
 */
static inline uint32_t mavlink_msg_novatel_diag_get_receiverStatus(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field solStatus from novatel_diag message
 *
 * @return solution Status. See table 44 page 197
 */
static inline uint8_t mavlink_msg_novatel_diag_get_solStatus(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field posType from novatel_diag message
 *
 * @return position type. See table 43 page 196
 */
static inline uint8_t mavlink_msg_novatel_diag_get_posType(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field velType from novatel_diag message
 *
 * @return velocity type. See table 43 page 196
 */
static inline uint8_t mavlink_msg_novatel_diag_get_velType(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field posSolAge from novatel_diag message
 *
 * @return Age of the position solution in seconds
 */
static inline float mavlink_msg_novatel_diag_get_posSolAge(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field csFails from novatel_diag message
 *
 * @return Times the CRC has failed since boot
 */
static inline uint16_t mavlink_msg_novatel_diag_get_csFails(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Decode a novatel_diag message into a struct
 *
 * @param msg The message to decode
 * @param novatel_diag C-struct to decode the message contents into
 */
static inline void mavlink_msg_novatel_diag_decode(const mavlink_message_t* msg, mavlink_novatel_diag_t* novatel_diag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    novatel_diag->receiverStatus = mavlink_msg_novatel_diag_get_receiverStatus(msg);
    novatel_diag->posSolAge = mavlink_msg_novatel_diag_get_posSolAge(msg);
    novatel_diag->csFails = mavlink_msg_novatel_diag_get_csFails(msg);
    novatel_diag->timeStatus = mavlink_msg_novatel_diag_get_timeStatus(msg);
    novatel_diag->solStatus = mavlink_msg_novatel_diag_get_solStatus(msg);
    novatel_diag->posType = mavlink_msg_novatel_diag_get_posType(msg);
    novatel_diag->velType = mavlink_msg_novatel_diag_get_velType(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NOVATEL_DIAG_LEN? msg->len : MAVLINK_MSG_ID_NOVATEL_DIAG_LEN;
        memset(novatel_diag, 0, MAVLINK_MSG_ID_NOVATEL_DIAG_LEN);
    memcpy(novatel_diag, _MAV_PAYLOAD(msg), len);
#endif
}
