#pragma once
// MESSAGE BOOT PACKING

#define MAVLINK_MSG_ID_BOOT 197

MAVPACKED(
typedef struct __mavlink_boot_t {
 uint32_t version; /*< The onboard software version*/
}) mavlink_boot_t;

#define MAVLINK_MSG_ID_BOOT_LEN 4
#define MAVLINK_MSG_ID_BOOT_MIN_LEN 4
#define MAVLINK_MSG_ID_197_LEN 4
#define MAVLINK_MSG_ID_197_MIN_LEN 4

#define MAVLINK_MSG_ID_BOOT_CRC 39
#define MAVLINK_MSG_ID_197_CRC 39



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BOOT { \
    197, \
    "BOOT", \
    1, \
    {  { "version", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_boot_t, version) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BOOT { \
    "BOOT", \
    1, \
    {  { "version", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_boot_t, version) }, \
         } \
}
#endif

/**
 * @brief Pack a boot message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param version The onboard software version
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_boot_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BOOT_LEN];
    _mav_put_uint32_t(buf, 0, version);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BOOT_LEN);
#else
    mavlink_boot_t packet;
    packet.version = version;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BOOT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BOOT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BOOT_MIN_LEN, MAVLINK_MSG_ID_BOOT_LEN, MAVLINK_MSG_ID_BOOT_CRC);
}

/**
 * @brief Pack a boot message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param version The onboard software version
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_boot_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BOOT_LEN];
    _mav_put_uint32_t(buf, 0, version);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BOOT_LEN);
#else
    mavlink_boot_t packet;
    packet.version = version;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BOOT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BOOT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BOOT_MIN_LEN, MAVLINK_MSG_ID_BOOT_LEN, MAVLINK_MSG_ID_BOOT_CRC);
}

/**
 * @brief Encode a boot struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param boot C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_boot_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_boot_t* boot)
{
    return mavlink_msg_boot_pack(system_id, component_id, msg, boot->version);
}

/**
 * @brief Encode a boot struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param boot C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_boot_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_boot_t* boot)
{
    return mavlink_msg_boot_pack_chan(system_id, component_id, chan, msg, boot->version);
}

/**
 * @brief Send a boot message
 * @param chan MAVLink channel to send the message
 *
 * @param version The onboard software version
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_boot_send(mavlink_channel_t chan, uint32_t version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BOOT_LEN];
    _mav_put_uint32_t(buf, 0, version);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOOT, buf, MAVLINK_MSG_ID_BOOT_MIN_LEN, MAVLINK_MSG_ID_BOOT_LEN, MAVLINK_MSG_ID_BOOT_CRC);
#else
    mavlink_boot_t packet;
    packet.version = version;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOOT, (const char *)&packet, MAVLINK_MSG_ID_BOOT_MIN_LEN, MAVLINK_MSG_ID_BOOT_LEN, MAVLINK_MSG_ID_BOOT_CRC);
#endif
}

/**
 * @brief Send a boot message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_boot_send_struct(mavlink_channel_t chan, const mavlink_boot_t* boot)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_boot_send(chan, boot->version);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOOT, (const char *)boot, MAVLINK_MSG_ID_BOOT_MIN_LEN, MAVLINK_MSG_ID_BOOT_LEN, MAVLINK_MSG_ID_BOOT_CRC);
#endif
}

#if MAVLINK_MSG_ID_BOOT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_boot_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, version);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOOT, buf, MAVLINK_MSG_ID_BOOT_MIN_LEN, MAVLINK_MSG_ID_BOOT_LEN, MAVLINK_MSG_ID_BOOT_CRC);
#else
    mavlink_boot_t *packet = (mavlink_boot_t *)msgbuf;
    packet->version = version;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOOT, (const char *)packet, MAVLINK_MSG_ID_BOOT_MIN_LEN, MAVLINK_MSG_ID_BOOT_LEN, MAVLINK_MSG_ID_BOOT_CRC);
#endif
}
#endif

#endif

// MESSAGE BOOT UNPACKING


/**
 * @brief Get field version from boot message
 *
 * @return The onboard software version
 */
static inline uint32_t mavlink_msg_boot_get_version(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a boot message into a struct
 *
 * @param msg The message to decode
 * @param boot C-struct to decode the message contents into
 */
static inline void mavlink_msg_boot_decode(const mavlink_message_t* msg, mavlink_boot_t* boot)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    boot->version = mavlink_msg_boot_get_version(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BOOT_LEN? msg->len : MAVLINK_MSG_ID_BOOT_LEN;
        memset(boot, 0, MAVLINK_MSG_ID_BOOT_LEN);
    memcpy(boot, _MAV_PAYLOAD(msg), len);
#endif
}
