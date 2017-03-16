#pragma once
// MESSAGE CTRL_SRFC_PT PACKING

#define MAVLINK_MSG_ID_CTRL_SRFC_PT 181

MAVPACKED(
typedef struct __mavlink_ctrl_srfc_pt_t {
 uint16_t bitfieldPt; /*< Bitfield containing the passthrough configuration, see CONTROL_SURFACE_FLAG ENUM.*/
 uint8_t target; /*< The system setting the commands*/
}) mavlink_ctrl_srfc_pt_t;

#define MAVLINK_MSG_ID_CTRL_SRFC_PT_LEN 3
#define MAVLINK_MSG_ID_CTRL_SRFC_PT_MIN_LEN 3
#define MAVLINK_MSG_ID_181_LEN 3
#define MAVLINK_MSG_ID_181_MIN_LEN 3

#define MAVLINK_MSG_ID_CTRL_SRFC_PT_CRC 104
#define MAVLINK_MSG_ID_181_CRC 104



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CTRL_SRFC_PT { \
    181, \
    "CTRL_SRFC_PT", \
    2, \
    {  { "bitfieldPt", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_ctrl_srfc_pt_t, bitfieldPt) }, \
         { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_ctrl_srfc_pt_t, target) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CTRL_SRFC_PT { \
    "CTRL_SRFC_PT", \
    2, \
    {  { "bitfieldPt", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_ctrl_srfc_pt_t, bitfieldPt) }, \
         { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_ctrl_srfc_pt_t, target) }, \
         } \
}
#endif

/**
 * @brief Pack a ctrl_srfc_pt message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The system setting the commands
 * @param bitfieldPt Bitfield containing the passthrough configuration, see CONTROL_SURFACE_FLAG ENUM.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ctrl_srfc_pt_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target, uint16_t bitfieldPt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CTRL_SRFC_PT_LEN];
    _mav_put_uint16_t(buf, 0, bitfieldPt);
    _mav_put_uint8_t(buf, 2, target);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CTRL_SRFC_PT_LEN);
#else
    mavlink_ctrl_srfc_pt_t packet;
    packet.bitfieldPt = bitfieldPt;
    packet.target = target;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CTRL_SRFC_PT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CTRL_SRFC_PT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CTRL_SRFC_PT_MIN_LEN, MAVLINK_MSG_ID_CTRL_SRFC_PT_LEN, MAVLINK_MSG_ID_CTRL_SRFC_PT_CRC);
}

/**
 * @brief Pack a ctrl_srfc_pt message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system setting the commands
 * @param bitfieldPt Bitfield containing the passthrough configuration, see CONTROL_SURFACE_FLAG ENUM.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ctrl_srfc_pt_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target,uint16_t bitfieldPt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CTRL_SRFC_PT_LEN];
    _mav_put_uint16_t(buf, 0, bitfieldPt);
    _mav_put_uint8_t(buf, 2, target);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CTRL_SRFC_PT_LEN);
#else
    mavlink_ctrl_srfc_pt_t packet;
    packet.bitfieldPt = bitfieldPt;
    packet.target = target;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CTRL_SRFC_PT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CTRL_SRFC_PT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CTRL_SRFC_PT_MIN_LEN, MAVLINK_MSG_ID_CTRL_SRFC_PT_LEN, MAVLINK_MSG_ID_CTRL_SRFC_PT_CRC);
}

/**
 * @brief Encode a ctrl_srfc_pt struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ctrl_srfc_pt C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ctrl_srfc_pt_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ctrl_srfc_pt_t* ctrl_srfc_pt)
{
    return mavlink_msg_ctrl_srfc_pt_pack(system_id, component_id, msg, ctrl_srfc_pt->target, ctrl_srfc_pt->bitfieldPt);
}

/**
 * @brief Encode a ctrl_srfc_pt struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ctrl_srfc_pt C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ctrl_srfc_pt_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ctrl_srfc_pt_t* ctrl_srfc_pt)
{
    return mavlink_msg_ctrl_srfc_pt_pack_chan(system_id, component_id, chan, msg, ctrl_srfc_pt->target, ctrl_srfc_pt->bitfieldPt);
}

/**
 * @brief Send a ctrl_srfc_pt message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system setting the commands
 * @param bitfieldPt Bitfield containing the passthrough configuration, see CONTROL_SURFACE_FLAG ENUM.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ctrl_srfc_pt_send(mavlink_channel_t chan, uint8_t target, uint16_t bitfieldPt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CTRL_SRFC_PT_LEN];
    _mav_put_uint16_t(buf, 0, bitfieldPt);
    _mav_put_uint8_t(buf, 2, target);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CTRL_SRFC_PT, buf, MAVLINK_MSG_ID_CTRL_SRFC_PT_MIN_LEN, MAVLINK_MSG_ID_CTRL_SRFC_PT_LEN, MAVLINK_MSG_ID_CTRL_SRFC_PT_CRC);
#else
    mavlink_ctrl_srfc_pt_t packet;
    packet.bitfieldPt = bitfieldPt;
    packet.target = target;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CTRL_SRFC_PT, (const char *)&packet, MAVLINK_MSG_ID_CTRL_SRFC_PT_MIN_LEN, MAVLINK_MSG_ID_CTRL_SRFC_PT_LEN, MAVLINK_MSG_ID_CTRL_SRFC_PT_CRC);
#endif
}

/**
 * @brief Send a ctrl_srfc_pt message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ctrl_srfc_pt_send_struct(mavlink_channel_t chan, const mavlink_ctrl_srfc_pt_t* ctrl_srfc_pt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ctrl_srfc_pt_send(chan, ctrl_srfc_pt->target, ctrl_srfc_pt->bitfieldPt);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CTRL_SRFC_PT, (const char *)ctrl_srfc_pt, MAVLINK_MSG_ID_CTRL_SRFC_PT_MIN_LEN, MAVLINK_MSG_ID_CTRL_SRFC_PT_LEN, MAVLINK_MSG_ID_CTRL_SRFC_PT_CRC);
#endif
}

#if MAVLINK_MSG_ID_CTRL_SRFC_PT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ctrl_srfc_pt_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target, uint16_t bitfieldPt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, bitfieldPt);
    _mav_put_uint8_t(buf, 2, target);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CTRL_SRFC_PT, buf, MAVLINK_MSG_ID_CTRL_SRFC_PT_MIN_LEN, MAVLINK_MSG_ID_CTRL_SRFC_PT_LEN, MAVLINK_MSG_ID_CTRL_SRFC_PT_CRC);
#else
    mavlink_ctrl_srfc_pt_t *packet = (mavlink_ctrl_srfc_pt_t *)msgbuf;
    packet->bitfieldPt = bitfieldPt;
    packet->target = target;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CTRL_SRFC_PT, (const char *)packet, MAVLINK_MSG_ID_CTRL_SRFC_PT_MIN_LEN, MAVLINK_MSG_ID_CTRL_SRFC_PT_LEN, MAVLINK_MSG_ID_CTRL_SRFC_PT_CRC);
#endif
}
#endif

#endif

// MESSAGE CTRL_SRFC_PT UNPACKING


/**
 * @brief Get field target from ctrl_srfc_pt message
 *
 * @return The system setting the commands
 */
static inline uint8_t mavlink_msg_ctrl_srfc_pt_get_target(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field bitfieldPt from ctrl_srfc_pt message
 *
 * @return Bitfield containing the passthrough configuration, see CONTROL_SURFACE_FLAG ENUM.
 */
static inline uint16_t mavlink_msg_ctrl_srfc_pt_get_bitfieldPt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a ctrl_srfc_pt message into a struct
 *
 * @param msg The message to decode
 * @param ctrl_srfc_pt C-struct to decode the message contents into
 */
static inline void mavlink_msg_ctrl_srfc_pt_decode(const mavlink_message_t* msg, mavlink_ctrl_srfc_pt_t* ctrl_srfc_pt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ctrl_srfc_pt->bitfieldPt = mavlink_msg_ctrl_srfc_pt_get_bitfieldPt(msg);
    ctrl_srfc_pt->target = mavlink_msg_ctrl_srfc_pt_get_target(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CTRL_SRFC_PT_LEN? msg->len : MAVLINK_MSG_ID_CTRL_SRFC_PT_LEN;
        memset(ctrl_srfc_pt, 0, MAVLINK_MSG_ID_CTRL_SRFC_PT_LEN);
    memcpy(ctrl_srfc_pt, _MAV_PAYLOAD(msg), len);
#endif
}
