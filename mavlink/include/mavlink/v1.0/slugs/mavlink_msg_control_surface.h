#pragma once
// MESSAGE CONTROL_SURFACE PACKING

#define MAVLINK_MSG_ID_CONTROL_SURFACE 185

MAVPACKED(
typedef struct __mavlink_control_surface_t {
 float mControl; /*< Pending*/
 float bControl; /*< Order to origin*/
 uint8_t target; /*< The system setting the commands*/
 uint8_t idSurface; /*< ID control surface send 0: throttle 1: aileron 2: elevator 3: rudder*/
}) mavlink_control_surface_t;

#define MAVLINK_MSG_ID_CONTROL_SURFACE_LEN 10
#define MAVLINK_MSG_ID_CONTROL_SURFACE_MIN_LEN 10
#define MAVLINK_MSG_ID_185_LEN 10
#define MAVLINK_MSG_ID_185_MIN_LEN 10

#define MAVLINK_MSG_ID_CONTROL_SURFACE_CRC 113
#define MAVLINK_MSG_ID_185_CRC 113



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CONTROL_SURFACE { \
    185, \
    "CONTROL_SURFACE", \
    4, \
    {  { "mControl", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_control_surface_t, mControl) }, \
         { "bControl", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_control_surface_t, bControl) }, \
         { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_control_surface_t, target) }, \
         { "idSurface", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_control_surface_t, idSurface) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CONTROL_SURFACE { \
    "CONTROL_SURFACE", \
    4, \
    {  { "mControl", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_control_surface_t, mControl) }, \
         { "bControl", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_control_surface_t, bControl) }, \
         { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_control_surface_t, target) }, \
         { "idSurface", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_control_surface_t, idSurface) }, \
         } \
}
#endif

/**
 * @brief Pack a control_surface message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The system setting the commands
 * @param idSurface ID control surface send 0: throttle 1: aileron 2: elevator 3: rudder
 * @param mControl Pending
 * @param bControl Order to origin
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_control_surface_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target, uint8_t idSurface, float mControl, float bControl)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONTROL_SURFACE_LEN];
    _mav_put_float(buf, 0, mControl);
    _mav_put_float(buf, 4, bControl);
    _mav_put_uint8_t(buf, 8, target);
    _mav_put_uint8_t(buf, 9, idSurface);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CONTROL_SURFACE_LEN);
#else
    mavlink_control_surface_t packet;
    packet.mControl = mControl;
    packet.bControl = bControl;
    packet.target = target;
    packet.idSurface = idSurface;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CONTROL_SURFACE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CONTROL_SURFACE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CONTROL_SURFACE_MIN_LEN, MAVLINK_MSG_ID_CONTROL_SURFACE_LEN, MAVLINK_MSG_ID_CONTROL_SURFACE_CRC);
}

/**
 * @brief Pack a control_surface message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system setting the commands
 * @param idSurface ID control surface send 0: throttle 1: aileron 2: elevator 3: rudder
 * @param mControl Pending
 * @param bControl Order to origin
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_control_surface_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target,uint8_t idSurface,float mControl,float bControl)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONTROL_SURFACE_LEN];
    _mav_put_float(buf, 0, mControl);
    _mav_put_float(buf, 4, bControl);
    _mav_put_uint8_t(buf, 8, target);
    _mav_put_uint8_t(buf, 9, idSurface);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CONTROL_SURFACE_LEN);
#else
    mavlink_control_surface_t packet;
    packet.mControl = mControl;
    packet.bControl = bControl;
    packet.target = target;
    packet.idSurface = idSurface;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CONTROL_SURFACE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CONTROL_SURFACE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CONTROL_SURFACE_MIN_LEN, MAVLINK_MSG_ID_CONTROL_SURFACE_LEN, MAVLINK_MSG_ID_CONTROL_SURFACE_CRC);
}

/**
 * @brief Encode a control_surface struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param control_surface C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_control_surface_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_control_surface_t* control_surface)
{
    return mavlink_msg_control_surface_pack(system_id, component_id, msg, control_surface->target, control_surface->idSurface, control_surface->mControl, control_surface->bControl);
}

/**
 * @brief Encode a control_surface struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param control_surface C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_control_surface_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_control_surface_t* control_surface)
{
    return mavlink_msg_control_surface_pack_chan(system_id, component_id, chan, msg, control_surface->target, control_surface->idSurface, control_surface->mControl, control_surface->bControl);
}

/**
 * @brief Send a control_surface message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system setting the commands
 * @param idSurface ID control surface send 0: throttle 1: aileron 2: elevator 3: rudder
 * @param mControl Pending
 * @param bControl Order to origin
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_control_surface_send(mavlink_channel_t chan, uint8_t target, uint8_t idSurface, float mControl, float bControl)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONTROL_SURFACE_LEN];
    _mav_put_float(buf, 0, mControl);
    _mav_put_float(buf, 4, bControl);
    _mav_put_uint8_t(buf, 8, target);
    _mav_put_uint8_t(buf, 9, idSurface);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_SURFACE, buf, MAVLINK_MSG_ID_CONTROL_SURFACE_MIN_LEN, MAVLINK_MSG_ID_CONTROL_SURFACE_LEN, MAVLINK_MSG_ID_CONTROL_SURFACE_CRC);
#else
    mavlink_control_surface_t packet;
    packet.mControl = mControl;
    packet.bControl = bControl;
    packet.target = target;
    packet.idSurface = idSurface;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_SURFACE, (const char *)&packet, MAVLINK_MSG_ID_CONTROL_SURFACE_MIN_LEN, MAVLINK_MSG_ID_CONTROL_SURFACE_LEN, MAVLINK_MSG_ID_CONTROL_SURFACE_CRC);
#endif
}

/**
 * @brief Send a control_surface message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_control_surface_send_struct(mavlink_channel_t chan, const mavlink_control_surface_t* control_surface)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_control_surface_send(chan, control_surface->target, control_surface->idSurface, control_surface->mControl, control_surface->bControl);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_SURFACE, (const char *)control_surface, MAVLINK_MSG_ID_CONTROL_SURFACE_MIN_LEN, MAVLINK_MSG_ID_CONTROL_SURFACE_LEN, MAVLINK_MSG_ID_CONTROL_SURFACE_CRC);
#endif
}

#if MAVLINK_MSG_ID_CONTROL_SURFACE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_control_surface_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target, uint8_t idSurface, float mControl, float bControl)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, mControl);
    _mav_put_float(buf, 4, bControl);
    _mav_put_uint8_t(buf, 8, target);
    _mav_put_uint8_t(buf, 9, idSurface);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_SURFACE, buf, MAVLINK_MSG_ID_CONTROL_SURFACE_MIN_LEN, MAVLINK_MSG_ID_CONTROL_SURFACE_LEN, MAVLINK_MSG_ID_CONTROL_SURFACE_CRC);
#else
    mavlink_control_surface_t *packet = (mavlink_control_surface_t *)msgbuf;
    packet->mControl = mControl;
    packet->bControl = bControl;
    packet->target = target;
    packet->idSurface = idSurface;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_SURFACE, (const char *)packet, MAVLINK_MSG_ID_CONTROL_SURFACE_MIN_LEN, MAVLINK_MSG_ID_CONTROL_SURFACE_LEN, MAVLINK_MSG_ID_CONTROL_SURFACE_CRC);
#endif
}
#endif

#endif

// MESSAGE CONTROL_SURFACE UNPACKING


/**
 * @brief Get field target from control_surface message
 *
 * @return The system setting the commands
 */
static inline uint8_t mavlink_msg_control_surface_get_target(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field idSurface from control_surface message
 *
 * @return ID control surface send 0: throttle 1: aileron 2: elevator 3: rudder
 */
static inline uint8_t mavlink_msg_control_surface_get_idSurface(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field mControl from control_surface message
 *
 * @return Pending
 */
static inline float mavlink_msg_control_surface_get_mControl(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field bControl from control_surface message
 *
 * @return Order to origin
 */
static inline float mavlink_msg_control_surface_get_bControl(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a control_surface message into a struct
 *
 * @param msg The message to decode
 * @param control_surface C-struct to decode the message contents into
 */
static inline void mavlink_msg_control_surface_decode(const mavlink_message_t* msg, mavlink_control_surface_t* control_surface)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    control_surface->mControl = mavlink_msg_control_surface_get_mControl(msg);
    control_surface->bControl = mavlink_msg_control_surface_get_bControl(msg);
    control_surface->target = mavlink_msg_control_surface_get_target(msg);
    control_surface->idSurface = mavlink_msg_control_surface_get_idSurface(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CONTROL_SURFACE_LEN? msg->len : MAVLINK_MSG_ID_CONTROL_SURFACE_LEN;
        memset(control_surface, 0, MAVLINK_MSG_ID_CONTROL_SURFACE_LEN);
    memcpy(control_surface, _MAV_PAYLOAD(msg), len);
#endif
}
