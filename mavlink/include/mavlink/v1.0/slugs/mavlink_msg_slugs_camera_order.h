#pragma once
// MESSAGE SLUGS_CAMERA_ORDER PACKING

#define MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER 184

MAVPACKED(
typedef struct __mavlink_slugs_camera_order_t {
 uint8_t target; /*< The system reporting the action*/
 int8_t pan; /*< Order the mount to pan: -1 left, 0 No pan motion, +1 right*/
 int8_t tilt; /*< Order the mount to tilt: -1 down, 0 No tilt motion, +1 up*/
 int8_t zoom; /*< Order the zoom values 0 to 10*/
 int8_t moveHome; /*< Orders the camera mount to move home. The other fields are ignored when this field is set. 1: move home, 0 ignored*/
}) mavlink_slugs_camera_order_t;

#define MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_LEN 5
#define MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_MIN_LEN 5
#define MAVLINK_MSG_ID_184_LEN 5
#define MAVLINK_MSG_ID_184_MIN_LEN 5

#define MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_CRC 45
#define MAVLINK_MSG_ID_184_CRC 45



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SLUGS_CAMERA_ORDER { \
    184, \
    "SLUGS_CAMERA_ORDER", \
    5, \
    {  { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_slugs_camera_order_t, target) }, \
         { "pan", NULL, MAVLINK_TYPE_INT8_T, 0, 1, offsetof(mavlink_slugs_camera_order_t, pan) }, \
         { "tilt", NULL, MAVLINK_TYPE_INT8_T, 0, 2, offsetof(mavlink_slugs_camera_order_t, tilt) }, \
         { "zoom", NULL, MAVLINK_TYPE_INT8_T, 0, 3, offsetof(mavlink_slugs_camera_order_t, zoom) }, \
         { "moveHome", NULL, MAVLINK_TYPE_INT8_T, 0, 4, offsetof(mavlink_slugs_camera_order_t, moveHome) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SLUGS_CAMERA_ORDER { \
    "SLUGS_CAMERA_ORDER", \
    5, \
    {  { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_slugs_camera_order_t, target) }, \
         { "pan", NULL, MAVLINK_TYPE_INT8_T, 0, 1, offsetof(mavlink_slugs_camera_order_t, pan) }, \
         { "tilt", NULL, MAVLINK_TYPE_INT8_T, 0, 2, offsetof(mavlink_slugs_camera_order_t, tilt) }, \
         { "zoom", NULL, MAVLINK_TYPE_INT8_T, 0, 3, offsetof(mavlink_slugs_camera_order_t, zoom) }, \
         { "moveHome", NULL, MAVLINK_TYPE_INT8_T, 0, 4, offsetof(mavlink_slugs_camera_order_t, moveHome) }, \
         } \
}
#endif

/**
 * @brief Pack a slugs_camera_order message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The system reporting the action
 * @param pan Order the mount to pan: -1 left, 0 No pan motion, +1 right
 * @param tilt Order the mount to tilt: -1 down, 0 No tilt motion, +1 up
 * @param zoom Order the zoom values 0 to 10
 * @param moveHome Orders the camera mount to move home. The other fields are ignored when this field is set. 1: move home, 0 ignored
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_slugs_camera_order_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target, int8_t pan, int8_t tilt, int8_t zoom, int8_t moveHome)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_LEN];
    _mav_put_uint8_t(buf, 0, target);
    _mav_put_int8_t(buf, 1, pan);
    _mav_put_int8_t(buf, 2, tilt);
    _mav_put_int8_t(buf, 3, zoom);
    _mav_put_int8_t(buf, 4, moveHome);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_LEN);
#else
    mavlink_slugs_camera_order_t packet;
    packet.target = target;
    packet.pan = pan;
    packet.tilt = tilt;
    packet.zoom = zoom;
    packet.moveHome = moveHome;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_MIN_LEN, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_LEN, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_CRC);
}

/**
 * @brief Pack a slugs_camera_order message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system reporting the action
 * @param pan Order the mount to pan: -1 left, 0 No pan motion, +1 right
 * @param tilt Order the mount to tilt: -1 down, 0 No tilt motion, +1 up
 * @param zoom Order the zoom values 0 to 10
 * @param moveHome Orders the camera mount to move home. The other fields are ignored when this field is set. 1: move home, 0 ignored
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_slugs_camera_order_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target,int8_t pan,int8_t tilt,int8_t zoom,int8_t moveHome)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_LEN];
    _mav_put_uint8_t(buf, 0, target);
    _mav_put_int8_t(buf, 1, pan);
    _mav_put_int8_t(buf, 2, tilt);
    _mav_put_int8_t(buf, 3, zoom);
    _mav_put_int8_t(buf, 4, moveHome);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_LEN);
#else
    mavlink_slugs_camera_order_t packet;
    packet.target = target;
    packet.pan = pan;
    packet.tilt = tilt;
    packet.zoom = zoom;
    packet.moveHome = moveHome;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_MIN_LEN, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_LEN, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_CRC);
}

/**
 * @brief Encode a slugs_camera_order struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param slugs_camera_order C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_slugs_camera_order_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_slugs_camera_order_t* slugs_camera_order)
{
    return mavlink_msg_slugs_camera_order_pack(system_id, component_id, msg, slugs_camera_order->target, slugs_camera_order->pan, slugs_camera_order->tilt, slugs_camera_order->zoom, slugs_camera_order->moveHome);
}

/**
 * @brief Encode a slugs_camera_order struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param slugs_camera_order C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_slugs_camera_order_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_slugs_camera_order_t* slugs_camera_order)
{
    return mavlink_msg_slugs_camera_order_pack_chan(system_id, component_id, chan, msg, slugs_camera_order->target, slugs_camera_order->pan, slugs_camera_order->tilt, slugs_camera_order->zoom, slugs_camera_order->moveHome);
}

/**
 * @brief Send a slugs_camera_order message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system reporting the action
 * @param pan Order the mount to pan: -1 left, 0 No pan motion, +1 right
 * @param tilt Order the mount to tilt: -1 down, 0 No tilt motion, +1 up
 * @param zoom Order the zoom values 0 to 10
 * @param moveHome Orders the camera mount to move home. The other fields are ignored when this field is set. 1: move home, 0 ignored
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_slugs_camera_order_send(mavlink_channel_t chan, uint8_t target, int8_t pan, int8_t tilt, int8_t zoom, int8_t moveHome)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_LEN];
    _mav_put_uint8_t(buf, 0, target);
    _mav_put_int8_t(buf, 1, pan);
    _mav_put_int8_t(buf, 2, tilt);
    _mav_put_int8_t(buf, 3, zoom);
    _mav_put_int8_t(buf, 4, moveHome);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER, buf, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_MIN_LEN, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_LEN, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_CRC);
#else
    mavlink_slugs_camera_order_t packet;
    packet.target = target;
    packet.pan = pan;
    packet.tilt = tilt;
    packet.zoom = zoom;
    packet.moveHome = moveHome;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER, (const char *)&packet, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_MIN_LEN, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_LEN, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_CRC);
#endif
}

/**
 * @brief Send a slugs_camera_order message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_slugs_camera_order_send_struct(mavlink_channel_t chan, const mavlink_slugs_camera_order_t* slugs_camera_order)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_slugs_camera_order_send(chan, slugs_camera_order->target, slugs_camera_order->pan, slugs_camera_order->tilt, slugs_camera_order->zoom, slugs_camera_order->moveHome);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER, (const char *)slugs_camera_order, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_MIN_LEN, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_LEN, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_CRC);
#endif
}

#if MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_slugs_camera_order_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target, int8_t pan, int8_t tilt, int8_t zoom, int8_t moveHome)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target);
    _mav_put_int8_t(buf, 1, pan);
    _mav_put_int8_t(buf, 2, tilt);
    _mav_put_int8_t(buf, 3, zoom);
    _mav_put_int8_t(buf, 4, moveHome);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER, buf, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_MIN_LEN, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_LEN, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_CRC);
#else
    mavlink_slugs_camera_order_t *packet = (mavlink_slugs_camera_order_t *)msgbuf;
    packet->target = target;
    packet->pan = pan;
    packet->tilt = tilt;
    packet->zoom = zoom;
    packet->moveHome = moveHome;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER, (const char *)packet, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_MIN_LEN, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_LEN, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_CRC);
#endif
}
#endif

#endif

// MESSAGE SLUGS_CAMERA_ORDER UNPACKING


/**
 * @brief Get field target from slugs_camera_order message
 *
 * @return The system reporting the action
 */
static inline uint8_t mavlink_msg_slugs_camera_order_get_target(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field pan from slugs_camera_order message
 *
 * @return Order the mount to pan: -1 left, 0 No pan motion, +1 right
 */
static inline int8_t mavlink_msg_slugs_camera_order_get_pan(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  1);
}

/**
 * @brief Get field tilt from slugs_camera_order message
 *
 * @return Order the mount to tilt: -1 down, 0 No tilt motion, +1 up
 */
static inline int8_t mavlink_msg_slugs_camera_order_get_tilt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  2);
}

/**
 * @brief Get field zoom from slugs_camera_order message
 *
 * @return Order the zoom values 0 to 10
 */
static inline int8_t mavlink_msg_slugs_camera_order_get_zoom(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  3);
}

/**
 * @brief Get field moveHome from slugs_camera_order message
 *
 * @return Orders the camera mount to move home. The other fields are ignored when this field is set. 1: move home, 0 ignored
 */
static inline int8_t mavlink_msg_slugs_camera_order_get_moveHome(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  4);
}

/**
 * @brief Decode a slugs_camera_order message into a struct
 *
 * @param msg The message to decode
 * @param slugs_camera_order C-struct to decode the message contents into
 */
static inline void mavlink_msg_slugs_camera_order_decode(const mavlink_message_t* msg, mavlink_slugs_camera_order_t* slugs_camera_order)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    slugs_camera_order->target = mavlink_msg_slugs_camera_order_get_target(msg);
    slugs_camera_order->pan = mavlink_msg_slugs_camera_order_get_pan(msg);
    slugs_camera_order->tilt = mavlink_msg_slugs_camera_order_get_tilt(msg);
    slugs_camera_order->zoom = mavlink_msg_slugs_camera_order_get_zoom(msg);
    slugs_camera_order->moveHome = mavlink_msg_slugs_camera_order_get_moveHome(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_LEN? msg->len : MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_LEN;
        memset(slugs_camera_order, 0, MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_LEN);
    memcpy(slugs_camera_order, _MAV_PAYLOAD(msg), len);
#endif
}
