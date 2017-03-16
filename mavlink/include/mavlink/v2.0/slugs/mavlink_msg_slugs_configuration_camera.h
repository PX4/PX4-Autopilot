#pragma once
// MESSAGE SLUGS_CONFIGURATION_CAMERA PACKING

#define MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA 188

MAVPACKED(
typedef struct __mavlink_slugs_configuration_camera_t {
 uint8_t target; /*< The system setting the commands*/
 uint8_t idOrder; /*< ID 0: brightness 1: aperture 2: iris 3: ICR 4: backlight*/
 uint8_t order; /*<  1: up/on 2: down/off 3: auto/reset/no action*/
}) mavlink_slugs_configuration_camera_t;

#define MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_LEN 3
#define MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_MIN_LEN 3
#define MAVLINK_MSG_ID_188_LEN 3
#define MAVLINK_MSG_ID_188_MIN_LEN 3

#define MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_CRC 5
#define MAVLINK_MSG_ID_188_CRC 5



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SLUGS_CONFIGURATION_CAMERA { \
    188, \
    "SLUGS_CONFIGURATION_CAMERA", \
    3, \
    {  { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_slugs_configuration_camera_t, target) }, \
         { "idOrder", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_slugs_configuration_camera_t, idOrder) }, \
         { "order", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_slugs_configuration_camera_t, order) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SLUGS_CONFIGURATION_CAMERA { \
    "SLUGS_CONFIGURATION_CAMERA", \
    3, \
    {  { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_slugs_configuration_camera_t, target) }, \
         { "idOrder", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_slugs_configuration_camera_t, idOrder) }, \
         { "order", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_slugs_configuration_camera_t, order) }, \
         } \
}
#endif

/**
 * @brief Pack a slugs_configuration_camera message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The system setting the commands
 * @param idOrder ID 0: brightness 1: aperture 2: iris 3: ICR 4: backlight
 * @param order  1: up/on 2: down/off 3: auto/reset/no action
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_slugs_configuration_camera_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target, uint8_t idOrder, uint8_t order)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_LEN];
    _mav_put_uint8_t(buf, 0, target);
    _mav_put_uint8_t(buf, 1, idOrder);
    _mav_put_uint8_t(buf, 2, order);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_LEN);
#else
    mavlink_slugs_configuration_camera_t packet;
    packet.target = target;
    packet.idOrder = idOrder;
    packet.order = order;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_MIN_LEN, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_LEN, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_CRC);
}

/**
 * @brief Pack a slugs_configuration_camera message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system setting the commands
 * @param idOrder ID 0: brightness 1: aperture 2: iris 3: ICR 4: backlight
 * @param order  1: up/on 2: down/off 3: auto/reset/no action
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_slugs_configuration_camera_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target,uint8_t idOrder,uint8_t order)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_LEN];
    _mav_put_uint8_t(buf, 0, target);
    _mav_put_uint8_t(buf, 1, idOrder);
    _mav_put_uint8_t(buf, 2, order);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_LEN);
#else
    mavlink_slugs_configuration_camera_t packet;
    packet.target = target;
    packet.idOrder = idOrder;
    packet.order = order;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_MIN_LEN, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_LEN, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_CRC);
}

/**
 * @brief Encode a slugs_configuration_camera struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param slugs_configuration_camera C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_slugs_configuration_camera_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_slugs_configuration_camera_t* slugs_configuration_camera)
{
    return mavlink_msg_slugs_configuration_camera_pack(system_id, component_id, msg, slugs_configuration_camera->target, slugs_configuration_camera->idOrder, slugs_configuration_camera->order);
}

/**
 * @brief Encode a slugs_configuration_camera struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param slugs_configuration_camera C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_slugs_configuration_camera_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_slugs_configuration_camera_t* slugs_configuration_camera)
{
    return mavlink_msg_slugs_configuration_camera_pack_chan(system_id, component_id, chan, msg, slugs_configuration_camera->target, slugs_configuration_camera->idOrder, slugs_configuration_camera->order);
}

/**
 * @brief Send a slugs_configuration_camera message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system setting the commands
 * @param idOrder ID 0: brightness 1: aperture 2: iris 3: ICR 4: backlight
 * @param order  1: up/on 2: down/off 3: auto/reset/no action
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_slugs_configuration_camera_send(mavlink_channel_t chan, uint8_t target, uint8_t idOrder, uint8_t order)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_LEN];
    _mav_put_uint8_t(buf, 0, target);
    _mav_put_uint8_t(buf, 1, idOrder);
    _mav_put_uint8_t(buf, 2, order);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA, buf, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_MIN_LEN, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_LEN, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_CRC);
#else
    mavlink_slugs_configuration_camera_t packet;
    packet.target = target;
    packet.idOrder = idOrder;
    packet.order = order;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA, (const char *)&packet, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_MIN_LEN, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_LEN, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_CRC);
#endif
}

/**
 * @brief Send a slugs_configuration_camera message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_slugs_configuration_camera_send_struct(mavlink_channel_t chan, const mavlink_slugs_configuration_camera_t* slugs_configuration_camera)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_slugs_configuration_camera_send(chan, slugs_configuration_camera->target, slugs_configuration_camera->idOrder, slugs_configuration_camera->order);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA, (const char *)slugs_configuration_camera, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_MIN_LEN, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_LEN, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_CRC);
#endif
}

#if MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_slugs_configuration_camera_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target, uint8_t idOrder, uint8_t order)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target);
    _mav_put_uint8_t(buf, 1, idOrder);
    _mav_put_uint8_t(buf, 2, order);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA, buf, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_MIN_LEN, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_LEN, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_CRC);
#else
    mavlink_slugs_configuration_camera_t *packet = (mavlink_slugs_configuration_camera_t *)msgbuf;
    packet->target = target;
    packet->idOrder = idOrder;
    packet->order = order;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA, (const char *)packet, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_MIN_LEN, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_LEN, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_CRC);
#endif
}
#endif

#endif

// MESSAGE SLUGS_CONFIGURATION_CAMERA UNPACKING


/**
 * @brief Get field target from slugs_configuration_camera message
 *
 * @return The system setting the commands
 */
static inline uint8_t mavlink_msg_slugs_configuration_camera_get_target(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field idOrder from slugs_configuration_camera message
 *
 * @return ID 0: brightness 1: aperture 2: iris 3: ICR 4: backlight
 */
static inline uint8_t mavlink_msg_slugs_configuration_camera_get_idOrder(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field order from slugs_configuration_camera message
 *
 * @return  1: up/on 2: down/off 3: auto/reset/no action
 */
static inline uint8_t mavlink_msg_slugs_configuration_camera_get_order(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a slugs_configuration_camera message into a struct
 *
 * @param msg The message to decode
 * @param slugs_configuration_camera C-struct to decode the message contents into
 */
static inline void mavlink_msg_slugs_configuration_camera_decode(const mavlink_message_t* msg, mavlink_slugs_configuration_camera_t* slugs_configuration_camera)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    slugs_configuration_camera->target = mavlink_msg_slugs_configuration_camera_get_target(msg);
    slugs_configuration_camera->idOrder = mavlink_msg_slugs_configuration_camera_get_idOrder(msg);
    slugs_configuration_camera->order = mavlink_msg_slugs_configuration_camera_get_order(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_LEN? msg->len : MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_LEN;
        memset(slugs_configuration_camera, 0, MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_LEN);
    memcpy(slugs_configuration_camera, _MAV_PAYLOAD(msg), len);
#endif
}
