#pragma once
// MESSAGE SERIAL_UDB_EXTRA_F18 PACKING

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18 184

MAVPACKED(
typedef struct __mavlink_serial_udb_extra_f18_t {
 float angle_of_attack_normal; /*< SUE Angle of Attack Normal*/
 float angle_of_attack_inverted; /*< SUE Angle of Attack Inverted*/
 float elevator_trim_normal; /*< SUE Elevator Trim Normal*/
 float elevator_trim_inverted; /*< SUE Elevator Trim Inverted*/
 float reference_speed; /*< SUE reference_speed*/
}) mavlink_serial_udb_extra_f18_t;

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_LEN 20
#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_MIN_LEN 20
#define MAVLINK_MSG_ID_184_LEN 20
#define MAVLINK_MSG_ID_184_MIN_LEN 20

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_CRC 41
#define MAVLINK_MSG_ID_184_CRC 41



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F18 { \
    184, \
    "SERIAL_UDB_EXTRA_F18", \
    5, \
    {  { "angle_of_attack_normal", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_serial_udb_extra_f18_t, angle_of_attack_normal) }, \
         { "angle_of_attack_inverted", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_serial_udb_extra_f18_t, angle_of_attack_inverted) }, \
         { "elevator_trim_normal", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_serial_udb_extra_f18_t, elevator_trim_normal) }, \
         { "elevator_trim_inverted", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_serial_udb_extra_f18_t, elevator_trim_inverted) }, \
         { "reference_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_serial_udb_extra_f18_t, reference_speed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F18 { \
    "SERIAL_UDB_EXTRA_F18", \
    5, \
    {  { "angle_of_attack_normal", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_serial_udb_extra_f18_t, angle_of_attack_normal) }, \
         { "angle_of_attack_inverted", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_serial_udb_extra_f18_t, angle_of_attack_inverted) }, \
         { "elevator_trim_normal", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_serial_udb_extra_f18_t, elevator_trim_normal) }, \
         { "elevator_trim_inverted", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_serial_udb_extra_f18_t, elevator_trim_inverted) }, \
         { "reference_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_serial_udb_extra_f18_t, reference_speed) }, \
         } \
}
#endif

/**
 * @brief Pack a serial_udb_extra_f18 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param angle_of_attack_normal SUE Angle of Attack Normal
 * @param angle_of_attack_inverted SUE Angle of Attack Inverted
 * @param elevator_trim_normal SUE Elevator Trim Normal
 * @param elevator_trim_inverted SUE Elevator Trim Inverted
 * @param reference_speed SUE reference_speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f18_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float angle_of_attack_normal, float angle_of_attack_inverted, float elevator_trim_normal, float elevator_trim_inverted, float reference_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_LEN];
    _mav_put_float(buf, 0, angle_of_attack_normal);
    _mav_put_float(buf, 4, angle_of_attack_inverted);
    _mav_put_float(buf, 8, elevator_trim_normal);
    _mav_put_float(buf, 12, elevator_trim_inverted);
    _mav_put_float(buf, 16, reference_speed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_LEN);
#else
    mavlink_serial_udb_extra_f18_t packet;
    packet.angle_of_attack_normal = angle_of_attack_normal;
    packet.angle_of_attack_inverted = angle_of_attack_inverted;
    packet.elevator_trim_normal = elevator_trim_normal;
    packet.elevator_trim_inverted = elevator_trim_inverted;
    packet.reference_speed = reference_speed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_CRC);
}

/**
 * @brief Pack a serial_udb_extra_f18 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param angle_of_attack_normal SUE Angle of Attack Normal
 * @param angle_of_attack_inverted SUE Angle of Attack Inverted
 * @param elevator_trim_normal SUE Elevator Trim Normal
 * @param elevator_trim_inverted SUE Elevator Trim Inverted
 * @param reference_speed SUE reference_speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f18_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float angle_of_attack_normal,float angle_of_attack_inverted,float elevator_trim_normal,float elevator_trim_inverted,float reference_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_LEN];
    _mav_put_float(buf, 0, angle_of_attack_normal);
    _mav_put_float(buf, 4, angle_of_attack_inverted);
    _mav_put_float(buf, 8, elevator_trim_normal);
    _mav_put_float(buf, 12, elevator_trim_inverted);
    _mav_put_float(buf, 16, reference_speed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_LEN);
#else
    mavlink_serial_udb_extra_f18_t packet;
    packet.angle_of_attack_normal = angle_of_attack_normal;
    packet.angle_of_attack_inverted = angle_of_attack_inverted;
    packet.elevator_trim_normal = elevator_trim_normal;
    packet.elevator_trim_inverted = elevator_trim_inverted;
    packet.reference_speed = reference_speed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_CRC);
}

/**
 * @brief Encode a serial_udb_extra_f18 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f18 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f18_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_serial_udb_extra_f18_t* serial_udb_extra_f18)
{
    return mavlink_msg_serial_udb_extra_f18_pack(system_id, component_id, msg, serial_udb_extra_f18->angle_of_attack_normal, serial_udb_extra_f18->angle_of_attack_inverted, serial_udb_extra_f18->elevator_trim_normal, serial_udb_extra_f18->elevator_trim_inverted, serial_udb_extra_f18->reference_speed);
}

/**
 * @brief Encode a serial_udb_extra_f18 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f18 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f18_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_serial_udb_extra_f18_t* serial_udb_extra_f18)
{
    return mavlink_msg_serial_udb_extra_f18_pack_chan(system_id, component_id, chan, msg, serial_udb_extra_f18->angle_of_attack_normal, serial_udb_extra_f18->angle_of_attack_inverted, serial_udb_extra_f18->elevator_trim_normal, serial_udb_extra_f18->elevator_trim_inverted, serial_udb_extra_f18->reference_speed);
}

/**
 * @brief Send a serial_udb_extra_f18 message
 * @param chan MAVLink channel to send the message
 *
 * @param angle_of_attack_normal SUE Angle of Attack Normal
 * @param angle_of_attack_inverted SUE Angle of Attack Inverted
 * @param elevator_trim_normal SUE Elevator Trim Normal
 * @param elevator_trim_inverted SUE Elevator Trim Inverted
 * @param reference_speed SUE reference_speed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_serial_udb_extra_f18_send(mavlink_channel_t chan, float angle_of_attack_normal, float angle_of_attack_inverted, float elevator_trim_normal, float elevator_trim_inverted, float reference_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_LEN];
    _mav_put_float(buf, 0, angle_of_attack_normal);
    _mav_put_float(buf, 4, angle_of_attack_inverted);
    _mav_put_float(buf, 8, elevator_trim_normal);
    _mav_put_float(buf, 12, elevator_trim_inverted);
    _mav_put_float(buf, 16, reference_speed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_CRC);
#else
    mavlink_serial_udb_extra_f18_t packet;
    packet.angle_of_attack_normal = angle_of_attack_normal;
    packet.angle_of_attack_inverted = angle_of_attack_inverted;
    packet.elevator_trim_normal = elevator_trim_normal;
    packet.elevator_trim_inverted = elevator_trim_inverted;
    packet.reference_speed = reference_speed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_CRC);
#endif
}

/**
 * @brief Send a serial_udb_extra_f18 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_serial_udb_extra_f18_send_struct(mavlink_channel_t chan, const mavlink_serial_udb_extra_f18_t* serial_udb_extra_f18)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_serial_udb_extra_f18_send(chan, serial_udb_extra_f18->angle_of_attack_normal, serial_udb_extra_f18->angle_of_attack_inverted, serial_udb_extra_f18->elevator_trim_normal, serial_udb_extra_f18->elevator_trim_inverted, serial_udb_extra_f18->reference_speed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18, (const char *)serial_udb_extra_f18, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_CRC);
#endif
}

#if MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_serial_udb_extra_f18_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float angle_of_attack_normal, float angle_of_attack_inverted, float elevator_trim_normal, float elevator_trim_inverted, float reference_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, angle_of_attack_normal);
    _mav_put_float(buf, 4, angle_of_attack_inverted);
    _mav_put_float(buf, 8, elevator_trim_normal);
    _mav_put_float(buf, 12, elevator_trim_inverted);
    _mav_put_float(buf, 16, reference_speed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_CRC);
#else
    mavlink_serial_udb_extra_f18_t *packet = (mavlink_serial_udb_extra_f18_t *)msgbuf;
    packet->angle_of_attack_normal = angle_of_attack_normal;
    packet->angle_of_attack_inverted = angle_of_attack_inverted;
    packet->elevator_trim_normal = elevator_trim_normal;
    packet->elevator_trim_inverted = elevator_trim_inverted;
    packet->reference_speed = reference_speed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18, (const char *)packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_CRC);
#endif
}
#endif

#endif

// MESSAGE SERIAL_UDB_EXTRA_F18 UNPACKING


/**
 * @brief Get field angle_of_attack_normal from serial_udb_extra_f18 message
 *
 * @return SUE Angle of Attack Normal
 */
static inline float mavlink_msg_serial_udb_extra_f18_get_angle_of_attack_normal(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field angle_of_attack_inverted from serial_udb_extra_f18 message
 *
 * @return SUE Angle of Attack Inverted
 */
static inline float mavlink_msg_serial_udb_extra_f18_get_angle_of_attack_inverted(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field elevator_trim_normal from serial_udb_extra_f18 message
 *
 * @return SUE Elevator Trim Normal
 */
static inline float mavlink_msg_serial_udb_extra_f18_get_elevator_trim_normal(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field elevator_trim_inverted from serial_udb_extra_f18 message
 *
 * @return SUE Elevator Trim Inverted
 */
static inline float mavlink_msg_serial_udb_extra_f18_get_elevator_trim_inverted(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field reference_speed from serial_udb_extra_f18 message
 *
 * @return SUE reference_speed
 */
static inline float mavlink_msg_serial_udb_extra_f18_get_reference_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a serial_udb_extra_f18 message into a struct
 *
 * @param msg The message to decode
 * @param serial_udb_extra_f18 C-struct to decode the message contents into
 */
static inline void mavlink_msg_serial_udb_extra_f18_decode(const mavlink_message_t* msg, mavlink_serial_udb_extra_f18_t* serial_udb_extra_f18)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    serial_udb_extra_f18->angle_of_attack_normal = mavlink_msg_serial_udb_extra_f18_get_angle_of_attack_normal(msg);
    serial_udb_extra_f18->angle_of_attack_inverted = mavlink_msg_serial_udb_extra_f18_get_angle_of_attack_inverted(msg);
    serial_udb_extra_f18->elevator_trim_normal = mavlink_msg_serial_udb_extra_f18_get_elevator_trim_normal(msg);
    serial_udb_extra_f18->elevator_trim_inverted = mavlink_msg_serial_udb_extra_f18_get_elevator_trim_inverted(msg);
    serial_udb_extra_f18->reference_speed = mavlink_msg_serial_udb_extra_f18_get_reference_speed(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_LEN? msg->len : MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_LEN;
        memset(serial_udb_extra_f18, 0, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F18_LEN);
    memcpy(serial_udb_extra_f18, _MAV_PAYLOAD(msg), len);
#endif
}
