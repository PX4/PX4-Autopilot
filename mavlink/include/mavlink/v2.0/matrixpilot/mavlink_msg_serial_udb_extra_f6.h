#pragma once
// MESSAGE SERIAL_UDB_EXTRA_F6 PACKING

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6 174

MAVPACKED(
typedef struct __mavlink_serial_udb_extra_f6_t {
 float sue_PITCHGAIN; /*< Serial UDB Extra PITCHGAIN Proportional Control*/
 float sue_PITCHKD; /*< Serial UDB Extra Pitch Rate Control*/
 float sue_RUDDER_ELEV_MIX; /*< Serial UDB Extra Rudder to Elevator Mix*/
 float sue_ROLL_ELEV_MIX; /*< Serial UDB Extra Roll to Elevator Mix*/
 float sue_ELEVATOR_BOOST; /*< Gain For Boosting Manual Elevator control When Plane Stabilized*/
}) mavlink_serial_udb_extra_f6_t;

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_LEN 20
#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_MIN_LEN 20
#define MAVLINK_MSG_ID_174_LEN 20
#define MAVLINK_MSG_ID_174_MIN_LEN 20

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_CRC 54
#define MAVLINK_MSG_ID_174_CRC 54



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F6 { \
    174, \
    "SERIAL_UDB_EXTRA_F6", \
    5, \
    {  { "sue_PITCHGAIN", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_serial_udb_extra_f6_t, sue_PITCHGAIN) }, \
         { "sue_PITCHKD", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_serial_udb_extra_f6_t, sue_PITCHKD) }, \
         { "sue_RUDDER_ELEV_MIX", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_serial_udb_extra_f6_t, sue_RUDDER_ELEV_MIX) }, \
         { "sue_ROLL_ELEV_MIX", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_serial_udb_extra_f6_t, sue_ROLL_ELEV_MIX) }, \
         { "sue_ELEVATOR_BOOST", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_serial_udb_extra_f6_t, sue_ELEVATOR_BOOST) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F6 { \
    "SERIAL_UDB_EXTRA_F6", \
    5, \
    {  { "sue_PITCHGAIN", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_serial_udb_extra_f6_t, sue_PITCHGAIN) }, \
         { "sue_PITCHKD", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_serial_udb_extra_f6_t, sue_PITCHKD) }, \
         { "sue_RUDDER_ELEV_MIX", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_serial_udb_extra_f6_t, sue_RUDDER_ELEV_MIX) }, \
         { "sue_ROLL_ELEV_MIX", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_serial_udb_extra_f6_t, sue_ROLL_ELEV_MIX) }, \
         { "sue_ELEVATOR_BOOST", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_serial_udb_extra_f6_t, sue_ELEVATOR_BOOST) }, \
         } \
}
#endif

/**
 * @brief Pack a serial_udb_extra_f6 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sue_PITCHGAIN Serial UDB Extra PITCHGAIN Proportional Control
 * @param sue_PITCHKD Serial UDB Extra Pitch Rate Control
 * @param sue_RUDDER_ELEV_MIX Serial UDB Extra Rudder to Elevator Mix
 * @param sue_ROLL_ELEV_MIX Serial UDB Extra Roll to Elevator Mix
 * @param sue_ELEVATOR_BOOST Gain For Boosting Manual Elevator control When Plane Stabilized
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f6_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float sue_PITCHGAIN, float sue_PITCHKD, float sue_RUDDER_ELEV_MIX, float sue_ROLL_ELEV_MIX, float sue_ELEVATOR_BOOST)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_LEN];
    _mav_put_float(buf, 0, sue_PITCHGAIN);
    _mav_put_float(buf, 4, sue_PITCHKD);
    _mav_put_float(buf, 8, sue_RUDDER_ELEV_MIX);
    _mav_put_float(buf, 12, sue_ROLL_ELEV_MIX);
    _mav_put_float(buf, 16, sue_ELEVATOR_BOOST);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_LEN);
#else
    mavlink_serial_udb_extra_f6_t packet;
    packet.sue_PITCHGAIN = sue_PITCHGAIN;
    packet.sue_PITCHKD = sue_PITCHKD;
    packet.sue_RUDDER_ELEV_MIX = sue_RUDDER_ELEV_MIX;
    packet.sue_ROLL_ELEV_MIX = sue_ROLL_ELEV_MIX;
    packet.sue_ELEVATOR_BOOST = sue_ELEVATOR_BOOST;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_CRC);
}

/**
 * @brief Pack a serial_udb_extra_f6 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sue_PITCHGAIN Serial UDB Extra PITCHGAIN Proportional Control
 * @param sue_PITCHKD Serial UDB Extra Pitch Rate Control
 * @param sue_RUDDER_ELEV_MIX Serial UDB Extra Rudder to Elevator Mix
 * @param sue_ROLL_ELEV_MIX Serial UDB Extra Roll to Elevator Mix
 * @param sue_ELEVATOR_BOOST Gain For Boosting Manual Elevator control When Plane Stabilized
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f6_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float sue_PITCHGAIN,float sue_PITCHKD,float sue_RUDDER_ELEV_MIX,float sue_ROLL_ELEV_MIX,float sue_ELEVATOR_BOOST)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_LEN];
    _mav_put_float(buf, 0, sue_PITCHGAIN);
    _mav_put_float(buf, 4, sue_PITCHKD);
    _mav_put_float(buf, 8, sue_RUDDER_ELEV_MIX);
    _mav_put_float(buf, 12, sue_ROLL_ELEV_MIX);
    _mav_put_float(buf, 16, sue_ELEVATOR_BOOST);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_LEN);
#else
    mavlink_serial_udb_extra_f6_t packet;
    packet.sue_PITCHGAIN = sue_PITCHGAIN;
    packet.sue_PITCHKD = sue_PITCHKD;
    packet.sue_RUDDER_ELEV_MIX = sue_RUDDER_ELEV_MIX;
    packet.sue_ROLL_ELEV_MIX = sue_ROLL_ELEV_MIX;
    packet.sue_ELEVATOR_BOOST = sue_ELEVATOR_BOOST;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_CRC);
}

/**
 * @brief Encode a serial_udb_extra_f6 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f6 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f6_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_serial_udb_extra_f6_t* serial_udb_extra_f6)
{
    return mavlink_msg_serial_udb_extra_f6_pack(system_id, component_id, msg, serial_udb_extra_f6->sue_PITCHGAIN, serial_udb_extra_f6->sue_PITCHKD, serial_udb_extra_f6->sue_RUDDER_ELEV_MIX, serial_udb_extra_f6->sue_ROLL_ELEV_MIX, serial_udb_extra_f6->sue_ELEVATOR_BOOST);
}

/**
 * @brief Encode a serial_udb_extra_f6 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f6 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f6_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_serial_udb_extra_f6_t* serial_udb_extra_f6)
{
    return mavlink_msg_serial_udb_extra_f6_pack_chan(system_id, component_id, chan, msg, serial_udb_extra_f6->sue_PITCHGAIN, serial_udb_extra_f6->sue_PITCHKD, serial_udb_extra_f6->sue_RUDDER_ELEV_MIX, serial_udb_extra_f6->sue_ROLL_ELEV_MIX, serial_udb_extra_f6->sue_ELEVATOR_BOOST);
}

/**
 * @brief Send a serial_udb_extra_f6 message
 * @param chan MAVLink channel to send the message
 *
 * @param sue_PITCHGAIN Serial UDB Extra PITCHGAIN Proportional Control
 * @param sue_PITCHKD Serial UDB Extra Pitch Rate Control
 * @param sue_RUDDER_ELEV_MIX Serial UDB Extra Rudder to Elevator Mix
 * @param sue_ROLL_ELEV_MIX Serial UDB Extra Roll to Elevator Mix
 * @param sue_ELEVATOR_BOOST Gain For Boosting Manual Elevator control When Plane Stabilized
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_serial_udb_extra_f6_send(mavlink_channel_t chan, float sue_PITCHGAIN, float sue_PITCHKD, float sue_RUDDER_ELEV_MIX, float sue_ROLL_ELEV_MIX, float sue_ELEVATOR_BOOST)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_LEN];
    _mav_put_float(buf, 0, sue_PITCHGAIN);
    _mav_put_float(buf, 4, sue_PITCHKD);
    _mav_put_float(buf, 8, sue_RUDDER_ELEV_MIX);
    _mav_put_float(buf, 12, sue_ROLL_ELEV_MIX);
    _mav_put_float(buf, 16, sue_ELEVATOR_BOOST);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_CRC);
#else
    mavlink_serial_udb_extra_f6_t packet;
    packet.sue_PITCHGAIN = sue_PITCHGAIN;
    packet.sue_PITCHKD = sue_PITCHKD;
    packet.sue_RUDDER_ELEV_MIX = sue_RUDDER_ELEV_MIX;
    packet.sue_ROLL_ELEV_MIX = sue_ROLL_ELEV_MIX;
    packet.sue_ELEVATOR_BOOST = sue_ELEVATOR_BOOST;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_CRC);
#endif
}

/**
 * @brief Send a serial_udb_extra_f6 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_serial_udb_extra_f6_send_struct(mavlink_channel_t chan, const mavlink_serial_udb_extra_f6_t* serial_udb_extra_f6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_serial_udb_extra_f6_send(chan, serial_udb_extra_f6->sue_PITCHGAIN, serial_udb_extra_f6->sue_PITCHKD, serial_udb_extra_f6->sue_RUDDER_ELEV_MIX, serial_udb_extra_f6->sue_ROLL_ELEV_MIX, serial_udb_extra_f6->sue_ELEVATOR_BOOST);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6, (const char *)serial_udb_extra_f6, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_CRC);
#endif
}

#if MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_serial_udb_extra_f6_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float sue_PITCHGAIN, float sue_PITCHKD, float sue_RUDDER_ELEV_MIX, float sue_ROLL_ELEV_MIX, float sue_ELEVATOR_BOOST)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, sue_PITCHGAIN);
    _mav_put_float(buf, 4, sue_PITCHKD);
    _mav_put_float(buf, 8, sue_RUDDER_ELEV_MIX);
    _mav_put_float(buf, 12, sue_ROLL_ELEV_MIX);
    _mav_put_float(buf, 16, sue_ELEVATOR_BOOST);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_CRC);
#else
    mavlink_serial_udb_extra_f6_t *packet = (mavlink_serial_udb_extra_f6_t *)msgbuf;
    packet->sue_PITCHGAIN = sue_PITCHGAIN;
    packet->sue_PITCHKD = sue_PITCHKD;
    packet->sue_RUDDER_ELEV_MIX = sue_RUDDER_ELEV_MIX;
    packet->sue_ROLL_ELEV_MIX = sue_ROLL_ELEV_MIX;
    packet->sue_ELEVATOR_BOOST = sue_ELEVATOR_BOOST;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6, (const char *)packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_CRC);
#endif
}
#endif

#endif

// MESSAGE SERIAL_UDB_EXTRA_F6 UNPACKING


/**
 * @brief Get field sue_PITCHGAIN from serial_udb_extra_f6 message
 *
 * @return Serial UDB Extra PITCHGAIN Proportional Control
 */
static inline float mavlink_msg_serial_udb_extra_f6_get_sue_PITCHGAIN(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field sue_PITCHKD from serial_udb_extra_f6 message
 *
 * @return Serial UDB Extra Pitch Rate Control
 */
static inline float mavlink_msg_serial_udb_extra_f6_get_sue_PITCHKD(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field sue_RUDDER_ELEV_MIX from serial_udb_extra_f6 message
 *
 * @return Serial UDB Extra Rudder to Elevator Mix
 */
static inline float mavlink_msg_serial_udb_extra_f6_get_sue_RUDDER_ELEV_MIX(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field sue_ROLL_ELEV_MIX from serial_udb_extra_f6 message
 *
 * @return Serial UDB Extra Roll to Elevator Mix
 */
static inline float mavlink_msg_serial_udb_extra_f6_get_sue_ROLL_ELEV_MIX(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field sue_ELEVATOR_BOOST from serial_udb_extra_f6 message
 *
 * @return Gain For Boosting Manual Elevator control When Plane Stabilized
 */
static inline float mavlink_msg_serial_udb_extra_f6_get_sue_ELEVATOR_BOOST(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a serial_udb_extra_f6 message into a struct
 *
 * @param msg The message to decode
 * @param serial_udb_extra_f6 C-struct to decode the message contents into
 */
static inline void mavlink_msg_serial_udb_extra_f6_decode(const mavlink_message_t* msg, mavlink_serial_udb_extra_f6_t* serial_udb_extra_f6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    serial_udb_extra_f6->sue_PITCHGAIN = mavlink_msg_serial_udb_extra_f6_get_sue_PITCHGAIN(msg);
    serial_udb_extra_f6->sue_PITCHKD = mavlink_msg_serial_udb_extra_f6_get_sue_PITCHKD(msg);
    serial_udb_extra_f6->sue_RUDDER_ELEV_MIX = mavlink_msg_serial_udb_extra_f6_get_sue_RUDDER_ELEV_MIX(msg);
    serial_udb_extra_f6->sue_ROLL_ELEV_MIX = mavlink_msg_serial_udb_extra_f6_get_sue_ROLL_ELEV_MIX(msg);
    serial_udb_extra_f6->sue_ELEVATOR_BOOST = mavlink_msg_serial_udb_extra_f6_get_sue_ELEVATOR_BOOST(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_LEN? msg->len : MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_LEN;
        memset(serial_udb_extra_f6, 0, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F6_LEN);
    memcpy(serial_udb_extra_f6, _MAV_PAYLOAD(msg), len);
#endif
}
