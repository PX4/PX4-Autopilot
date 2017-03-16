#pragma once
// MESSAGE SERIAL_UDB_EXTRA_F5 PACKING

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5 173

MAVPACKED(
typedef struct __mavlink_serial_udb_extra_f5_t {
 float sue_YAWKP_AILERON; /*< Serial UDB YAWKP_AILERON Gain for Proporional control of navigation*/
 float sue_YAWKD_AILERON; /*< Serial UDB YAWKD_AILERON Gain for Rate control of navigation*/
 float sue_ROLLKP; /*< Serial UDB Extra ROLLKP Gain for Proportional control of roll stabilization*/
 float sue_ROLLKD; /*< Serial UDB Extra ROLLKD Gain for Rate control of roll stabilization*/
}) mavlink_serial_udb_extra_f5_t;

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN 16
#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_MIN_LEN 16
#define MAVLINK_MSG_ID_173_LEN 16
#define MAVLINK_MSG_ID_173_MIN_LEN 16

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_CRC 54
#define MAVLINK_MSG_ID_173_CRC 54



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F5 { \
    173, \
    "SERIAL_UDB_EXTRA_F5", \
    4, \
    {  { "sue_YAWKP_AILERON", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_serial_udb_extra_f5_t, sue_YAWKP_AILERON) }, \
         { "sue_YAWKD_AILERON", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_serial_udb_extra_f5_t, sue_YAWKD_AILERON) }, \
         { "sue_ROLLKP", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_serial_udb_extra_f5_t, sue_ROLLKP) }, \
         { "sue_ROLLKD", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_serial_udb_extra_f5_t, sue_ROLLKD) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F5 { \
    "SERIAL_UDB_EXTRA_F5", \
    4, \
    {  { "sue_YAWKP_AILERON", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_serial_udb_extra_f5_t, sue_YAWKP_AILERON) }, \
         { "sue_YAWKD_AILERON", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_serial_udb_extra_f5_t, sue_YAWKD_AILERON) }, \
         { "sue_ROLLKP", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_serial_udb_extra_f5_t, sue_ROLLKP) }, \
         { "sue_ROLLKD", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_serial_udb_extra_f5_t, sue_ROLLKD) }, \
         } \
}
#endif

/**
 * @brief Pack a serial_udb_extra_f5 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sue_YAWKP_AILERON Serial UDB YAWKP_AILERON Gain for Proporional control of navigation
 * @param sue_YAWKD_AILERON Serial UDB YAWKD_AILERON Gain for Rate control of navigation
 * @param sue_ROLLKP Serial UDB Extra ROLLKP Gain for Proportional control of roll stabilization
 * @param sue_ROLLKD Serial UDB Extra ROLLKD Gain for Rate control of roll stabilization
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f5_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float sue_YAWKP_AILERON, float sue_YAWKD_AILERON, float sue_ROLLKP, float sue_ROLLKD)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN];
    _mav_put_float(buf, 0, sue_YAWKP_AILERON);
    _mav_put_float(buf, 4, sue_YAWKD_AILERON);
    _mav_put_float(buf, 8, sue_ROLLKP);
    _mav_put_float(buf, 12, sue_ROLLKD);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN);
#else
    mavlink_serial_udb_extra_f5_t packet;
    packet.sue_YAWKP_AILERON = sue_YAWKP_AILERON;
    packet.sue_YAWKD_AILERON = sue_YAWKD_AILERON;
    packet.sue_ROLLKP = sue_ROLLKP;
    packet.sue_ROLLKD = sue_ROLLKD;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_CRC);
}

/**
 * @brief Pack a serial_udb_extra_f5 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sue_YAWKP_AILERON Serial UDB YAWKP_AILERON Gain for Proporional control of navigation
 * @param sue_YAWKD_AILERON Serial UDB YAWKD_AILERON Gain for Rate control of navigation
 * @param sue_ROLLKP Serial UDB Extra ROLLKP Gain for Proportional control of roll stabilization
 * @param sue_ROLLKD Serial UDB Extra ROLLKD Gain for Rate control of roll stabilization
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f5_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float sue_YAWKP_AILERON,float sue_YAWKD_AILERON,float sue_ROLLKP,float sue_ROLLKD)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN];
    _mav_put_float(buf, 0, sue_YAWKP_AILERON);
    _mav_put_float(buf, 4, sue_YAWKD_AILERON);
    _mav_put_float(buf, 8, sue_ROLLKP);
    _mav_put_float(buf, 12, sue_ROLLKD);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN);
#else
    mavlink_serial_udb_extra_f5_t packet;
    packet.sue_YAWKP_AILERON = sue_YAWKP_AILERON;
    packet.sue_YAWKD_AILERON = sue_YAWKD_AILERON;
    packet.sue_ROLLKP = sue_ROLLKP;
    packet.sue_ROLLKD = sue_ROLLKD;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_CRC);
}

/**
 * @brief Encode a serial_udb_extra_f5 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f5 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f5_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_serial_udb_extra_f5_t* serial_udb_extra_f5)
{
    return mavlink_msg_serial_udb_extra_f5_pack(system_id, component_id, msg, serial_udb_extra_f5->sue_YAWKP_AILERON, serial_udb_extra_f5->sue_YAWKD_AILERON, serial_udb_extra_f5->sue_ROLLKP, serial_udb_extra_f5->sue_ROLLKD);
}

/**
 * @brief Encode a serial_udb_extra_f5 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f5 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f5_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_serial_udb_extra_f5_t* serial_udb_extra_f5)
{
    return mavlink_msg_serial_udb_extra_f5_pack_chan(system_id, component_id, chan, msg, serial_udb_extra_f5->sue_YAWKP_AILERON, serial_udb_extra_f5->sue_YAWKD_AILERON, serial_udb_extra_f5->sue_ROLLKP, serial_udb_extra_f5->sue_ROLLKD);
}

/**
 * @brief Send a serial_udb_extra_f5 message
 * @param chan MAVLink channel to send the message
 *
 * @param sue_YAWKP_AILERON Serial UDB YAWKP_AILERON Gain for Proporional control of navigation
 * @param sue_YAWKD_AILERON Serial UDB YAWKD_AILERON Gain for Rate control of navigation
 * @param sue_ROLLKP Serial UDB Extra ROLLKP Gain for Proportional control of roll stabilization
 * @param sue_ROLLKD Serial UDB Extra ROLLKD Gain for Rate control of roll stabilization
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_serial_udb_extra_f5_send(mavlink_channel_t chan, float sue_YAWKP_AILERON, float sue_YAWKD_AILERON, float sue_ROLLKP, float sue_ROLLKD)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN];
    _mav_put_float(buf, 0, sue_YAWKP_AILERON);
    _mav_put_float(buf, 4, sue_YAWKD_AILERON);
    _mav_put_float(buf, 8, sue_ROLLKP);
    _mav_put_float(buf, 12, sue_ROLLKD);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_CRC);
#else
    mavlink_serial_udb_extra_f5_t packet;
    packet.sue_YAWKP_AILERON = sue_YAWKP_AILERON;
    packet.sue_YAWKD_AILERON = sue_YAWKD_AILERON;
    packet.sue_ROLLKP = sue_ROLLKP;
    packet.sue_ROLLKD = sue_ROLLKD;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_CRC);
#endif
}

/**
 * @brief Send a serial_udb_extra_f5 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_serial_udb_extra_f5_send_struct(mavlink_channel_t chan, const mavlink_serial_udb_extra_f5_t* serial_udb_extra_f5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_serial_udb_extra_f5_send(chan, serial_udb_extra_f5->sue_YAWKP_AILERON, serial_udb_extra_f5->sue_YAWKD_AILERON, serial_udb_extra_f5->sue_ROLLKP, serial_udb_extra_f5->sue_ROLLKD);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5, (const char *)serial_udb_extra_f5, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_CRC);
#endif
}

#if MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_serial_udb_extra_f5_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float sue_YAWKP_AILERON, float sue_YAWKD_AILERON, float sue_ROLLKP, float sue_ROLLKD)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, sue_YAWKP_AILERON);
    _mav_put_float(buf, 4, sue_YAWKD_AILERON);
    _mav_put_float(buf, 8, sue_ROLLKP);
    _mav_put_float(buf, 12, sue_ROLLKD);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_CRC);
#else
    mavlink_serial_udb_extra_f5_t *packet = (mavlink_serial_udb_extra_f5_t *)msgbuf;
    packet->sue_YAWKP_AILERON = sue_YAWKP_AILERON;
    packet->sue_YAWKD_AILERON = sue_YAWKD_AILERON;
    packet->sue_ROLLKP = sue_ROLLKP;
    packet->sue_ROLLKD = sue_ROLLKD;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5, (const char *)packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_CRC);
#endif
}
#endif

#endif

// MESSAGE SERIAL_UDB_EXTRA_F5 UNPACKING


/**
 * @brief Get field sue_YAWKP_AILERON from serial_udb_extra_f5 message
 *
 * @return Serial UDB YAWKP_AILERON Gain for Proporional control of navigation
 */
static inline float mavlink_msg_serial_udb_extra_f5_get_sue_YAWKP_AILERON(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field sue_YAWKD_AILERON from serial_udb_extra_f5 message
 *
 * @return Serial UDB YAWKD_AILERON Gain for Rate control of navigation
 */
static inline float mavlink_msg_serial_udb_extra_f5_get_sue_YAWKD_AILERON(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field sue_ROLLKP from serial_udb_extra_f5 message
 *
 * @return Serial UDB Extra ROLLKP Gain for Proportional control of roll stabilization
 */
static inline float mavlink_msg_serial_udb_extra_f5_get_sue_ROLLKP(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field sue_ROLLKD from serial_udb_extra_f5 message
 *
 * @return Serial UDB Extra ROLLKD Gain for Rate control of roll stabilization
 */
static inline float mavlink_msg_serial_udb_extra_f5_get_sue_ROLLKD(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a serial_udb_extra_f5 message into a struct
 *
 * @param msg The message to decode
 * @param serial_udb_extra_f5 C-struct to decode the message contents into
 */
static inline void mavlink_msg_serial_udb_extra_f5_decode(const mavlink_message_t* msg, mavlink_serial_udb_extra_f5_t* serial_udb_extra_f5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    serial_udb_extra_f5->sue_YAWKP_AILERON = mavlink_msg_serial_udb_extra_f5_get_sue_YAWKP_AILERON(msg);
    serial_udb_extra_f5->sue_YAWKD_AILERON = mavlink_msg_serial_udb_extra_f5_get_sue_YAWKD_AILERON(msg);
    serial_udb_extra_f5->sue_ROLLKP = mavlink_msg_serial_udb_extra_f5_get_sue_ROLLKP(msg);
    serial_udb_extra_f5->sue_ROLLKD = mavlink_msg_serial_udb_extra_f5_get_sue_ROLLKD(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN? msg->len : MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN;
        memset(serial_udb_extra_f5, 0, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN);
    memcpy(serial_udb_extra_f5, _MAV_PAYLOAD(msg), len);
#endif
}
