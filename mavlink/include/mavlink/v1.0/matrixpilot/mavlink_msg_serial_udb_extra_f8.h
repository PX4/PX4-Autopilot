// MESSAGE SERIAL_UDB_EXTRA_F8 PACKING

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8 176

typedef struct __mavlink_serial_udb_extra_f8_t
{
 float sue_HEIGHT_TARGET_MAX; ///< Serial UDB Extra HEIGHT_TARGET_MAX
 float sue_HEIGHT_TARGET_MIN; ///< Serial UDB Extra HEIGHT_TARGET_MIN
 float sue_ALT_HOLD_THROTTLE_MIN; ///< Serial UDB Extra ALT_HOLD_THROTTLE_MIN
 float sue_ALT_HOLD_THROTTLE_MAX; ///< Serial UDB Extra ALT_HOLD_THROTTLE_MAX
 float sue_ALT_HOLD_PITCH_MIN; ///< Serial UDB Extra ALT_HOLD_PITCH_MIN
 float sue_ALT_HOLD_PITCH_MAX; ///< Serial UDB Extra ALT_HOLD_PITCH_MAX
 float sue_ALT_HOLD_PITCH_HIGH; ///< Serial UDB Extra ALT_HOLD_PITCH_HIGH
} mavlink_serial_udb_extra_f8_t;

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_LEN 28
#define MAVLINK_MSG_ID_176_LEN 28

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_CRC 142
#define MAVLINK_MSG_ID_176_CRC 142



#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F8 { \
	"SERIAL_UDB_EXTRA_F8", \
	7, \
	{  { "sue_HEIGHT_TARGET_MAX", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_serial_udb_extra_f8_t, sue_HEIGHT_TARGET_MAX) }, \
         { "sue_HEIGHT_TARGET_MIN", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_serial_udb_extra_f8_t, sue_HEIGHT_TARGET_MIN) }, \
         { "sue_ALT_HOLD_THROTTLE_MIN", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_serial_udb_extra_f8_t, sue_ALT_HOLD_THROTTLE_MIN) }, \
         { "sue_ALT_HOLD_THROTTLE_MAX", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_serial_udb_extra_f8_t, sue_ALT_HOLD_THROTTLE_MAX) }, \
         { "sue_ALT_HOLD_PITCH_MIN", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_serial_udb_extra_f8_t, sue_ALT_HOLD_PITCH_MIN) }, \
         { "sue_ALT_HOLD_PITCH_MAX", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_serial_udb_extra_f8_t, sue_ALT_HOLD_PITCH_MAX) }, \
         { "sue_ALT_HOLD_PITCH_HIGH", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_serial_udb_extra_f8_t, sue_ALT_HOLD_PITCH_HIGH) }, \
         } \
}


/**
 * @brief Pack a serial_udb_extra_f8 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sue_HEIGHT_TARGET_MAX Serial UDB Extra HEIGHT_TARGET_MAX
 * @param sue_HEIGHT_TARGET_MIN Serial UDB Extra HEIGHT_TARGET_MIN
 * @param sue_ALT_HOLD_THROTTLE_MIN Serial UDB Extra ALT_HOLD_THROTTLE_MIN
 * @param sue_ALT_HOLD_THROTTLE_MAX Serial UDB Extra ALT_HOLD_THROTTLE_MAX
 * @param sue_ALT_HOLD_PITCH_MIN Serial UDB Extra ALT_HOLD_PITCH_MIN
 * @param sue_ALT_HOLD_PITCH_MAX Serial UDB Extra ALT_HOLD_PITCH_MAX
 * @param sue_ALT_HOLD_PITCH_HIGH Serial UDB Extra ALT_HOLD_PITCH_HIGH
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f8_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float sue_HEIGHT_TARGET_MAX, float sue_HEIGHT_TARGET_MIN, float sue_ALT_HOLD_THROTTLE_MIN, float sue_ALT_HOLD_THROTTLE_MAX, float sue_ALT_HOLD_PITCH_MIN, float sue_ALT_HOLD_PITCH_MAX, float sue_ALT_HOLD_PITCH_HIGH)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_LEN];
	_mav_put_float(buf, 0, sue_HEIGHT_TARGET_MAX);
	_mav_put_float(buf, 4, sue_HEIGHT_TARGET_MIN);
	_mav_put_float(buf, 8, sue_ALT_HOLD_THROTTLE_MIN);
	_mav_put_float(buf, 12, sue_ALT_HOLD_THROTTLE_MAX);
	_mav_put_float(buf, 16, sue_ALT_HOLD_PITCH_MIN);
	_mav_put_float(buf, 20, sue_ALT_HOLD_PITCH_MAX);
	_mav_put_float(buf, 24, sue_ALT_HOLD_PITCH_HIGH);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_LEN);
#else
	mavlink_serial_udb_extra_f8_t packet;
	packet.sue_HEIGHT_TARGET_MAX = sue_HEIGHT_TARGET_MAX;
	packet.sue_HEIGHT_TARGET_MIN = sue_HEIGHT_TARGET_MIN;
	packet.sue_ALT_HOLD_THROTTLE_MIN = sue_ALT_HOLD_THROTTLE_MIN;
	packet.sue_ALT_HOLD_THROTTLE_MAX = sue_ALT_HOLD_THROTTLE_MAX;
	packet.sue_ALT_HOLD_PITCH_MIN = sue_ALT_HOLD_PITCH_MIN;
	packet.sue_ALT_HOLD_PITCH_MAX = sue_ALT_HOLD_PITCH_MAX;
	packet.sue_ALT_HOLD_PITCH_HIGH = sue_ALT_HOLD_PITCH_HIGH;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_LEN);
#endif
}

/**
 * @brief Pack a serial_udb_extra_f8 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sue_HEIGHT_TARGET_MAX Serial UDB Extra HEIGHT_TARGET_MAX
 * @param sue_HEIGHT_TARGET_MIN Serial UDB Extra HEIGHT_TARGET_MIN
 * @param sue_ALT_HOLD_THROTTLE_MIN Serial UDB Extra ALT_HOLD_THROTTLE_MIN
 * @param sue_ALT_HOLD_THROTTLE_MAX Serial UDB Extra ALT_HOLD_THROTTLE_MAX
 * @param sue_ALT_HOLD_PITCH_MIN Serial UDB Extra ALT_HOLD_PITCH_MIN
 * @param sue_ALT_HOLD_PITCH_MAX Serial UDB Extra ALT_HOLD_PITCH_MAX
 * @param sue_ALT_HOLD_PITCH_HIGH Serial UDB Extra ALT_HOLD_PITCH_HIGH
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f8_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float sue_HEIGHT_TARGET_MAX,float sue_HEIGHT_TARGET_MIN,float sue_ALT_HOLD_THROTTLE_MIN,float sue_ALT_HOLD_THROTTLE_MAX,float sue_ALT_HOLD_PITCH_MIN,float sue_ALT_HOLD_PITCH_MAX,float sue_ALT_HOLD_PITCH_HIGH)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_LEN];
	_mav_put_float(buf, 0, sue_HEIGHT_TARGET_MAX);
	_mav_put_float(buf, 4, sue_HEIGHT_TARGET_MIN);
	_mav_put_float(buf, 8, sue_ALT_HOLD_THROTTLE_MIN);
	_mav_put_float(buf, 12, sue_ALT_HOLD_THROTTLE_MAX);
	_mav_put_float(buf, 16, sue_ALT_HOLD_PITCH_MIN);
	_mav_put_float(buf, 20, sue_ALT_HOLD_PITCH_MAX);
	_mav_put_float(buf, 24, sue_ALT_HOLD_PITCH_HIGH);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_LEN);
#else
	mavlink_serial_udb_extra_f8_t packet;
	packet.sue_HEIGHT_TARGET_MAX = sue_HEIGHT_TARGET_MAX;
	packet.sue_HEIGHT_TARGET_MIN = sue_HEIGHT_TARGET_MIN;
	packet.sue_ALT_HOLD_THROTTLE_MIN = sue_ALT_HOLD_THROTTLE_MIN;
	packet.sue_ALT_HOLD_THROTTLE_MAX = sue_ALT_HOLD_THROTTLE_MAX;
	packet.sue_ALT_HOLD_PITCH_MIN = sue_ALT_HOLD_PITCH_MIN;
	packet.sue_ALT_HOLD_PITCH_MAX = sue_ALT_HOLD_PITCH_MAX;
	packet.sue_ALT_HOLD_PITCH_HIGH = sue_ALT_HOLD_PITCH_HIGH;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_LEN);
#endif
}

/**
 * @brief Encode a serial_udb_extra_f8 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f8 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f8_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_serial_udb_extra_f8_t* serial_udb_extra_f8)
{
	return mavlink_msg_serial_udb_extra_f8_pack(system_id, component_id, msg, serial_udb_extra_f8->sue_HEIGHT_TARGET_MAX, serial_udb_extra_f8->sue_HEIGHT_TARGET_MIN, serial_udb_extra_f8->sue_ALT_HOLD_THROTTLE_MIN, serial_udb_extra_f8->sue_ALT_HOLD_THROTTLE_MAX, serial_udb_extra_f8->sue_ALT_HOLD_PITCH_MIN, serial_udb_extra_f8->sue_ALT_HOLD_PITCH_MAX, serial_udb_extra_f8->sue_ALT_HOLD_PITCH_HIGH);
}

/**
 * @brief Encode a serial_udb_extra_f8 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f8 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f8_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_serial_udb_extra_f8_t* serial_udb_extra_f8)
{
	return mavlink_msg_serial_udb_extra_f8_pack_chan(system_id, component_id, chan, msg, serial_udb_extra_f8->sue_HEIGHT_TARGET_MAX, serial_udb_extra_f8->sue_HEIGHT_TARGET_MIN, serial_udb_extra_f8->sue_ALT_HOLD_THROTTLE_MIN, serial_udb_extra_f8->sue_ALT_HOLD_THROTTLE_MAX, serial_udb_extra_f8->sue_ALT_HOLD_PITCH_MIN, serial_udb_extra_f8->sue_ALT_HOLD_PITCH_MAX, serial_udb_extra_f8->sue_ALT_HOLD_PITCH_HIGH);
}

/**
 * @brief Send a serial_udb_extra_f8 message
 * @param chan MAVLink channel to send the message
 *
 * @param sue_HEIGHT_TARGET_MAX Serial UDB Extra HEIGHT_TARGET_MAX
 * @param sue_HEIGHT_TARGET_MIN Serial UDB Extra HEIGHT_TARGET_MIN
 * @param sue_ALT_HOLD_THROTTLE_MIN Serial UDB Extra ALT_HOLD_THROTTLE_MIN
 * @param sue_ALT_HOLD_THROTTLE_MAX Serial UDB Extra ALT_HOLD_THROTTLE_MAX
 * @param sue_ALT_HOLD_PITCH_MIN Serial UDB Extra ALT_HOLD_PITCH_MIN
 * @param sue_ALT_HOLD_PITCH_MAX Serial UDB Extra ALT_HOLD_PITCH_MAX
 * @param sue_ALT_HOLD_PITCH_HIGH Serial UDB Extra ALT_HOLD_PITCH_HIGH
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_serial_udb_extra_f8_send(mavlink_channel_t chan, float sue_HEIGHT_TARGET_MAX, float sue_HEIGHT_TARGET_MIN, float sue_ALT_HOLD_THROTTLE_MIN, float sue_ALT_HOLD_THROTTLE_MAX, float sue_ALT_HOLD_PITCH_MIN, float sue_ALT_HOLD_PITCH_MAX, float sue_ALT_HOLD_PITCH_HIGH)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_LEN];
	_mav_put_float(buf, 0, sue_HEIGHT_TARGET_MAX);
	_mav_put_float(buf, 4, sue_HEIGHT_TARGET_MIN);
	_mav_put_float(buf, 8, sue_ALT_HOLD_THROTTLE_MIN);
	_mav_put_float(buf, 12, sue_ALT_HOLD_THROTTLE_MAX);
	_mav_put_float(buf, 16, sue_ALT_HOLD_PITCH_MIN);
	_mav_put_float(buf, 20, sue_ALT_HOLD_PITCH_MAX);
	_mav_put_float(buf, 24, sue_ALT_HOLD_PITCH_HIGH);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_LEN);
#endif
#else
	mavlink_serial_udb_extra_f8_t packet;
	packet.sue_HEIGHT_TARGET_MAX = sue_HEIGHT_TARGET_MAX;
	packet.sue_HEIGHT_TARGET_MIN = sue_HEIGHT_TARGET_MIN;
	packet.sue_ALT_HOLD_THROTTLE_MIN = sue_ALT_HOLD_THROTTLE_MIN;
	packet.sue_ALT_HOLD_THROTTLE_MAX = sue_ALT_HOLD_THROTTLE_MAX;
	packet.sue_ALT_HOLD_PITCH_MIN = sue_ALT_HOLD_PITCH_MIN;
	packet.sue_ALT_HOLD_PITCH_MAX = sue_ALT_HOLD_PITCH_MAX;
	packet.sue_ALT_HOLD_PITCH_HIGH = sue_ALT_HOLD_PITCH_HIGH;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_serial_udb_extra_f8_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float sue_HEIGHT_TARGET_MAX, float sue_HEIGHT_TARGET_MIN, float sue_ALT_HOLD_THROTTLE_MIN, float sue_ALT_HOLD_THROTTLE_MAX, float sue_ALT_HOLD_PITCH_MIN, float sue_ALT_HOLD_PITCH_MAX, float sue_ALT_HOLD_PITCH_HIGH)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, sue_HEIGHT_TARGET_MAX);
	_mav_put_float(buf, 4, sue_HEIGHT_TARGET_MIN);
	_mav_put_float(buf, 8, sue_ALT_HOLD_THROTTLE_MIN);
	_mav_put_float(buf, 12, sue_ALT_HOLD_THROTTLE_MAX);
	_mav_put_float(buf, 16, sue_ALT_HOLD_PITCH_MIN);
	_mav_put_float(buf, 20, sue_ALT_HOLD_PITCH_MAX);
	_mav_put_float(buf, 24, sue_ALT_HOLD_PITCH_HIGH);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_LEN);
#endif
#else
	mavlink_serial_udb_extra_f8_t *packet = (mavlink_serial_udb_extra_f8_t *)msgbuf;
	packet->sue_HEIGHT_TARGET_MAX = sue_HEIGHT_TARGET_MAX;
	packet->sue_HEIGHT_TARGET_MIN = sue_HEIGHT_TARGET_MIN;
	packet->sue_ALT_HOLD_THROTTLE_MIN = sue_ALT_HOLD_THROTTLE_MIN;
	packet->sue_ALT_HOLD_THROTTLE_MAX = sue_ALT_HOLD_THROTTLE_MAX;
	packet->sue_ALT_HOLD_PITCH_MIN = sue_ALT_HOLD_PITCH_MIN;
	packet->sue_ALT_HOLD_PITCH_MAX = sue_ALT_HOLD_PITCH_MAX;
	packet->sue_ALT_HOLD_PITCH_HIGH = sue_ALT_HOLD_PITCH_HIGH;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8, (const char *)packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8, (const char *)packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SERIAL_UDB_EXTRA_F8 UNPACKING


/**
 * @brief Get field sue_HEIGHT_TARGET_MAX from serial_udb_extra_f8 message
 *
 * @return Serial UDB Extra HEIGHT_TARGET_MAX
 */
static inline float mavlink_msg_serial_udb_extra_f8_get_sue_HEIGHT_TARGET_MAX(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field sue_HEIGHT_TARGET_MIN from serial_udb_extra_f8 message
 *
 * @return Serial UDB Extra HEIGHT_TARGET_MIN
 */
static inline float mavlink_msg_serial_udb_extra_f8_get_sue_HEIGHT_TARGET_MIN(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field sue_ALT_HOLD_THROTTLE_MIN from serial_udb_extra_f8 message
 *
 * @return Serial UDB Extra ALT_HOLD_THROTTLE_MIN
 */
static inline float mavlink_msg_serial_udb_extra_f8_get_sue_ALT_HOLD_THROTTLE_MIN(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field sue_ALT_HOLD_THROTTLE_MAX from serial_udb_extra_f8 message
 *
 * @return Serial UDB Extra ALT_HOLD_THROTTLE_MAX
 */
static inline float mavlink_msg_serial_udb_extra_f8_get_sue_ALT_HOLD_THROTTLE_MAX(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field sue_ALT_HOLD_PITCH_MIN from serial_udb_extra_f8 message
 *
 * @return Serial UDB Extra ALT_HOLD_PITCH_MIN
 */
static inline float mavlink_msg_serial_udb_extra_f8_get_sue_ALT_HOLD_PITCH_MIN(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field sue_ALT_HOLD_PITCH_MAX from serial_udb_extra_f8 message
 *
 * @return Serial UDB Extra ALT_HOLD_PITCH_MAX
 */
static inline float mavlink_msg_serial_udb_extra_f8_get_sue_ALT_HOLD_PITCH_MAX(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field sue_ALT_HOLD_PITCH_HIGH from serial_udb_extra_f8 message
 *
 * @return Serial UDB Extra ALT_HOLD_PITCH_HIGH
 */
static inline float mavlink_msg_serial_udb_extra_f8_get_sue_ALT_HOLD_PITCH_HIGH(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a serial_udb_extra_f8 message into a struct
 *
 * @param msg The message to decode
 * @param serial_udb_extra_f8 C-struct to decode the message contents into
 */
static inline void mavlink_msg_serial_udb_extra_f8_decode(const mavlink_message_t* msg, mavlink_serial_udb_extra_f8_t* serial_udb_extra_f8)
{
#if MAVLINK_NEED_BYTE_SWAP
	serial_udb_extra_f8->sue_HEIGHT_TARGET_MAX = mavlink_msg_serial_udb_extra_f8_get_sue_HEIGHT_TARGET_MAX(msg);
	serial_udb_extra_f8->sue_HEIGHT_TARGET_MIN = mavlink_msg_serial_udb_extra_f8_get_sue_HEIGHT_TARGET_MIN(msg);
	serial_udb_extra_f8->sue_ALT_HOLD_THROTTLE_MIN = mavlink_msg_serial_udb_extra_f8_get_sue_ALT_HOLD_THROTTLE_MIN(msg);
	serial_udb_extra_f8->sue_ALT_HOLD_THROTTLE_MAX = mavlink_msg_serial_udb_extra_f8_get_sue_ALT_HOLD_THROTTLE_MAX(msg);
	serial_udb_extra_f8->sue_ALT_HOLD_PITCH_MIN = mavlink_msg_serial_udb_extra_f8_get_sue_ALT_HOLD_PITCH_MIN(msg);
	serial_udb_extra_f8->sue_ALT_HOLD_PITCH_MAX = mavlink_msg_serial_udb_extra_f8_get_sue_ALT_HOLD_PITCH_MAX(msg);
	serial_udb_extra_f8->sue_ALT_HOLD_PITCH_HIGH = mavlink_msg_serial_udb_extra_f8_get_sue_ALT_HOLD_PITCH_HIGH(msg);
#else
	memcpy(serial_udb_extra_f8, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F8_LEN);
#endif
}
