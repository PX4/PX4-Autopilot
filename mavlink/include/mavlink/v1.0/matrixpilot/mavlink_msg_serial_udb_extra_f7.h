// MESSAGE SERIAL_UDB_EXTRA_F7 PACKING

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7 175

typedef struct __mavlink_serial_udb_extra_f7_t
{
 float sue_YAWKP_RUDDER; ///< Serial UDB YAWKP_RUDDER Gain for Proporional control of navigation
 float sue_YAWKD_RUDDER; ///< Serial UDB YAWKD_RUDDER Gain for Rate control of navigation
 float sue_ROLLKP_RUDDER; ///< Serial UDB Extra ROLLKP_RUDDER Gain for Proportional control of roll stabilization
 float sue_ROLLKD_RUDDER; ///< Serial UDB Extra ROLLKD_RUDDER Gain for Rate control of roll stabilization
 float sue_RUDDER_BOOST; ///< SERIAL UDB EXTRA Rudder Boost Gain to Manual Control when stabilized
 float sue_RTL_PITCH_DOWN; ///< Serial UDB Extra Return To Landing - Angle to Pitch Plane Down
} mavlink_serial_udb_extra_f7_t;

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7_LEN 24
#define MAVLINK_MSG_ID_175_LEN 24

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7_CRC 171
#define MAVLINK_MSG_ID_175_CRC 171



#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F7 { \
	"SERIAL_UDB_EXTRA_F7", \
	6, \
	{  { "sue_YAWKP_RUDDER", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_serial_udb_extra_f7_t, sue_YAWKP_RUDDER) }, \
         { "sue_YAWKD_RUDDER", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_serial_udb_extra_f7_t, sue_YAWKD_RUDDER) }, \
         { "sue_ROLLKP_RUDDER", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_serial_udb_extra_f7_t, sue_ROLLKP_RUDDER) }, \
         { "sue_ROLLKD_RUDDER", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_serial_udb_extra_f7_t, sue_ROLLKD_RUDDER) }, \
         { "sue_RUDDER_BOOST", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_serial_udb_extra_f7_t, sue_RUDDER_BOOST) }, \
         { "sue_RTL_PITCH_DOWN", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_serial_udb_extra_f7_t, sue_RTL_PITCH_DOWN) }, \
         } \
}


/**
 * @brief Pack a serial_udb_extra_f7 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sue_YAWKP_RUDDER Serial UDB YAWKP_RUDDER Gain for Proporional control of navigation
 * @param sue_YAWKD_RUDDER Serial UDB YAWKD_RUDDER Gain for Rate control of navigation
 * @param sue_ROLLKP_RUDDER Serial UDB Extra ROLLKP_RUDDER Gain for Proportional control of roll stabilization
 * @param sue_ROLLKD_RUDDER Serial UDB Extra ROLLKD_RUDDER Gain for Rate control of roll stabilization
 * @param sue_RUDDER_BOOST SERIAL UDB EXTRA Rudder Boost Gain to Manual Control when stabilized
 * @param sue_RTL_PITCH_DOWN Serial UDB Extra Return To Landing - Angle to Pitch Plane Down
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f7_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float sue_YAWKP_RUDDER, float sue_YAWKD_RUDDER, float sue_ROLLKP_RUDDER, float sue_ROLLKD_RUDDER, float sue_RUDDER_BOOST, float sue_RTL_PITCH_DOWN)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7_LEN];
	_mav_put_float(buf, 0, sue_YAWKP_RUDDER);
	_mav_put_float(buf, 4, sue_YAWKD_RUDDER);
	_mav_put_float(buf, 8, sue_ROLLKP_RUDDER);
	_mav_put_float(buf, 12, sue_ROLLKD_RUDDER);
	_mav_put_float(buf, 16, sue_RUDDER_BOOST);
	_mav_put_float(buf, 20, sue_RTL_PITCH_DOWN);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7_LEN);
#else
	mavlink_serial_udb_extra_f7_t packet;
	packet.sue_YAWKP_RUDDER = sue_YAWKP_RUDDER;
	packet.sue_YAWKD_RUDDER = sue_YAWKD_RUDDER;
	packet.sue_ROLLKP_RUDDER = sue_ROLLKP_RUDDER;
	packet.sue_ROLLKD_RUDDER = sue_ROLLKD_RUDDER;
	packet.sue_RUDDER_BOOST = sue_RUDDER_BOOST;
	packet.sue_RTL_PITCH_DOWN = sue_RTL_PITCH_DOWN;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7_LEN);
#endif
}

/**
 * @brief Pack a serial_udb_extra_f7 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param sue_YAWKP_RUDDER Serial UDB YAWKP_RUDDER Gain for Proporional control of navigation
 * @param sue_YAWKD_RUDDER Serial UDB YAWKD_RUDDER Gain for Rate control of navigation
 * @param sue_ROLLKP_RUDDER Serial UDB Extra ROLLKP_RUDDER Gain for Proportional control of roll stabilization
 * @param sue_ROLLKD_RUDDER Serial UDB Extra ROLLKD_RUDDER Gain for Rate control of roll stabilization
 * @param sue_RUDDER_BOOST SERIAL UDB EXTRA Rudder Boost Gain to Manual Control when stabilized
 * @param sue_RTL_PITCH_DOWN Serial UDB Extra Return To Landing - Angle to Pitch Plane Down
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f7_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float sue_YAWKP_RUDDER,float sue_YAWKD_RUDDER,float sue_ROLLKP_RUDDER,float sue_ROLLKD_RUDDER,float sue_RUDDER_BOOST,float sue_RTL_PITCH_DOWN)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7_LEN];
	_mav_put_float(buf, 0, sue_YAWKP_RUDDER);
	_mav_put_float(buf, 4, sue_YAWKD_RUDDER);
	_mav_put_float(buf, 8, sue_ROLLKP_RUDDER);
	_mav_put_float(buf, 12, sue_ROLLKD_RUDDER);
	_mav_put_float(buf, 16, sue_RUDDER_BOOST);
	_mav_put_float(buf, 20, sue_RTL_PITCH_DOWN);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7_LEN);
#else
	mavlink_serial_udb_extra_f7_t packet;
	packet.sue_YAWKP_RUDDER = sue_YAWKP_RUDDER;
	packet.sue_YAWKD_RUDDER = sue_YAWKD_RUDDER;
	packet.sue_ROLLKP_RUDDER = sue_ROLLKP_RUDDER;
	packet.sue_ROLLKD_RUDDER = sue_ROLLKD_RUDDER;
	packet.sue_RUDDER_BOOST = sue_RUDDER_BOOST;
	packet.sue_RTL_PITCH_DOWN = sue_RTL_PITCH_DOWN;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7_LEN);
#endif
}

/**
 * @brief Encode a serial_udb_extra_f7 struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f7 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f7_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_serial_udb_extra_f7_t* serial_udb_extra_f7)
{
	return mavlink_msg_serial_udb_extra_f7_pack(system_id, component_id, msg, serial_udb_extra_f7->sue_YAWKP_RUDDER, serial_udb_extra_f7->sue_YAWKD_RUDDER, serial_udb_extra_f7->sue_ROLLKP_RUDDER, serial_udb_extra_f7->sue_ROLLKD_RUDDER, serial_udb_extra_f7->sue_RUDDER_BOOST, serial_udb_extra_f7->sue_RTL_PITCH_DOWN);
}

/**
 * @brief Send a serial_udb_extra_f7 message
 * @param chan MAVLink channel to send the message
 *
 * @param sue_YAWKP_RUDDER Serial UDB YAWKP_RUDDER Gain for Proporional control of navigation
 * @param sue_YAWKD_RUDDER Serial UDB YAWKD_RUDDER Gain for Rate control of navigation
 * @param sue_ROLLKP_RUDDER Serial UDB Extra ROLLKP_RUDDER Gain for Proportional control of roll stabilization
 * @param sue_ROLLKD_RUDDER Serial UDB Extra ROLLKD_RUDDER Gain for Rate control of roll stabilization
 * @param sue_RUDDER_BOOST SERIAL UDB EXTRA Rudder Boost Gain to Manual Control when stabilized
 * @param sue_RTL_PITCH_DOWN Serial UDB Extra Return To Landing - Angle to Pitch Plane Down
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_serial_udb_extra_f7_send(mavlink_channel_t chan, float sue_YAWKP_RUDDER, float sue_YAWKD_RUDDER, float sue_ROLLKP_RUDDER, float sue_ROLLKD_RUDDER, float sue_RUDDER_BOOST, float sue_RTL_PITCH_DOWN)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7_LEN];
	_mav_put_float(buf, 0, sue_YAWKP_RUDDER);
	_mav_put_float(buf, 4, sue_YAWKD_RUDDER);
	_mav_put_float(buf, 8, sue_ROLLKP_RUDDER);
	_mav_put_float(buf, 12, sue_ROLLKD_RUDDER);
	_mav_put_float(buf, 16, sue_RUDDER_BOOST);
	_mav_put_float(buf, 20, sue_RTL_PITCH_DOWN);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7_LEN);
#endif
#else
	mavlink_serial_udb_extra_f7_t packet;
	packet.sue_YAWKP_RUDDER = sue_YAWKP_RUDDER;
	packet.sue_YAWKD_RUDDER = sue_YAWKD_RUDDER;
	packet.sue_ROLLKP_RUDDER = sue_ROLLKP_RUDDER;
	packet.sue_ROLLKD_RUDDER = sue_ROLLKD_RUDDER;
	packet.sue_RUDDER_BOOST = sue_RUDDER_BOOST;
	packet.sue_RTL_PITCH_DOWN = sue_RTL_PITCH_DOWN;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7_LEN);
#endif
#endif
}

#endif

// MESSAGE SERIAL_UDB_EXTRA_F7 UNPACKING


/**
 * @brief Get field sue_YAWKP_RUDDER from serial_udb_extra_f7 message
 *
 * @return Serial UDB YAWKP_RUDDER Gain for Proporional control of navigation
 */
static inline float mavlink_msg_serial_udb_extra_f7_get_sue_YAWKP_RUDDER(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field sue_YAWKD_RUDDER from serial_udb_extra_f7 message
 *
 * @return Serial UDB YAWKD_RUDDER Gain for Rate control of navigation
 */
static inline float mavlink_msg_serial_udb_extra_f7_get_sue_YAWKD_RUDDER(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field sue_ROLLKP_RUDDER from serial_udb_extra_f7 message
 *
 * @return Serial UDB Extra ROLLKP_RUDDER Gain for Proportional control of roll stabilization
 */
static inline float mavlink_msg_serial_udb_extra_f7_get_sue_ROLLKP_RUDDER(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field sue_ROLLKD_RUDDER from serial_udb_extra_f7 message
 *
 * @return Serial UDB Extra ROLLKD_RUDDER Gain for Rate control of roll stabilization
 */
static inline float mavlink_msg_serial_udb_extra_f7_get_sue_ROLLKD_RUDDER(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field sue_RUDDER_BOOST from serial_udb_extra_f7 message
 *
 * @return SERIAL UDB EXTRA Rudder Boost Gain to Manual Control when stabilized
 */
static inline float mavlink_msg_serial_udb_extra_f7_get_sue_RUDDER_BOOST(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field sue_RTL_PITCH_DOWN from serial_udb_extra_f7 message
 *
 * @return Serial UDB Extra Return To Landing - Angle to Pitch Plane Down
 */
static inline float mavlink_msg_serial_udb_extra_f7_get_sue_RTL_PITCH_DOWN(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a serial_udb_extra_f7 message into a struct
 *
 * @param msg The message to decode
 * @param serial_udb_extra_f7 C-struct to decode the message contents into
 */
static inline void mavlink_msg_serial_udb_extra_f7_decode(const mavlink_message_t* msg, mavlink_serial_udb_extra_f7_t* serial_udb_extra_f7)
{
#if MAVLINK_NEED_BYTE_SWAP
	serial_udb_extra_f7->sue_YAWKP_RUDDER = mavlink_msg_serial_udb_extra_f7_get_sue_YAWKP_RUDDER(msg);
	serial_udb_extra_f7->sue_YAWKD_RUDDER = mavlink_msg_serial_udb_extra_f7_get_sue_YAWKD_RUDDER(msg);
	serial_udb_extra_f7->sue_ROLLKP_RUDDER = mavlink_msg_serial_udb_extra_f7_get_sue_ROLLKP_RUDDER(msg);
	serial_udb_extra_f7->sue_ROLLKD_RUDDER = mavlink_msg_serial_udb_extra_f7_get_sue_ROLLKD_RUDDER(msg);
	serial_udb_extra_f7->sue_RUDDER_BOOST = mavlink_msg_serial_udb_extra_f7_get_sue_RUDDER_BOOST(msg);
	serial_udb_extra_f7->sue_RTL_PITCH_DOWN = mavlink_msg_serial_udb_extra_f7_get_sue_RTL_PITCH_DOWN(msg);
#else
	memcpy(serial_udb_extra_f7, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F7_LEN);
#endif
}
