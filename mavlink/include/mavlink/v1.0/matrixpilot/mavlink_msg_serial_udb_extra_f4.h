// MESSAGE SERIAL_UDB_EXTRA_F4 PACKING

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4 172

typedef struct __mavlink_serial_udb_extra_f4_t
{
 uint8_t sue_ROLL_STABILIZATION_AILERONS; ///< Serial UDB Extra Roll Stabilization with Ailerons Enabled
 uint8_t sue_ROLL_STABILIZATION_RUDDER; ///< Serial UDB Extra Roll Stabilization with Rudder Enabled
 uint8_t sue_PITCH_STABILIZATION; ///< Serial UDB Extra Pitch Stabilization Enabled
 uint8_t sue_YAW_STABILIZATION_RUDDER; ///< Serial UDB Extra Yaw Stabilization using Rudder Enabled
 uint8_t sue_YAW_STABILIZATION_AILERON; ///< Serial UDB Extra Yaw Stabilization using Ailerons Enabled
 uint8_t sue_AILERON_NAVIGATION; ///< Serial UDB Extra Navigation with Ailerons Enabled
 uint8_t sue_RUDDER_NAVIGATION; ///< Serial UDB Extra Navigation with Rudder Enabled
 uint8_t sue_ALTITUDEHOLD_STABILIZED; ///< Serial UDB Extra Type of Alitude Hold when in Stabilized Mode
 uint8_t sue_ALTITUDEHOLD_WAYPOINT; ///< Serial UDB Extra Type of Alitude Hold when in Waypoint Mode
 uint8_t sue_RACING_MODE; ///< Serial UDB Extra Firmware racing mode enabled
} mavlink_serial_udb_extra_f4_t;

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4_LEN 10
#define MAVLINK_MSG_ID_172_LEN 10

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4_CRC 191
#define MAVLINK_MSG_ID_172_CRC 191



#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F4 { \
	"SERIAL_UDB_EXTRA_F4", \
	10, \
	{  { "sue_ROLL_STABILIZATION_AILERONS", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_serial_udb_extra_f4_t, sue_ROLL_STABILIZATION_AILERONS) }, \
         { "sue_ROLL_STABILIZATION_RUDDER", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_serial_udb_extra_f4_t, sue_ROLL_STABILIZATION_RUDDER) }, \
         { "sue_PITCH_STABILIZATION", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_serial_udb_extra_f4_t, sue_PITCH_STABILIZATION) }, \
         { "sue_YAW_STABILIZATION_RUDDER", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_serial_udb_extra_f4_t, sue_YAW_STABILIZATION_RUDDER) }, \
         { "sue_YAW_STABILIZATION_AILERON", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_serial_udb_extra_f4_t, sue_YAW_STABILIZATION_AILERON) }, \
         { "sue_AILERON_NAVIGATION", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_serial_udb_extra_f4_t, sue_AILERON_NAVIGATION) }, \
         { "sue_RUDDER_NAVIGATION", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_serial_udb_extra_f4_t, sue_RUDDER_NAVIGATION) }, \
         { "sue_ALTITUDEHOLD_STABILIZED", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_serial_udb_extra_f4_t, sue_ALTITUDEHOLD_STABILIZED) }, \
         { "sue_ALTITUDEHOLD_WAYPOINT", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_serial_udb_extra_f4_t, sue_ALTITUDEHOLD_WAYPOINT) }, \
         { "sue_RACING_MODE", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_serial_udb_extra_f4_t, sue_RACING_MODE) }, \
         } \
}


/**
 * @brief Pack a serial_udb_extra_f4 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sue_ROLL_STABILIZATION_AILERONS Serial UDB Extra Roll Stabilization with Ailerons Enabled
 * @param sue_ROLL_STABILIZATION_RUDDER Serial UDB Extra Roll Stabilization with Rudder Enabled
 * @param sue_PITCH_STABILIZATION Serial UDB Extra Pitch Stabilization Enabled
 * @param sue_YAW_STABILIZATION_RUDDER Serial UDB Extra Yaw Stabilization using Rudder Enabled
 * @param sue_YAW_STABILIZATION_AILERON Serial UDB Extra Yaw Stabilization using Ailerons Enabled
 * @param sue_AILERON_NAVIGATION Serial UDB Extra Navigation with Ailerons Enabled
 * @param sue_RUDDER_NAVIGATION Serial UDB Extra Navigation with Rudder Enabled
 * @param sue_ALTITUDEHOLD_STABILIZED Serial UDB Extra Type of Alitude Hold when in Stabilized Mode
 * @param sue_ALTITUDEHOLD_WAYPOINT Serial UDB Extra Type of Alitude Hold when in Waypoint Mode
 * @param sue_RACING_MODE Serial UDB Extra Firmware racing mode enabled
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f4_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t sue_ROLL_STABILIZATION_AILERONS, uint8_t sue_ROLL_STABILIZATION_RUDDER, uint8_t sue_PITCH_STABILIZATION, uint8_t sue_YAW_STABILIZATION_RUDDER, uint8_t sue_YAW_STABILIZATION_AILERON, uint8_t sue_AILERON_NAVIGATION, uint8_t sue_RUDDER_NAVIGATION, uint8_t sue_ALTITUDEHOLD_STABILIZED, uint8_t sue_ALTITUDEHOLD_WAYPOINT, uint8_t sue_RACING_MODE)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4_LEN];
	_mav_put_uint8_t(buf, 0, sue_ROLL_STABILIZATION_AILERONS);
	_mav_put_uint8_t(buf, 1, sue_ROLL_STABILIZATION_RUDDER);
	_mav_put_uint8_t(buf, 2, sue_PITCH_STABILIZATION);
	_mav_put_uint8_t(buf, 3, sue_YAW_STABILIZATION_RUDDER);
	_mav_put_uint8_t(buf, 4, sue_YAW_STABILIZATION_AILERON);
	_mav_put_uint8_t(buf, 5, sue_AILERON_NAVIGATION);
	_mav_put_uint8_t(buf, 6, sue_RUDDER_NAVIGATION);
	_mav_put_uint8_t(buf, 7, sue_ALTITUDEHOLD_STABILIZED);
	_mav_put_uint8_t(buf, 8, sue_ALTITUDEHOLD_WAYPOINT);
	_mav_put_uint8_t(buf, 9, sue_RACING_MODE);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4_LEN);
#else
	mavlink_serial_udb_extra_f4_t packet;
	packet.sue_ROLL_STABILIZATION_AILERONS = sue_ROLL_STABILIZATION_AILERONS;
	packet.sue_ROLL_STABILIZATION_RUDDER = sue_ROLL_STABILIZATION_RUDDER;
	packet.sue_PITCH_STABILIZATION = sue_PITCH_STABILIZATION;
	packet.sue_YAW_STABILIZATION_RUDDER = sue_YAW_STABILIZATION_RUDDER;
	packet.sue_YAW_STABILIZATION_AILERON = sue_YAW_STABILIZATION_AILERON;
	packet.sue_AILERON_NAVIGATION = sue_AILERON_NAVIGATION;
	packet.sue_RUDDER_NAVIGATION = sue_RUDDER_NAVIGATION;
	packet.sue_ALTITUDEHOLD_STABILIZED = sue_ALTITUDEHOLD_STABILIZED;
	packet.sue_ALTITUDEHOLD_WAYPOINT = sue_ALTITUDEHOLD_WAYPOINT;
	packet.sue_RACING_MODE = sue_RACING_MODE;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4_LEN);
#endif
}

/**
 * @brief Pack a serial_udb_extra_f4 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param sue_ROLL_STABILIZATION_AILERONS Serial UDB Extra Roll Stabilization with Ailerons Enabled
 * @param sue_ROLL_STABILIZATION_RUDDER Serial UDB Extra Roll Stabilization with Rudder Enabled
 * @param sue_PITCH_STABILIZATION Serial UDB Extra Pitch Stabilization Enabled
 * @param sue_YAW_STABILIZATION_RUDDER Serial UDB Extra Yaw Stabilization using Rudder Enabled
 * @param sue_YAW_STABILIZATION_AILERON Serial UDB Extra Yaw Stabilization using Ailerons Enabled
 * @param sue_AILERON_NAVIGATION Serial UDB Extra Navigation with Ailerons Enabled
 * @param sue_RUDDER_NAVIGATION Serial UDB Extra Navigation with Rudder Enabled
 * @param sue_ALTITUDEHOLD_STABILIZED Serial UDB Extra Type of Alitude Hold when in Stabilized Mode
 * @param sue_ALTITUDEHOLD_WAYPOINT Serial UDB Extra Type of Alitude Hold when in Waypoint Mode
 * @param sue_RACING_MODE Serial UDB Extra Firmware racing mode enabled
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f4_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t sue_ROLL_STABILIZATION_AILERONS,uint8_t sue_ROLL_STABILIZATION_RUDDER,uint8_t sue_PITCH_STABILIZATION,uint8_t sue_YAW_STABILIZATION_RUDDER,uint8_t sue_YAW_STABILIZATION_AILERON,uint8_t sue_AILERON_NAVIGATION,uint8_t sue_RUDDER_NAVIGATION,uint8_t sue_ALTITUDEHOLD_STABILIZED,uint8_t sue_ALTITUDEHOLD_WAYPOINT,uint8_t sue_RACING_MODE)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4_LEN];
	_mav_put_uint8_t(buf, 0, sue_ROLL_STABILIZATION_AILERONS);
	_mav_put_uint8_t(buf, 1, sue_ROLL_STABILIZATION_RUDDER);
	_mav_put_uint8_t(buf, 2, sue_PITCH_STABILIZATION);
	_mav_put_uint8_t(buf, 3, sue_YAW_STABILIZATION_RUDDER);
	_mav_put_uint8_t(buf, 4, sue_YAW_STABILIZATION_AILERON);
	_mav_put_uint8_t(buf, 5, sue_AILERON_NAVIGATION);
	_mav_put_uint8_t(buf, 6, sue_RUDDER_NAVIGATION);
	_mav_put_uint8_t(buf, 7, sue_ALTITUDEHOLD_STABILIZED);
	_mav_put_uint8_t(buf, 8, sue_ALTITUDEHOLD_WAYPOINT);
	_mav_put_uint8_t(buf, 9, sue_RACING_MODE);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4_LEN);
#else
	mavlink_serial_udb_extra_f4_t packet;
	packet.sue_ROLL_STABILIZATION_AILERONS = sue_ROLL_STABILIZATION_AILERONS;
	packet.sue_ROLL_STABILIZATION_RUDDER = sue_ROLL_STABILIZATION_RUDDER;
	packet.sue_PITCH_STABILIZATION = sue_PITCH_STABILIZATION;
	packet.sue_YAW_STABILIZATION_RUDDER = sue_YAW_STABILIZATION_RUDDER;
	packet.sue_YAW_STABILIZATION_AILERON = sue_YAW_STABILIZATION_AILERON;
	packet.sue_AILERON_NAVIGATION = sue_AILERON_NAVIGATION;
	packet.sue_RUDDER_NAVIGATION = sue_RUDDER_NAVIGATION;
	packet.sue_ALTITUDEHOLD_STABILIZED = sue_ALTITUDEHOLD_STABILIZED;
	packet.sue_ALTITUDEHOLD_WAYPOINT = sue_ALTITUDEHOLD_WAYPOINT;
	packet.sue_RACING_MODE = sue_RACING_MODE;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4_LEN);
#endif
}

/**
 * @brief Encode a serial_udb_extra_f4 struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f4 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f4_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_serial_udb_extra_f4_t* serial_udb_extra_f4)
{
	return mavlink_msg_serial_udb_extra_f4_pack(system_id, component_id, msg, serial_udb_extra_f4->sue_ROLL_STABILIZATION_AILERONS, serial_udb_extra_f4->sue_ROLL_STABILIZATION_RUDDER, serial_udb_extra_f4->sue_PITCH_STABILIZATION, serial_udb_extra_f4->sue_YAW_STABILIZATION_RUDDER, serial_udb_extra_f4->sue_YAW_STABILIZATION_AILERON, serial_udb_extra_f4->sue_AILERON_NAVIGATION, serial_udb_extra_f4->sue_RUDDER_NAVIGATION, serial_udb_extra_f4->sue_ALTITUDEHOLD_STABILIZED, serial_udb_extra_f4->sue_ALTITUDEHOLD_WAYPOINT, serial_udb_extra_f4->sue_RACING_MODE);
}

/**
 * @brief Send a serial_udb_extra_f4 message
 * @param chan MAVLink channel to send the message
 *
 * @param sue_ROLL_STABILIZATION_AILERONS Serial UDB Extra Roll Stabilization with Ailerons Enabled
 * @param sue_ROLL_STABILIZATION_RUDDER Serial UDB Extra Roll Stabilization with Rudder Enabled
 * @param sue_PITCH_STABILIZATION Serial UDB Extra Pitch Stabilization Enabled
 * @param sue_YAW_STABILIZATION_RUDDER Serial UDB Extra Yaw Stabilization using Rudder Enabled
 * @param sue_YAW_STABILIZATION_AILERON Serial UDB Extra Yaw Stabilization using Ailerons Enabled
 * @param sue_AILERON_NAVIGATION Serial UDB Extra Navigation with Ailerons Enabled
 * @param sue_RUDDER_NAVIGATION Serial UDB Extra Navigation with Rudder Enabled
 * @param sue_ALTITUDEHOLD_STABILIZED Serial UDB Extra Type of Alitude Hold when in Stabilized Mode
 * @param sue_ALTITUDEHOLD_WAYPOINT Serial UDB Extra Type of Alitude Hold when in Waypoint Mode
 * @param sue_RACING_MODE Serial UDB Extra Firmware racing mode enabled
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_serial_udb_extra_f4_send(mavlink_channel_t chan, uint8_t sue_ROLL_STABILIZATION_AILERONS, uint8_t sue_ROLL_STABILIZATION_RUDDER, uint8_t sue_PITCH_STABILIZATION, uint8_t sue_YAW_STABILIZATION_RUDDER, uint8_t sue_YAW_STABILIZATION_AILERON, uint8_t sue_AILERON_NAVIGATION, uint8_t sue_RUDDER_NAVIGATION, uint8_t sue_ALTITUDEHOLD_STABILIZED, uint8_t sue_ALTITUDEHOLD_WAYPOINT, uint8_t sue_RACING_MODE)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4_LEN];
	_mav_put_uint8_t(buf, 0, sue_ROLL_STABILIZATION_AILERONS);
	_mav_put_uint8_t(buf, 1, sue_ROLL_STABILIZATION_RUDDER);
	_mav_put_uint8_t(buf, 2, sue_PITCH_STABILIZATION);
	_mav_put_uint8_t(buf, 3, sue_YAW_STABILIZATION_RUDDER);
	_mav_put_uint8_t(buf, 4, sue_YAW_STABILIZATION_AILERON);
	_mav_put_uint8_t(buf, 5, sue_AILERON_NAVIGATION);
	_mav_put_uint8_t(buf, 6, sue_RUDDER_NAVIGATION);
	_mav_put_uint8_t(buf, 7, sue_ALTITUDEHOLD_STABILIZED);
	_mav_put_uint8_t(buf, 8, sue_ALTITUDEHOLD_WAYPOINT);
	_mav_put_uint8_t(buf, 9, sue_RACING_MODE);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4_LEN);
#endif
#else
	mavlink_serial_udb_extra_f4_t packet;
	packet.sue_ROLL_STABILIZATION_AILERONS = sue_ROLL_STABILIZATION_AILERONS;
	packet.sue_ROLL_STABILIZATION_RUDDER = sue_ROLL_STABILIZATION_RUDDER;
	packet.sue_PITCH_STABILIZATION = sue_PITCH_STABILIZATION;
	packet.sue_YAW_STABILIZATION_RUDDER = sue_YAW_STABILIZATION_RUDDER;
	packet.sue_YAW_STABILIZATION_AILERON = sue_YAW_STABILIZATION_AILERON;
	packet.sue_AILERON_NAVIGATION = sue_AILERON_NAVIGATION;
	packet.sue_RUDDER_NAVIGATION = sue_RUDDER_NAVIGATION;
	packet.sue_ALTITUDEHOLD_STABILIZED = sue_ALTITUDEHOLD_STABILIZED;
	packet.sue_ALTITUDEHOLD_WAYPOINT = sue_ALTITUDEHOLD_WAYPOINT;
	packet.sue_RACING_MODE = sue_RACING_MODE;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4_LEN);
#endif
#endif
}

#endif

// MESSAGE SERIAL_UDB_EXTRA_F4 UNPACKING


/**
 * @brief Get field sue_ROLL_STABILIZATION_AILERONS from serial_udb_extra_f4 message
 *
 * @return Serial UDB Extra Roll Stabilization with Ailerons Enabled
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f4_get_sue_ROLL_STABILIZATION_AILERONS(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field sue_ROLL_STABILIZATION_RUDDER from serial_udb_extra_f4 message
 *
 * @return Serial UDB Extra Roll Stabilization with Rudder Enabled
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f4_get_sue_ROLL_STABILIZATION_RUDDER(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field sue_PITCH_STABILIZATION from serial_udb_extra_f4 message
 *
 * @return Serial UDB Extra Pitch Stabilization Enabled
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f4_get_sue_PITCH_STABILIZATION(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field sue_YAW_STABILIZATION_RUDDER from serial_udb_extra_f4 message
 *
 * @return Serial UDB Extra Yaw Stabilization using Rudder Enabled
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f4_get_sue_YAW_STABILIZATION_RUDDER(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field sue_YAW_STABILIZATION_AILERON from serial_udb_extra_f4 message
 *
 * @return Serial UDB Extra Yaw Stabilization using Ailerons Enabled
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f4_get_sue_YAW_STABILIZATION_AILERON(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field sue_AILERON_NAVIGATION from serial_udb_extra_f4 message
 *
 * @return Serial UDB Extra Navigation with Ailerons Enabled
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f4_get_sue_AILERON_NAVIGATION(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field sue_RUDDER_NAVIGATION from serial_udb_extra_f4 message
 *
 * @return Serial UDB Extra Navigation with Rudder Enabled
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f4_get_sue_RUDDER_NAVIGATION(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field sue_ALTITUDEHOLD_STABILIZED from serial_udb_extra_f4 message
 *
 * @return Serial UDB Extra Type of Alitude Hold when in Stabilized Mode
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f4_get_sue_ALTITUDEHOLD_STABILIZED(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field sue_ALTITUDEHOLD_WAYPOINT from serial_udb_extra_f4 message
 *
 * @return Serial UDB Extra Type of Alitude Hold when in Waypoint Mode
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f4_get_sue_ALTITUDEHOLD_WAYPOINT(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field sue_RACING_MODE from serial_udb_extra_f4 message
 *
 * @return Serial UDB Extra Firmware racing mode enabled
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f4_get_sue_RACING_MODE(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Decode a serial_udb_extra_f4 message into a struct
 *
 * @param msg The message to decode
 * @param serial_udb_extra_f4 C-struct to decode the message contents into
 */
static inline void mavlink_msg_serial_udb_extra_f4_decode(const mavlink_message_t* msg, mavlink_serial_udb_extra_f4_t* serial_udb_extra_f4)
{
#if MAVLINK_NEED_BYTE_SWAP
	serial_udb_extra_f4->sue_ROLL_STABILIZATION_AILERONS = mavlink_msg_serial_udb_extra_f4_get_sue_ROLL_STABILIZATION_AILERONS(msg);
	serial_udb_extra_f4->sue_ROLL_STABILIZATION_RUDDER = mavlink_msg_serial_udb_extra_f4_get_sue_ROLL_STABILIZATION_RUDDER(msg);
	serial_udb_extra_f4->sue_PITCH_STABILIZATION = mavlink_msg_serial_udb_extra_f4_get_sue_PITCH_STABILIZATION(msg);
	serial_udb_extra_f4->sue_YAW_STABILIZATION_RUDDER = mavlink_msg_serial_udb_extra_f4_get_sue_YAW_STABILIZATION_RUDDER(msg);
	serial_udb_extra_f4->sue_YAW_STABILIZATION_AILERON = mavlink_msg_serial_udb_extra_f4_get_sue_YAW_STABILIZATION_AILERON(msg);
	serial_udb_extra_f4->sue_AILERON_NAVIGATION = mavlink_msg_serial_udb_extra_f4_get_sue_AILERON_NAVIGATION(msg);
	serial_udb_extra_f4->sue_RUDDER_NAVIGATION = mavlink_msg_serial_udb_extra_f4_get_sue_RUDDER_NAVIGATION(msg);
	serial_udb_extra_f4->sue_ALTITUDEHOLD_STABILIZED = mavlink_msg_serial_udb_extra_f4_get_sue_ALTITUDEHOLD_STABILIZED(msg);
	serial_udb_extra_f4->sue_ALTITUDEHOLD_WAYPOINT = mavlink_msg_serial_udb_extra_f4_get_sue_ALTITUDEHOLD_WAYPOINT(msg);
	serial_udb_extra_f4->sue_RACING_MODE = mavlink_msg_serial_udb_extra_f4_get_sue_RACING_MODE(msg);
#else
	memcpy(serial_udb_extra_f4, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F4_LEN);
#endif
}
