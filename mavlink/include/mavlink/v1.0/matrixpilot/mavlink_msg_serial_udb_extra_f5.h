// MESSAGE SERIAL_UDB_EXTRA_F5 PACKING

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5 173

typedef struct __mavlink_serial_udb_extra_f5_t
{
 float sue_YAWKP_AILERON; ///< Serial UDB YAWKP_AILERON Gain for Proporional control of navigation
 float sue_YAWKD_AILERON; ///< Serial UDB YAWKD_AILERON Gain for Rate control of navigation
 float sue_ROLLKP; ///< Serial UDB Extra ROLLKP Gain for Proportional control of roll stabilization
 float sue_ROLLKD; ///< Serial UDB Extra ROLLKD Gain for Rate control of roll stabilization
 float sue_YAW_STABILIZATION_AILERON; ///< YAW_STABILIZATION_AILERON Proportional control
 float sue_AILERON_BOOST; ///< Gain For Boosting Manual Aileron control When Plane Stabilized
} mavlink_serial_udb_extra_f5_t;

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN 24
#define MAVLINK_MSG_ID_173_LEN 24

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_CRC 121
#define MAVLINK_MSG_ID_173_CRC 121



#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F5 { \
	"SERIAL_UDB_EXTRA_F5", \
	6, \
	{  { "sue_YAWKP_AILERON", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_serial_udb_extra_f5_t, sue_YAWKP_AILERON) }, \
         { "sue_YAWKD_AILERON", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_serial_udb_extra_f5_t, sue_YAWKD_AILERON) }, \
         { "sue_ROLLKP", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_serial_udb_extra_f5_t, sue_ROLLKP) }, \
         { "sue_ROLLKD", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_serial_udb_extra_f5_t, sue_ROLLKD) }, \
         { "sue_YAW_STABILIZATION_AILERON", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_serial_udb_extra_f5_t, sue_YAW_STABILIZATION_AILERON) }, \
         { "sue_AILERON_BOOST", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_serial_udb_extra_f5_t, sue_AILERON_BOOST) }, \
         } \
}


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
 * @param sue_YAW_STABILIZATION_AILERON YAW_STABILIZATION_AILERON Proportional control
 * @param sue_AILERON_BOOST Gain For Boosting Manual Aileron control When Plane Stabilized
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f5_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float sue_YAWKP_AILERON, float sue_YAWKD_AILERON, float sue_ROLLKP, float sue_ROLLKD, float sue_YAW_STABILIZATION_AILERON, float sue_AILERON_BOOST)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN];
	_mav_put_float(buf, 0, sue_YAWKP_AILERON);
	_mav_put_float(buf, 4, sue_YAWKD_AILERON);
	_mav_put_float(buf, 8, sue_ROLLKP);
	_mav_put_float(buf, 12, sue_ROLLKD);
	_mav_put_float(buf, 16, sue_YAW_STABILIZATION_AILERON);
	_mav_put_float(buf, 20, sue_AILERON_BOOST);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN);
#else
	mavlink_serial_udb_extra_f5_t packet;
	packet.sue_YAWKP_AILERON = sue_YAWKP_AILERON;
	packet.sue_YAWKD_AILERON = sue_YAWKD_AILERON;
	packet.sue_ROLLKP = sue_ROLLKP;
	packet.sue_ROLLKD = sue_ROLLKD;
	packet.sue_YAW_STABILIZATION_AILERON = sue_YAW_STABILIZATION_AILERON;
	packet.sue_AILERON_BOOST = sue_AILERON_BOOST;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN);
#endif
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
 * @param sue_YAW_STABILIZATION_AILERON YAW_STABILIZATION_AILERON Proportional control
 * @param sue_AILERON_BOOST Gain For Boosting Manual Aileron control When Plane Stabilized
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f5_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float sue_YAWKP_AILERON,float sue_YAWKD_AILERON,float sue_ROLLKP,float sue_ROLLKD,float sue_YAW_STABILIZATION_AILERON,float sue_AILERON_BOOST)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN];
	_mav_put_float(buf, 0, sue_YAWKP_AILERON);
	_mav_put_float(buf, 4, sue_YAWKD_AILERON);
	_mav_put_float(buf, 8, sue_ROLLKP);
	_mav_put_float(buf, 12, sue_ROLLKD);
	_mav_put_float(buf, 16, sue_YAW_STABILIZATION_AILERON);
	_mav_put_float(buf, 20, sue_AILERON_BOOST);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN);
#else
	mavlink_serial_udb_extra_f5_t packet;
	packet.sue_YAWKP_AILERON = sue_YAWKP_AILERON;
	packet.sue_YAWKD_AILERON = sue_YAWKD_AILERON;
	packet.sue_ROLLKP = sue_ROLLKP;
	packet.sue_ROLLKD = sue_ROLLKD;
	packet.sue_YAW_STABILIZATION_AILERON = sue_YAW_STABILIZATION_AILERON;
	packet.sue_AILERON_BOOST = sue_AILERON_BOOST;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN);
#endif
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
	return mavlink_msg_serial_udb_extra_f5_pack(system_id, component_id, msg, serial_udb_extra_f5->sue_YAWKP_AILERON, serial_udb_extra_f5->sue_YAWKD_AILERON, serial_udb_extra_f5->sue_ROLLKP, serial_udb_extra_f5->sue_ROLLKD, serial_udb_extra_f5->sue_YAW_STABILIZATION_AILERON, serial_udb_extra_f5->sue_AILERON_BOOST);
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
	return mavlink_msg_serial_udb_extra_f5_pack_chan(system_id, component_id, chan, msg, serial_udb_extra_f5->sue_YAWKP_AILERON, serial_udb_extra_f5->sue_YAWKD_AILERON, serial_udb_extra_f5->sue_ROLLKP, serial_udb_extra_f5->sue_ROLLKD, serial_udb_extra_f5->sue_YAW_STABILIZATION_AILERON, serial_udb_extra_f5->sue_AILERON_BOOST);
}

/**
 * @brief Send a serial_udb_extra_f5 message
 * @param chan MAVLink channel to send the message
 *
 * @param sue_YAWKP_AILERON Serial UDB YAWKP_AILERON Gain for Proporional control of navigation
 * @param sue_YAWKD_AILERON Serial UDB YAWKD_AILERON Gain for Rate control of navigation
 * @param sue_ROLLKP Serial UDB Extra ROLLKP Gain for Proportional control of roll stabilization
 * @param sue_ROLLKD Serial UDB Extra ROLLKD Gain for Rate control of roll stabilization
 * @param sue_YAW_STABILIZATION_AILERON YAW_STABILIZATION_AILERON Proportional control
 * @param sue_AILERON_BOOST Gain For Boosting Manual Aileron control When Plane Stabilized
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_serial_udb_extra_f5_send(mavlink_channel_t chan, float sue_YAWKP_AILERON, float sue_YAWKD_AILERON, float sue_ROLLKP, float sue_ROLLKD, float sue_YAW_STABILIZATION_AILERON, float sue_AILERON_BOOST)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN];
	_mav_put_float(buf, 0, sue_YAWKP_AILERON);
	_mav_put_float(buf, 4, sue_YAWKD_AILERON);
	_mav_put_float(buf, 8, sue_ROLLKP);
	_mav_put_float(buf, 12, sue_ROLLKD);
	_mav_put_float(buf, 16, sue_YAW_STABILIZATION_AILERON);
	_mav_put_float(buf, 20, sue_AILERON_BOOST);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN);
#endif
#else
	mavlink_serial_udb_extra_f5_t packet;
	packet.sue_YAWKP_AILERON = sue_YAWKP_AILERON;
	packet.sue_YAWKD_AILERON = sue_YAWKD_AILERON;
	packet.sue_ROLLKP = sue_ROLLKP;
	packet.sue_ROLLKD = sue_ROLLKD;
	packet.sue_YAW_STABILIZATION_AILERON = sue_YAW_STABILIZATION_AILERON;
	packet.sue_AILERON_BOOST = sue_AILERON_BOOST;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN);
#endif
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
static inline void mavlink_msg_serial_udb_extra_f5_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float sue_YAWKP_AILERON, float sue_YAWKD_AILERON, float sue_ROLLKP, float sue_ROLLKD, float sue_YAW_STABILIZATION_AILERON, float sue_AILERON_BOOST)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, sue_YAWKP_AILERON);
	_mav_put_float(buf, 4, sue_YAWKD_AILERON);
	_mav_put_float(buf, 8, sue_ROLLKP);
	_mav_put_float(buf, 12, sue_ROLLKD);
	_mav_put_float(buf, 16, sue_YAW_STABILIZATION_AILERON);
	_mav_put_float(buf, 20, sue_AILERON_BOOST);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN);
#endif
#else
	mavlink_serial_udb_extra_f5_t *packet = (mavlink_serial_udb_extra_f5_t *)msgbuf;
	packet->sue_YAWKP_AILERON = sue_YAWKP_AILERON;
	packet->sue_YAWKD_AILERON = sue_YAWKD_AILERON;
	packet->sue_ROLLKP = sue_ROLLKP;
	packet->sue_ROLLKD = sue_ROLLKD;
	packet->sue_YAW_STABILIZATION_AILERON = sue_YAW_STABILIZATION_AILERON;
	packet->sue_AILERON_BOOST = sue_AILERON_BOOST;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5, (const char *)packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5, (const char *)packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN);
#endif
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
 * @brief Get field sue_YAW_STABILIZATION_AILERON from serial_udb_extra_f5 message
 *
 * @return YAW_STABILIZATION_AILERON Proportional control
 */
static inline float mavlink_msg_serial_udb_extra_f5_get_sue_YAW_STABILIZATION_AILERON(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field sue_AILERON_BOOST from serial_udb_extra_f5 message
 *
 * @return Gain For Boosting Manual Aileron control When Plane Stabilized
 */
static inline float mavlink_msg_serial_udb_extra_f5_get_sue_AILERON_BOOST(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a serial_udb_extra_f5 message into a struct
 *
 * @param msg The message to decode
 * @param serial_udb_extra_f5 C-struct to decode the message contents into
 */
static inline void mavlink_msg_serial_udb_extra_f5_decode(const mavlink_message_t* msg, mavlink_serial_udb_extra_f5_t* serial_udb_extra_f5)
{
#if MAVLINK_NEED_BYTE_SWAP
	serial_udb_extra_f5->sue_YAWKP_AILERON = mavlink_msg_serial_udb_extra_f5_get_sue_YAWKP_AILERON(msg);
	serial_udb_extra_f5->sue_YAWKD_AILERON = mavlink_msg_serial_udb_extra_f5_get_sue_YAWKD_AILERON(msg);
	serial_udb_extra_f5->sue_ROLLKP = mavlink_msg_serial_udb_extra_f5_get_sue_ROLLKP(msg);
	serial_udb_extra_f5->sue_ROLLKD = mavlink_msg_serial_udb_extra_f5_get_sue_ROLLKD(msg);
	serial_udb_extra_f5->sue_YAW_STABILIZATION_AILERON = mavlink_msg_serial_udb_extra_f5_get_sue_YAW_STABILIZATION_AILERON(msg);
	serial_udb_extra_f5->sue_AILERON_BOOST = mavlink_msg_serial_udb_extra_f5_get_sue_AILERON_BOOST(msg);
#else
	memcpy(serial_udb_extra_f5, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F5_LEN);
#endif
}
