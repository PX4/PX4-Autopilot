// MESSAGE ALTITUDES PACKING

#define MAVLINK_MSG_ID_ALTITUDES 181

typedef struct __mavlink_altitudes_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 int32_t alt_gps; ///< GPS altitude in meters, expressed as * 1000 (millimeters), above MSL
 int32_t alt_imu; ///< IMU altitude above ground in meters, expressed as * 1000 (millimeters)
 int32_t alt_barometric; ///< barometeric altitude above ground in meters, expressed as * 1000 (millimeters)
 int32_t alt_optical_flow; ///< Optical flow altitude above ground in meters, expressed as * 1000 (millimeters)
 int32_t alt_range_finder; ///< Rangefinder Altitude above ground in meters, expressed as * 1000 (millimeters)
 int32_t alt_extra; ///< Extra altitude above ground in meters, expressed as * 1000 (millimeters)
} mavlink_altitudes_t;

#define MAVLINK_MSG_ID_ALTITUDES_LEN 28
#define MAVLINK_MSG_ID_181_LEN 28

#define MAVLINK_MSG_ID_ALTITUDES_CRC 55
#define MAVLINK_MSG_ID_181_CRC 55



#define MAVLINK_MESSAGE_INFO_ALTITUDES { \
	"ALTITUDES", \
	7, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_altitudes_t, time_boot_ms) }, \
         { "alt_gps", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_altitudes_t, alt_gps) }, \
         { "alt_imu", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_altitudes_t, alt_imu) }, \
         { "alt_barometric", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_altitudes_t, alt_barometric) }, \
         { "alt_optical_flow", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_altitudes_t, alt_optical_flow) }, \
         { "alt_range_finder", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_altitudes_t, alt_range_finder) }, \
         { "alt_extra", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_altitudes_t, alt_extra) }, \
         } \
}


/**
 * @brief Pack a altitudes message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param alt_gps GPS altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param alt_imu IMU altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param alt_barometric barometeric altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param alt_optical_flow Optical flow altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param alt_range_finder Rangefinder Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param alt_extra Extra altitude above ground in meters, expressed as * 1000 (millimeters)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_altitudes_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, int32_t alt_gps, int32_t alt_imu, int32_t alt_barometric, int32_t alt_optical_flow, int32_t alt_range_finder, int32_t alt_extra)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ALTITUDES_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, alt_gps);
	_mav_put_int32_t(buf, 8, alt_imu);
	_mav_put_int32_t(buf, 12, alt_barometric);
	_mav_put_int32_t(buf, 16, alt_optical_flow);
	_mav_put_int32_t(buf, 20, alt_range_finder);
	_mav_put_int32_t(buf, 24, alt_extra);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ALTITUDES_LEN);
#else
	mavlink_altitudes_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.alt_gps = alt_gps;
	packet.alt_imu = alt_imu;
	packet.alt_barometric = alt_barometric;
	packet.alt_optical_flow = alt_optical_flow;
	packet.alt_range_finder = alt_range_finder;
	packet.alt_extra = alt_extra;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ALTITUDES_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ALTITUDES;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ALTITUDES_LEN, MAVLINK_MSG_ID_ALTITUDES_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ALTITUDES_LEN);
#endif
}

/**
 * @brief Pack a altitudes message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param alt_gps GPS altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param alt_imu IMU altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param alt_barometric barometeric altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param alt_optical_flow Optical flow altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param alt_range_finder Rangefinder Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param alt_extra Extra altitude above ground in meters, expressed as * 1000 (millimeters)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_altitudes_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,int32_t alt_gps,int32_t alt_imu,int32_t alt_barometric,int32_t alt_optical_flow,int32_t alt_range_finder,int32_t alt_extra)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ALTITUDES_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, alt_gps);
	_mav_put_int32_t(buf, 8, alt_imu);
	_mav_put_int32_t(buf, 12, alt_barometric);
	_mav_put_int32_t(buf, 16, alt_optical_flow);
	_mav_put_int32_t(buf, 20, alt_range_finder);
	_mav_put_int32_t(buf, 24, alt_extra);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ALTITUDES_LEN);
#else
	mavlink_altitudes_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.alt_gps = alt_gps;
	packet.alt_imu = alt_imu;
	packet.alt_barometric = alt_barometric;
	packet.alt_optical_flow = alt_optical_flow;
	packet.alt_range_finder = alt_range_finder;
	packet.alt_extra = alt_extra;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ALTITUDES_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ALTITUDES;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ALTITUDES_LEN, MAVLINK_MSG_ID_ALTITUDES_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ALTITUDES_LEN);
#endif
}

/**
 * @brief Encode a altitudes struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param altitudes C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_altitudes_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_altitudes_t* altitudes)
{
	return mavlink_msg_altitudes_pack(system_id, component_id, msg, altitudes->time_boot_ms, altitudes->alt_gps, altitudes->alt_imu, altitudes->alt_barometric, altitudes->alt_optical_flow, altitudes->alt_range_finder, altitudes->alt_extra);
}

/**
 * @brief Encode a altitudes struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param altitudes C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_altitudes_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_altitudes_t* altitudes)
{
	return mavlink_msg_altitudes_pack_chan(system_id, component_id, chan, msg, altitudes->time_boot_ms, altitudes->alt_gps, altitudes->alt_imu, altitudes->alt_barometric, altitudes->alt_optical_flow, altitudes->alt_range_finder, altitudes->alt_extra);
}

/**
 * @brief Send a altitudes message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param alt_gps GPS altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param alt_imu IMU altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param alt_barometric barometeric altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param alt_optical_flow Optical flow altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param alt_range_finder Rangefinder Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param alt_extra Extra altitude above ground in meters, expressed as * 1000 (millimeters)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_altitudes_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t alt_gps, int32_t alt_imu, int32_t alt_barometric, int32_t alt_optical_flow, int32_t alt_range_finder, int32_t alt_extra)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ALTITUDES_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, alt_gps);
	_mav_put_int32_t(buf, 8, alt_imu);
	_mav_put_int32_t(buf, 12, alt_barometric);
	_mav_put_int32_t(buf, 16, alt_optical_flow);
	_mav_put_int32_t(buf, 20, alt_range_finder);
	_mav_put_int32_t(buf, 24, alt_extra);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ALTITUDES, buf, MAVLINK_MSG_ID_ALTITUDES_LEN, MAVLINK_MSG_ID_ALTITUDES_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ALTITUDES, buf, MAVLINK_MSG_ID_ALTITUDES_LEN);
#endif
#else
	mavlink_altitudes_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.alt_gps = alt_gps;
	packet.alt_imu = alt_imu;
	packet.alt_barometric = alt_barometric;
	packet.alt_optical_flow = alt_optical_flow;
	packet.alt_range_finder = alt_range_finder;
	packet.alt_extra = alt_extra;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ALTITUDES, (const char *)&packet, MAVLINK_MSG_ID_ALTITUDES_LEN, MAVLINK_MSG_ID_ALTITUDES_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ALTITUDES, (const char *)&packet, MAVLINK_MSG_ID_ALTITUDES_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ALTITUDES_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_altitudes_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, int32_t alt_gps, int32_t alt_imu, int32_t alt_barometric, int32_t alt_optical_flow, int32_t alt_range_finder, int32_t alt_extra)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, alt_gps);
	_mav_put_int32_t(buf, 8, alt_imu);
	_mav_put_int32_t(buf, 12, alt_barometric);
	_mav_put_int32_t(buf, 16, alt_optical_flow);
	_mav_put_int32_t(buf, 20, alt_range_finder);
	_mav_put_int32_t(buf, 24, alt_extra);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ALTITUDES, buf, MAVLINK_MSG_ID_ALTITUDES_LEN, MAVLINK_MSG_ID_ALTITUDES_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ALTITUDES, buf, MAVLINK_MSG_ID_ALTITUDES_LEN);
#endif
#else
	mavlink_altitudes_t *packet = (mavlink_altitudes_t *)msgbuf;
	packet->time_boot_ms = time_boot_ms;
	packet->alt_gps = alt_gps;
	packet->alt_imu = alt_imu;
	packet->alt_barometric = alt_barometric;
	packet->alt_optical_flow = alt_optical_flow;
	packet->alt_range_finder = alt_range_finder;
	packet->alt_extra = alt_extra;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ALTITUDES, (const char *)packet, MAVLINK_MSG_ID_ALTITUDES_LEN, MAVLINK_MSG_ID_ALTITUDES_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ALTITUDES, (const char *)packet, MAVLINK_MSG_ID_ALTITUDES_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ALTITUDES UNPACKING


/**
 * @brief Get field time_boot_ms from altitudes message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_altitudes_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field alt_gps from altitudes message
 *
 * @return GPS altitude in meters, expressed as * 1000 (millimeters), above MSL
 */
static inline int32_t mavlink_msg_altitudes_get_alt_gps(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field alt_imu from altitudes message
 *
 * @return IMU altitude above ground in meters, expressed as * 1000 (millimeters)
 */
static inline int32_t mavlink_msg_altitudes_get_alt_imu(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field alt_barometric from altitudes message
 *
 * @return barometeric altitude above ground in meters, expressed as * 1000 (millimeters)
 */
static inline int32_t mavlink_msg_altitudes_get_alt_barometric(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field alt_optical_flow from altitudes message
 *
 * @return Optical flow altitude above ground in meters, expressed as * 1000 (millimeters)
 */
static inline int32_t mavlink_msg_altitudes_get_alt_optical_flow(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field alt_range_finder from altitudes message
 *
 * @return Rangefinder Altitude above ground in meters, expressed as * 1000 (millimeters)
 */
static inline int32_t mavlink_msg_altitudes_get_alt_range_finder(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field alt_extra from altitudes message
 *
 * @return Extra altitude above ground in meters, expressed as * 1000 (millimeters)
 */
static inline int32_t mavlink_msg_altitudes_get_alt_extra(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  24);
}

/**
 * @brief Decode a altitudes message into a struct
 *
 * @param msg The message to decode
 * @param altitudes C-struct to decode the message contents into
 */
static inline void mavlink_msg_altitudes_decode(const mavlink_message_t* msg, mavlink_altitudes_t* altitudes)
{
#if MAVLINK_NEED_BYTE_SWAP
	altitudes->time_boot_ms = mavlink_msg_altitudes_get_time_boot_ms(msg);
	altitudes->alt_gps = mavlink_msg_altitudes_get_alt_gps(msg);
	altitudes->alt_imu = mavlink_msg_altitudes_get_alt_imu(msg);
	altitudes->alt_barometric = mavlink_msg_altitudes_get_alt_barometric(msg);
	altitudes->alt_optical_flow = mavlink_msg_altitudes_get_alt_optical_flow(msg);
	altitudes->alt_range_finder = mavlink_msg_altitudes_get_alt_range_finder(msg);
	altitudes->alt_extra = mavlink_msg_altitudes_get_alt_extra(msg);
#else
	memcpy(altitudes, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ALTITUDES_LEN);
#endif
}
