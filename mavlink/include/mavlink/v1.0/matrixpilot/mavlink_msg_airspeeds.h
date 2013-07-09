// MESSAGE AIRSPEEDS PACKING

#define MAVLINK_MSG_ID_AIRSPEEDS 182

typedef struct __mavlink_airspeeds_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 int16_t airspeed_imu; ///< Airspeed estimate from IMU, cm/s
 int16_t airspeed_pitot; ///< Pitot measured forward airpseed, cm/s
 int16_t airspeed_hot_wire; ///< Hot wire anenometer measured airspeed, cm/s
 int16_t airspeed_ultrasonic; ///< Ultrasonic measured airspeed, cm/s
 int16_t aoa; ///< Angle of attack sensor, degrees * 10
 int16_t aoy; ///< Yaw angle sensor, degrees * 10
} mavlink_airspeeds_t;

#define MAVLINK_MSG_ID_AIRSPEEDS_LEN 16
#define MAVLINK_MSG_ID_182_LEN 16

#define MAVLINK_MSG_ID_AIRSPEEDS_CRC 154
#define MAVLINK_MSG_ID_182_CRC 154



#define MAVLINK_MESSAGE_INFO_AIRSPEEDS { \
	"AIRSPEEDS", \
	7, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_airspeeds_t, time_boot_ms) }, \
         { "airspeed_imu", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_airspeeds_t, airspeed_imu) }, \
         { "airspeed_pitot", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_airspeeds_t, airspeed_pitot) }, \
         { "airspeed_hot_wire", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_airspeeds_t, airspeed_hot_wire) }, \
         { "airspeed_ultrasonic", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_airspeeds_t, airspeed_ultrasonic) }, \
         { "aoa", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_airspeeds_t, aoa) }, \
         { "aoy", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_airspeeds_t, aoy) }, \
         } \
}


/**
 * @brief Pack a airspeeds message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param airspeed_imu Airspeed estimate from IMU, cm/s
 * @param airspeed_pitot Pitot measured forward airpseed, cm/s
 * @param airspeed_hot_wire Hot wire anenometer measured airspeed, cm/s
 * @param airspeed_ultrasonic Ultrasonic measured airspeed, cm/s
 * @param aoa Angle of attack sensor, degrees * 10
 * @param aoy Yaw angle sensor, degrees * 10
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airspeeds_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, int16_t airspeed_imu, int16_t airspeed_pitot, int16_t airspeed_hot_wire, int16_t airspeed_ultrasonic, int16_t aoa, int16_t aoy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_AIRSPEEDS_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int16_t(buf, 4, airspeed_imu);
	_mav_put_int16_t(buf, 6, airspeed_pitot);
	_mav_put_int16_t(buf, 8, airspeed_hot_wire);
	_mav_put_int16_t(buf, 10, airspeed_ultrasonic);
	_mav_put_int16_t(buf, 12, aoa);
	_mav_put_int16_t(buf, 14, aoy);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRSPEEDS_LEN);
#else
	mavlink_airspeeds_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.airspeed_imu = airspeed_imu;
	packet.airspeed_pitot = airspeed_pitot;
	packet.airspeed_hot_wire = airspeed_hot_wire;
	packet.airspeed_ultrasonic = airspeed_ultrasonic;
	packet.aoa = aoa;
	packet.aoy = aoy;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRSPEEDS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_AIRSPEEDS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AIRSPEEDS_LEN, MAVLINK_MSG_ID_AIRSPEEDS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AIRSPEEDS_LEN);
#endif
}

/**
 * @brief Pack a airspeeds message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param airspeed_imu Airspeed estimate from IMU, cm/s
 * @param airspeed_pitot Pitot measured forward airpseed, cm/s
 * @param airspeed_hot_wire Hot wire anenometer measured airspeed, cm/s
 * @param airspeed_ultrasonic Ultrasonic measured airspeed, cm/s
 * @param aoa Angle of attack sensor, degrees * 10
 * @param aoy Yaw angle sensor, degrees * 10
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airspeeds_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,int16_t airspeed_imu,int16_t airspeed_pitot,int16_t airspeed_hot_wire,int16_t airspeed_ultrasonic,int16_t aoa,int16_t aoy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_AIRSPEEDS_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int16_t(buf, 4, airspeed_imu);
	_mav_put_int16_t(buf, 6, airspeed_pitot);
	_mav_put_int16_t(buf, 8, airspeed_hot_wire);
	_mav_put_int16_t(buf, 10, airspeed_ultrasonic);
	_mav_put_int16_t(buf, 12, aoa);
	_mav_put_int16_t(buf, 14, aoy);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRSPEEDS_LEN);
#else
	mavlink_airspeeds_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.airspeed_imu = airspeed_imu;
	packet.airspeed_pitot = airspeed_pitot;
	packet.airspeed_hot_wire = airspeed_hot_wire;
	packet.airspeed_ultrasonic = airspeed_ultrasonic;
	packet.aoa = aoa;
	packet.aoy = aoy;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRSPEEDS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_AIRSPEEDS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AIRSPEEDS_LEN, MAVLINK_MSG_ID_AIRSPEEDS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AIRSPEEDS_LEN);
#endif
}

/**
 * @brief Encode a airspeeds struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param airspeeds C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airspeeds_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_airspeeds_t* airspeeds)
{
	return mavlink_msg_airspeeds_pack(system_id, component_id, msg, airspeeds->time_boot_ms, airspeeds->airspeed_imu, airspeeds->airspeed_pitot, airspeeds->airspeed_hot_wire, airspeeds->airspeed_ultrasonic, airspeeds->aoa, airspeeds->aoy);
}

/**
 * @brief Send a airspeeds message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param airspeed_imu Airspeed estimate from IMU, cm/s
 * @param airspeed_pitot Pitot measured forward airpseed, cm/s
 * @param airspeed_hot_wire Hot wire anenometer measured airspeed, cm/s
 * @param airspeed_ultrasonic Ultrasonic measured airspeed, cm/s
 * @param aoa Angle of attack sensor, degrees * 10
 * @param aoy Yaw angle sensor, degrees * 10
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_airspeeds_send(mavlink_channel_t chan, uint32_t time_boot_ms, int16_t airspeed_imu, int16_t airspeed_pitot, int16_t airspeed_hot_wire, int16_t airspeed_ultrasonic, int16_t aoa, int16_t aoy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_AIRSPEEDS_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int16_t(buf, 4, airspeed_imu);
	_mav_put_int16_t(buf, 6, airspeed_pitot);
	_mav_put_int16_t(buf, 8, airspeed_hot_wire);
	_mav_put_int16_t(buf, 10, airspeed_ultrasonic);
	_mav_put_int16_t(buf, 12, aoa);
	_mav_put_int16_t(buf, 14, aoy);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRSPEEDS, buf, MAVLINK_MSG_ID_AIRSPEEDS_LEN, MAVLINK_MSG_ID_AIRSPEEDS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRSPEEDS, buf, MAVLINK_MSG_ID_AIRSPEEDS_LEN);
#endif
#else
	mavlink_airspeeds_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.airspeed_imu = airspeed_imu;
	packet.airspeed_pitot = airspeed_pitot;
	packet.airspeed_hot_wire = airspeed_hot_wire;
	packet.airspeed_ultrasonic = airspeed_ultrasonic;
	packet.aoa = aoa;
	packet.aoy = aoy;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRSPEEDS, (const char *)&packet, MAVLINK_MSG_ID_AIRSPEEDS_LEN, MAVLINK_MSG_ID_AIRSPEEDS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRSPEEDS, (const char *)&packet, MAVLINK_MSG_ID_AIRSPEEDS_LEN);
#endif
#endif
}

#endif

// MESSAGE AIRSPEEDS UNPACKING


/**
 * @brief Get field time_boot_ms from airspeeds message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_airspeeds_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field airspeed_imu from airspeeds message
 *
 * @return Airspeed estimate from IMU, cm/s
 */
static inline int16_t mavlink_msg_airspeeds_get_airspeed_imu(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field airspeed_pitot from airspeeds message
 *
 * @return Pitot measured forward airpseed, cm/s
 */
static inline int16_t mavlink_msg_airspeeds_get_airspeed_pitot(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field airspeed_hot_wire from airspeeds message
 *
 * @return Hot wire anenometer measured airspeed, cm/s
 */
static inline int16_t mavlink_msg_airspeeds_get_airspeed_hot_wire(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field airspeed_ultrasonic from airspeeds message
 *
 * @return Ultrasonic measured airspeed, cm/s
 */
static inline int16_t mavlink_msg_airspeeds_get_airspeed_ultrasonic(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field aoa from airspeeds message
 *
 * @return Angle of attack sensor, degrees * 10
 */
static inline int16_t mavlink_msg_airspeeds_get_aoa(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field aoy from airspeeds message
 *
 * @return Yaw angle sensor, degrees * 10
 */
static inline int16_t mavlink_msg_airspeeds_get_aoy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Decode a airspeeds message into a struct
 *
 * @param msg The message to decode
 * @param airspeeds C-struct to decode the message contents into
 */
static inline void mavlink_msg_airspeeds_decode(const mavlink_message_t* msg, mavlink_airspeeds_t* airspeeds)
{
#if MAVLINK_NEED_BYTE_SWAP
	airspeeds->time_boot_ms = mavlink_msg_airspeeds_get_time_boot_ms(msg);
	airspeeds->airspeed_imu = mavlink_msg_airspeeds_get_airspeed_imu(msg);
	airspeeds->airspeed_pitot = mavlink_msg_airspeeds_get_airspeed_pitot(msg);
	airspeeds->airspeed_hot_wire = mavlink_msg_airspeeds_get_airspeed_hot_wire(msg);
	airspeeds->airspeed_ultrasonic = mavlink_msg_airspeeds_get_airspeed_ultrasonic(msg);
	airspeeds->aoa = mavlink_msg_airspeeds_get_aoa(msg);
	airspeeds->aoy = mavlink_msg_airspeeds_get_aoy(msg);
#else
	memcpy(airspeeds, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_AIRSPEEDS_LEN);
#endif
}
