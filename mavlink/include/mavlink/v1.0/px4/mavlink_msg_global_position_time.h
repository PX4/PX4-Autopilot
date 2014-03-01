// MESSAGE GLOBAL_POSITION_TIME PACKING

#define MAVLINK_MSG_ID_GLOBAL_POSITION_TIME 126

typedef struct __mavlink_global_position_time_t
{
 uint64_t time; ///< GPS time (microseconds since UNIX epoch)
 int32_t lat; ///< Latitude, expressed as * 1E7
 int32_t lon; ///< Longitude, expressed as * 1E7
 float alt; ///< Altitude in meters, expressed as * 1000 (millimeters), above MSL
 float vx; ///< Ground X Speed (Latitude), expressed as m/s * 100
 float vy; ///< Ground Y Speed (Longitude), expressed as m/s * 100
 float vz; ///< Ground Z Speed (Altitude), expressed as m/s * 100
 uint16_t hdg; ///< Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
} mavlink_global_position_time_t;

#define MAVLINK_MSG_ID_GLOBAL_POSITION_TIME_LEN 34
#define MAVLINK_MSG_ID_126_LEN 34

#define MAVLINK_MSG_ID_GLOBAL_POSITION_TIME_CRC 237
#define MAVLINK_MSG_ID_126_CRC 237



#define MAVLINK_MESSAGE_INFO_GLOBAL_POSITION_TIME { \
	"GLOBAL_POSITION_TIME", \
	8, \
	{  { "time", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_global_position_time_t, time) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_global_position_time_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_global_position_time_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_global_position_time_t, alt) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_global_position_time_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_global_position_time_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_global_position_time_t, vz) }, \
         { "hdg", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_global_position_time_t, hdg) }, \
         } \
}


/**
 * @brief Pack a global_position_time message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time GPS time (microseconds since UNIX epoch)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_position_time_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time, int32_t lat, int32_t lon, float alt, float vx, float vy, float vz, uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_TIME_LEN];
	_mav_put_uint64_t(buf, 0, time);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_float(buf, 16, alt);
	_mav_put_float(buf, 20, vx);
	_mav_put_float(buf, 24, vy);
	_mav_put_float(buf, 28, vz);
	_mav_put_uint16_t(buf, 32, hdg);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GLOBAL_POSITION_TIME_LEN);
#else
	mavlink_global_position_time_t packet;
	packet.time = time;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.hdg = hdg;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GLOBAL_POSITION_TIME_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_TIME;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GLOBAL_POSITION_TIME_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_TIME_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GLOBAL_POSITION_TIME_LEN);
#endif
}

/**
 * @brief Pack a global_position_time message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time GPS time (microseconds since UNIX epoch)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_position_time_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time,int32_t lat,int32_t lon,float alt,float vx,float vy,float vz,uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_TIME_LEN];
	_mav_put_uint64_t(buf, 0, time);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_float(buf, 16, alt);
	_mav_put_float(buf, 20, vx);
	_mav_put_float(buf, 24, vy);
	_mav_put_float(buf, 28, vz);
	_mav_put_uint16_t(buf, 32, hdg);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GLOBAL_POSITION_TIME_LEN);
#else
	mavlink_global_position_time_t packet;
	packet.time = time;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.hdg = hdg;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GLOBAL_POSITION_TIME_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_TIME;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GLOBAL_POSITION_TIME_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_TIME_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GLOBAL_POSITION_TIME_LEN);
#endif
}

/**
 * @brief Encode a global_position_time struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param global_position_time C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_position_time_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_global_position_time_t* global_position_time)
{
	return mavlink_msg_global_position_time_pack(system_id, component_id, msg, global_position_time->time, global_position_time->lat, global_position_time->lon, global_position_time->alt, global_position_time->vx, global_position_time->vy, global_position_time->vz, global_position_time->hdg);
}

/**
 * @brief Encode a global_position_time struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param global_position_time C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_position_time_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_global_position_time_t* global_position_time)
{
	return mavlink_msg_global_position_time_pack_chan(system_id, component_id, chan, msg, global_position_time->time, global_position_time->lat, global_position_time->lon, global_position_time->alt, global_position_time->vx, global_position_time->vy, global_position_time->vz, global_position_time->hdg);
}

/**
 * @brief Send a global_position_time message
 * @param chan MAVLink channel to send the message
 *
 * @param time GPS time (microseconds since UNIX epoch)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_global_position_time_send(mavlink_channel_t chan, uint64_t time, int32_t lat, int32_t lon, float alt, float vx, float vy, float vz, uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_TIME_LEN];
	_mav_put_uint64_t(buf, 0, time);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_float(buf, 16, alt);
	_mav_put_float(buf, 20, vx);
	_mav_put_float(buf, 24, vy);
	_mav_put_float(buf, 28, vz);
	_mav_put_uint16_t(buf, 32, hdg);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_TIME, buf, MAVLINK_MSG_ID_GLOBAL_POSITION_TIME_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_TIME_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_TIME, buf, MAVLINK_MSG_ID_GLOBAL_POSITION_TIME_LEN);
#endif
#else
	mavlink_global_position_time_t packet;
	packet.time = time;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.hdg = hdg;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_TIME, (const char *)&packet, MAVLINK_MSG_ID_GLOBAL_POSITION_TIME_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_TIME_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_TIME, (const char *)&packet, MAVLINK_MSG_ID_GLOBAL_POSITION_TIME_LEN);
#endif
#endif
}

#endif

// MESSAGE GLOBAL_POSITION_TIME UNPACKING


/**
 * @brief Get field time from global_position_time message
 *
 * @return GPS time (microseconds since UNIX epoch)
 */
static inline uint64_t mavlink_msg_global_position_time_get_time(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field lat from global_position_time message
 *
 * @return Latitude, expressed as * 1E7
 */
static inline int32_t mavlink_msg_global_position_time_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field lon from global_position_time message
 *
 * @return Longitude, expressed as * 1E7
 */
static inline int32_t mavlink_msg_global_position_time_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field alt from global_position_time message
 *
 * @return Altitude in meters, expressed as * 1000 (millimeters), above MSL
 */
static inline float mavlink_msg_global_position_time_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vx from global_position_time message
 *
 * @return Ground X Speed (Latitude), expressed as m/s * 100
 */
static inline float mavlink_msg_global_position_time_get_vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field vy from global_position_time message
 *
 * @return Ground Y Speed (Longitude), expressed as m/s * 100
 */
static inline float mavlink_msg_global_position_time_get_vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field vz from global_position_time message
 *
 * @return Ground Z Speed (Altitude), expressed as m/s * 100
 */
static inline float mavlink_msg_global_position_time_get_vz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field hdg from global_position_time message
 *
 * @return Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 */
static inline uint16_t mavlink_msg_global_position_time_get_hdg(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Decode a global_position_time message into a struct
 *
 * @param msg The message to decode
 * @param global_position_time C-struct to decode the message contents into
 */
static inline void mavlink_msg_global_position_time_decode(const mavlink_message_t* msg, mavlink_global_position_time_t* global_position_time)
{
#if MAVLINK_NEED_BYTE_SWAP
	global_position_time->time = mavlink_msg_global_position_time_get_time(msg);
	global_position_time->lat = mavlink_msg_global_position_time_get_lat(msg);
	global_position_time->lon = mavlink_msg_global_position_time_get_lon(msg);
	global_position_time->alt = mavlink_msg_global_position_time_get_alt(msg);
	global_position_time->vx = mavlink_msg_global_position_time_get_vx(msg);
	global_position_time->vy = mavlink_msg_global_position_time_get_vy(msg);
	global_position_time->vz = mavlink_msg_global_position_time_get_vz(msg);
	global_position_time->hdg = mavlink_msg_global_position_time_get_hdg(msg);
#else
	memcpy(global_position_time, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GLOBAL_POSITION_TIME_LEN);
#endif
}
