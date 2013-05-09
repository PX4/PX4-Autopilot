// MESSAGE DISCRETE_RADAR PACKING

#define MAVLINK_MSG_ID_DISCRETE_RADAR 107

typedef struct __mavlink_discrete_radar_t
{
 uint64_t time_usec; ///< Timestamp (microseconds, synced to UNIX time or since system boot)
 float sonar; ///< Front sonar distance in m
 int16_t distances[32]; ///< 360 degree (in 32 sectors) distances in mm
 uint8_t sensor_id; ///< Sensor ID
 uint8_t quality; ///< Radar quality / confidence. 0: bad, 255: maximum quality
} mavlink_discrete_radar_t;

#define MAVLINK_MSG_ID_DISCRETE_RADAR_LEN 78
#define MAVLINK_MSG_ID_107_LEN 78

#define MAVLINK_MSG_DISCRETE_RADAR_FIELD_DISTANCES_LEN 32

#define MAVLINK_MESSAGE_INFO_DISCRETE_RADAR { \
	"DISCRETE_RADAR", \
	5, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_discrete_radar_t, time_usec) }, \
         { "sonar", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_discrete_radar_t, sonar) }, \
         { "distances", NULL, MAVLINK_TYPE_INT16_T, 32, 12, offsetof(mavlink_discrete_radar_t, distances) }, \
         { "sensor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 76, offsetof(mavlink_discrete_radar_t, sensor_id) }, \
         { "quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 77, offsetof(mavlink_discrete_radar_t, quality) }, \
         } \
}


/**
 * @brief Pack a discrete_radar message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param sensor_id Sensor ID
 * @param distances 360 degree (in 32 sectors) distances in mm
 * @param sonar Front sonar distance in m
 * @param quality Radar quality / confidence. 0: bad, 255: maximum quality
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_discrete_radar_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint8_t sensor_id, const int16_t *distances, float sonar, uint8_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[78];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, sonar);
	_mav_put_uint8_t(buf, 76, sensor_id);
	_mav_put_uint8_t(buf, 77, quality);
	_mav_put_int16_t_array(buf, 12, distances, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 78);
#else
	mavlink_discrete_radar_t packet;
	packet.time_usec = time_usec;
	packet.sonar = sonar;
	packet.sensor_id = sensor_id;
	packet.quality = quality;
	mav_array_memcpy(packet.distances, distances, sizeof(int16_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 78);
#endif

	msg->msgid = MAVLINK_MSG_ID_DISCRETE_RADAR;
	return mavlink_finalize_message(msg, system_id, component_id, 78, 85);
}

/**
 * @brief Pack a discrete_radar message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param sensor_id Sensor ID
 * @param distances 360 degree (in 32 sectors) distances in mm
 * @param sonar Front sonar distance in m
 * @param quality Radar quality / confidence. 0: bad, 255: maximum quality
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_discrete_radar_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,uint8_t sensor_id,const int16_t *distances,float sonar,uint8_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[78];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, sonar);
	_mav_put_uint8_t(buf, 76, sensor_id);
	_mav_put_uint8_t(buf, 77, quality);
	_mav_put_int16_t_array(buf, 12, distances, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 78);
#else
	mavlink_discrete_radar_t packet;
	packet.time_usec = time_usec;
	packet.sonar = sonar;
	packet.sensor_id = sensor_id;
	packet.quality = quality;
	mav_array_memcpy(packet.distances, distances, sizeof(int16_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 78);
#endif

	msg->msgid = MAVLINK_MSG_ID_DISCRETE_RADAR;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 78, 85);
}

/**
 * @brief Encode a discrete_radar struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param discrete_radar C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_discrete_radar_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_discrete_radar_t* discrete_radar)
{
	return mavlink_msg_discrete_radar_pack(system_id, component_id, msg, discrete_radar->time_usec, discrete_radar->sensor_id, discrete_radar->distances, discrete_radar->sonar, discrete_radar->quality);
}

/**
 * @brief Send a discrete_radar message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param sensor_id Sensor ID
 * @param distances 360 degree (in 32 sectors) distances in mm
 * @param sonar Front sonar distance in m
 * @param quality Radar quality / confidence. 0: bad, 255: maximum quality
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_discrete_radar_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t sensor_id, const int16_t *distances, float sonar, uint8_t quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[78];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, sonar);
	_mav_put_uint8_t(buf, 76, sensor_id);
	_mav_put_uint8_t(buf, 77, quality);
	_mav_put_int16_t_array(buf, 12, distances, 32);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DISCRETE_RADAR, buf, 78, 85);
#else
	mavlink_discrete_radar_t packet;
	packet.time_usec = time_usec;
	packet.sonar = sonar;
	packet.sensor_id = sensor_id;
	packet.quality = quality;
	mav_array_memcpy(packet.distances, distances, sizeof(int16_t)*32);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DISCRETE_RADAR, (const char *)&packet, 78, 85);
#endif
}

#endif

// MESSAGE DISCRETE_RADAR UNPACKING


/**
 * @brief Get field time_usec from discrete_radar message
 *
 * @return Timestamp (microseconds, synced to UNIX time or since system boot)
 */
static inline uint64_t mavlink_msg_discrete_radar_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field sensor_id from discrete_radar message
 *
 * @return Sensor ID
 */
static inline uint8_t mavlink_msg_discrete_radar_get_sensor_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  76);
}

/**
 * @brief Get field distances from discrete_radar message
 *
 * @return 360 degree (in 32 sectors) distances in mm
 */
static inline uint16_t mavlink_msg_discrete_radar_get_distances(const mavlink_message_t* msg, int16_t *distances)
{
	return _MAV_RETURN_int16_t_array(msg, distances, 32,  12);
}

/**
 * @brief Get field sonar from discrete_radar message
 *
 * @return Front sonar distance in m
 */
static inline float mavlink_msg_discrete_radar_get_sonar(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field quality from discrete_radar message
 *
 * @return Radar quality / confidence. 0: bad, 255: maximum quality
 */
static inline uint8_t mavlink_msg_discrete_radar_get_quality(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  77);
}

/**
 * @brief Decode a discrete_radar message into a struct
 *
 * @param msg The message to decode
 * @param discrete_radar C-struct to decode the message contents into
 */
static inline void mavlink_msg_discrete_radar_decode(const mavlink_message_t* msg, mavlink_discrete_radar_t* discrete_radar)
{
#if MAVLINK_NEED_BYTE_SWAP
	discrete_radar->time_usec = mavlink_msg_discrete_radar_get_time_usec(msg);
	discrete_radar->sonar = mavlink_msg_discrete_radar_get_sonar(msg);
	mavlink_msg_discrete_radar_get_distances(msg, discrete_radar->distances);
	discrete_radar->sensor_id = mavlink_msg_discrete_radar_get_sensor_id(msg);
	discrete_radar->quality = mavlink_msg_discrete_radar_get_quality(msg);
#else
	memcpy(discrete_radar, _MAV_PAYLOAD(msg), 78);
#endif
}
