// MESSAGE OBSTACLE_POSITION_P2G PACKING

#define MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G 166

MAVPACKED(
typedef struct __mavlink_obstacle_position_p2g_t {
 uint64_t timestamp; /*< timestamp*/
 float obstacle_x; /*< obstacle X position (NED), in m*/
 float obstacle_y; /*< obstacle Y position (NED), in m*/
 float obstacle_z; /*< obstacle Z position (NED), in m*/
 uint8_t obstacle_valid; /*< whether the obstacle is valid*/
}) mavlink_obstacle_position_p2g_t;

#define MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_LEN 21
#define MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_MIN_LEN 21
#define MAVLINK_MSG_ID_166_LEN 21
#define MAVLINK_MSG_ID_166_MIN_LEN 21

#define MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_CRC 218
#define MAVLINK_MSG_ID_166_CRC 218



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_OBSTACLE_POSITION_P2G { \
	166, \
	"OBSTACLE_POSITION_P2G", \
	5, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_obstacle_position_p2g_t, timestamp) }, \
         { "obstacle_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_obstacle_position_p2g_t, obstacle_x) }, \
         { "obstacle_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_obstacle_position_p2g_t, obstacle_y) }, \
         { "obstacle_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_obstacle_position_p2g_t, obstacle_z) }, \
         { "obstacle_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_obstacle_position_p2g_t, obstacle_valid) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_OBSTACLE_POSITION_P2G { \
	"OBSTACLE_POSITION_P2G", \
	5, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_obstacle_position_p2g_t, timestamp) }, \
         { "obstacle_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_obstacle_position_p2g_t, obstacle_x) }, \
         { "obstacle_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_obstacle_position_p2g_t, obstacle_y) }, \
         { "obstacle_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_obstacle_position_p2g_t, obstacle_z) }, \
         { "obstacle_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_obstacle_position_p2g_t, obstacle_valid) }, \
         } \
}
#endif

/**
 * @brief Pack a obstacle_position_p2g message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp timestamp
 * @param obstacle_x obstacle X position (NED), in m
 * @param obstacle_y obstacle Y position (NED), in m
 * @param obstacle_z obstacle Z position (NED), in m
 * @param obstacle_valid whether the obstacle is valid
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_obstacle_position_p2g_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, float obstacle_x, float obstacle_y, float obstacle_z, uint8_t obstacle_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, obstacle_x);
	_mav_put_float(buf, 12, obstacle_y);
	_mav_put_float(buf, 16, obstacle_z);
	_mav_put_uint8_t(buf, 20, obstacle_valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_LEN);
#else
	mavlink_obstacle_position_p2g_t packet;
	packet.timestamp = timestamp;
	packet.obstacle_x = obstacle_x;
	packet.obstacle_y = obstacle_y;
	packet.obstacle_z = obstacle_z;
	packet.obstacle_valid = obstacle_valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_MIN_LEN, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_LEN, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_CRC);
}

/**
 * @brief Pack a obstacle_position_p2g message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp timestamp
 * @param obstacle_x obstacle X position (NED), in m
 * @param obstacle_y obstacle Y position (NED), in m
 * @param obstacle_z obstacle Z position (NED), in m
 * @param obstacle_valid whether the obstacle is valid
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_obstacle_position_p2g_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,float obstacle_x,float obstacle_y,float obstacle_z,uint8_t obstacle_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, obstacle_x);
	_mav_put_float(buf, 12, obstacle_y);
	_mav_put_float(buf, 16, obstacle_z);
	_mav_put_uint8_t(buf, 20, obstacle_valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_LEN);
#else
	mavlink_obstacle_position_p2g_t packet;
	packet.timestamp = timestamp;
	packet.obstacle_x = obstacle_x;
	packet.obstacle_y = obstacle_y;
	packet.obstacle_z = obstacle_z;
	packet.obstacle_valid = obstacle_valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_MIN_LEN, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_LEN, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_CRC);
}

/**
 * @brief Encode a obstacle_position_p2g struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param obstacle_position_p2g C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_obstacle_position_p2g_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_obstacle_position_p2g_t* obstacle_position_p2g)
{
	return mavlink_msg_obstacle_position_p2g_pack(system_id, component_id, msg, obstacle_position_p2g->timestamp, obstacle_position_p2g->obstacle_x, obstacle_position_p2g->obstacle_y, obstacle_position_p2g->obstacle_z, obstacle_position_p2g->obstacle_valid);
}

/**
 * @brief Encode a obstacle_position_p2g struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param obstacle_position_p2g C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_obstacle_position_p2g_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_obstacle_position_p2g_t* obstacle_position_p2g)
{
	return mavlink_msg_obstacle_position_p2g_pack_chan(system_id, component_id, chan, msg, obstacle_position_p2g->timestamp, obstacle_position_p2g->obstacle_x, obstacle_position_p2g->obstacle_y, obstacle_position_p2g->obstacle_z, obstacle_position_p2g->obstacle_valid);
}

/**
 * @brief Send a obstacle_position_p2g message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp timestamp
 * @param obstacle_x obstacle X position (NED), in m
 * @param obstacle_y obstacle Y position (NED), in m
 * @param obstacle_z obstacle Z position (NED), in m
 * @param obstacle_valid whether the obstacle is valid
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_obstacle_position_p2g_send(mavlink_channel_t chan, uint64_t timestamp, float obstacle_x, float obstacle_y, float obstacle_z, uint8_t obstacle_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, obstacle_x);
	_mav_put_float(buf, 12, obstacle_y);
	_mav_put_float(buf, 16, obstacle_z);
	_mav_put_uint8_t(buf, 20, obstacle_valid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G, buf, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_MIN_LEN, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_LEN, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_CRC);
#else
	mavlink_obstacle_position_p2g_t packet;
	packet.timestamp = timestamp;
	packet.obstacle_x = obstacle_x;
	packet.obstacle_y = obstacle_y;
	packet.obstacle_z = obstacle_z;
	packet.obstacle_valid = obstacle_valid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G, (const char *)&packet, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_MIN_LEN, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_LEN, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_CRC);
#endif
}

/**
 * @brief Send a obstacle_position_p2g message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_obstacle_position_p2g_send_struct(mavlink_channel_t chan, const mavlink_obstacle_position_p2g_t* obstacle_position_p2g)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_obstacle_position_p2g_send(chan, obstacle_position_p2g->timestamp, obstacle_position_p2g->obstacle_x, obstacle_position_p2g->obstacle_y, obstacle_position_p2g->obstacle_z, obstacle_position_p2g->obstacle_valid);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G, (const char *)obstacle_position_p2g, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_MIN_LEN, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_LEN, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_CRC);
#endif
}

#if MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_obstacle_position_p2g_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float obstacle_x, float obstacle_y, float obstacle_z, uint8_t obstacle_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, obstacle_x);
	_mav_put_float(buf, 12, obstacle_y);
	_mav_put_float(buf, 16, obstacle_z);
	_mav_put_uint8_t(buf, 20, obstacle_valid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G, buf, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_MIN_LEN, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_LEN, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_CRC);
#else
	mavlink_obstacle_position_p2g_t *packet = (mavlink_obstacle_position_p2g_t *)msgbuf;
	packet->timestamp = timestamp;
	packet->obstacle_x = obstacle_x;
	packet->obstacle_y = obstacle_y;
	packet->obstacle_z = obstacle_z;
	packet->obstacle_valid = obstacle_valid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G, (const char *)packet, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_MIN_LEN, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_LEN, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_CRC);
#endif
}
#endif

#endif

// MESSAGE OBSTACLE_POSITION_P2G UNPACKING


/**
 * @brief Get field timestamp from obstacle_position_p2g message
 *
 * @return timestamp
 */
static inline uint64_t mavlink_msg_obstacle_position_p2g_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field obstacle_x from obstacle_position_p2g message
 *
 * @return obstacle X position (NED), in m
 */
static inline float mavlink_msg_obstacle_position_p2g_get_obstacle_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field obstacle_y from obstacle_position_p2g message
 *
 * @return obstacle Y position (NED), in m
 */
static inline float mavlink_msg_obstacle_position_p2g_get_obstacle_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field obstacle_z from obstacle_position_p2g message
 *
 * @return obstacle Z position (NED), in m
 */
static inline float mavlink_msg_obstacle_position_p2g_get_obstacle_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field obstacle_valid from obstacle_position_p2g message
 *
 * @return whether the obstacle is valid
 */
static inline uint8_t mavlink_msg_obstacle_position_p2g_get_obstacle_valid(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Decode a obstacle_position_p2g message into a struct
 *
 * @param msg The message to decode
 * @param obstacle_position_p2g C-struct to decode the message contents into
 */
static inline void mavlink_msg_obstacle_position_p2g_decode(const mavlink_message_t* msg, mavlink_obstacle_position_p2g_t* obstacle_position_p2g)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	obstacle_position_p2g->timestamp = mavlink_msg_obstacle_position_p2g_get_timestamp(msg);
	obstacle_position_p2g->obstacle_x = mavlink_msg_obstacle_position_p2g_get_obstacle_x(msg);
	obstacle_position_p2g->obstacle_y = mavlink_msg_obstacle_position_p2g_get_obstacle_y(msg);
	obstacle_position_p2g->obstacle_z = mavlink_msg_obstacle_position_p2g_get_obstacle_z(msg);
	obstacle_position_p2g->obstacle_valid = mavlink_msg_obstacle_position_p2g_get_obstacle_valid(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_LEN? msg->len : MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_LEN;
        memset(obstacle_position_p2g, 0, MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_LEN);
	memcpy(obstacle_position_p2g, _MAV_PAYLOAD(msg), len);
#endif
}
