// MESSAGE SET_POSITION_CONTROL_OFFSET PACKING

#define MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET 160

typedef struct __mavlink_set_position_control_offset_t
{
 float x; ///< x position offset
 float y; ///< y position offset
 float z; ///< z position offset
 float yaw; ///< yaw orientation offset in radians, 0 = NORTH
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
} mavlink_set_position_control_offset_t;

#define MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_LEN 18
#define MAVLINK_MSG_ID_160_LEN 18

#define MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_CRC 22
#define MAVLINK_MSG_ID_160_CRC 22



#define MAVLINK_MESSAGE_INFO_SET_POSITION_CONTROL_OFFSET { \
	"SET_POSITION_CONTROL_OFFSET", \
	6, \
	{  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_position_control_offset_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_position_control_offset_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_position_control_offset_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_position_control_offset_t, yaw) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_set_position_control_offset_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_set_position_control_offset_t, target_component) }, \
         } \
}


/**
 * @brief Pack a set_position_control_offset message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param x x position offset
 * @param y y position offset
 * @param z z position offset
 * @param yaw yaw orientation offset in radians, 0 = NORTH
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_position_control_offset_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, yaw);
	_mav_put_uint8_t(buf, 16, target_system);
	_mav_put_uint8_t(buf, 17, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_LEN);
#else
	mavlink_set_position_control_offset_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_LEN, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_LEN);
#endif
}

/**
 * @brief Pack a set_position_control_offset message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param x x position offset
 * @param y y position offset
 * @param z z position offset
 * @param yaw yaw orientation offset in radians, 0 = NORTH
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_position_control_offset_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,float x,float y,float z,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, yaw);
	_mav_put_uint8_t(buf, 16, target_system);
	_mav_put_uint8_t(buf, 17, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_LEN);
#else
	mavlink_set_position_control_offset_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_LEN, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_LEN);
#endif
}

/**
 * @brief Encode a set_position_control_offset struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_position_control_offset C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_position_control_offset_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_position_control_offset_t* set_position_control_offset)
{
	return mavlink_msg_set_position_control_offset_pack(system_id, component_id, msg, set_position_control_offset->target_system, set_position_control_offset->target_component, set_position_control_offset->x, set_position_control_offset->y, set_position_control_offset->z, set_position_control_offset->yaw);
}

/**
 * @brief Encode a set_position_control_offset struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_position_control_offset C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_position_control_offset_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_position_control_offset_t* set_position_control_offset)
{
	return mavlink_msg_set_position_control_offset_pack_chan(system_id, component_id, chan, msg, set_position_control_offset->target_system, set_position_control_offset->target_component, set_position_control_offset->x, set_position_control_offset->y, set_position_control_offset->z, set_position_control_offset->yaw);
}

/**
 * @brief Send a set_position_control_offset message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param x x position offset
 * @param y y position offset
 * @param z z position offset
 * @param yaw yaw orientation offset in radians, 0 = NORTH
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_position_control_offset_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, yaw);
	_mav_put_uint8_t(buf, 16, target_system);
	_mav_put_uint8_t(buf, 17, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET, buf, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_LEN, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET, buf, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_LEN);
#endif
#else
	mavlink_set_position_control_offset_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.target_system = target_system;
	packet.target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET, (const char *)&packet, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_LEN, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET, (const char *)&packet, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_position_control_offset_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, yaw);
	_mav_put_uint8_t(buf, 16, target_system);
	_mav_put_uint8_t(buf, 17, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET, buf, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_LEN, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET, buf, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_LEN);
#endif
#else
	mavlink_set_position_control_offset_t *packet = (mavlink_set_position_control_offset_t *)msgbuf;
	packet->x = x;
	packet->y = y;
	packet->z = z;
	packet->yaw = yaw;
	packet->target_system = target_system;
	packet->target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET, (const char *)packet, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_LEN, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET, (const char *)packet, MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SET_POSITION_CONTROL_OFFSET UNPACKING


/**
 * @brief Get field target_system from set_position_control_offset message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_set_position_control_offset_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field target_component from set_position_control_offset message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_set_position_control_offset_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field x from set_position_control_offset message
 *
 * @return x position offset
 */
static inline float mavlink_msg_set_position_control_offset_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from set_position_control_offset message
 *
 * @return y position offset
 */
static inline float mavlink_msg_set_position_control_offset_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from set_position_control_offset message
 *
 * @return z position offset
 */
static inline float mavlink_msg_set_position_control_offset_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from set_position_control_offset message
 *
 * @return yaw orientation offset in radians, 0 = NORTH
 */
static inline float mavlink_msg_set_position_control_offset_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a set_position_control_offset message into a struct
 *
 * @param msg The message to decode
 * @param set_position_control_offset C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_position_control_offset_decode(const mavlink_message_t* msg, mavlink_set_position_control_offset_t* set_position_control_offset)
{
#if MAVLINK_NEED_BYTE_SWAP
	set_position_control_offset->x = mavlink_msg_set_position_control_offset_get_x(msg);
	set_position_control_offset->y = mavlink_msg_set_position_control_offset_get_y(msg);
	set_position_control_offset->z = mavlink_msg_set_position_control_offset_get_z(msg);
	set_position_control_offset->yaw = mavlink_msg_set_position_control_offset_get_yaw(msg);
	set_position_control_offset->target_system = mavlink_msg_set_position_control_offset_get_target_system(msg);
	set_position_control_offset->target_component = mavlink_msg_set_position_control_offset_get_target_component(msg);
#else
	memcpy(set_position_control_offset, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SET_POSITION_CONTROL_OFFSET_LEN);
#endif
}
