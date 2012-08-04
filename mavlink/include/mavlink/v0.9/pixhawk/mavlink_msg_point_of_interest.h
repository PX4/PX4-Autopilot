// MESSAGE POINT_OF_INTEREST PACKING

#define MAVLINK_MSG_ID_POINT_OF_INTEREST 191

typedef struct __mavlink_point_of_interest_t
{
 uint8_t type; ///< 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 uint8_t color; ///< 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 uint8_t coordinate_system; ///< 0: global, 1:local
 uint16_t timeout; ///< 0: no timeout, >1: timeout in seconds
 float x; ///< X Position
 float y; ///< Y Position
 float z; ///< Z Position
 char name[26]; ///< POI name
} mavlink_point_of_interest_t;

#define MAVLINK_MSG_ID_POINT_OF_INTEREST_LEN 43
#define MAVLINK_MSG_ID_191_LEN 43

#define MAVLINK_MSG_POINT_OF_INTEREST_FIELD_NAME_LEN 26

#define MAVLINK_MESSAGE_INFO_POINT_OF_INTEREST { \
	"POINT_OF_INTEREST", \
	8, \
	{  { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_point_of_interest_t, type) }, \
         { "color", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_point_of_interest_t, color) }, \
         { "coordinate_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_point_of_interest_t, coordinate_system) }, \
         { "timeout", NULL, MAVLINK_TYPE_UINT16_T, 0, 3, offsetof(mavlink_point_of_interest_t, timeout) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 5, offsetof(mavlink_point_of_interest_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 9, offsetof(mavlink_point_of_interest_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 13, offsetof(mavlink_point_of_interest_t, z) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 26, 17, offsetof(mavlink_point_of_interest_t, name) }, \
         } \
}


/**
 * @brief Pack a point_of_interest message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 * @param color 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 * @param coordinate_system 0: global, 1:local
 * @param timeout 0: no timeout, >1: timeout in seconds
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @param name POI name
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_point_of_interest_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t type, uint8_t color, uint8_t coordinate_system, uint16_t timeout, float x, float y, float z, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[43];
	_mav_put_uint8_t(buf, 0, type);
	_mav_put_uint8_t(buf, 1, color);
	_mav_put_uint8_t(buf, 2, coordinate_system);
	_mav_put_uint16_t(buf, 3, timeout);
	_mav_put_float(buf, 5, x);
	_mav_put_float(buf, 9, y);
	_mav_put_float(buf, 13, z);
	_mav_put_char_array(buf, 17, name, 26);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 43);
#else
	mavlink_point_of_interest_t packet;
	packet.type = type;
	packet.color = color;
	packet.coordinate_system = coordinate_system;
	packet.timeout = timeout;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	mav_array_memcpy(packet.name, name, sizeof(char)*26);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 43);
#endif

	msg->msgid = MAVLINK_MSG_ID_POINT_OF_INTEREST;
	return mavlink_finalize_message(msg, system_id, component_id, 43);
}

/**
 * @brief Pack a point_of_interest message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param type 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 * @param color 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 * @param coordinate_system 0: global, 1:local
 * @param timeout 0: no timeout, >1: timeout in seconds
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @param name POI name
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_point_of_interest_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t type,uint8_t color,uint8_t coordinate_system,uint16_t timeout,float x,float y,float z,const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[43];
	_mav_put_uint8_t(buf, 0, type);
	_mav_put_uint8_t(buf, 1, color);
	_mav_put_uint8_t(buf, 2, coordinate_system);
	_mav_put_uint16_t(buf, 3, timeout);
	_mav_put_float(buf, 5, x);
	_mav_put_float(buf, 9, y);
	_mav_put_float(buf, 13, z);
	_mav_put_char_array(buf, 17, name, 26);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 43);
#else
	mavlink_point_of_interest_t packet;
	packet.type = type;
	packet.color = color;
	packet.coordinate_system = coordinate_system;
	packet.timeout = timeout;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	mav_array_memcpy(packet.name, name, sizeof(char)*26);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 43);
#endif

	msg->msgid = MAVLINK_MSG_ID_POINT_OF_INTEREST;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 43);
}

/**
 * @brief Encode a point_of_interest struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param point_of_interest C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_point_of_interest_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_point_of_interest_t* point_of_interest)
{
	return mavlink_msg_point_of_interest_pack(system_id, component_id, msg, point_of_interest->type, point_of_interest->color, point_of_interest->coordinate_system, point_of_interest->timeout, point_of_interest->x, point_of_interest->y, point_of_interest->z, point_of_interest->name);
}

/**
 * @brief Send a point_of_interest message
 * @param chan MAVLink channel to send the message
 *
 * @param type 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 * @param color 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 * @param coordinate_system 0: global, 1:local
 * @param timeout 0: no timeout, >1: timeout in seconds
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @param name POI name
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_point_of_interest_send(mavlink_channel_t chan, uint8_t type, uint8_t color, uint8_t coordinate_system, uint16_t timeout, float x, float y, float z, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[43];
	_mav_put_uint8_t(buf, 0, type);
	_mav_put_uint8_t(buf, 1, color);
	_mav_put_uint8_t(buf, 2, coordinate_system);
	_mav_put_uint16_t(buf, 3, timeout);
	_mav_put_float(buf, 5, x);
	_mav_put_float(buf, 9, y);
	_mav_put_float(buf, 13, z);
	_mav_put_char_array(buf, 17, name, 26);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POINT_OF_INTEREST, buf, 43);
#else
	mavlink_point_of_interest_t packet;
	packet.type = type;
	packet.color = color;
	packet.coordinate_system = coordinate_system;
	packet.timeout = timeout;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	mav_array_memcpy(packet.name, name, sizeof(char)*26);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POINT_OF_INTEREST, (const char *)&packet, 43);
#endif
}

#endif

// MESSAGE POINT_OF_INTEREST UNPACKING


/**
 * @brief Get field type from point_of_interest message
 *
 * @return 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 */
static inline uint8_t mavlink_msg_point_of_interest_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field color from point_of_interest message
 *
 * @return 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 */
static inline uint8_t mavlink_msg_point_of_interest_get_color(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field coordinate_system from point_of_interest message
 *
 * @return 0: global, 1:local
 */
static inline uint8_t mavlink_msg_point_of_interest_get_coordinate_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field timeout from point_of_interest message
 *
 * @return 0: no timeout, >1: timeout in seconds
 */
static inline uint16_t mavlink_msg_point_of_interest_get_timeout(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  3);
}

/**
 * @brief Get field x from point_of_interest message
 *
 * @return X Position
 */
static inline float mavlink_msg_point_of_interest_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  5);
}

/**
 * @brief Get field y from point_of_interest message
 *
 * @return Y Position
 */
static inline float mavlink_msg_point_of_interest_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  9);
}

/**
 * @brief Get field z from point_of_interest message
 *
 * @return Z Position
 */
static inline float mavlink_msg_point_of_interest_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  13);
}

/**
 * @brief Get field name from point_of_interest message
 *
 * @return POI name
 */
static inline uint16_t mavlink_msg_point_of_interest_get_name(const mavlink_message_t* msg, char *name)
{
	return _MAV_RETURN_char_array(msg, name, 26,  17);
}

/**
 * @brief Decode a point_of_interest message into a struct
 *
 * @param msg The message to decode
 * @param point_of_interest C-struct to decode the message contents into
 */
static inline void mavlink_msg_point_of_interest_decode(const mavlink_message_t* msg, mavlink_point_of_interest_t* point_of_interest)
{
#if MAVLINK_NEED_BYTE_SWAP
	point_of_interest->type = mavlink_msg_point_of_interest_get_type(msg);
	point_of_interest->color = mavlink_msg_point_of_interest_get_color(msg);
	point_of_interest->coordinate_system = mavlink_msg_point_of_interest_get_coordinate_system(msg);
	point_of_interest->timeout = mavlink_msg_point_of_interest_get_timeout(msg);
	point_of_interest->x = mavlink_msg_point_of_interest_get_x(msg);
	point_of_interest->y = mavlink_msg_point_of_interest_get_y(msg);
	point_of_interest->z = mavlink_msg_point_of_interest_get_z(msg);
	mavlink_msg_point_of_interest_get_name(msg, point_of_interest->name);
#else
	memcpy(point_of_interest, _MAV_PAYLOAD(msg), 43);
#endif
}
