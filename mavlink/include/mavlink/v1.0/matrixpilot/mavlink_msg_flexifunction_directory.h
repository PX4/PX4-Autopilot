// MESSAGE FLEXIFUNCTION_DIRECTORY PACKING

#define MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY 155

typedef struct __mavlink_flexifunction_directory_t
{
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 uint8_t directory_type; ///< 0=inputs, 1=outputs
 uint8_t start_index; ///< index of first directory entry to write
 uint8_t count; ///< count of directory entries to write
 int8_t directory_data[48]; ///< Settings data
} mavlink_flexifunction_directory_t;

#define MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_LEN 53
#define MAVLINK_MSG_ID_155_LEN 53

#define MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_CRC 12
#define MAVLINK_MSG_ID_155_CRC 12

#define MAVLINK_MSG_FLEXIFUNCTION_DIRECTORY_FIELD_DIRECTORY_DATA_LEN 48

#define MAVLINK_MESSAGE_INFO_FLEXIFUNCTION_DIRECTORY { \
	"FLEXIFUNCTION_DIRECTORY", \
	6, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_flexifunction_directory_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_flexifunction_directory_t, target_component) }, \
         { "directory_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_flexifunction_directory_t, directory_type) }, \
         { "start_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_flexifunction_directory_t, start_index) }, \
         { "count", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_flexifunction_directory_t, count) }, \
         { "directory_data", NULL, MAVLINK_TYPE_INT8_T, 48, 5, offsetof(mavlink_flexifunction_directory_t, directory_data) }, \
         } \
}


/**
 * @brief Pack a flexifunction_directory message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param directory_type 0=inputs, 1=outputs
 * @param start_index index of first directory entry to write
 * @param count count of directory entries to write
 * @param directory_data Settings data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flexifunction_directory_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint8_t directory_type, uint8_t start_index, uint8_t count, const int8_t *directory_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, directory_type);
	_mav_put_uint8_t(buf, 3, start_index);
	_mav_put_uint8_t(buf, 4, count);
	_mav_put_int8_t_array(buf, 5, directory_data, 48);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_LEN);
#else
	mavlink_flexifunction_directory_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.directory_type = directory_type;
	packet.start_index = start_index;
	packet.count = count;
	mav_array_memcpy(packet.directory_data, directory_data, sizeof(int8_t)*48);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_LEN);
#endif
}

/**
 * @brief Pack a flexifunction_directory message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param directory_type 0=inputs, 1=outputs
 * @param start_index index of first directory entry to write
 * @param count count of directory entries to write
 * @param directory_data Settings data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flexifunction_directory_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint8_t directory_type,uint8_t start_index,uint8_t count,const int8_t *directory_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, directory_type);
	_mav_put_uint8_t(buf, 3, start_index);
	_mav_put_uint8_t(buf, 4, count);
	_mav_put_int8_t_array(buf, 5, directory_data, 48);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_LEN);
#else
	mavlink_flexifunction_directory_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.directory_type = directory_type;
	packet.start_index = start_index;
	packet.count = count;
	mav_array_memcpy(packet.directory_data, directory_data, sizeof(int8_t)*48);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_LEN);
#endif
}

/**
 * @brief Encode a flexifunction_directory struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param flexifunction_directory C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flexifunction_directory_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_flexifunction_directory_t* flexifunction_directory)
{
	return mavlink_msg_flexifunction_directory_pack(system_id, component_id, msg, flexifunction_directory->target_system, flexifunction_directory->target_component, flexifunction_directory->directory_type, flexifunction_directory->start_index, flexifunction_directory->count, flexifunction_directory->directory_data);
}

/**
 * @brief Encode a flexifunction_directory struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param flexifunction_directory C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flexifunction_directory_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_flexifunction_directory_t* flexifunction_directory)
{
	return mavlink_msg_flexifunction_directory_pack_chan(system_id, component_id, chan, msg, flexifunction_directory->target_system, flexifunction_directory->target_component, flexifunction_directory->directory_type, flexifunction_directory->start_index, flexifunction_directory->count, flexifunction_directory->directory_data);
}

/**
 * @brief Send a flexifunction_directory message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param directory_type 0=inputs, 1=outputs
 * @param start_index index of first directory entry to write
 * @param count count of directory entries to write
 * @param directory_data Settings data
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_flexifunction_directory_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t directory_type, uint8_t start_index, uint8_t count, const int8_t *directory_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, directory_type);
	_mav_put_uint8_t(buf, 3, start_index);
	_mav_put_uint8_t(buf, 4, count);
	_mav_put_int8_t_array(buf, 5, directory_data, 48);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY, buf, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY, buf, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_LEN);
#endif
#else
	mavlink_flexifunction_directory_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.directory_type = directory_type;
	packet.start_index = start_index;
	packet.count = count;
	mav_array_memcpy(packet.directory_data, directory_data, sizeof(int8_t)*48);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY, (const char *)&packet, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY, (const char *)&packet, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_flexifunction_directory_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t directory_type, uint8_t start_index, uint8_t count, const int8_t *directory_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, directory_type);
	_mav_put_uint8_t(buf, 3, start_index);
	_mav_put_uint8_t(buf, 4, count);
	_mav_put_int8_t_array(buf, 5, directory_data, 48);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY, buf, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY, buf, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_LEN);
#endif
#else
	mavlink_flexifunction_directory_t *packet = (mavlink_flexifunction_directory_t *)msgbuf;
	packet->target_system = target_system;
	packet->target_component = target_component;
	packet->directory_type = directory_type;
	packet->start_index = start_index;
	packet->count = count;
	mav_array_memcpy(packet->directory_data, directory_data, sizeof(int8_t)*48);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY, (const char *)packet, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY, (const char *)packet, MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE FLEXIFUNCTION_DIRECTORY UNPACKING


/**
 * @brief Get field target_system from flexifunction_directory message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_flexifunction_directory_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from flexifunction_directory message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_flexifunction_directory_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field directory_type from flexifunction_directory message
 *
 * @return 0=inputs, 1=outputs
 */
static inline uint8_t mavlink_msg_flexifunction_directory_get_directory_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field start_index from flexifunction_directory message
 *
 * @return index of first directory entry to write
 */
static inline uint8_t mavlink_msg_flexifunction_directory_get_start_index(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field count from flexifunction_directory message
 *
 * @return count of directory entries to write
 */
static inline uint8_t mavlink_msg_flexifunction_directory_get_count(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field directory_data from flexifunction_directory message
 *
 * @return Settings data
 */
static inline uint16_t mavlink_msg_flexifunction_directory_get_directory_data(const mavlink_message_t* msg, int8_t *directory_data)
{
	return _MAV_RETURN_int8_t_array(msg, directory_data, 48,  5);
}

/**
 * @brief Decode a flexifunction_directory message into a struct
 *
 * @param msg The message to decode
 * @param flexifunction_directory C-struct to decode the message contents into
 */
static inline void mavlink_msg_flexifunction_directory_decode(const mavlink_message_t* msg, mavlink_flexifunction_directory_t* flexifunction_directory)
{
#if MAVLINK_NEED_BYTE_SWAP
	flexifunction_directory->target_system = mavlink_msg_flexifunction_directory_get_target_system(msg);
	flexifunction_directory->target_component = mavlink_msg_flexifunction_directory_get_target_component(msg);
	flexifunction_directory->directory_type = mavlink_msg_flexifunction_directory_get_directory_type(msg);
	flexifunction_directory->start_index = mavlink_msg_flexifunction_directory_get_start_index(msg);
	flexifunction_directory->count = mavlink_msg_flexifunction_directory_get_count(msg);
	mavlink_msg_flexifunction_directory_get_directory_data(msg, flexifunction_directory->directory_data);
#else
	memcpy(flexifunction_directory, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_FLEXIFUNCTION_DIRECTORY_LEN);
#endif
}
