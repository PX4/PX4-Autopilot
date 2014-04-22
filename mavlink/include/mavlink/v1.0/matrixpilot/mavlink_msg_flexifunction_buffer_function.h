// MESSAGE FLEXIFUNCTION_BUFFER_FUNCTION PACKING

#define MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION 152

typedef struct __mavlink_flexifunction_buffer_function_t
{
 uint16_t func_index; ///< Function index
 uint16_t func_count; ///< Total count of functions
 uint16_t data_address; ///< Address in the flexifunction data, Set to 0xFFFF to use address in target memory
 uint16_t data_size; ///< Size of the 
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 int8_t data[48]; ///< Settings data
} mavlink_flexifunction_buffer_function_t;

#define MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_LEN 58
#define MAVLINK_MSG_ID_152_LEN 58

#define MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_CRC 101
#define MAVLINK_MSG_ID_152_CRC 101

#define MAVLINK_MSG_FLEXIFUNCTION_BUFFER_FUNCTION_FIELD_DATA_LEN 48

#define MAVLINK_MESSAGE_INFO_FLEXIFUNCTION_BUFFER_FUNCTION { \
	"FLEXIFUNCTION_BUFFER_FUNCTION", \
	7, \
	{  { "func_index", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_flexifunction_buffer_function_t, func_index) }, \
         { "func_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_flexifunction_buffer_function_t, func_count) }, \
         { "data_address", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_flexifunction_buffer_function_t, data_address) }, \
         { "data_size", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_flexifunction_buffer_function_t, data_size) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_flexifunction_buffer_function_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_flexifunction_buffer_function_t, target_component) }, \
         { "data", NULL, MAVLINK_TYPE_INT8_T, 48, 10, offsetof(mavlink_flexifunction_buffer_function_t, data) }, \
         } \
}


/**
 * @brief Pack a flexifunction_buffer_function message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param func_index Function index
 * @param func_count Total count of functions
 * @param data_address Address in the flexifunction data, Set to 0xFFFF to use address in target memory
 * @param data_size Size of the 
 * @param data Settings data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flexifunction_buffer_function_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint16_t func_index, uint16_t func_count, uint16_t data_address, uint16_t data_size, const int8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_LEN];
	_mav_put_uint16_t(buf, 0, func_index);
	_mav_put_uint16_t(buf, 2, func_count);
	_mav_put_uint16_t(buf, 4, data_address);
	_mav_put_uint16_t(buf, 6, data_size);
	_mav_put_uint8_t(buf, 8, target_system);
	_mav_put_uint8_t(buf, 9, target_component);
	_mav_put_int8_t_array(buf, 10, data, 48);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_LEN);
#else
	mavlink_flexifunction_buffer_function_t packet;
	packet.func_index = func_index;
	packet.func_count = func_count;
	packet.data_address = data_address;
	packet.data_size = data_size;
	packet.target_system = target_system;
	packet.target_component = target_component;
	mav_array_memcpy(packet.data, data, sizeof(int8_t)*48);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_LEN);
#endif
}

/**
 * @brief Pack a flexifunction_buffer_function message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param func_index Function index
 * @param func_count Total count of functions
 * @param data_address Address in the flexifunction data, Set to 0xFFFF to use address in target memory
 * @param data_size Size of the 
 * @param data Settings data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flexifunction_buffer_function_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint16_t func_index,uint16_t func_count,uint16_t data_address,uint16_t data_size,const int8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_LEN];
	_mav_put_uint16_t(buf, 0, func_index);
	_mav_put_uint16_t(buf, 2, func_count);
	_mav_put_uint16_t(buf, 4, data_address);
	_mav_put_uint16_t(buf, 6, data_size);
	_mav_put_uint8_t(buf, 8, target_system);
	_mav_put_uint8_t(buf, 9, target_component);
	_mav_put_int8_t_array(buf, 10, data, 48);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_LEN);
#else
	mavlink_flexifunction_buffer_function_t packet;
	packet.func_index = func_index;
	packet.func_count = func_count;
	packet.data_address = data_address;
	packet.data_size = data_size;
	packet.target_system = target_system;
	packet.target_component = target_component;
	mav_array_memcpy(packet.data, data, sizeof(int8_t)*48);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_LEN);
#endif
}

/**
 * @brief Encode a flexifunction_buffer_function struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param flexifunction_buffer_function C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flexifunction_buffer_function_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_flexifunction_buffer_function_t* flexifunction_buffer_function)
{
	return mavlink_msg_flexifunction_buffer_function_pack(system_id, component_id, msg, flexifunction_buffer_function->target_system, flexifunction_buffer_function->target_component, flexifunction_buffer_function->func_index, flexifunction_buffer_function->func_count, flexifunction_buffer_function->data_address, flexifunction_buffer_function->data_size, flexifunction_buffer_function->data);
}

/**
 * @brief Encode a flexifunction_buffer_function struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param flexifunction_buffer_function C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flexifunction_buffer_function_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_flexifunction_buffer_function_t* flexifunction_buffer_function)
{
	return mavlink_msg_flexifunction_buffer_function_pack_chan(system_id, component_id, chan, msg, flexifunction_buffer_function->target_system, flexifunction_buffer_function->target_component, flexifunction_buffer_function->func_index, flexifunction_buffer_function->func_count, flexifunction_buffer_function->data_address, flexifunction_buffer_function->data_size, flexifunction_buffer_function->data);
}

/**
 * @brief Send a flexifunction_buffer_function message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param func_index Function index
 * @param func_count Total count of functions
 * @param data_address Address in the flexifunction data, Set to 0xFFFF to use address in target memory
 * @param data_size Size of the 
 * @param data Settings data
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_flexifunction_buffer_function_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t func_index, uint16_t func_count, uint16_t data_address, uint16_t data_size, const int8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_LEN];
	_mav_put_uint16_t(buf, 0, func_index);
	_mav_put_uint16_t(buf, 2, func_count);
	_mav_put_uint16_t(buf, 4, data_address);
	_mav_put_uint16_t(buf, 6, data_size);
	_mav_put_uint8_t(buf, 8, target_system);
	_mav_put_uint8_t(buf, 9, target_component);
	_mav_put_int8_t_array(buf, 10, data, 48);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION, buf, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION, buf, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_LEN);
#endif
#else
	mavlink_flexifunction_buffer_function_t packet;
	packet.func_index = func_index;
	packet.func_count = func_count;
	packet.data_address = data_address;
	packet.data_size = data_size;
	packet.target_system = target_system;
	packet.target_component = target_component;
	mav_array_memcpy(packet.data, data, sizeof(int8_t)*48);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION, (const char *)&packet, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION, (const char *)&packet, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_flexifunction_buffer_function_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint16_t func_index, uint16_t func_count, uint16_t data_address, uint16_t data_size, const int8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, func_index);
	_mav_put_uint16_t(buf, 2, func_count);
	_mav_put_uint16_t(buf, 4, data_address);
	_mav_put_uint16_t(buf, 6, data_size);
	_mav_put_uint8_t(buf, 8, target_system);
	_mav_put_uint8_t(buf, 9, target_component);
	_mav_put_int8_t_array(buf, 10, data, 48);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION, buf, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION, buf, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_LEN);
#endif
#else
	mavlink_flexifunction_buffer_function_t *packet = (mavlink_flexifunction_buffer_function_t *)msgbuf;
	packet->func_index = func_index;
	packet->func_count = func_count;
	packet->data_address = data_address;
	packet->data_size = data_size;
	packet->target_system = target_system;
	packet->target_component = target_component;
	mav_array_memcpy(packet->data, data, sizeof(int8_t)*48);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION, (const char *)packet, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION, (const char *)packet, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE FLEXIFUNCTION_BUFFER_FUNCTION UNPACKING


/**
 * @brief Get field target_system from flexifunction_buffer_function message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_flexifunction_buffer_function_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field target_component from flexifunction_buffer_function message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_flexifunction_buffer_function_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field func_index from flexifunction_buffer_function message
 *
 * @return Function index
 */
static inline uint16_t mavlink_msg_flexifunction_buffer_function_get_func_index(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field func_count from flexifunction_buffer_function message
 *
 * @return Total count of functions
 */
static inline uint16_t mavlink_msg_flexifunction_buffer_function_get_func_count(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field data_address from flexifunction_buffer_function message
 *
 * @return Address in the flexifunction data, Set to 0xFFFF to use address in target memory
 */
static inline uint16_t mavlink_msg_flexifunction_buffer_function_get_data_address(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field data_size from flexifunction_buffer_function message
 *
 * @return Size of the 
 */
static inline uint16_t mavlink_msg_flexifunction_buffer_function_get_data_size(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field data from flexifunction_buffer_function message
 *
 * @return Settings data
 */
static inline uint16_t mavlink_msg_flexifunction_buffer_function_get_data(const mavlink_message_t* msg, int8_t *data)
{
	return _MAV_RETURN_int8_t_array(msg, data, 48,  10);
}

/**
 * @brief Decode a flexifunction_buffer_function message into a struct
 *
 * @param msg The message to decode
 * @param flexifunction_buffer_function C-struct to decode the message contents into
 */
static inline void mavlink_msg_flexifunction_buffer_function_decode(const mavlink_message_t* msg, mavlink_flexifunction_buffer_function_t* flexifunction_buffer_function)
{
#if MAVLINK_NEED_BYTE_SWAP
	flexifunction_buffer_function->func_index = mavlink_msg_flexifunction_buffer_function_get_func_index(msg);
	flexifunction_buffer_function->func_count = mavlink_msg_flexifunction_buffer_function_get_func_count(msg);
	flexifunction_buffer_function->data_address = mavlink_msg_flexifunction_buffer_function_get_data_address(msg);
	flexifunction_buffer_function->data_size = mavlink_msg_flexifunction_buffer_function_get_data_size(msg);
	flexifunction_buffer_function->target_system = mavlink_msg_flexifunction_buffer_function_get_target_system(msg);
	flexifunction_buffer_function->target_component = mavlink_msg_flexifunction_buffer_function_get_target_component(msg);
	mavlink_msg_flexifunction_buffer_function_get_data(msg, flexifunction_buffer_function->data);
#else
	memcpy(flexifunction_buffer_function, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_LEN);
#endif
}
