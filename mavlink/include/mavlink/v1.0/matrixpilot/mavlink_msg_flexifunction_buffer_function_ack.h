// MESSAGE FLEXIFUNCTION_BUFFER_FUNCTION_ACK PACKING

#define MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK 153

typedef struct __mavlink_flexifunction_buffer_function_ack_t
{
 uint16_t func_index; ///< Function index
 uint16_t result; ///< result of acknowledge, 0=fail, 1=good
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
} mavlink_flexifunction_buffer_function_ack_t;

#define MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_LEN 6
#define MAVLINK_MSG_ID_153_LEN 6

#define MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_CRC 109
#define MAVLINK_MSG_ID_153_CRC 109



#define MAVLINK_MESSAGE_INFO_FLEXIFUNCTION_BUFFER_FUNCTION_ACK { \
	"FLEXIFUNCTION_BUFFER_FUNCTION_ACK", \
	4, \
	{  { "func_index", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_flexifunction_buffer_function_ack_t, func_index) }, \
         { "result", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_flexifunction_buffer_function_ack_t, result) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_flexifunction_buffer_function_ack_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_flexifunction_buffer_function_ack_t, target_component) }, \
         } \
}


/**
 * @brief Pack a flexifunction_buffer_function_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param func_index Function index
 * @param result result of acknowledge, 0=fail, 1=good
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flexifunction_buffer_function_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint16_t func_index, uint16_t result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_LEN];
	_mav_put_uint16_t(buf, 0, func_index);
	_mav_put_uint16_t(buf, 2, result);
	_mav_put_uint8_t(buf, 4, target_system);
	_mav_put_uint8_t(buf, 5, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_LEN);
#else
	mavlink_flexifunction_buffer_function_ack_t packet;
	packet.func_index = func_index;
	packet.result = result;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_LEN);
#endif
}

/**
 * @brief Pack a flexifunction_buffer_function_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param func_index Function index
 * @param result result of acknowledge, 0=fail, 1=good
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flexifunction_buffer_function_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint16_t func_index,uint16_t result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_LEN];
	_mav_put_uint16_t(buf, 0, func_index);
	_mav_put_uint16_t(buf, 2, result);
	_mav_put_uint8_t(buf, 4, target_system);
	_mav_put_uint8_t(buf, 5, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_LEN);
#else
	mavlink_flexifunction_buffer_function_ack_t packet;
	packet.func_index = func_index;
	packet.result = result;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_LEN);
#endif
}

/**
 * @brief Encode a flexifunction_buffer_function_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param flexifunction_buffer_function_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flexifunction_buffer_function_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_flexifunction_buffer_function_ack_t* flexifunction_buffer_function_ack)
{
	return mavlink_msg_flexifunction_buffer_function_ack_pack(system_id, component_id, msg, flexifunction_buffer_function_ack->target_system, flexifunction_buffer_function_ack->target_component, flexifunction_buffer_function_ack->func_index, flexifunction_buffer_function_ack->result);
}

/**
 * @brief Encode a flexifunction_buffer_function_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param flexifunction_buffer_function_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flexifunction_buffer_function_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_flexifunction_buffer_function_ack_t* flexifunction_buffer_function_ack)
{
	return mavlink_msg_flexifunction_buffer_function_ack_pack_chan(system_id, component_id, chan, msg, flexifunction_buffer_function_ack->target_system, flexifunction_buffer_function_ack->target_component, flexifunction_buffer_function_ack->func_index, flexifunction_buffer_function_ack->result);
}

/**
 * @brief Send a flexifunction_buffer_function_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param func_index Function index
 * @param result result of acknowledge, 0=fail, 1=good
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_flexifunction_buffer_function_ack_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t func_index, uint16_t result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_LEN];
	_mav_put_uint16_t(buf, 0, func_index);
	_mav_put_uint16_t(buf, 2, result);
	_mav_put_uint8_t(buf, 4, target_system);
	_mav_put_uint8_t(buf, 5, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK, buf, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK, buf, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_LEN);
#endif
#else
	mavlink_flexifunction_buffer_function_ack_t packet;
	packet.func_index = func_index;
	packet.result = result;
	packet.target_system = target_system;
	packet.target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK, (const char *)&packet, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK, (const char *)&packet, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_flexifunction_buffer_function_ack_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint16_t func_index, uint16_t result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, func_index);
	_mav_put_uint16_t(buf, 2, result);
	_mav_put_uint8_t(buf, 4, target_system);
	_mav_put_uint8_t(buf, 5, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK, buf, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK, buf, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_LEN);
#endif
#else
	mavlink_flexifunction_buffer_function_ack_t *packet = (mavlink_flexifunction_buffer_function_ack_t *)msgbuf;
	packet->func_index = func_index;
	packet->result = result;
	packet->target_system = target_system;
	packet->target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK, (const char *)packet, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK, (const char *)packet, MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE FLEXIFUNCTION_BUFFER_FUNCTION_ACK UNPACKING


/**
 * @brief Get field target_system from flexifunction_buffer_function_ack message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_flexifunction_buffer_function_ack_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field target_component from flexifunction_buffer_function_ack message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_flexifunction_buffer_function_ack_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field func_index from flexifunction_buffer_function_ack message
 *
 * @return Function index
 */
static inline uint16_t mavlink_msg_flexifunction_buffer_function_ack_get_func_index(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field result from flexifunction_buffer_function_ack message
 *
 * @return result of acknowledge, 0=fail, 1=good
 */
static inline uint16_t mavlink_msg_flexifunction_buffer_function_ack_get_result(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Decode a flexifunction_buffer_function_ack message into a struct
 *
 * @param msg The message to decode
 * @param flexifunction_buffer_function_ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_flexifunction_buffer_function_ack_decode(const mavlink_message_t* msg, mavlink_flexifunction_buffer_function_ack_t* flexifunction_buffer_function_ack)
{
#if MAVLINK_NEED_BYTE_SWAP
	flexifunction_buffer_function_ack->func_index = mavlink_msg_flexifunction_buffer_function_ack_get_func_index(msg);
	flexifunction_buffer_function_ack->result = mavlink_msg_flexifunction_buffer_function_ack_get_result(msg);
	flexifunction_buffer_function_ack->target_system = mavlink_msg_flexifunction_buffer_function_ack_get_target_system(msg);
	flexifunction_buffer_function_ack->target_component = mavlink_msg_flexifunction_buffer_function_ack_get_target_component(msg);
#else
	memcpy(flexifunction_buffer_function_ack, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_LEN);
#endif
}
