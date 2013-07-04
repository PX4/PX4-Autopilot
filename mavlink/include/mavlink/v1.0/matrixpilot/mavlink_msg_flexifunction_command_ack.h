// MESSAGE FLEXIFUNCTION_COMMAND_ACK PACKING

#define MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK 158

typedef struct __mavlink_flexifunction_command_ack_t
{
 uint16_t command_type; ///< Command acknowledged
 uint16_t result; ///< result of acknowledge
} mavlink_flexifunction_command_ack_t;

#define MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK_LEN 4
#define MAVLINK_MSG_ID_158_LEN 4

#define MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK_CRC 208
#define MAVLINK_MSG_ID_158_CRC 208



#define MAVLINK_MESSAGE_INFO_FLEXIFUNCTION_COMMAND_ACK { \
	"FLEXIFUNCTION_COMMAND_ACK", \
	2, \
	{  { "command_type", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_flexifunction_command_ack_t, command_type) }, \
         { "result", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_flexifunction_command_ack_t, result) }, \
         } \
}


/**
 * @brief Pack a flexifunction_command_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param command_type Command acknowledged
 * @param result result of acknowledge
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flexifunction_command_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t command_type, uint16_t result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK_LEN];
	_mav_put_uint16_t(buf, 0, command_type);
	_mav_put_uint16_t(buf, 2, result);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK_LEN);
#else
	mavlink_flexifunction_command_ack_t packet;
	packet.command_type = command_type;
	packet.result = result;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK_LEN);
#endif
}

/**
 * @brief Pack a flexifunction_command_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param command_type Command acknowledged
 * @param result result of acknowledge
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flexifunction_command_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t command_type,uint16_t result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK_LEN];
	_mav_put_uint16_t(buf, 0, command_type);
	_mav_put_uint16_t(buf, 2, result);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK_LEN);
#else
	mavlink_flexifunction_command_ack_t packet;
	packet.command_type = command_type;
	packet.result = result;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK_LEN);
#endif
}

/**
 * @brief Encode a flexifunction_command_ack struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param flexifunction_command_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flexifunction_command_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_flexifunction_command_ack_t* flexifunction_command_ack)
{
	return mavlink_msg_flexifunction_command_ack_pack(system_id, component_id, msg, flexifunction_command_ack->command_type, flexifunction_command_ack->result);
}

/**
 * @brief Send a flexifunction_command_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param command_type Command acknowledged
 * @param result result of acknowledge
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_flexifunction_command_ack_send(mavlink_channel_t chan, uint16_t command_type, uint16_t result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK_LEN];
	_mav_put_uint16_t(buf, 0, command_type);
	_mav_put_uint16_t(buf, 2, result);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK, buf, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK, buf, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK_LEN);
#endif
#else
	mavlink_flexifunction_command_ack_t packet;
	packet.command_type = command_type;
	packet.result = result;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK, (const char *)&packet, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK, (const char *)&packet, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK_LEN);
#endif
#endif
}

#endif

// MESSAGE FLEXIFUNCTION_COMMAND_ACK UNPACKING


/**
 * @brief Get field command_type from flexifunction_command_ack message
 *
 * @return Command acknowledged
 */
static inline uint16_t mavlink_msg_flexifunction_command_ack_get_command_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field result from flexifunction_command_ack message
 *
 * @return result of acknowledge
 */
static inline uint16_t mavlink_msg_flexifunction_command_ack_get_result(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Decode a flexifunction_command_ack message into a struct
 *
 * @param msg The message to decode
 * @param flexifunction_command_ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_flexifunction_command_ack_decode(const mavlink_message_t* msg, mavlink_flexifunction_command_ack_t* flexifunction_command_ack)
{
#if MAVLINK_NEED_BYTE_SWAP
	flexifunction_command_ack->command_type = mavlink_msg_flexifunction_command_ack_get_command_type(msg);
	flexifunction_command_ack->result = mavlink_msg_flexifunction_command_ack_get_result(msg);
#else
	memcpy(flexifunction_command_ack, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_ACK_LEN);
#endif
}
